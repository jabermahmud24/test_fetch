from __future__ import annotations
import math, random, json, os, itertools
from pathlib import Path
from typing import List, Tuple, Dict

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import torch.optim.lr_scheduler as lr_scheduler

# 1.  GLOBAL CONFIG
DT              = 0.1
T_SEG           = 1.0
N_MID           = 15          # Total midpoints (incl. 3 fixed human-guided)
N_EPOCHS        = 50
BATCH_SIZE      = 32
LR_UPPER        = 5e-5
LR_LOWER        = 1e-4
N_LOWER_ITERS   = 50
CG_ITERS        = 15
DEVICE          = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

# Loss weights (can be tuned offline)
W = {
    'goal'         : 1.0,
    'clear'        : 6.0,
    'bounds'       : 1.0,
    'orient_pass'  : 2.0,
    'orient_track' : 2.0,
    'smooth'       : 0.5,
    'reg'          : 0.01,
    'memory'       : 20.0,
    'miss'         : 5.0,
}

# 2.  GEOMETRY HELPERS

def load_object_vertices(path: str | Path) -> torch.Tensor:
    """Load a list/npz/ply/obj of vertices - here we expect a simple Nx3 .npy."""
    verts = np.load(path)
    com   = verts.mean(0)
    return torch.tensor(verts - com, dtype=torch.float32, device=DEVICE)

# Default E-block if user does not supply a mesh
_default_e_block = np.array([
    [ 2.0, 0.0, 0.0], [2.5, 0.0, 0.0], [2.5, 0.0, 0.5], [2.0, 0.0, 0.5],
    [ 2.0, 0.2, 0.0], [2.5, 0.2, 0.0], [2.5, 0.2, 0.5], [2.0, 0.2, 0.5],
])
OBJ_VERTS_LOCAL = torch.tensor(_default_e_block - _default_e_block.mean(0),
                              dtype=torch.float32, device=DEVICE)

# Workspace bounds (modifiable per scene)
WS_DEFAULT = dict(x=(0.0, 3.0), y=(0.0, 2.0), z=(0.0, 2.0))

# RPY � Rotation matrix batch helper

def rpy_to_matrix(rpy: torch.Tensor) -> torch.Tensor:
    roll, pitch, yaw = rpy.unbind(-1)
    cz, sz = torch.cos(yaw), torch.sin(yaw)
    cy, sy = torch.cos(pitch), torch.sin(pitch)
    cx, sx = torch.cos(roll), torch.sin(roll)
    Rz = torch.stack([torch.stack([cz,-sz, torch.zeros_like(cz)], -1),
                      torch.stack([sz, cz, torch.zeros_like(cz)], -1),
                      torch.stack([torch.zeros_like(cz), torch.zeros_like(cz), torch.ones_like(cz)], -1)], -2)
    Ry = torch.stack([torch.stack([cy, torch.zeros_like(cy), sy], -1),
                      torch.stack([torch.zeros_like(cy), torch.ones_like(cy), torch.zeros_like(cy)], -1),
                      torch.stack([-sy, torch.zeros_like(cy), cy], -1)], -2)
    Rx = torch.stack([torch.stack([torch.ones_like(cx), torch.zeros_like(cx), torch.zeros_like(cx)], -1),
                      torch.stack([torch.zeros_like(cx), cx, -sx], -1),
                      torch.stack([torch.zeros_like(cx), sx, cx], -1)], -2)
    return Rz @ Ry @ Rx

# Signed-distance to axis-aligned box (vectorised)

def signed_distance_box(pts: torch.Tensor, box: Dict[str, Tuple[float, float]]) -> torch.Tensor:
    xmin,xmax = box['x']; ymin,ymax = box['y']; zmin,zmax = box['z']
    dx_out = torch.where(pts[...,0]<xmin, xmin-pts[...,0], torch.where(pts[...,0]>xmax, pts[...,0]-xmax, 0.0))
    dy_out = torch.where(pts[...,1]<ymin, ymin-pts[...,1], torch.where(pts[...,1]>ymax, pts[...,1]-ymax, 0.0))
    dz_out = torch.where(pts[...,2]<zmin, zmin-pts[...,2], torch.where(pts[...,2]>zmax, pts[...,2]-zmax, 0.0))
    outside = torch.linalg.norm(torch.stack((dx_out,dy_out,dz_out),-1), dim=-1)
    dx_in = torch.minimum(pts[...,0]-xmin, xmax-pts[...,0])
    dy_in = torch.minimum(pts[...,1]-ymin, ymax-pts[...,1])
    dz_in = torch.minimum(pts[...,2]-zmin, zmax-pts[...,2])
    inside = -torch.minimum(torch.minimum(dx_in,dy_in), dz_in)
    return torch.where(outside>0, outside, inside)

# Penalty from a set of boxes

def collision_cost(verts: torch.Tensor, boxes: List[Dict], clearance: float=0.01):
    cost = 0.0
    for b in boxes:
        cost += torch.relu(clearance - signed_distance_box(verts, b)).sum(-1)
    return cost.mean() / (len(boxes) * verts.shape[-2])

# Voxel-grid encoder (very light) for environment
class VoxelEncoder(nn.Module):
    def __init__(self, grid_dim=(20,20,20)):
        super().__init__()
        self.grid_dim = grid_dim
        self.conv = nn.Sequential(
            nn.Conv3d(1, 8, 3, stride=2, padding=1), nn.ReLU(),
            nn.Conv3d(8,16,3,stride=2,padding=1), nn.ReLU(),
            nn.Flatten(),
        )
        # Calculate flat dim
        dummy = torch.zeros(1,1,*grid_dim)
        flat = self.conv(dummy).shape[1]
        self.fc = nn.Linear(flat, 64)
    def forward(self, voxel):
        return self.fc(self.conv(voxel))

# 3.  LLM-ASSISTED CONSTRAINT GENERATOR (OFFLINE)

class ConstraintLLM:
    """Offline helper that queries an LLM (or returns scripted rules in this stub)."""
    def __init__(self, model_name: str = 'gpt-4o', temperature: float = 0.2):
        self.model_name = model_name
        self.temperature = temperature
    def generate(self, scene_desc: str) -> Dict:
        """Return a dict describing extra constraints or waypoints."""
        # In production, call OpenAI API here.  For now we demo one rule.
        if 'narrow door' in scene_desc.lower():
            return {
                'type': 'orient_align',
                'axis': 'yaw',
                'target': 0.0,
                'weight': 2.5,
            }
        # default - no extra constraints
        return {}

LLM_AGENT = ConstraintLLM()

# Convert LLM constraint dict into a differentiable penalty

def llm_constraint_penalty(traj: torch.Tensor, c: Dict):
    if not c: return torch.tensor(0.0, device=traj.device)
    if c.get('type') == 'orient_align':
        axis_idx = {'roll':0, 'pitch':1, 'yaw':2}[c['axis']]
        target   = c['target']
        weight   = c['weight']
        return weight * (traj[:,:,3+axis_idx]-target).pow(2).mean()
    return torch.tensor(0.0, device=traj.device)

# 4.  WAYPOINT NETWORK (with env voxel + start/goal)

class WaypointNet(nn.Module):
    def __init__(self, n_mid=N_MID, enc_dim=64):
        super().__init__()
        self.n_mid = n_mid
        self.encoder = VoxelEncoder()  # encode 3-D occupancy grid
        self.fc = nn.Sequential(
            nn.Linear(enc_dim + 12, 256), nn.ReLU(),
            nn.Linear(256,256), nn.ReLU(),
            nn.Linear(256,n_mid*6)
        )
        # bias init near workspace centre
        nn.init.zeros_(self.fc[-1].weight)
        with torch.no_grad():
            self.fc[-1].bias.uniform_(-0.5,0.5)
    def forward(self, start, goal, voxel):
        feat = self.encoder(voxel)
        x = torch.cat([start, goal, feat],-1)
        out = self.fc(x)
        return out.view(-1, self.n_mid, 6)

# 5.  CATMULL-ROM SPLINE

def catmull_rom(ctrl: torch.Tensor, k: int) -> torch.Tensor:
    B,K,D = ctrl.shape
    t = torch.linspace(0,1,k+1, device=ctrl.device)[:-1]
    t2, t3 = t*t, t*t*t
    segs = []
    for i in range(K-1):
        P0 = ctrl[:,i-1] if i>0 else ctrl[:,i]
        P1,P2 = ctrl[:,i], ctrl[:,i+1]
        P3 = ctrl[:,i+2] if i+2<K else ctrl[:,i+1]
        a = 2*P1
        b = P2-P0
        c = 2*P0-5*P1+4*P2-P3
        d = -P0+3*P1-3*P2+P3
        seg = 0.5*(a[:,None]+b[:,None]*t[:,None]+c[:,None]*t2[:,None]+d[:,None]*t3[:,None])
        segs.append(seg)
    segs.append(ctrl[:,-1:].expand(B,1,D))
    return torch.cat(segs,1)

# 6.  MEMORY FOR COLLISIONS

class CollisionMemory:
    def __init__(self, max_size=200):
        self.buf: List[Tuple[torch.Tensor, float]] = []
        self.max_size=max_size
    def add(self, mid, cost):
        self.buf.append((mid.detach().cpu(), cost))
        if len(self.buf) > self.max_size:
            self.buf.pop(0)
    def repel_penalty(self, mid: torch.Tensor, sigma2=0.1):
        if not self.buf: return torch.tensor(0.0, device=mid.device)
        pen=0.0
        for m,c in self.buf:
            m=m.to(mid.device)
            pen+=c*torch.exp(-(mid-m).pow(2).sum()/sigma2)
        return pen/len(self.buf)
MEM = CollisionMemory()

# 7.  LOSSES

def lower_loss(traj, mid, mid_init, boxes, ws, extra_pen):
    B,T,_ = traj.shape
    verts_local = OBJ_VERTS_LOCAL[None].expand(B,-1,-1)
    # Collision cost
    L_clear = 0.0
    for t in range(T):
        pose = traj[:,t]
        R = rpy_to_matrix(pose[:,3:])
        # verts = (R @ verts_local.permute(0,2,1)).permute(0,2,1) + pose[:,:3,None]
        verts = (R @ verts_local.permute(0,2,1)).permute(0,2,1) + pose[:, :3].unsqueeze(1)

        L_clear += collision_cost(verts, boxes)
    L_clear /= T
    # Bounds
    L_bounds = sum(torch.relu(ws[k][0]-traj[:,:,i]+0.1)+torch.relu(traj[:,:,i]-ws[k][1]+0.1)
                   for i,k in enumerate(('x','y','z'))).mean()
    # Smooth (pos+ori)
    diff = traj[:,1:,:]-traj[:,:-1,:]
    L_smooth = diff.pow(2).mean()
    # Regularise to net initial
    L_reg = (mid-mid_init).pow(2).mean()
    return (W['clear']*L_clear + W['bounds']*L_bounds + W['smooth']*L_smooth + W['reg']*L_reg + extra_pen)


def upper_loss(traj, start, goal, boxes, miss_pen):
    end_pos = (traj[:,-2,:3]-goal[:,:3]).pow(2).sum(-1)
    # end_pos = (traj[:,-1,:3]-goal[:,:3]).pow(2).sum(-1)
    end_ori = (traj[:,-2,3:]-goal[:,3:]).pow(2).sum(-1)
    # end_ori = (traj[:,-1,3:]-goal[:,3:]).pow(2).sum(-1)
    L_goal = (end_pos + 0.1*end_ori).mean()
    # Collision quick check
    L_coll = 0.0
    for t in range(0,traj.shape[1],4):
        pose = traj[:,t]
        R = rpy_to_matrix(pose[:,3:])
        # verts = (R @ OBJ_VERTS_LOCAL.T).permute(1,0,2) + pose[:,:3,None]
        verts_local = OBJ_VERTS_LOCAL.unsqueeze(0).expand(R.size(0), -1, -1)
        rotated     = (R @ verts_local.permute(0,2,1)).permute(0,2,1)
        translation = pose[:, :3].unsqueeze(1)
        verts       = rotated + translation
        L_coll += collision_cost(verts, boxes)
    # L_coll = tray.shape[1]//4
    return (W['goal']*L_goal + W['miss']*miss_pen + W['clear']*L_coll)

# 8.  TRAINING UTILITIES

def random_scene(batch: int):
    """Generate random obstacles and start/goal poses."""
    boxes=[]
    # two walls with small gap (door) as before
    DOOR_X_MIN,DOOR_X_MAX=1.4,1.43
    DOOR_Y_MIN,DOOR_Y_MAX=0.9,1.2
    boxes.append(dict(x=(DOOR_X_MIN,DOOR_X_MAX), y=(0.0,DOOR_Y_MIN), z=(0.0,1.2)))
    boxes.append(dict(x=(DOOR_X_MIN,DOOR_X_MAX), y=(DOOR_Y_MAX,2.0), z=(0.0,1.2)))
    # Add a few random blocks
    for _ in range(random.randint(1,4)):
        cx,cy,cz=[random.uniform(0.3,2.7), random.uniform(0.3,1.7), 0.0]
        w,h,d=[random.uniform(0.1,0.3) for _ in range(3)]
        boxes.append(dict(x=(cx,cx+w), y=(cy,cy+h), z=(cz,cz+d)))
    start=torch.tensor([[2.2,0.6,0.25, math.pi/2,math.pi/4,math.pi/8]],device=DEVICE)
    goal =torch.tensor([[0.5,1.2,0.4, math.pi/4,math.pi/8,math.pi/6]],device=DEVICE)
    start=start.repeat(batch,1)
    goal =goal.repeat(batch,1)
    return start,goal,boxes,WS_DEFAULT

# Occupancy voxel grid generator

def boxes_to_voxel(boxes, ws, grid=(20,20,20)):
    gx,gy,gz = grid
    xs = torch.linspace(ws['x'][0],ws['x'][1],gx)
    ys = torch.linspace(ws['y'][0],ws['y'][1],gy)
    zs = torch.linspace(ws['z'][0],ws['z'][1],gz)
    xv,yv,zv = torch.meshgrid(xs,ys,zs,indexing='ij')
    occ = torch.zeros(gx,gy,gz)
    pts = torch.stack((xv,yv,zv),-1)
    for b in boxes:
        mask = (pts[...,0]>=b['x'][0])&(pts[...,0]<=b['x'][1]) & \
               (pts[...,1]>=b['y'][0])&(pts[...,1]<=b['y'][1]) & \
               (pts[...,2]>=b['z'][0])&(pts[...,2]<=b['z'][1])
        occ[mask] = 1
    return occ.unsqueeze(0)   # (1,gx,gy,gz)

# 9.  LOWER LEVEL OPTIMISER

def optimise_lower(mid_init, start, goal, boxes, ws, llm_pen, k):
    batch = start.size(0)
    # mid_var = mid_init[:,3:,:].clone().requires_grad_(True)  # skip first 3 guided
    mid_var = mid_init[:,3:,:].clone().detach().requires_grad_(True)

    opt = optim.Adam([mid_var], lr=LR_LOWER)
    sched= lr_scheduler.StepLR(opt,10,0.9)
    for _ in range(N_LOWER_ITERS):
        opt.zero_grad()
        mid_full = torch.cat([mid_init[:,:3,:].detach(), mid_var],1)
        ctrl = torch.cat([start[:,None,:], mid_full, goal[:,None,:]],1)
        traj = catmull_rom(ctrl,k)
        loss = lower_loss(traj, mid_full, mid_init.detach(), boxes, ws, llm_pen)
        loss.backward()
        torch.nn.utils.clip_grad_norm_([mid_var],1.0)
        opt.step(); sched.step()
    # mid_final = torch.cat([mid_init[:,:3,:].detach(), mid_var.detach()],1)
    mid_final = torch.cat([mid_init[:,:3,:].detach(), mid_var], 1)
    return mid_final

# 10.  IMPLICIT GRADIENT�STEP

def cg_solve(hvp, b, n=CG_ITERS):
    x = torch.zeros_like(b); r=b.clone(); p=r.clone(); rs=torch.dot(r.view(-1),r.view(-1))
    for _ in range(n):
        Ap=hvp(p); a=rs/(torch.dot(p.view(-1),Ap.view(-1))+1e-9)
        x+=a*p; r-=a*Ap; rs_new=torch.dot(r.view(-1),r.view(-1))
        if torch.sqrt(rs_new)<1e-8: break
        p=r+rs_new/rs*p; rs=rs_new
    return x

def implicit_update(loss_upper, loss_lower, net, mids_var):
    psi=list(net.parameters())
    # v = torch.autograd.grad(loss_upper, mids_var, retain_graph=True, allow_unused=True)[0]
    v = torch.autograd.grad(loss_upper, mids_var, retain_graph=True, allow_unused=True)[0]
    if v is None:
        # nothing to do if loss_upper doesn't depend on mids_var
        return
    print("implicit v is", v)
    def hvp(p):
        g=torch.autograd.grad(loss_lower, mids_var, create_graph=True, retain_graph=True)[0]
        return torch.autograd.grad(g, mids_var, grad_outputs=p, retain_graph=True)[0] + 0.1*p
    q = cg_solve(hvp, v)
    g_phi=torch.autograd.grad(loss_lower, mids_var, create_graph=True)[0]
    implicit_grads=torch.autograd.grad(g_phi, psi, grad_outputs=-q.detach())
    for p,g in zip(psi,implicit_grads):
        p.grad=g

# 11.  TRAIN LOOP

def train():
    net = WaypointNet().to(DEVICE)
    opt_upper = optim.Adam(net.parameters(), lr=LR_UPPER)
    sched_up  = lr_scheduler.StepLR(opt_upper, 30, 0.8)
    k = int(T_SEG/DT)
    for ep in range(1, N_EPOCHS+1):
        start,goal,boxes,ws = random_scene(BATCH_SIZE)
        # Scene description for LLM (string)
        scene_str = f"start={start[0,:3].tolist()}, goal={goal[0,:3].tolist()}, boxes={len(boxes)}"
        extra_c   = LLM_AGENT.generate(scene_str)
        # Pre-compute voxel grid
        vox = boxes_to_voxel(boxes, ws).to(DEVICE).repeat(BATCH_SIZE,1,1,1,1)
        mid_init = net(start, goal, vox)
        # Optimise lower level
        llm_pen_dummy = torch.tensor(0.0, device=DEVICE)  # will add later if needed
        mid_opt = optimise_lower(mid_init, start, goal, boxes, ws, llm_pen_dummy, k)
        ctrl    = torch.cat([start[:,None,:], mid_opt, goal[:,None,:]],1)
        traj    = catmull_rom(ctrl,k)
        miss    = torch.tensor(0.0, device=DEVICE)  # door miss penalty placeholder
        loss_up = upper_loss(traj, start, goal, boxes, miss)
        loss_lo = lower_loss(traj, mid_opt, mid_init, boxes, ws, llm_pen_dummy)
        # Implicit diff
        opt_upper.zero_grad(); implicit_update(loss_up, loss_lo, net, mid_opt[:,3:,:]); opt_upper.step(); sched_up.step()
        if ep%1==0:
            print(f"Ep {ep:03d}  upper={loss_up.item():.4f}  lower={loss_lo.item():.4f}")
    torch.save(net.state_dict(), "waypointnet_general_with_LLM.pth")

if __name__ == "__main__":
    train()