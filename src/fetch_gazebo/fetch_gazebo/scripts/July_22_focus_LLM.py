from __future__ import annotations
import math, random, json, os, functools, pathlib
from typing import List, Tuple, Dict, Union

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import torch.optim.lr_scheduler as lr_scheduler
import openai
import time 
import re
import ollama


# 1.  GLOBAL CONFIG
DT, T_SEG = 0.1, 1.0
N_MID, N_GUIDED = 25, 3                     # total | human-guided
N_EPOCHS, BATCH_SIZE = 300, 32
LR_UPPER, LR_LOWER = 5e-5, 1e-4
N_LOWER_ITERS, CG_ITERS = 50, 15
DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

W = {           # loss weights
    'goal'  : 1.0,
    'clear' : 5.0,
    'bounds': 3.0,
    'smooth': 0.5,
    'reg'   : 0.1,
    'memory': 20.0,
    'orient_pass' : 0.1,
    'orient_track': 0.1,
    'miss'        : 4.0,
}

# 2.  ENVIRONMENT GEOMETRY
H_GUIDED = torch.tensor([
    [2.2, 0.6, 0.25, math.pi/2, math.pi, 0],
    [2.1, 0.7, 0.30, math.pi/2, math.pi, 0],
    [2.0, 0.8, 0.35, math.pi/2, math.pi, 0],
], dtype=torch.float32, device=DEVICE)                       # (3,6)

DOOR_X_MIN, DOOR_X_MAX = 1.4, 1.43
DOOR_Y_MIN, DOOR_Y_MAX = 0.9, 1.2
DOOR_Z_MIN, DOOR_Z_MAX = 0.0, 1.2
WS_DEFAULT = {"x": (0.0, 3.0), "y": (0.0, 2.0), "z": (0.0, 2.0)}
WALLS = (
    {"x": (DOOR_X_MIN, DOOR_X_MAX), "y": (0.0, DOOR_Y_MIN), "z": (DOOR_Z_MIN, DOOR_Z_MAX)},
    {"x": (DOOR_X_MIN, DOOR_X_MAX), "y": (DOOR_Y_MAX, WS_DEFAULT["y"][1]), "z": (DOOR_Z_MIN, DOOR_Z_MAX)},
)

OBJ_VERTS_LOCAL = torch.tensor([
     [2.0, 0.0, 0.0], [2.5, 0.0, 0.0], [2.5, 0.0, 0.5], [2.0, 0.0, 0.5],
     [2.0, 0.2, 0.0], [2.5, 0.2, 0.0], [2.5, 0.2, 0.5], [2.0, 0.2, 0.5],
], dtype=torch.float32, device=DEVICE)
LOCAL_COM       = OBJ_VERTS_LOCAL.mean(dim=0)
LOCAL_COM_NP    = LOCAL_COM.cpu().numpy()

# 3.  MATH HELPERS
def rpy_to_matrix(rpy: torch.Tensor) -> torch.Tensor:
    roll, pitch, yaw = rpy.unbind(-1)
    cz, sz = torch.cos(yaw), torch.sin(yaw)
    cy, sy = torch.cos(pitch), torch.sin(pitch)
    cx, sx = torch.cos(roll), torch.sin(roll)
    Rz = torch.stack([torch.stack([cz,-sz,torch.zeros_like(cz)],-1),
                      torch.stack([sz, cz,torch.zeros_like(cz)],-1),
                      torch.stack([torch.zeros_like(cz),torch.zeros_like(cz),torch.ones_like(cz)],-1)], -2)
    Ry = torch.stack([torch.stack([cy,torch.zeros_like(cy),sy],-1),
                      torch.stack([torch.zeros_like(cy),torch.ones_like(cy),torch.zeros_like(cy)],-1),
                      torch.stack([-sy,torch.zeros_like(cy),cy],-1)], -2)
    Rx = torch.stack([torch.stack([torch.ones_like(cx),torch.zeros_like(cx),torch.zeros_like(cx)],-1),
                      torch.stack([torch.zeros_like(cx),cx,-sx],-1),
                      torch.stack([torch.zeros_like(cx),sx, cx],-1)], -2)
    return Rz @ Ry @ Rx

def signed_distance_box(pts: torch.Tensor, box: dict) -> torch.Tensor:
    xmin,xmax = box['x']; ymin,ymax = box['y']; zmin,zmax = box['z']
    dx = torch.where(pts[...,0]<xmin, xmin-pts[...,0],
         torch.where(pts[...,0]>xmax, pts[...,0]-xmax, torch.zeros_like(pts[...,0])))
    dy = torch.where(pts[...,1]<ymin, ymin-pts[...,1],
         torch.where(pts[...,1]>ymax, pts[...,1]-ymax, torch.zeros_like(pts[...,1])))
    dz = torch.where(pts[...,2]<zmin, zmin-pts[...,2],
         torch.where(pts[...,2]>zmax, pts[...,2]-zmax, torch.zeros_like(pts[...,2])))
    outside = torch.linalg.norm(torch.stack((dx,dy,dz),dim=-1),dim=-1)
    dx_in = torch.minimum(pts[...,0]-xmin, xmax-pts[...,0])
    dy_in = torch.minimum(pts[...,1]-ymin, ymax-pts[...,1])
    dz_in = torch.minimum(pts[...,2]-zmin, zmax-pts[...,2])
    inside = -torch.minimum(torch.minimum(dx_in,dy_in), dz_in)
    return torch.where(outside>0, outside, inside)

def collision_cost(verts: torch.Tensor, boxes: List[Dict], clr:float=0.01):
    loss = 0.0
    for b in boxes:
        loss += torch.relu(clr - signed_distance_box(verts,b)).sum(-1)
    return loss.mean() / (len(boxes)*verts.shape[-2])

# 4.  VOXEL ENCODER
class VoxelEncoder(nn.Module):
    def __init__(self, grid=(20,20,20)):
        super().__init__()
        self.conv = nn.Sequential(
            nn.Conv3d(1,8,3,stride=2,padding=1), nn.ReLU(),
            nn.Conv3d(8,16,3,stride=2,padding=1), nn.ReLU(),
            nn.Flatten()
        )
        dummy = torch.zeros(1,1,*grid)
        flat = self.conv(dummy).shape[1]
        self.fc = nn.Linear(flat,64)
    def forward(self,x): return self.fc(self.conv(x))

# 5.  LLM WRAPPER
# def scene_to_prompt(boxes, start, goal):
#     def fmt(b): return f"x[{b['x'][0]:.2f},{b['x'][1]:.2f}] y[{b['y'][0]:.2f},{b['y'][1]:.2f}] z[{b['z'][0]:.2f},{b['z'][1]:.2f}]"
#     txt  = "Obstacles:\n" + "\n".join(map(fmt, boxes))
#     txt += f"\nStart pose: {start.tolist()}\nGoal pose: {goal.tolist()}"
#     txt += "\nReturn JSON with fields 'constraints' (list) and 'waypoints' (list of 6-D)."
#     return txt


def scene_to_prompt(boxes, start, goal):
    # allow start/goal to be either torch.Tensor or plain list
    if hasattr(start, "tolist"):
        start_list = start.tolist()
    else:
        start_list = start
    if hasattr(goal, "tolist"):
        goal_list = goal.tolist()
    else:
        goal_list = goal

    def fmt(b):
        return f"x[{b['x'][0]:.2f},{b['x'][1]:.2f}] y[{b['y'][0]:.2f},{b['y'][1]:.2f}] z[{b['z'][0]:.2f},{b['z'][1]:.2f}]"
    txt  = "Obstacles:\n" + "\n".join(map(fmt, boxes))
    txt += f"\nStart pose: {start_list}\nGoal pose: {goal_list}"
    txt += "\nReturn JSON with fields 'constraints' (list) and 'waypoints' (list of 6-D)."
    return txt

system_msg = """
You are a robotics motion-planning assistant. I will give you:
1. A list called “Obstacles” (each with x, y, z ranges).
2. A “Start pose” and a “Goal pose” (each as a 6-element array).

Respond with exactly one JSON object (no extra text) containing:
- "constraints": an array of objects, each with:
    • "type": must be one of ["avoid_box", "orient_align", "height_min"]
    • if "avoid_box": include "region" (with x, y, z range arrays) and "weight" (number)
    • if "orient_align": include "axis", "target", "region", and "weight"
    • if "height_min": include "z_min" and "weight"
- "waypoints": an array of 6-element numeric arrays

Here’s an example of exactly what to return (including braces, brackets, quotes, and commas):

```json
{
  "constraints": [
    {
      "type": "avoid_box",
      "region": { "x":[1.4,1.43], "y":[0.0,0.9], "z":[0.0,1.2] },
      "weight": 6.0
    },
    {
      "type": "height_min",
      "z_min": 0.25,
      "weight": 1.0
    }
  ],
  "waypoints": [
    [2.2, 0.6, 0.25, 1.5708, 3.1416, 0.0],
    [0.5, 1.2, 0.4, 0.7854, 0.3927, 0.5236]
  ]
}
"""

class ConstraintLLM:
    def __init__(self, model: str = "llama3"):
        self.model = model
        self._cache = {}

    def _call_model(self, prompt: str) -> dict:
        # 1) Query the model
        resp = ollama.chat(
            model=self.model,
            messages=[
                {"role":"system", 
                "content": system_msg},
                # "content":"You are a motion-planning assistant. Return ONLY valid JSON—no prose."},
                {"role":"user","content":prompt}
            ]
        )
        raw = resp["message"]["content"]

        # 2) Extract from first '{' through last '}'
        start = raw.find("{")
        end   = raw.rfind("}") + 1
        if start < 0 or end < 1:
            raise RuntimeError(f"No JSON object found:\n{raw!r}")
        blob = raw[start:end]

        # 3) Auto-balance braces if truncated
        opens  = blob.count("{")
        closes = blob.count("}")
        if closes < opens:
            blob += "}" * (opens - closes)

        # 4) Remove any trailing commas which break strict JSON
        #    e.g. turn `[...,]` into `[...]`, same for objects
        blob = re.sub(r",\s*([\]}])", r"\1", blob)

        # 5) Attempt to parse full JSON
        return json.loads(blob)

    def generate(self, boxes, start, goal) -> dict:
        prompt = scene_to_prompt(boxes, start, goal)
        if prompt in self._cache:
            return self._cache[prompt]

        try:
            answer = self._call_model(prompt)
            # ensure minimal schema
            answer.setdefault("constraints", [])
            answer.setdefault("waypoints", [])
        except Exception as e:
            # on ANY parsing error, log and fallback to no constraints
            print(f"[LLM parse error] {e!r}\nFalling back to empty constraints.")
            answer = {"constraints": [], "waypoints": []}

        # print(answer)
        self._cache[prompt] = answer
        return answer
    



    ###################################################



    # def _call_model(self, prompt: str) -> dict:
    #     resp = ollama.chat(
    #         model=self.model,
    #         messages=[
    #             {"role":"system",
    #             "content":"You are a motion-planning assistant. Return ONLY valid JSON—no placeholders, no prose."},
    #             {"role":"user","content":prompt}
    #         ]
    #     )
    #     raw = resp["message"]["content"]

    #     # 1) Isolate from first '{' to last '}' (inclusive)
    #     start = raw.find("{")
    #     end   = raw.rfind("}") + 1
    #     if start < 0 or end < 1:
    #         raise RuntimeError(f"No JSON object found in:\n{raw!r}")
    #     blob = raw[start:end]

    #     # 2) Auto-balance braces: count opens vs closes and append missing
    #     opens  = blob.count("{")
    #     closes = blob.count("}")
    #     if closes < opens:
    #         blob += "}" * (opens - closes)

    #     # 3) Attempt full parse
    #     try:
    #         return json.loads(blob)
    #     except json.JSONDecodeError:
    #         # 4) Fallback: extract constraints array only
    #         m = re.search(r'"constraints"\s*:\s*(\[[\s\S]*?\])', blob)
    #         if not m:
    #             raise RuntimeError(f"Could not parse constraints from:\n{blob!r}")
    #         constraints = json.loads(m.group(1))
    #         return {"constraints": constraints, "waypoints": []}

    # def generate(self, boxes, start, goal):
    #     prompt = scene_to_prompt(boxes, start, goal)
    #     if prompt in self._cache:
    #         return self._cache[prompt]

    #     answer = self._call_model(prompt)

    #     # 1) pull raw list
    #     raw = answer.get("constraints", [])

    #     # 2) normalized list
    #     normalized = []
    #     for c in raw:
    #         if "type" in c:
    #             normalized.append(c)   # already a typed constraint
    #         else:
    #             # assume this is a box dict ⇒ avoid-collision penalty
    #             normalized.append({
    #                 "type": "avoid_box",
    #                 "region": {"x": c["x"], "y": c["y"], "z": c["z"]},
    #                 "weight": 6.0    # same weight as your 'clear' loss
    #             })
    #     answer["constraints"] = normalized
    #     answer.setdefault("waypoints", [])
    #     self._cache[prompt] = answer
    #     return answer



# class ConstraintLLM:
#     """Queries OpenAI if key present, otherwise returns heuristic."""
#     def __init__(self, model="gpt-4o-mini", temp=0.2):
#         self.model, self.temp = model, temp
#         self.use_api = bool(os.getenv("OPENAI_API_KEY"))
#         if self.use_api:
#             from openai import OpenAI
#             # import openai
#             self.openai = OpenAI
#             # self.openai = openai
#         self._cache:Dict[str,dict] = {}
#     def generate(self, boxes, start, goal):
#         prompt = scene_to_prompt(boxes, start, goal)
#         if prompt in self._cache: return self._cache[prompt]
#         if self.use_api:
#             rsp = self.openai.ChatCompletion.create(
#                 model=self.model, temperature=self.temp,
#                 messages=[{"role":"user","content":prompt}]
#             )
#             out = json.loads(rsp.choices[0].message.content)
#         else:   # rule-based fallback
#             out = {
#                 "constraints":[
#                     {"type":"orient_align", "axis":"yaw", "target":0.0, "weight":3.0,
#                      "region":{"x":[DOOR_X_MIN,DOOR_X_MAX], "y":[DOOR_Y_MIN,DOOR_Y_MAX]}},
#                     {"type":"height_min", "z_min":0.25, "weight":1.0}
#                 ],
#                 "waypoints":[
#                     [(DOOR_X_MAX+0.15), (DOOR_Y_MIN+DOOR_Y_MAX)/2, 0.3,
#                      math.pi/2, math.pi, 0.0]
#                 ]
#             }
#         self._cache[prompt] = out
#         print(out)
#         return out

LLM_AGENT = ConstraintLLM()


def llm_constraint_penalty(traj, info):
    total = torch.tensor(0., device=traj.device)
    for c in info.get("constraints", []):
        if c["type"] == "orient_align":
            axis={'roll':0,'pitch':1,'yaw':2}[c["axis"]]
            x,y = traj[:,:,0], traj[:,:,1]
            mask = (x>=c["region"]["x"][0])&(x<=c["region"]["x"][1])&\
                   (y>=c["region"]["y"][0])&(y<=c["region"]["y"][1])
            total += c["weight"] * ((traj[:,:,3+axis]-c["target"])**2 * mask).mean()
        elif c["type"] == "height_min":
            total += c["weight"]*torch.relu(c["z_min"]-traj[:,:,2]).mean()
        elif c["type"] == "avoid_box":
            # treat region exactly like collision_cost but as a soft penalty
            # build verts for one time‐step (for simplicity, use first frame)
            # inside llm_constraint_penalty, for each batch element B

            # 1) pick the pose at t=0
            #    traj has shape (B, T, 6) → pose0 is (B, 6)
            pose0 = traj[:, 0, :]              

            # 2) build a rotation matrix for each batch from the RPY angles
            #    pose0[:,3:] are the roll/pitch/yaw
            R0 = rpy_to_matrix(pose0[:, 3:])   # shape (B, 3, 3)

            # 3) get your local‐mesh vertices, expanded to the batch
            #    OBJ_VERTS_LOCAL is (Nv, 3), so we unsqueeze + expand → (B, Nv, 3)
            verts_local = OBJ_VERTS_LOCAL.unsqueeze(0).expand(traj.shape[0], -1, -1)

            # 4) rotate & translate into world frame:
            #    (B, 3, 3) @ (B, 3, Nv) → (B, 3, Nv), then permute → (B, Nv, 3)
            world_verts = (R0 @ verts_local.permute(0, 2, 1)).permute(0, 2, 1)
            verts = world_verts + pose0[:, :3].unsqueeze(1)  # add the x,y,z offset

            # `verts` now has shape (B, Nv, 3) giving each vertex’s world‐coords at t=0

            # compute how far inside that box the verts are
            total += c["weight"] * torch.relu(
                0.01 - signed_distance_box(verts, c["region"])
            ).mean()
    return total


# def llm_constraint_penalty(traj: torch.Tensor, info: Dict)->torch.Tensor:
#     if not info: return torch.tensor(0.0,device=traj.device)
#     total = torch.tensor(0., device=traj.device)
#     for c in info.get("constraints", []):
#         if c["type"]=="orient_align":
#             axis={'roll':0,'pitch':1,'yaw':2}[c["axis"]]
#             x,y = traj[:,:,0], traj[:,:,1]
#             mask = (x>=c["region"]["x"][0])&(x<=c["region"]["x"][1])&\
#                    (y>=c["region"]["y"][0])&(y<=c["region"]["y"][1])
#             total += c["weight"] * ((traj[:,:,3+axis]-c["target"])**2 * mask).mean()
#         elif c["type"]=="height_min":
#             total += c["weight"]*torch.relu(c["z_min"]-traj[:,:,2]).mean()
#     return total

# 6.  WAYPOINT NETWORK
class WaypointNet(nn.Module):
    def __init__(self, n_mid=N_MID):
        super().__init__()
        self.n_mid = n_mid
        self.enc   = VoxelEncoder()
        self.fc    = nn.Sequential(
            nn.Linear(64+12,256), nn.ReLU(),
            nn.Linear(256,256), nn.ReLU(),
            nn.Linear(256,256), nn.ReLU(),
            nn.Linear(256,n_mid*6)
        )
        # init bias to last guided wp
        with torch.no_grad():
            bias = H_GUIDED[-1].repeat(n_mid)
            self.fc[-1].bias.copy_(bias)
            nn.init.zeros_(self.fc[-1].weight)
    def forward(self,start,goal,vox):
        feat=self.enc(vox)
        x=torch.cat([start,goal,feat],-1)
        return self.fc(x).view(-1,self.n_mid,6)

# 7.  SPLINE, MEMORY, LOSS
# (unchanged except llm penalty injection & weight dict)
def catmull_rom(ctrl: torch.Tensor, steps:int)->torch.Tensor:
    B,K,D=ctrl.shape
    t = torch.linspace(0,1,steps+1,device=ctrl.device)[:-1]
    t2,t3=t*t,t*t*t
    out=[]
    for i in range(K-1):
        P0=ctrl[:,i-1] if i>0 else ctrl[:,i]
        P1,P2=ctrl[:,i],ctrl[:,i+1]
        P3=ctrl[:,i+2] if i+2<K else ctrl[:,i+1]
        a,b,c,d = 2*P1, P2-P0, 2*P0-5*P1+4*P2-P3, -P0+3*P1-3*P2+P3
        seg=0.5*(a[:,None]+b[:,None]*t[:,None]+c[:,None]*t2[:,None]+d[:,None]*t3[:,None])
        out.append(seg)
    out.append(ctrl[:,-1:].expand(B,1,D))
    return torch.cat(out,1)            # (B,T,D)

class CollisionMemory:
    def __init__(self,max_size=200):
        self.buf:List[Tuple[torch.Tensor,float]]=[]
        self.max_size=max_size
    def add(self,mid,cost):
        self.buf.append((mid.detach().cpu(),cost))
    #     if len(self.buf)>self.max_size: self.buf.pop(0)
    # def repel_penalty(self,mid,sigma2=0.1):
    #     if not self.buf: return torch.tensor(0.0,device=mid.device)
    #     pen=0.0
    #     for m,c in self.buf:
    #         m=m.to(mid.device)
    #         pen+=c*torch.exp(-(mid-m).pow(2).sum()/sigma2)
    #     return pen/len(self.buf)

    def repel_penalty(self, mid: torch.Tensor, sigma2: float = 0.1) -> torch.Tensor:
        """
        Compute a repulsion penalty between `mid` (shape [B, M, D]) and each
        stored buffer entry of matching shape.  Ignores any with a different M.
        """
        if not self.buf:
            return torch.tensor(0.0, device=mid.device)

        pen = torch.tensor(0.0, device=mid.device)
        valid_count = 0

        for m, c in self.buf:
            # m: CPU tensor of shape [B, M_mem, D]
            # we only compare if M_mem == M_current
            if m.ndim != mid.cpu().ndim or m.shape[1] != mid.shape[1]:
                continue

            # move to same device and dtype
            m_dev = m.to(mid.device).type_as(mid)
            # compute squared-distance repulsion
            dist2 = (mid - m_dev).pow(2).sum(dim=(1,2))  # sum over mid-points & dims
            pen   += c * torch.exp(-dist2 / sigma2).mean()
            valid_count += 1

        if valid_count == 0:
            return torch.tensor(0.0, device=mid.device)

        # average over the valid entries
        return pen / valid_count
MEM = CollisionMemory()

def door_miss_penalty(pos):
    x,y=pos[:,:,0],pos[:,:,1]
    slab=(x>=DOOR_X_MIN)&(x<=DOOR_X_MAX)
    inside=(y>=DOOR_Y_MIN)&(y<=DOOR_Y_MAX)
    success=(slab&inside).any(-1).float()
    return (1.0-success).mean()

def orient_pass_penalty(traj):
    x=traj[:,:,0]; yaw=traj[:,:,5]
    mask=((x>=DOOR_X_MIN)&(x<=DOOR_X_MAX)).float()
    denom=mask.sum()+1e-6
    return (mask*yaw.abs()).sum()/denom

def orient_track_penalty(traj,start,goal):
    B,T,_=traj.shape
    t=torch.linspace(0,1,T,device=traj.device)[None,:,None]
    target=start[:,None,3:]+t*(goal[:,None,3:]-start[:,None,3:])
    return (traj[:,:,3:]-target).pow(2).mean()

def lower_loss(traj,mid,mid_init,boxes,ws,llm_info):
    B,T,_=traj.shape
    verts_local=OBJ_VERTS_LOCAL[None].expand(B,-1,-1)
    L_clear=0.0
    for t in range(T):
        pose=traj[:,t]
        R=rpy_to_matrix(pose[:,3:])
        verts=(R@verts_local.permute(0,2,1)).permute(0,2,1)+pose[:,:3].unsqueeze(1)
        L_clear+=collision_cost(verts,boxes)
    L_clear/=T
    L_bounds=sum(torch.relu(ws[k][0]-traj[:,:,i]+0.1)+torch.relu(traj[:,:,i]-ws[k][1]+0.1)
                 for i,k in enumerate(('x','y','z'))).mean()
    L_smooth=(traj[:,1:,:]-traj[:,:-1,:]).pow(2).mean()
    L_reg=(mid-mid_init).pow(2).mean()
    L_mem=MEM.repel_penalty(mid)
    L_llm=llm_constraint_penalty(traj,llm_info)
    return (W['clear']*L_clear + W['bounds']*L_bounds + W['smooth']*L_smooth +
            W['reg']*L_reg + W['memory']*L_mem + L_llm)

def upper_loss(traj,start,goal,boxes):
    L_goal=((traj[:,-1,:3]-goal[:,:3]).pow(2).sum(-1)+0.1*(traj[:,-1,3:]-goal[:,3:]).pow(2).sum(-1)).mean()
    L_coll=0.0
    for t in range(0,traj.shape[1],4):
        pose=traj[:,t]; R=rpy_to_matrix(pose[:,3:])
        verts=(R@OBJ_VERTS_LOCAL.T).permute(0,2,1)+pose[:,:3].unsqueeze(1)
        L_coll+=collision_cost(verts,boxes)
    L_orient_pass=orient_pass_penalty(traj)
    L_orient_track=orient_track_penalty(traj,start,goal)
    L_miss=door_miss_penalty(traj[:,:,:3])
    return (W['goal']*L_goal + W['clear']*L_coll +
            W['orient_pass']*L_orient_pass + W['orient_track']*L_orient_track +
            W['miss']*L_miss)

# 8.  SCENE & VOXEL
def random_scene(batch:int):
    boxes=[dict(x=(DOOR_X_MIN,DOOR_X_MAX), y=(0.0,DOOR_Y_MIN), z=(0.0,1.2)),
           dict(x=(DOOR_X_MIN,DOOR_X_MAX), y=(DOOR_Y_MAX,2.0),   z=(0.0,1.2))]
    for _ in range(random.randint(1,4)):
        cx,cy,cz=[random.uniform(0.3,2.7), random.uniform(0.3,1.7), 0.0]
        w,h,d=[random.uniform(0.1,0.3) for _ in range(3)]
        boxes.append(dict(x=(cx,cx+w), y=(cy,cy+h), z=(cz,cz+d)))
    start=torch.tensor([[2.2,0.6,0.25, math.pi/2,math.pi/4,math.pi/8]],device=DEVICE).repeat(batch,1)
    goal =torch.tensor([[0.5,1.2,0.4,  math.pi/4,math.pi/8,math.pi/6]],device=DEVICE).repeat(batch,1)
    return start,goal,boxes,WS_DEFAULT

def boxes_to_voxel(boxes,ws,grid=(20,20,20)):
    gx,gy,gz=grid
    xs=torch.linspace(ws['x'][0],ws['x'][1],gx)
    ys=torch.linspace(ws['y'][0],ws['y'][1],gy)
    zs=torch.linspace(ws['z'][0],ws['z'][1],gz)
    xv,yv,zv=torch.meshgrid(xs,ys,zs,indexing='ij')
    occ=torch.zeros(gx,gy,gz)
    pts=torch.stack((xv,yv,zv),-1)
    for b in boxes:
        mask=(pts[...,0]>=b['x'][0])&(pts[...,0]<=b['x'][1]) & \
             (pts[...,1]>=b['y'][0])&(pts[...,1]<=b['y'][1]) & \
             (pts[...,2]>=b['z'][0])&(pts[...,2]<=b['z'][1])
        occ[mask]=1
    return occ.unsqueeze(0)        # (1,gx,gy,gz)

# 9.  LOWER-LEVEL SOLVER (same API)
def optimise_lower(mid_init,start,goal,boxes,ws,llm_info,k):
    B=start.shape[0]
    mid0 = mid_init.detach().clone()
    mid_var = mid0[:,N_GUIDED:,:].requires_grad_(True)
    opt=optim.Adam([mid_var],lr=LR_LOWER); sched=lr_scheduler.StepLR(opt,10,0.9)
    for _ in range(N_LOWER_ITERS):
        opt.zero_grad()
        mid_full=torch.cat([H_GUIDED[None].expand(B,-1,-1).detach(), mid_var],1)
        ctrl=torch.cat([start[:,None,:], mid_full, goal[:,None,:]],1)
        traj=catmull_rom(ctrl,k)
        loss=lower_loss(traj,mid_full,mid_init,boxes,ws,llm_info)
        loss.backward(); torch.nn.utils.clip_grad_norm_([mid_var],1.0)
        opt.step(); sched.step()
    mid_final=torch.cat([H_GUIDED[None].expand(B,-1,-1).detach(), mid_var.detach()],1)
    # store in memory
    ctrl=torch.cat([start[:,None,:], mid_final, goal[:,None,:]],1)
    traj=catmull_rom(ctrl,k)
    verts_local=OBJ_VERTS_LOCAL[None].expand(B,-1,-1)
    Lclear=0.0
    for t in range(traj.shape[1]):
        pose=traj[:,t]; R=rpy_to_matrix(pose[:,3:])
        verts = (R @ verts_local.permute(0,2,1)).permute(0,2,1) + pose[:, :3].unsqueeze(1)
        Lclear+=collision_cost(verts,boxes)
    MEM.add(mid_final,Lclear/traj.shape[1])
    return mid_final

# 10.  IMPLICIT GRADIENT
def cg_solve(hvp,b,n=CG_ITERS):
    x=torch.zeros_like(b); r=b.clone(); p=r.clone(); rs=torch.dot(r.view(-1),r.view(-1))
    for _ in range(n):
        Ap=hvp(p); a=rs/(torch.dot(p.view(-1),Ap.view(-1))+1e-9)
        x+=a*p; r-=a*Ap; rs_new=torch.dot(r.view(-1),r.view(-1))
        if torch.sqrt(rs_new)<1e-8: break
        p=r+rs_new/rs*p; rs=rs_new
    return x

def implicit_update(loss_up, loss_lo, net, mid_var):
    psi=list(net.parameters())
    v=torch.autograd.grad(loss_up, mid_var, retain_graph=True)[0]
    def hvp(p):
        g=torch.autograd.grad(loss_lo, mid_var, create_graph=True, retain_graph=True)[0]
        return torch.autograd.grad(g, mid_var, grad_outputs=p, retain_graph=True)[0] + 0.1*p
    q=cg_solve(hvp,v)
    g_phi=torch.autograd.grad(loss_lo, mid_var, create_graph=True)[0]
    implicit_grads=torch.autograd.grad(g_phi, psi, grad_outputs=-q.detach(), allow_unused=True)
    for p,g in zip(psi,implicit_grads): p.grad=torch.zeros_like(p) if g is None else g


import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.animation import FuncAnimation

def make_prism(x0, x1, y0, y1, z0, z1):
    v = np.array([[x0,y0,z0],[x1,y0,z0],[x1,y1,z0],[x0,y1,z0], [x0,y0,z1],[x1,y0,z1],[x1,y1,z1],[x0,y1,z1]])
    faces = [[0,1,2,3],[4,5,6,7],[0,1,5,4], [2,3,7,6],[1,2,6,5],[3,0,4,7]]
    return [[v[i] for i in f] for f in faces]
def make_E_blocks():
    e_x0,e_x1,e_y0,e_y1 = 2.0, 2.5, 0.0, 0.2
    e_z0,e_z1,th_x,th_z = 0.0, 0.5, 0.1, 0.1
    spine      = make_prism(e_x0, e_x0+th_x, e_y0, e_y1, e_z0,   e_z1)
    bottom_bar = make_prism(e_x0, e_x1,       e_y0, e_y1, e_z0,   e_z0+th_z)
    top_bar    = make_prism(e_x0, e_x1,       e_y0, e_y1, e_z1-th_z, e_z1)
    return spine + bottom_bar + top_bar


def _make_box_faces(box: Dict):
    """Return a list of 6 quad faces for an axis-aligned box."""
    (x0,x1),(y0,y1),(z0,z1) = box['x'], box['y'], box['z']
    v = np.array([[x0,y0,z0],[x1,y0,z0],[x1,y1,z0],[x0,y1,z0],
                  [x0,y0,z1],[x1,y0,z1],[x1,y1,z1],[x0,y1,z1]])
    idx = [[0,1,2,3],[4,5,6,7],[0,1,5,4],[2,3,7,6],[1,2,6,5],[3,0,4,7]]
    return [[v[i] for i in f] for f in idx]

walls_mesh = [face for w in WALLS for face in make_prism(*w['x'], *w['y'], *w['z'])]
def in_box(pt, box):
    x,y,z = pt
    return (box['x'][0]<=x<=box['x'][1] and box['y'][0]<=y<=box['y'][1] and box['z'][0]<=z<=box['z'][1])
def collision_detect(verts):
    return any(in_box(v, WALLS[0]) or in_box(v, WALLS[1]) for v in verts)


def animate_full(traj_np: np.ndarray,
                 goal_np: np.ndarray,
                 boxes: List[Dict]=None,
                 save_path: str=None):
    """
    traj_np : (T,6)  - x,y,z,roll,pitch,yaw
    goal_np : (6,)   - goal pose (for reference axes)
    boxes   : list   - obstacle boxes to render (optional)
    """
    if boxes is None:
        boxes = []   # nothing extra - walls etc. already - built in

    # --- figure & static objects ------------------------------------------------
    fig = plt.figure(figsize=(6,5))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_box_aspect((1,1.5,1))
    ax.set_xlim(0,3); ax.set_ylim(0,2); ax.set_zlim(0,2)
    ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
    ax.set_title("Learned trajectory")

    # walls / door slabs
    wall_faces = [
        dict(x=(DOOR_X_MIN, DOOR_X_MAX), y=(0.0, DOOR_Y_MIN), z=(0.0, 1.2)),
        dict(x=(DOOR_X_MIN, DOOR_X_MAX), y=(DOOR_Y_MAX, 2.0), z=(0.0, 1.2))
    ]
    for b in wall_faces + (boxes or []):
        for f in _make_box_faces(b):
            pc = Poly3DCollection([f], facecolors="lightgrey", alpha=0.5, edgecolors="k")
            ax.add_collection3d(pc)

    # goalframe coordinate axes
    gx,gy,gz,gr,gp,gyaw = goal_np
    Rg = rpy_to_matrix(torch.tensor([[gr,gp,gyaw]])).squeeze(0).cpu().numpy()
    for i,color in enumerate(("r","g","b")):
        v = Rg[:,i]; ax.quiver(gx,gy,gz, *v, length=0.25, color=color, linewidth=2)

    # --- artists we will update -------------------------------------------------
    path_line, = ax.plot([],[],[], lw=2, c='blue')          # full trail
    # one Poly3DCollection per block
    object_pcs = []
    for blk in make_E_blocks():
        pc = Poly3DCollection([blk], facecolors="royalblue", alpha=0.9, edgecolors="k")
        ax.add_collection3d(pc)
        object_pcs.append(pc)

    verts_local = OBJ_VERTS_LOCAL.cpu().numpy()             # (Nv,3)
    axes_artists = []

    def update(frame):
        nonlocal axes_artists
        for art in axes_artists: art.remove()
        axes_artists = []
        x,y,z,roll,pitch,yaw = traj_np[frame]
        R = rpy_to_matrix(torch.tensor([[roll,pitch,yaw]], device=DEVICE)).squeeze(0).cpu().numpy()
        world_vertices = []
        for blk, pc in zip(make_E_blocks(), object_pcs):
            verts = []
            for p in blk:
                p_loc = np.array(p) - LOCAL_COM_NP
                p_w   = R.dot(p_loc) + np.array([x, y, z])
                verts.append(p_w)
                world_vertices.append(p_w)
            pc.set_verts([verts])

        if collision_detect(world_vertices): 
             print(f"<- Collision at t={frame*DT:.2f}s - stopping animation.")
             anim.event_source.stop()
        length = 0.2
        for idx, col in enumerate(("r","g","b")):
            v = R[:, idx]
            axes_artists.append(ax.quiver(x,y,z,v[0],v[1],v[2],length=length,color=col,linewidth=3))
        return object_pcs + axes_artists
    roll, pitch, yaw = goal_np[3:]; Rg = rpy_to_matrix(torch.tensor([[roll,pitch,yaw]], device=DEVICE)).squeeze(0).cpu().numpy()
    for idx, col in enumerate(("r","g","b")): ax.quiver(*goal_np[:3], *Rg[:, idx], length=0.2, color=col, linewidth=3)
    anim = FuncAnimation(fig, update, frames=traj_np.shape[0], interval=DT*1000, blit=False)
    return anim

# 11.  TRAIN LOOP
def train():
    net, k = WaypointNet().to(DEVICE), int(T_SEG/DT)
    opt_up = optim.Adam(net.parameters(), lr=LR_UPPER)
    sched_up = lr_scheduler.StepLR(opt_up, 30, 0.8)

    for ep in range(1,N_EPOCHS+1):
        start,goal,boxes,ws = random_scene(BATCH_SIZE)
        # llm_out = LLM_AGENT.generate(boxes, start[0].cpu().tolist(), goal[0].cpu().tolist())

        llm_out = LLM_AGENT.generate(boxes, start[0], goal[0])

        # voxel grid
        vox = boxes_to_voxel(boxes,ws).to(DEVICE).repeat(BATCH_SIZE,1,1,1,1)

        # network guess
        mid_net = net(start,goal,vox).detach()
        mid_full_init = torch.cat([H_GUIDED[None].expand(BATCH_SIZE,-1,-1), mid_net[:,N_GUIDED:,:]],1)

        # prepend LLM-suggested extra waypoints (if any)
        if llm_out.get("waypoints"):
            extra = torch.tensor(llm_out["waypoints"],dtype=torch.float32,device=DEVICE)
            extra = extra.unsqueeze(0).expand(BATCH_SIZE,-1,6)
            mid_full_init = torch.cat([H_GUIDED[None].expand(BATCH_SIZE,-1,-1),
                                       extra,
                                       mid_net[:,N_GUIDED:,:]],1)

        # lower-level optimise
        mid_opt = optimise_lower(mid_full_init,start,goal,boxes,ws,llm_out,k)

        # implicit diff
        # mid_var = mid_opt[:,N_GUIDED:,:].clone().requires_grad_(True)
        # ctrl = torch.cat([start.unsqueeze(1), mid_opt, goal.unsqueeze(1)],1)
        # traj = catmull_rom(ctrl,k)
        # loss_lo = lower_loss(traj,mid_opt,mid_full_init,boxes,ws,llm_out)
        # loss_up = upper_loss(traj,start,goal,boxes)

        # opt_up.zero_grad()
        # implicit_update(loss_up,loss_lo,net,mid_var)
        # opt_up.step(); sched_up.step()


        mid_var = mid_opt[:, N_GUIDED:, :].clone().requires_grad_(True)
        # rebuild full control points from mid_var so losses depend on it
        mid_full_grad = torch.cat([
            H_GUIDED[None].expand(BATCH_SIZE, -1, -1),
            mid_var
        ], dim=1)
        ctrl_grad = torch.cat([start.unsqueeze(1), mid_full_grad, goal.unsqueeze(1)], dim=1)
        traj_grad = catmull_rom(ctrl_grad, k)
        loss_lo_grad = lower_loss(traj_grad, mid_full_grad, mid_full_init, boxes, ws, llm_out)
        loss_up_grad = upper_loss(traj_grad, start, goal, boxes)

        opt_up.zero_grad()
        implicit_update(loss_up_grad, loss_lo_grad, net, mid_var)
        opt_up.step()
        sched_up.step()

        # # 5) visualize the *first* trajectory in the batch
        # traj_np = traj_grad[0].detach().cpu().numpy()    # shape (T,6)
        # goal_np = goal[0].detach().cpu().numpy()         # shape (6,)
        # anim = animate_full(traj_np, goal_np)           # returns a FuncAnimation
        # plt.show()    

        if ep%1==0:
            print(f"Ep {ep:03d}  Upper={loss_up_grad.item():.4f}  Lower={loss_lo_grad.item():.4f}")

    torch.save(net.state_dict(),"waypointnet_full.pth")
    print("Training finished  waypointnet_full.pth")

if __name__=="__main__":
    train()