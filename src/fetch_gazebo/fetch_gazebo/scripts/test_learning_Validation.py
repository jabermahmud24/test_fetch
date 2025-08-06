

# import math
# import numpy as np
# import torch
# import torch.nn as nn
# import torch.optim as optim
# import torch.optim.lr_scheduler as lr_scheduler # Needed for lower-level optimization
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d.art3d import Poly3DCollection
# from matplotlib.animation import FuncAnimation
# from itertools import chain

# DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
# # MODEL_PATH = "june30_implicit_net_4_22pm.pth" # Path to the trained model
# MODEL_PATH = "part_2_implicit_waypoint_net.pth" # Path to the trained model
# DT = 0.1
# T_SEG = 1.0
# N_MID = 100
# # N_MID = 20
# N_LOWER_ITERS = 200 # Iterations for the Conjugate Gradient solver
# LR_LOWER = 1e-4 # Learning rate for the lower-level optimization

# # Loss weights
# w_goal = 1.0

# w_door = 4.0
# w_clear = 4.0

# w_bounds= 1.0
# w_length_excess= 0.0

# # w_orient_door= 1.0 # FIX: Was -1.0
# w_orient_door= 0.0 # FIX: Was -1.0
# w_orient_traj= 0.0
# # w_orient_traj= 0.2

# w_smooth= 0.5
# w_reg= 0.0 # FIX: Reduced from 0.1
# w_coll_upper = 2.0
# w_coll_memory= 0.0
# w_track= 1.0
# w_miss = 2.0

# w_repel = 1.5

# # # Loss weights (needed for lower-level optimization)
# # w_goal = 1.0
# # w_door = 5.0
# # w_clear = 5.0
# # w_bounds= 1.0
# # w_length_excess= 1.0
# # w_orient_door= 3.0
# # w_orient_traj= 2.0
# # w_smooth= 0.5
# # w_reg= 0.01
# # w_coll_upper = 2.0
# # w_coll_memory= 20.0
# # w_track= 1.0
# # w_miss = 5.0

# # Number of fixed human-guided midpoints (must match training)
# n_guided = 3
# human_guided = torch.tensor([
#     [
#         [2.2, 0.6, 0.25, math.pi/2, math.pi, 0],
#         [2.1, 0.7, 0.30, math.pi/2, math.pi, 0],
#         [2.0, 0.8, 0.35, math.pi/2, math.pi, 0],
#     ]
# ], dtype=torch.float32, device=DEVICE)


# DOOR_X_MIN, DOOR_X_MAX = 1.4, 1.43
# DOOR_Y_OPEN_MIN, DOOR_Y_OPEN_MAX = 0.9, 1.2
# DOOR_Z_MIN, DOOR_Z_MAX= 0.0, 1.2
# WS = {"x": (0.0, 3.0), "y": (0.0, 2.0), "z": (0.0, 2.0)}
# th = 0.01
# WALL_DOOR_BOTTOM = {"x": (DOOR_X_MIN, DOOR_X_MAX), "y": (0.0, DOOR_Y_OPEN_MIN), "z": (DOOR_Z_MIN, DOOR_Z_MAX)}
# WALL_DOOR_TOP = {"x": (DOOR_X_MIN, DOOR_X_MAX), "y": (DOOR_Y_OPEN_MAX, WS["y"][1]), "z": (DOOR_Z_MIN, DOOR_Z_MAX)}
# WALLS = (WALL_DOOR_BOTTOM, WALL_DOOR_TOP)

# def make_prism(x0, x1, y0, y1, z0, z1):
#     v = np.array([[x0,y0,z0],[x1,y0,z0],[x1,y1,z0],[x0,y1,z0], [x0,y0,z1],[x1,y0,z1],[x1,y1,z1],[x0,y1,z1]])
#     faces = [[0,1,2,3],[4,5,6,7],[0,1,5,4], [2,3,7,6],[1,2,6,5],[3,0,4,7]]
#     return [[v[i] for i in f] for f in faces]

# def make_E_blocks():
#     e_x0,e_x1,e_y0,e_y1 = 2.0, 2.5, 0.0, 0.2
#     e_z0,e_z1,th_x,th_z = 0.0, 0.5, 0.1, 0.1
#     spine      = make_prism(e_x0, e_x0+th_x, e_y0, e_y1, e_z0,   e_z1)
#     bottom_bar = make_prism(e_x0, e_x1,        e_y0, e_y1, e_z0,   e_z0+th_z)
#     top_bar    = make_prism(e_x0, e_x1,        e_y0, e_y1, e_z1-th_z, e_z1)
#     return spine + bottom_bar + top_bar

# E_BLOCK_VS = np.vstack(list(chain.from_iterable(make_E_blocks())))
# LOCAL_COM = E_BLOCK_VS.mean(axis=0)
# OBJ_PTS_LOCAL = np.unique(E_BLOCK_VS, axis=0) - LOCAL_COM
# OBJ_PTS_LOCAL = torch.tensor(OBJ_PTS_LOCAL, dtype=torch.float32, device=DEVICE)

# def rpy_to_matrix(rpy: torch.Tensor) -> torch.Tensor:
#     roll, pitch, yaw = rpy.unbind(-1)
#     cz, sz = torch.cos(yaw), torch.sin(yaw)
#     cy, sy = torch.cos(pitch), torch.sin(pitch)
#     cx, sx = torch.cos(roll), torch.sin(roll)
#     Rz = torch.stack([torch.stack([cz,-sz,torch.zeros_like(cz)],-1), torch.stack([sz,cz,torch.zeros_like(cz)],-1), torch.stack([torch.zeros_like(cz),torch.zeros_like(cz),torch.ones_like(cz)],-1)],-2)
#     Ry = torch.stack([torch.stack([cy,torch.zeros_like(cy),sy],-1), torch.stack([torch.zeros_like(cy),torch.ones_like(cy),torch.zeros_like(cy)],-1), torch.stack([-sy,torch.zeros_like(cy),cy],-1)],-2)
#     Rx = torch.stack([torch.stack([torch.ones_like(cx),torch.zeros_like(cx),torch.zeros_like(cx)],-1), torch.stack([torch.zeros_like(cx),cx,-sx],-1), torch.stack([torch.zeros_like(cx),sx,cx],-1)],-2)
#     return Rz @ Ry @ Rx

# class CollisionMemory:
#     def __init__(self, max_size=100):
#         self.memory = []
#         self.max_size = max_size

#     def add_trajectory(self, traj, collision_cost):
#         traj_rounded = torch.round(traj, decimals=2)
#         self.memory.append((traj_rounded.detach().cpu(), collision_cost.detach().cpu()))
#         if len(self.memory) > self.max_size:
#             self.memory.pop(0)

#     def collision_penalty(self, traj):
#         if not self.memory:
#             return torch.tensor(0.0, device=traj.device)
#         penalty = 0.0
#         traj_rounded = torch.round(traj, decimals=2)
#         for mem_traj, mem_cost in self.memory:
#             mem_traj = mem_traj.to(traj.device)
#             diff = (traj_rounded - mem_traj).pow(2).mean()
#             penalty += mem_cost * torch.exp(-diff / 0.1)
#         return penalty / max(len(self.memory), 1)

# COLLISION_MEMORY = CollisionMemory(max_size=1000) # Re-initialize for validation if needed, or pass trained one

# def signed_distance_box(pts: torch.Tensor, box: dict) -> torch.Tensor:
#     xmin, xmax = box['x']; ymin, ymax = box['y']; zmin, zmax = box['z']
#     dx_out = torch.where(pts[...,0]<xmin,xmin-pts[...,0],torch.where(pts[...,0]>xmax,pts[...,0]-xmax,torch.zeros_like(pts[...,0])))
#     dy_out = torch.where(pts[...,1]<ymin,ymin-pts[...,1],torch.where(pts[...,1]>ymax,pts[...,1]-ymax,torch.zeros_like(pts[...,1])))
#     dz_out = torch.where(pts[...,2]<zmin,zmin-pts[...,2],torch.where(pts[...,2]>zmax,pts[...,2]-zmax,torch.zeros_like(pts[...,2])))
#     outside_dist = torch.linalg.norm(torch.stack((dx_out,dy_out,dz_out),dim=-1),dim=-1)
#     dx_in = torch.minimum(pts[...,0]-xmin,xmax-pts[...,0]); dy_in = torch.minimum(pts[...,1]-ymin,ymax-pts[...,1]); dz_in = torch.minimum(pts[...,2]-zmin,zmax-pts[...,2])
#     inside_dist = -torch.minimum(torch.minimum(dx_in,dy_in),dz_in)
#     return torch.where(outside_dist > 0, outside_dist, inside_dist)

# def esdf_penalty(pts: torch.Tensor, box: dict, clearance: float=0.01) -> torch.Tensor:
#     return torch.relu(clearance - signed_distance_box(pts, box))

# def collision_cost(vertices: torch.Tensor, clearance: float=0.01) -> torch.Tensor:
#     cost = 0.0
#     for w in WALLS: cost += esdf_penalty(vertices, w, clearance=clearance).sum(-1)
#     return cost.mean() / (len(WALLS) * vertices.shape[-2])

# def catmull_rom_spline(ctrl: torch.Tensor, steps_per_seg: int) -> torch.Tensor:
#     B, K, D = ctrl.shape
#     t = torch.linspace(0, 1, steps_per_seg + 1, device=ctrl.device)[:-1]
#     t2, t3 = t*t, t*t*t
#     traj_chunks = []
#     for i in range(K-1):
#         P0 = ctrl[:,i-1] if i>0 else ctrl[:,i]
#         P1, P2 = ctrl[:,i], ctrl[:,i+1]
#         P3 = ctrl[:,i+2] if i+2<K else ctrl[:,i+1]
#         a,b,c,d = 2*P1, (P2-P0), 2*P0-5*P1+4*P2-P3, -P0+3*P1-3*P2+P3
#         seg = 0.5 * (a.unsqueeze(1) + b.unsqueeze(1)*t[:,None] + c.unsqueeze(1)*t2[:,None] + d.unsqueeze(1)*t3[:,None])
#         traj_chunks.append(seg)
#     traj_chunks.append(ctrl[:, -1:].expand(B, 1, D))
#     return torch.cat(traj_chunks, dim=1)

# def bounds_penalty(pos: torch.Tensor, ws: dict, margin: float=0.1):
#     px, py, pz = pos.unbind(-1)
#     dx = torch.relu(ws['x'][0]-px+margin) + torch.relu(px-ws['x'][1]+margin)
#     dy = torch.relu(ws['y'][0]-py+margin) + torch.relu(py-ws['y'][1]+margin)
#     dz = torch.relu(ws['z'][0]-pz+margin) + torch.relu(pz-ws['z'][1]+margin)
#     return (dx+dy+dz).mean()

# def door_penalty(pos: torch.Tensor, margin: float = 0.1) -> torch.Tensor:
#     x, y, _ = pos.unbind(-1)
#     slab = (x >= DOOR_X_MIN - margin) & (x <= DOOR_X_MAX + margin)
#     outside_y = torch.relu(DOOR_Y_OPEN_MIN - margin - y) + torch.relu(y - (DOOR_Y_OPEN_MAX + margin))
#     outside_x = torch.relu(DOOR_X_MIN - margin - x) + torch.relu(x - (DOOR_X_MAX + margin))
#     outside_y = outside_y * slab.float()
#     success = (slab & (y >= DOOR_Y_OPEN_MIN) & (y <= DOOR_Y_OPEN_MAX)).any(dim=-1).float()
#     miss_pen = torch.clamp(1.0 - success, min=0.0, max=1.0) * 10.0
#     return outside_y.mean() + outside_x.mean() + miss_pen.mean()


# def compute_lower_loss(traj: torch.Tensor, mid: torch.Tensor, mid_init: torch.Tensor) -> torch.Tensor:
#     B, T, _ = traj.shape
#     verts_local = OBJ_PTS_LOCAL.unsqueeze(0).expand(B, -1, -1)
#     L_clear = 0.0
#     for t in range(0, T):
#         pose_t = traj[:, t]
#         R, trans = rpy_to_matrix(pose_t[:, 3:]), pose_t[:, :3].unsqueeze(1)
#         verts_w = (R @ verts_local.permute(0,2,1)).permute(0,2,1) + trans
#         L_clear += collision_cost(verts_w)
#     L_bounds = bounds_penalty(traj[:, :, :3], WS)
#     x_all, y_all = traj[:, :, 0], traj[:, :, 1]
#     slab = ((x_all >= DOOR_X_MIN) & (x_all <= DOOR_X_MAX)).float()
#     outside_y = torch.relu(DOOR_Y_OPEN_MIN - y_all) + torch.relu(y_all - DOOR_Y_OPEN_MAX)
#     L_door_strong = (slab * outside_y).mean()
#     dpos = traj[:, 1:, :3] - traj[:, :-1, :3]
#     L_smooth = dpos.pow(2).sum(-1).mean()
#     L_reg = (mid - mid_init).pow(2).mean()

#     L_repel = 0.0
#     w_repel = 1.0
#     sigma2  = 0.1
#     for mem_mid, _ in COLLISION_MEMORY.memory:
#         mem_mid = mem_mid.to(traj.device)
#         L_repel += torch.exp(- (mid - mem_mid).pow(2).sum(-1) / sigma2).mean()

#     t_lin = torch.linspace(0, 1, T, device=traj.device).view(1, T, 1).expand(B, T, 3)
#     straight_line = mid_init[:, 0, :3].unsqueeze(1) + t_lin * (mid_init[:, -1, :3].unsqueeze(1) - mid_init[:, 0, :3].unsqueeze(1))
#     L_track = (traj[:, :, :3] - straight_line).pow(2).sum(-1).mean()

#     coll_penalty = w_coll_memory * COLLISION_MEMORY.collision_penalty(mid)
#     return (w_clear*L_clear + w_bounds*L_bounds + w_door*L_door_strong + w_smooth*L_smooth + w_reg*L_reg + w_repel*L_repel + w_track*L_track) + coll_penalty


# class WaypointNet(nn.Module):
#     def __init__(self, n_mid=N_MID):
#         super().__init__()
#         self.n_mid = n_mid
#         self.fc = nn.Sequential(nn.Linear(12,256), nn.ReLU(), nn.Linear(256,256), nn.ReLU(), nn.Linear(256, n_mid * 6))

#         with torch.no_grad():
#             bias = []
#             for i in range(n_mid):
#                 alpha = float(i + 1) / (n_mid + 1)
#                 x = 1.8
#                 y = 1.0
#                 z = 0.5
#                 bias += [x, y, z]
#                 human_guided_rpy = human_guided[0, -1, 3:].cpu().numpy()
#                 goal_rpy = torch.tensor([math.pi/4, math.pi/8, math.pi/6]).cpu().numpy()
#                 rpy = human_guided_rpy 
#                 # rpy = human_guided_rpy + alpha * (goal_rpy - human_guided_rpy)
#                 bias += rpy.tolist()
#             self.fc[-1].bias.copy_(torch.tensor(bias, dtype=torch.float32))
#             nn.init.zeros_(self.fc[-1].weight)
   
#     def forward(self, start_pose, goal_pose):
#         return self.fc(torch.cat([start_pose, goal_pose], dim=-1)).view(-1, self.n_mid, 6)

# walls_mesh = [face for w in WALLS for face in make_prism(*w['x'], *w['y'], *w['z'])]
# def in_box(pt, box):
#     x,y,z = pt
#     return (box['x'][0]<=x<=box['x'][1] and box['y'][0]<=y<=box['y'][1] and box['z'][0]<=z<=box['z'][1])
# def collision_detect(verts):
#     return any(in_box(v, WALLS[0]) or in_box(v, WALLS[1]) for v in verts)
# def animate_full(traj: np.ndarray, goal_pose: np.ndarray):
#     fig = plt.figure(figsize=(8,6)); ax = fig.add_subplot(111, projection='3d')
#     for face in walls_mesh: ax.add_collection3d(Poly3DCollection([face], facecolors="lightgrey", edgecolors="k", alpha=0.8))
#     ax.set_box_aspect((1,1.5,1)); ax.set_xlim(0,3); ax.set_ylim(0,2); ax.set_zlim(0,2)
#     ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z"); ax.set_title("Learned trajectory")
#     poly_cols = [ax.add_collection3d(Poly3DCollection([], facecolors="royalblue", edgecolors="k")) for _ in make_E_blocks()]
#     axes_artists = []
#     def update(frame):
#         nonlocal axes_artists
#         for art in axes_artists: art.remove()
#         axes_artists = []
#         x,y,z,roll,pitch,yaw = traj[frame]
#         R = rpy_to_matrix(torch.tensor([[roll,pitch,yaw]], device=DEVICE)).squeeze(0).cpu().numpy()
#         world_vertices = []
#         for blk, pc in zip(make_E_blocks(), poly_cols):
#             verts = []
#             for p in blk:
#                 p_loc, p_w = np.array(p) - LOCAL_COM, R @ (np.array(p) - LOCAL_COM) + np.array([x,y,z])
#                 verts.append(p_w); world_vertices.append(p_w)
#             pc.set_verts([verts])
#         if collision_detect(world_vertices): 
#             print(f"< Collision at t={frame*DT:.2f}s  stopping animation.")
#             anim.event_source.stop()
#         length = 0.2
#         for idx, col in enumerate(("r","g","b")):
#             v = R[:, idx]
#             axes_artists.append(ax.quiver(x,y,z,v[0],v[1],v[2],length=length,color=col,linewidth=3))
#         return poly_cols + axes_artists
#     roll, pitch, yaw = goal_pose[3:]; Rg = rpy_to_matrix(torch.tensor([[roll,pitch,yaw]], device=DEVICE)).squeeze(0).cpu().numpy()
#     for idx, col in enumerate(("r","g","b")): ax.quiver(*goal_pose[:3], *Rg[:, idx], length=0.2, color=col, linewidth=3)
#     anim = FuncAnimation(fig, update, frames=traj.shape[0], interval=DT*1000, blit=False)
#     return anim

# def get_full_mid(mid_optimizable, batch_size):
#     """Helper to combine fixed human-guided points and trainable points."""
#     guided_block = human_guided.expand(batch_size, -1, -1)
#     return torch.cat([guided_block, mid_optimizable], dim=1)

# def get_full_ctrl(mid, start_pose, goal_pose):
#     """Helper to construct the full control point tensor for the spline."""
#     return torch.cat([start_pose.unsqueeze(1), mid, goal_pose.unsqueeze(1)], dim=1)

# def lower_level_optimize_validate(mid_init, start_pose, goal_pose, steps_per_seg):
#     """
#     Performs lower-level optimization for validation.
#     Similar to the training version but focused on getting the final trajectory.
#     """
#     batch_size = start_pose.shape[0]
#     mid_init_trainable = mid_init[:, n_guided:, :].detach()
#     mid_optimizable = mid_init_trainable.clone().requires_grad_(True)

#     optimizer = optim.Adam([mid_optimizable], lr=LR_LOWER)
#     scheduler = lr_scheduler.StepLR(optimizer, step_size=10, gamma=0.9) # Match training scheduler

#     for _ in range(N_LOWER_ITERS):
#         optimizer.zero_grad()
#         mid_full = get_full_mid(mid_optimizable, batch_size)
#         ctrl = get_full_ctrl(mid_full, start_pose, goal_pose)
#         traj = catmull_rom_spline(ctrl, steps_per_seg)
        
#         # The lower-level objective
#         loss = compute_lower_loss(traj, mid_full, mid_init.detach())
#         loss.backward()
        
#         torch.nn.utils.clip_grad_norm_([mid_optimizable], max_norm=1.0)
#         optimizer.step()
#         scheduler.step()
    
#     mid_opt = get_full_mid(mid_optimizable, batch_size)
#     ctrl_opt = get_full_ctrl(mid_opt, start_pose, goal_pose)
#     traj_opt = catmull_rom_spline(ctrl_opt, steps_per_seg)
#     return traj_opt, mid_opt

# def run_validation_episode(net, start_pose, goal_pose):
#     """Runs a single validation episode and returns the optimized trajectory and losses."""
#     net.eval() # Set network to evaluation mode
#     steps_per_seg = int(T_SEG / DT)

#     with torch.no_grad(): # No gradients needed for the upper-level network during validation
#         mid_init = net(start_pose.unsqueeze(0), goal_pose.unsqueeze(0))

#     # The lower-level optimization still needs gradients
#     traj_opt, mid_opt = lower_level_optimize_validate(mid_init, start_pose.unsqueeze(0), goal_pose.unsqueeze(0), steps_per_seg)
    
#     # Compute final losses for reporting
#     loss_lower_final = compute_lower_loss(traj_opt, mid_opt, mid_init)
#     loss_upper_final = compute_upper_loss_validate(traj_opt, goal_pose.unsqueeze(0), start_pose.unsqueeze(0))

#     return traj_opt.squeeze(0), loss_upper_final.item(), loss_lower_final.item()

# def upper_collision_loss_validate(traj: torch.Tensor, clearance: float=0.01, check_stride: int=1) -> torch.Tensor:
#     B, T, _ = traj.shape
#     Lc, checks = 0.0, 0
#     for t in range(0, T, check_stride):
#         pose = traj[:, t]
#         R = rpy_to_matrix(pose[:, 3:])
#         pts = OBJ_PTS_LOCAL.unsqueeze(0).expand(B, -1, -1)
#         world = (R @ pts.permute(0,2,1)).permute(0,2,1) + pose[:,:3].unsqueeze(1)
#         Lc += collision_cost(world, clearance)
#         checks += 1
#     return Lc / checks

# def compute_upper_loss_validate(traj: torch.Tensor, goal: torch.Tensor, start: torch.Tensor) -> torch.Tensor:
#     B, T, _ = traj.shape
#     end_err_pos = (traj[:,-1,:3]-goal[:,:3]).pow(2).sum(-1)
#     end_err_ori = (traj[:,-1,3:]-goal[:,3:]).pow(2).sum(-1)
#     L_goal = (end_err_pos + 0.1*end_err_ori).mean()
#     dpos = traj[:,1:,:3]-traj[:,:-1,:3]
#     L_path = torch.linalg.norm(dpos,dim=-1).sum(-1).mean()
#     straight_dist = torch.linalg.norm(goal[:,:3]-traj[:,0,:3],dim=-1).mean()
#     L_excess = torch.relu(L_path - straight_dist)
#     L_coll = upper_collision_loss_validate(traj)
#     x_all, rpy_all = traj[:,:,0], traj[:,:,3:6]
#     slab_mask = ((x_all >= DOOR_X_MIN) & (x_all <= DOOR_X_MAX)).float()
#     denom = slab_mask.sum() + 1e-6
#     L_orient_door = (slab_mask.unsqueeze(-1) * rpy_all.abs()).sum() / denom
#     t = torch.linspace(0,1,T,device=traj.device).view(1,T,1).expand(B,T,3)
#     target_rpy = start[:,3:].unsqueeze(1) + t*(goal[:,3:].unsqueeze(1)-start[:,3:].unsqueeze(1))
#     L_orient_traj = (traj[:,:,3:]-target_rpy).pow(2).sum(-1).mean()
#     L_miss = door_penalty(traj[:,:,:3])
#     return (w_goal*L_goal + w_length_excess*L_excess + w_coll_upper*L_coll + w_orient_door*L_orient_door + w_orient_traj*L_orient_traj + w_miss*L_miss)

# def validate():
#     net = WaypointNet().to(DEVICE)
#     net.load_state_dict(torch.load(MODEL_PATH, map_location=DEVICE))
#     net.eval() # Set the network to evaluation mode

#     print("--- Starting Validation ---")
    
#     # Define a few test cases or generate random ones
#     test_cases = [
#         (torch.tensor([2.2, 0.6, 0.5, math.pi/2, math.pi/4, math.pi/8], dtype=torch.float32, device=DEVICE),
#          torch.tensor([0.5, 1.2, 0.4, math.pi/4, math.pi/8, math.pi/6], dtype=torch.float32, device=DEVICE)),
#         (torch.tensor([2.5, 0.5, 0.5, math.pi, 0, 0], dtype=torch.float32, device=DEVICE),
#          torch.tensor([0.8, 1.5, 0.7, -math.pi/2, -math.pi/4, math.pi/2], dtype=torch.float32, device=DEVICE)),
#         (torch.tensor([2.0, 0.3, 0.5, math.pi/2, 0, -math.pi/2], dtype=torch.float32, device=DEVICE),
#          torch.tensor([0.3, 1.0, 0.3, math.pi/4, math.pi/4, 0], dtype=torch.float32, device=DEVICE)),
#         # Add more diverse test cases here if desired
#     ]

#     for i, (start_pose, goal_pose) in enumerate(test_cases):
#         print(f"\n--- Validation Episode {i+1} ---")
#         print(f"Start Pose: {start_pose.cpu().numpy()}")
#         print(f"Goal Pose:  {goal_pose.cpu().numpy()}")

#         traj_optimized, upper_loss, lower_loss = run_validation_episode(net, start_pose, goal_pose)
        
#         print(f"Validation Upper Loss: {upper_loss:.4f}")
#         print(f"Validation Lower Loss: {lower_loss:.4f}")
        
#         traj_np = traj_optimized.detach().cpu().numpy()
#         goal_np = goal_pose.detach().cpu().numpy()
        
#         anim = animate_full(traj_np, goal_np)
#         plt.show()

#     print("\n--- Validation Complete ---")

# if __name__ == "__main__":
#     validate()





import math
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import torch.optim.lr_scheduler as lr_scheduler # Needed for lower-level optimization
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.animation import FuncAnimation
from itertools import chain

DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
# MODEL_PATH = "june30_implicit_net_4_22pm.pth" # Path to the trained model
MODEL_PATH = "july9_part_2_implicit_waypoint_net.pth" # Path to the trained model
DT = 0.1
T_SEG = 1.0
N_MID = 5
# N_MID = 20
N_LOWER_ITERS = 50 # Iterations for the Conjugate Gradient solver
LR_LOWER = 1e-4 # Learning rate for the lower-level optimization

# Loss weights (needed for lower-level optimization)
w_goal = 1.0
w_door = 5.0
w_clear = 5.0
w_bounds= 1.0
w_length_excess= 1.0
w_orient_door= 3.0
w_orient_traj= 2.0
w_smooth= 0.5
w_reg= 0.01
w_coll_upper = 2.0
w_coll_memory= 20.0
w_track= 1.0
w_miss = 5.0

# Number of fixed human-guided midpoints (must match training)
n_guided = 3
human_guided = torch.tensor([
    [
        [2.2, 0.6, 0.25, math.pi/2, math.pi, 0],
        [2.1, 0.7, 0.30, math.pi/2, math.pi, 0],
        [2.0, 0.8, 0.35, math.pi/2, math.pi, 0],
    ]
], dtype=torch.float32, device=DEVICE)


DOOR_X_MIN, DOOR_X_MAX = 1.4, 1.43
DOOR_Y_OPEN_MIN, DOOR_Y_OPEN_MAX = 0.9, 1.2
DOOR_Z_MIN, DOOR_Z_MAX= 0.0, 1.2
WS = {"x": (0.0, 3.0), "y": (0.0, 2.0), "z": (0.0, 2.0)}
th = 0.01
WALL_DOOR_BOTTOM = {"x": (DOOR_X_MIN, DOOR_X_MAX), "y": (0.0, DOOR_Y_OPEN_MIN), "z": (DOOR_Z_MIN, DOOR_Z_MAX)}
WALL_DOOR_TOP = {"x": (DOOR_X_MIN, DOOR_X_MAX), "y": (DOOR_Y_OPEN_MAX, WS["y"][1]), "z": (DOOR_Z_MIN, DOOR_Z_MAX)}
WALLS = (WALL_DOOR_BOTTOM, WALL_DOOR_TOP)


def make_prism(x0, x1, y0, y1, z0, z1):
    v = np.array([[x0,y0,z0],[x1,y0,z0],[x1,y1,z0],[x0,y1,z0], [x0,y0,z1],[x1,y0,z1],[x1,y1,z1],[x0,y1,z1]])
    faces = [[0,1,2,3],[4,5,6,7],[0,1,5,4], [2,3,7,6],[1,2,6,5],[3,0,4,7]]
    return [[v[i] for i in f] for f in faces]

def make_E_blocks():
    e_x0,e_x1,e_y0,e_y1 = 2.0, 2.5, 0.0, 0.2
    e_z0,e_z1,th_x,th_z = 0.0, 0.5, 0.1, 0.1
    spine      = make_prism(e_x0, e_x0+th_x, e_y0, e_y1, e_z0,   e_z1)
    bottom_bar = make_prism(e_x0, e_x1,        e_y0, e_y1, e_z0,   e_z0+th_z)
    top_bar    = make_prism(e_x0, e_x1,        e_y0, e_y1, e_z1-th_z, e_z1)
    return spine + bottom_bar + top_bar

E_BLOCK_VS = np.vstack(list(chain.from_iterable(make_E_blocks())))
LOCAL_COM = E_BLOCK_VS.mean(axis=0)
OBJ_PTS_LOCAL = np.unique(E_BLOCK_VS, axis=0) - LOCAL_COM
OBJ_PTS_LOCAL = torch.tensor(OBJ_PTS_LOCAL, dtype=torch.float32, device=DEVICE)

def rpy_to_matrix(rpy: torch.Tensor) -> torch.Tensor:
    roll, pitch, yaw = rpy.unbind(-1)
    cz, sz = torch.cos(yaw), torch.sin(yaw)
    cy, sy = torch.cos(pitch), torch.sin(pitch)
    cx, sx = torch.cos(roll), torch.sin(roll)
    Rz = torch.stack([torch.stack([cz,-sz,torch.zeros_like(cz)],-1), torch.stack([sz,cz,torch.zeros_like(cz)],-1), torch.stack([torch.zeros_like(cz),torch.zeros_like(cz),torch.ones_like(cz)],-1)],-2)
    Ry = torch.stack([torch.stack([cy,torch.zeros_like(cy),sy],-1), torch.stack([torch.zeros_like(cy),torch.ones_like(cy),torch.zeros_like(cy)],-1), torch.stack([-sy,torch.zeros_like(cy),cy],-1)],-2)
    Rx = torch.stack([torch.stack([torch.ones_like(cx),torch.zeros_like(cx),torch.zeros_like(cx)],-1), torch.stack([torch.zeros_like(cx),cx,-sx],-1), torch.stack([torch.zeros_like(cx),sx,cx],-1)],-2)
    return Rz @ Ry @ Rx

class CollisionMemory:
    def __init__(self, max_size=100):
        self.memory = []
        self.max_size = max_size

    def add_trajectory(self, traj, collision_cost):
        traj_rounded = torch.round(traj, decimals=2)
        self.memory.append((traj_rounded.detach().cpu(), collision_cost.detach().cpu()))
        if len(self.memory) > self.max_size:
            self.memory.pop(0)

    def collision_penalty(self, traj):
        if not self.memory:
            return torch.tensor(0.0, device=traj.device)
        penalty = 0.0
        traj_rounded = torch.round(traj, decimals=2)
        for mem_traj, mem_cost in self.memory:
            mem_traj = mem_traj.to(traj.device)
            diff = (traj_rounded - mem_traj).pow(2).mean()
            penalty += mem_cost * torch.exp(-diff / 0.1)
        return penalty / max(len(self.memory), 1)

COLLISION_MEMORY = CollisionMemory(max_size=1000) # Re-initialize for validation if needed, or pass trained one

def signed_distance_box(pts: torch.Tensor, box: dict) -> torch.Tensor:
    xmin, xmax = box['x']; ymin, ymax = box['y']; zmin, zmax = box['z']
    dx_out = torch.where(pts[...,0]<xmin,xmin-pts[...,0],torch.where(pts[...,0]>xmax,pts[...,0]-xmax,torch.zeros_like(pts[...,0])))
    dy_out = torch.where(pts[...,1]<ymin,ymin-pts[...,1],torch.where(pts[...,1]>ymax,pts[...,1]-ymax,torch.zeros_like(pts[...,1])))
    dz_out = torch.where(pts[...,2]<zmin,zmin-pts[...,2],torch.where(pts[...,2]>zmax,pts[...,2]-zmax,torch.zeros_like(pts[...,2])))
    outside_dist = torch.linalg.norm(torch.stack((dx_out,dy_out,dz_out),dim=-1),dim=-1)
    dx_in = torch.minimum(pts[...,0]-xmin,xmax-pts[...,0]); dy_in = torch.minimum(pts[...,1]-ymin,ymax-pts[...,1]); dz_in = torch.minimum(pts[...,2]-zmin,zmax-pts[...,2])
    inside_dist = -torch.minimum(torch.minimum(dx_in,dy_in),dz_in)
    return torch.where(outside_dist > 0, outside_dist, inside_dist)

def esdf_penalty(pts: torch.Tensor, box: dict, clearance: float=0.01) -> torch.Tensor:
    return torch.relu(clearance - signed_distance_box(pts, box))

def collision_cost(vertices: torch.Tensor, clearance: float=0.01) -> torch.Tensor:
    cost = 0.0
    for w in WALLS: cost += esdf_penalty(vertices, w, clearance=clearance).sum(-1)
    return cost.mean() / (len(WALLS) * vertices.shape[-2])

def catmull_rom_spline(ctrl: torch.Tensor, steps_per_seg: int) -> torch.Tensor:
    B, K, D = ctrl.shape
    t = torch.linspace(0, 1, steps_per_seg + 1, device=ctrl.device)[:-1]
    t2, t3 = t*t, t*t*t
    traj_chunks = []
    for i in range(K-1):
        P0 = ctrl[:,i-1] if i>0 else ctrl[:,i]
        P1, P2 = ctrl[:,i], ctrl[:,i+1]
        P3 = ctrl[:,i+2] if i+2<K else ctrl[:,i+1]
        a,b,c,d = 2*P1, (P2-P0), 2*P0-5*P1+4*P2-P3, -P0+3*P1-3*P2+P3
        seg = 0.5 * (a.unsqueeze(1) + b.unsqueeze(1)*t[:,None] + c.unsqueeze(1)*t2[:,None] + d.unsqueeze(1)*t3[:,None])
        traj_chunks.append(seg)
    traj_chunks.append(ctrl[:, -1:].expand(B, 1, D))
    return torch.cat(traj_chunks, dim=1)

def bounds_penalty(pos: torch.Tensor, ws: dict, margin: float=0.1):
    px, py, pz = pos.unbind(-1)
    dx = torch.relu(ws['x'][0]-px+margin) + torch.relu(px-ws['x'][1]+margin)
    dy = torch.relu(ws['y'][0]-py+margin) + torch.relu(py-ws['y'][1]+margin)
    dz = torch.relu(ws['z'][0]-pz+margin) + torch.relu(pz-ws['z'][1]+margin)
    return (dx+dy+dz).mean()

def door_penalty(pos: torch.Tensor, margin: float = 0.1) -> torch.Tensor:
    x, y, _ = pos.unbind(-1)
    slab = (x >= DOOR_X_MIN - margin) & (x <= DOOR_X_MAX + margin)
    outside_y = torch.relu(DOOR_Y_OPEN_MIN - margin - y) + torch.relu(y - (DOOR_Y_OPEN_MAX + margin))
    outside_x = torch.relu(DOOR_X_MIN - margin - x) + torch.relu(x - (DOOR_X_MAX + margin))
    outside_y = outside_y * slab.float()
    success = (slab & (y >= DOOR_Y_OPEN_MIN) & (y <= DOOR_Y_OPEN_MAX)).any(dim=-1).float()
    miss_pen = torch.clamp(1.0 - success, min=0.0, max=1.0) * 10.0
    return outside_y.mean() + outside_x.mean() + miss_pen.mean()


def compute_lower_loss(traj: torch.Tensor, mid: torch.Tensor, mid_init: torch.Tensor) -> torch.Tensor:
    B, T, _ = traj.shape
    verts_local = OBJ_PTS_LOCAL.unsqueeze(0).expand(B, -1, -1)
    L_clear = 0.0
    for t in range(0, T):
        pose_t = traj[:, t]
        R, trans = rpy_to_matrix(pose_t[:, 3:]), pose_t[:, :3].unsqueeze(1)
        verts_w = (R @ verts_local.permute(0,2,1)).permute(0,2,1) + trans
        L_clear += collision_cost(verts_w)
    L_bounds = bounds_penalty(traj[:, :, :3], WS)
    x_all, y_all = traj[:, :, 0], traj[:, :, 1]
    slab = ((x_all >= DOOR_X_MIN) & (x_all <= DOOR_X_MAX)).float()
    outside_y = torch.relu(DOOR_Y_OPEN_MIN - y_all) + torch.relu(y_all - DOOR_Y_OPEN_MAX)
    L_door_strong = (slab * outside_y).mean()
    dpos = traj[:, 1:, :3] - traj[:, :-1, :3]
    L_smooth = dpos.pow(2).sum(-1).mean()
    L_reg = (mid - mid_init).pow(2).mean()

    L_repel = 0.0
    w_repel = 1.0
    sigma2  = 0.1
    for mem_mid, _ in COLLISION_MEMORY.memory:
        mem_mid = mem_mid.to(traj.device)
        L_repel += torch.exp(- (mid - mem_mid).pow(2).sum(-1) / sigma2).mean()

    t_lin = torch.linspace(0, 1, T, device=traj.device).view(1, T, 1).expand(B, T, 3)
    straight_line = mid_init[:, 0, :3].unsqueeze(1) + t_lin * (mid_init[:, -1, :3].unsqueeze(1) - mid_init[:, 0, :3].unsqueeze(1))
    L_track = (traj[:, :, :3] - straight_line).pow(2).sum(-1).mean()

    coll_penalty = w_coll_memory * COLLISION_MEMORY.collision_penalty(mid)
    return (w_clear*L_clear + w_bounds*L_bounds + w_door*L_door_strong + w_smooth*L_smooth + w_reg*L_reg + w_repel*L_repel )
    # return (w_clear*L_clear + w_bounds*L_bounds + w_door*L_door_strong + w_smooth*L_smooth + w_reg*L_reg + w_repel*L_repel + w_track*L_track) + coll_penalty

class WaypointNet(nn.Module):
    def __init__(self, n_mid=N_MID):
        super().__init__()
        self.n_mid = n_mid
        self.fc = nn.Sequential(nn.Linear(12,256), nn.ReLU(), nn.Linear(256,256), nn.ReLU(), nn.Linear(256, n_mid * 6))

        with torch.no_grad():
            bias = []
            for i in range(n_mid):
                alpha = float(i + 1) / (n_mid + 1)
                x = 1.8
                y = 1.0
                z = 0.5
                bias += [x, y, z]
                human_guided_rpy = human_guided[0, -1, 3:].cpu().numpy()
                goal_rpy = torch.tensor([math.pi/4, math.pi/8, math.pi/6]).cpu().numpy()
                rpy = human_guided_rpy 
                # rpy = human_guided_rpy + alpha * (goal_rpy - human_guided_rpy)
                bias += rpy.tolist()
            self.fc[-1].bias.copy_(torch.tensor(bias, dtype=torch.float32))
            nn.init.zeros_(self.fc[-1].weight)
   
    def forward(self, start_pose, goal_pose):
        return self.fc(torch.cat([start_pose, goal_pose], dim=-1)).view(-1, self.n_mid, 6)

walls_mesh = [face for w in WALLS for face in make_prism(*w['x'], *w['y'], *w['z'])]
def in_box(pt, box):
    x,y,z = pt
    return (box['x'][0]<=x<=box['x'][1] and box['y'][0]<=y<=box['y'][1] and box['z'][0]<=z<=box['z'][1])
def collision_detect(verts):
    return any(in_box(v, WALLS[0]) or in_box(v, WALLS[1]) for v in verts)
def animate_full(traj: np.ndarray, goal_pose: np.ndarray):
    fig = plt.figure(figsize=(8,6)); ax = fig.add_subplot(111, projection='3d')
    for face in walls_mesh: ax.add_collection3d(Poly3DCollection([face], facecolors="lightgrey", edgecolors="k", alpha=0.8))
    ax.set_box_aspect((1,1.5,1)); ax.set_xlim(0,3); ax.set_ylim(0,2); ax.set_zlim(0,2)
    ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z"); ax.set_title("Learned trajectory")
    poly_cols = [ax.add_collection3d(Poly3DCollection([], facecolors="royalblue", edgecolors="k")) for _ in make_E_blocks()]
    axes_artists = []
    def update(frame):
        nonlocal axes_artists
        for art in axes_artists: art.remove()
        axes_artists = []
        x,y,z,roll,pitch,yaw = traj[frame]
        R = rpy_to_matrix(torch.tensor([[roll,pitch,yaw]], device=DEVICE)).squeeze(0).cpu().numpy()
        world_vertices = []
        for blk, pc in zip(make_E_blocks(), poly_cols):
            verts = []
            for p in blk:
                p_loc, p_w = np.array(p) - LOCAL_COM, R @ (np.array(p) - LOCAL_COM) + np.array([x,y,z])
                verts.append(p_w); world_vertices.append(p_w)
            pc.set_verts([verts])
        if collision_detect(world_vertices): 
            print(f"<Collision at t={frame*DT:.2f}s stopping animation.")
            anim.event_source.stop()
        length = 0.2
        for idx, col in enumerate(("r","g","b")):
            v = R[:, idx]
            axes_artists.append(ax.quiver(x,y,z,v[0],v[1],v[2],length=length,color=col,linewidth=3))
        return poly_cols + axes_artists
    roll, pitch, yaw = goal_pose[3:]; Rg = rpy_to_matrix(torch.tensor([[roll,pitch,yaw]], device=DEVICE)).squeeze(0).cpu().numpy()
    for idx, col in enumerate(("r","g","b")): ax.quiver(*goal_pose[:3], *Rg[:, idx], length=0.2, color=col, linewidth=3)
    anim = FuncAnimation(fig, update, frames=traj.shape[0], interval=DT*1000, blit=False)
    return anim

def get_full_mid(mid_optimizable, batch_size):
    """Helper to combine fixed human-guided points and trainable points."""
    guided_block = human_guided.expand(batch_size, -1, -1)
    return torch.cat([guided_block, mid_optimizable], dim=1)

def get_full_ctrl(mid, start_pose, goal_pose):
    """Helper to construct the full control point tensor for the spline."""
    return torch.cat([start_pose.unsqueeze(1), mid, goal_pose.unsqueeze(1)], dim=1)

def lower_level_optimize_validate(mid_init, start_pose, goal_pose, steps_per_seg):
    """
    Performs lower-level optimization for validation.
    Similar to the training version but focused on getting the final trajectory.
    """
    batch_size = start_pose.shape[0]
    mid_init_trainable = mid_init[:, n_guided:, :].detach()
    mid_optimizable = mid_init_trainable.clone().requires_grad_(True)

    optimizer = optim.Adam([mid_optimizable], lr=LR_LOWER)
    scheduler = lr_scheduler.StepLR(optimizer, step_size=10, gamma=0.9) # Match training scheduler

    for _ in range(N_LOWER_ITERS):
        optimizer.zero_grad()
        mid_full = get_full_mid(mid_optimizable, batch_size)
        ctrl = get_full_ctrl(mid_full, start_pose, goal_pose)
        traj = catmull_rom_spline(ctrl, steps_per_seg)
        
        # The lower-level objective
        loss = compute_lower_loss(traj, mid_full, mid_init.detach())
        loss.backward()
        
        torch.nn.utils.clip_grad_norm_([mid_optimizable], max_norm=1.0)
        optimizer.step()
        scheduler.step()
    
    mid_opt = get_full_mid(mid_optimizable, batch_size)
    ctrl_opt = get_full_ctrl(mid_opt, start_pose, goal_pose)
    traj_opt = catmull_rom_spline(ctrl_opt, steps_per_seg)
    return traj_opt, mid_opt

def run_validation_episode(net, start_pose, goal_pose):
    """Runs a single validation episode and returns the optimized trajectory and losses."""
    net.eval() # Set network to evaluation mode
    steps_per_seg = int(T_SEG / DT)

    with torch.no_grad(): # No gradients needed for the upper-level network during validation
        mid_init = net(start_pose.unsqueeze(0), goal_pose.unsqueeze(0))

    # The lower-level optimization still needs gradients
    traj_opt, mid_opt = lower_level_optimize_validate(mid_init, start_pose.unsqueeze(0), goal_pose.unsqueeze(0), steps_per_seg)
    
    # Compute final losses for reporting
    loss_lower_final = compute_lower_loss(traj_opt, mid_opt, mid_init)
    loss_upper_final = compute_upper_loss_validate(traj_opt, goal_pose.unsqueeze(0), start_pose.unsqueeze(0))

    return traj_opt.squeeze(0), loss_upper_final.item(), loss_lower_final.item()

def upper_collision_loss_validate(traj: torch.Tensor, clearance: float=0.01, check_stride: int=1) -> torch.Tensor:
    B, T, _ = traj.shape
    Lc, checks = 0.0, 0
    for t in range(0, T, check_stride):
        pose = traj[:, t]
        R = rpy_to_matrix(pose[:, 3:])
        pts = OBJ_PTS_LOCAL.unsqueeze(0).expand(B, -1, -1)
        world = (R @ pts.permute(0,2,1)).permute(0,2,1) + pose[:,:3].unsqueeze(1)
        Lc += collision_cost(world, clearance)
        checks += 1
    return Lc / checks

def compute_upper_loss_validate(traj: torch.Tensor, goal: torch.Tensor, start: torch.Tensor) -> torch.Tensor:
    B, T, _ = traj.shape
    end_err_pos = (traj[:,-1,:3]-goal[:,:3]).pow(2).sum(-1)
    end_err_ori = (traj[:,-1,3:]-goal[:,3:]).pow(2).sum(-1)
    L_goal = (end_err_pos + 0.1*end_err_ori).mean()
    dpos = traj[:,1:,:3]-traj[:,:-1,:3]
    L_path = torch.linalg.norm(dpos,dim=-1).sum(-1).mean()
    straight_dist = torch.linalg.norm(goal[:,:3]-traj[:,0,:3],dim=-1).mean()
    L_excess = torch.relu(L_path - straight_dist)
    L_coll = upper_collision_loss_validate(traj)
    x_all, rpy_all = traj[:,:,0], traj[:,:,3:6]
    slab_mask = ((x_all >= DOOR_X_MIN) & (x_all <= DOOR_X_MAX)).float()
    denom = slab_mask.sum() + 1e-6
    L_orient_door = (slab_mask.unsqueeze(-1) * rpy_all.abs()).sum() / denom
    t = torch.linspace(0,1,T,device=traj.device).view(1,T,1).expand(B,T,3)
    target_rpy = start[:,3:].unsqueeze(1) + t*(goal[:,3:].unsqueeze(1)-start[:,3:].unsqueeze(1))
    L_orient_traj = (traj[:,:,3:]-target_rpy).pow(2).sum(-1).mean()
    L_miss = door_penalty(traj[:,:,:3])
    return (w_goal*L_goal +  w_coll_upper*L_coll  + w_miss*L_miss)
    # return (w_goal*L_goal + w_length_excess*L_excess + w_coll_upper*L_coll + w_orient_door*L_orient_door + w_orient_traj*L_orient_traj + w_miss*L_miss)

def validate():
    net = WaypointNet().to(DEVICE)
    net.load_state_dict(torch.load(MODEL_PATH, map_location=DEVICE))
    net.eval() # Set the network to evaluation mode

    print("--- Starting Validation ---")
    
    # Define a few test cases or generate random ones
    test_cases = [
        (torch.tensor([2.2, 0.6, 0.5, math.pi/2, math.pi/4, math.pi/8], dtype=torch.float32, device=DEVICE),
         torch.tensor([0.5, 1.2, 0.4, math.pi/4, math.pi/8, math.pi/6], dtype=torch.float32, device=DEVICE)),
        (torch.tensor([2.5, 0.5, 0.5, math.pi, 0, 0], dtype=torch.float32, device=DEVICE),
         torch.tensor([0.8, 1.5, 0.7, -math.pi/2, -math.pi/4, math.pi/2], dtype=torch.float32, device=DEVICE)),
        (torch.tensor([2.0, 0.3, 0.5, math.pi/2, 0, -math.pi/2], dtype=torch.float32, device=DEVICE),
         torch.tensor([0.3, 1.0, 0.3, math.pi/4, math.pi/4, 0], dtype=torch.float32, device=DEVICE)),
        # Add more diverse test cases here if desired
    ]

    for i, (start_pose, goal_pose) in enumerate(test_cases):
        print(f"\n--- Validation Episode {i+1} ---")
        print(f"Start Pose: {start_pose.cpu().numpy()}")
        print(f"Goal Pose:  {goal_pose.cpu().numpy()}")

        traj_optimized, upper_loss, lower_loss = run_validation_episode(net, start_pose, goal_pose)
        
        print(f"Validation Upper Loss: {upper_loss:.4f}")
        print(f"Validation Lower Loss: {lower_loss:.4f}")
        
        traj_np = traj_optimized.detach().cpu().numpy()
        goal_np = goal_pose.detach().cpu().numpy()
        
        anim = animate_full(traj_np, goal_np)
        plt.show()

    print("\n--- Validation Complete ---")

if __name__ == "__main__":
    validate()