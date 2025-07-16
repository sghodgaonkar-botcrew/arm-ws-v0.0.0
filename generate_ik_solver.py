#!/usr/bin/env python3
"""
generate_ik_solver.py

One‐shot 6‐DOF IK:
    minₓ ½ e(x,p)^T W e(x,p)
    e(x,p) = log6( p^{-1} ⋅ FK_frame(x) )
with N=1, joint‐limit bounds, and C code export.
"""

import numpy as np
import casadi as ca
from casadi import SX, transpose
from acados_template import AcadosOcp, AcadosModel
from pathlib import Path
import pinocchio as pin
import pinocchio.casadi as cpin
from pinocchio.casadi import (
    Model as CasadiModel,
    Data as CasadiData,
    SE3 as cSE3,
    log6 as cLog6,
)

# === 1) load URDF & build CasADi model/data ===
ROBOT_MODEL_PATH = Path(
    "/home/shanto/Documents/arm-ws-v0.0.0/urdf/ur10_v1.1.3/")
URDF_FILENAME = ROBOT_MODEL_PATH / "ur10.urdf"
END_EFFECTOR_NAME = "connection_frame"

pin_model = pin.buildModelFromUrdf(URDF_FILENAME)
cad_model = CasadiModel(pin_model)
cad_data = CasadiData(cad_model)

frame_id = pin_model.getFrameId(END_EFFECTOR_NAME)

# === 2) symbols ===
nq = pin_model.nq    # should be 6
np_target = 7                # [px,py,pz, qw,qx,qy,qz]
x = SX.sym('x', nq)
p = SX.sym('p', np_target)

# === 3) FK to get T_q ===
cpin.framesForwardKinematics(cad_model, cad_data, x)
# SE3Tpl<SX> from pinocchio.casadi :contentReference[oaicite:0]{index=0}
T_q = cad_data.oMf[frame_id]

# === 4) build T_target manually ===
# split translation & quaternion (w,x,y,z)
p_t = p[0:3]
p_quat = p[3:7]
qw, qx, qy, qz = p_quat[0], p_quat[1], p_quat[2], p_quat[3]

# quaternion→rotation matrix (CasADi SX)
r00 = 1 - 2*(qy*qy + qz*qz)
r01 = 2*(qx*qy - qw*qz)
r02 = 2*(qx*qz + qw*qy)
r10 = 2*(qx*qy + qw*qz)
r11 = 1 - 2*(qx*qx + qz*qz)
r12 = 2*(qy*qz - qw*qx)
r20 = 2*(qx*qz - qw*qy)
r21 = 2*(qy*qz + qw*qx)
r22 = 1 - 2*(qx*qx + qy*qy)

R = ca.vertcat(
    ca.horzcat(r00, r01, r02),
    ca.horzcat(r10, r11, r12),
    ca.horzcat(r20, r21, r22),
)

# default‐constructed identity SE3
# Construct the 4x4 homogeneous transform matrix
T_target = ca.vertcat(
    ca.horzcat(r00, r01, r02, p[0]),  # [R | t]
    ca.horzcat(r10, r11, r12, p[1]),
    ca.horzcat(r20, r21, r22, p[2]),
    ca.horzcat(0, 0, 0, 1)            # [0 | 1]
)
# copy in our translation & rotation :contentReference[oaicite:1]{index=1}
# cpin.copy(p_t,           T_target.translation())
# cpin.copy(R,             T_target.rotation())

# === 5) se(3) error & cost ===
E = cLog6(cad_model, cad_data, ca.transpose(T_target) @ T_q)
e_vec = E.toVector()     # 6×1 error
W = np.diag([1, 1, 1, 1, 1, 1])  # your pose_weights_

cost_expr = 0.5 * transpose(e_vec) @ W @ e_vec

# === 6) set up Acados OCP ===
ocp = AcadosOcp()
model = AcadosModel()

model.name = 'ik_model'
model.x = x
model.p = p
model.y = e_vec
model.y_e = e_vec
model.expr_y = cost_expr

ocp.model = model
ocp.dims.N = 1

ocp.solver_options.qp_solver = 'FULL_CONDENSING_QPOASES'
ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
ocp.solver_options.integrator_type = 'ERK'

# === 7) joint‐limit bounds ===
ll = pin_model.lowerPositionLimit
uu = pin_model.upperPositionLimit

ocp.constraints.xmin = ll
ocp.constraints.xmax = uu
ocp.constraints.idxbx = list(range(nq))

# only terminal cost
ocp.cost.W = np.zeros((6, 6))
ocp.cost.W_e = W

# === 8) export C code ===
output_dir = 'c_code/ik_solver_acados'
ocp.code_export_directory = output_dir
ocp.generate()

print(f"✅ Solver generated in {output_dir}/c_generated")
