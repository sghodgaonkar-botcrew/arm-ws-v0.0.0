import os
import datetime
from acados_template import AcadosOcpSolver
from re import S
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


ROBOT_MODEL_PATH = Path(
    "/home/shanto/Documents/arm-ws-v0.0.0/urdf/ur10_v1.1.3/")
URDF_FILENAME = ROBOT_MODEL_PATH / "ur10.urdf"
END_EFFECTOR_NAME = "connection_frame"


q_test = np.array([0, 0, 0, 0, 0, 0])
# Format: [x, y, z, qx, qy, qz, qw] - quaternion in [qx, qy, qz, qw] format
p_test = np.array([0.5, 0.5, 0.5, 0, -0.7071068, 0, 0.7071068])


# 1) Load your robot model and create the Data
pin_model = pin.buildModelFromUrdf(URDF_FILENAME)
pin_data = pin_model.createData()
frame_id = pin_model.getFrameId(END_EFFECTOR_NAME)
# 2) Wrap it for CasADi
cmodel = cpin.Model(pin_model)
cdata = cmodel.createData()

# 3) Create a CasADi symbolic joint vector of length 6
q = ca.SX.sym('q', pin_model.nq)

# 4) Record forward kinematics on the CasADi model
cpin.framesForwardKinematics(cmodel, cdata, q)

# 5) Extract the frame SE3 of your end-effector

pose = cdata.oMf[frame_id]

# 6) Grab rotation (3×3) and translation (3×1) as SX
R = pose.rotation    # Eigen::Matrix< SX,3,3 >
t = pose.translation  # "      "   vector< SX,3 >

# 7) Build the 4×4 homogeneous matrix
#    bottom row [0,0,0,1]
zero = ca.SX(0)
one = ca.SX(1)
# First three rows
row0 = ca.horzcat(R[0, 0], R[0, 1], R[0, 2], t[0])
row1 = ca.horzcat(R[1, 0], R[1, 1], R[1, 2], t[1])
row2 = ca.horzcat(R[2, 0], R[2, 1], R[2, 2], t[2])
# Last row
row3 = ca.horzcat(zero,   zero,   zero,   one)
# Stack into a 4×4 SX matrix
T_Fk = ca.vertcat(row0, row1, row2, row3)

fk = ca.Function('fk', [q], [T_Fk], ['q'], ['T_Fk'])

print(fk)

# q_test = np.array([0, 0, 0, 0, 0, 0])
print(f"FK from casadi: \n{np.round(np.array(fk(q_test)), 3)}")

p = SX.sym('p', 7)

p_t = p[0:3]
p_quat = p[3:7]
# Unpack quaternion in [qx, qy, qz, qw] format
qx, qy, qz, qw = p_quat[0], p_quat[1], p_quat[2], p_quat[3]

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

r_target = ca.vertcat(
    ca.horzcat(r00, r01, r02),
    ca.horzcat(r10, r11, r12),
    ca.horzcat(r20, r21, r22),
)

t_target = ca.vertcat(p[0], p[1], p[2])

# default‐constructed identity SE3
# Construct the 4x4 homogeneous transform matrix
T_target = ca.vertcat(
    ca.horzcat(r00, r01, r02, p[0]),  # [R | t]
    ca.horzcat(r10, r11, r12, p[1]),
    ca.horzcat(r20, r21, r22, p[2]),
    ca.horzcat(0, 0, 0, 1)            # [0 | 1]
)

r_target_t = ca.transpose(r_target)
t_inv = -r_target_t @ t_target

row0 = ca.horzcat(r_target_t[0, 0], r_target_t[0,
                  1], r_target_t[0, 2], t_inv[0])
row1 = ca.horzcat(r_target_t[1, 0], r_target_t[1,
                  1], r_target_t[1, 2], t_inv[1])
row2 = ca.horzcat(r_target_t[2, 0], r_target_t[2,
                  1], r_target_t[2, 2], t_inv[2])
row3 = ca.horzcat(zero,      zero,      zero,     one)

T_target_inv = ca.vertcat(row0, row1, row2, row3)

target = ca.Function('target', [p], [T_target], ['p'], ['T_target'])

print(f"Target from casadi: \n{np.round(np.array(target(p_test)), 3)}")

E = cLog6(T_target_inv @ T_Fk).vector    # 6×1 error
W = ca.SX(np.diag([1, 1, 1, 1, 1, 1]))  # your pose_weights_

cost_expr = 0.5 * E.T @ W @ E

cost = ca.Function('cost', [q, p], [cost_expr])

print(cost)


print(f"Cost from casadi: {cost(q_test, p_test)}")


pin.forwardKinematics(pin_model, pin_data, q_test)
pin.updateFramePlacements(pin_model, pin_data)
print(f"Frame placement: \n{np.round(pin_data.oMf[frame_id].homogeneous, 3)}")
print(f"Target : \n{np.round(pin.XYZQUATToSE3(p_test).homogeneous, 3)}")
# Format: [x, y, z, qx, qy, qz, qw] - quaternion in [qx, qy, qz, qw] format
error = pin.log6(pin.XYZQUATToSE3(p_test).inverse().homogeneous @
                 pin_data.oMf[frame_id].homogeneous).vector
print(
    f"Error from pinocchio: {0.5 * error.T @ np.diag([1, 1, 1, 1, 1, 1]) @ error}")


# === 8) set up Acados OCP ===
ll = pin_model.lowerPositionLimit
uu = pin_model.upperPositionLimit

ocp = AcadosOcp()
acados_model = AcadosModel()

acados_model.name = 'ik_model'
# 1) State = joint angles; no controls
acados_model.x = q              # q: SX.sym('q',6)
# acados_model.u omitted entirely (nu=0)

# 2) Parameter = target pose
acados_model.p = p              # p: SX.sym('p',7)

# 3) Cost = linear-LS on the 6×1 error vector E
acados_model.y = E            # E = log6( … )  (6×1)
acados_model.y_e = E
acados_model.expr_y = cost_expr  # ½ EᵀW E
acados_model.f_expl_expr = ca.SX.zeros(pin_model.nq, 1)
ocp.model = acados_model

# 4) Weight matrices
ocp.cost.W = np.zeros((6, 6))       # no stage cost
ocp.cost.W_e = np.diag([1]*6)         # terminal cost

# 5) Selection matrices
ocp.cost.Vx = np.eye(6)             # y = I·x
ocp.cost.Vu = np.zeros((6, 0))       # no u
ocp.cost.Vx_e = np.eye(6)

ny = 6

# since you’re doing a pure terminal cost, stage and terminal refs may both be zero
ocp.cost.yref = np.zeros(ny)    # stage-cost reference
ocp.cost.yref_e = np.zeros(ny)    # terminal reference

# 6) Bounds on x
ocp.constraints.lbx = pin_model.lowerPositionLimit
ocp.constraints.ubx = pin_model.upperPositionLimit
ocp.constraints.idxbx = np.arange(pin_model.nq)

# 7) (Optional) default params for codegen
ocp.parameter_values = np.zeros(7)
ocp.solver_options.N_horizon = 1
ocp.solver_options.tf = 1.0

ocp.solver_options.qp_solver = 'FULL_CONDENSING_HPIPM'
ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
ocp.solver_options.integrator_type = 'ERK'

ocp.solver_options.nlp_solver_type = 'SQP_RTI'

# === 7) generate & build the Acados solver ===


output_dir = 'c_code/ik_solver_acados'
os.makedirs(output_dir, exist_ok=True)
ocp.code_export_directory = output_dir
timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
# this will:
#   1) write acados_ocp.json into output_dir,
#   2) generate all C code under output_dir,
#   3) invoke make to build the shared library.
solver = AcadosOcpSolver(ocp,
                         json_file=f"{output_dir}/acados_ocp_{timestamp}.json",
                         build=True,      # compile the shared lib
                         generate=True,
                         verbose=True,)   # generate the C sources

print(f"✅ Solver generated and built in {output_dir}")
