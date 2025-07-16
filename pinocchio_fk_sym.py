import pinocchio as pin
import pinocchio.casadi as cpin
import casadi as ca
from pathlib import Path

ROBOT_MODEL_PATH = Path(
    "/home/shanto/Documents/arm-ws-v0.0.0/urdf/ur10_v1.1.3/")
URDF_FILENAME = ROBOT_MODEL_PATH / "ur10.urdf"
END_EFFECTOR_NAME = "connection_frame"

# 1) Load your robot model and create the Data
model = pin.buildModelFromUrdf(URDF_FILENAME)
data = model.createData()

# 2) Wrap it for CasADi
cmodel = cpin.Model(model)
cdata = cmodel.createData()

# 3) Create a CasADi symbolic joint vector of length 6
q = ca.SX.sym('q', model.nq)

# 4) Record forward kinematics on the CasADi model
cpin.framesForwardKinematics(cmodel, cdata, q)

# 5) Extract the frame SE3 of your end-effector
frame_id = model.getFrameId(END_EFFECTOR_NAME)
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
T = ca.vertcat(row0, row1, row2, row3)

# 8) Create a CasADi Function q → 4×4 homogeneous transform
fk_hom = ca.Function('fk_hom', [q], [T],
                     ['q'], ['T_hom'])

# 9) (Optional) Print its signature and test with a dummy
print(fk_hom)
# Evaluate numerically at q = [0,0,0,0,0,0]
# returns a DM 4×4 matrix directly:
T0 = fk_hom([0, 0, 0, 0, 0, 0])
print("T(0):\n", T0)
