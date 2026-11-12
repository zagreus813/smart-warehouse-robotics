"""student_arm - controller shared by all 6 UR3e arms (input arms and output arms).

Every arm runs its own copy of this program; robot.getName() tells you which one you are
("ur3e_in_0" ... "ur3e_out_2"). Assignment 1 does not use the arms, so this stub only holds
the arm in a safe ready pose. You will extend it from assignment 3 onwards.
"""
from controller import Robot

READY_POSE = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

robot = Robot()
timestep = int(robot.getBasicTimeStep())
name = robot.getName()

print(f"[{name}] devices:", ", ".join(robot.getDeviceByIndex(i).getName() for i in range(robot.getNumberOfDevices())))

motors = [robot.getDevice(j) for j in JOINTS]
for m, q in zip(motors, READY_POSE):
    m.setPosition(q)

while robot.step(timestep) != -1:
    pass  # TODO (assignment 3): pick / place logic
