"""student_mir - controller of the MiR100 robot(s).

Assignment 1 starter: lists the devices and shows how to read the battery / mission status.
Replace the body of the loop with your own code (odometry, go-to-pose, mission handling).
"""
import json

from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

print(f"[{robot.getName()}] devices:")
for i in range(robot.getNumberOfDevices()):
    d = robot.getDeviceByIndex(i)
    print(f"   {d.getName():<28} type={d.getNodeType()}")

left = robot.getDevice("middle_left_wheel_joint")
right = robot.getDevice("middle_right_wheel_joint")
for m in (left, right):
    m.setPosition(float("inf"))
    m.setVelocity(0.0)
left_enc = robot.getDevice("middle_left_wheel_joint_sensor")
right_enc = robot.getDevice("middle_right_wheel_joint_sensor")
left_enc.enable(timestep)
right_enc.enable(timestep)

next_print = 0.0
while robot.step(timestep) != -1:
    t = robot.getTime()
    status = json.loads(robot.getCustomData() or "{}")     # battery, charging, mission (see docs/03)
    if t >= next_print:
        print(f"t={t:.0f}s battery={status.get('level')} route={status.get('mission', {}).get('route')}")
        next_print = t + 10.0
    # TODO: odometry, go-to-pose, follow the mission route
