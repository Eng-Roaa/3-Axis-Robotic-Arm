import os
import roboticstoolbox as rtb

urdf_path = os.path.abspath(
    os.path.join(
        os.path.dirname(__file__),
        '..',
        'Robotics Arm URDF',
        'urdf',
        'Robotics ARm.SLDASM.urdf'
    )
)

print("URDF path:")
print(urdf_path)

print("\nFile exists:")
print(os.path.exists(urdf_path))

try:
    robot = rtb.Robot.URDF(urdf_path)

    print("\nSUCCESS!")
    print("Robot:", robot)
    print("Number of joints:", robot.n)

    for i, link in enumerate(robot.links):
        print(i, link)

except Exception as e:
    print("\nERROR:")
    print(type(e).__name__)
    print(e)