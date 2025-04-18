# IMPORTING THE LIBRARIES #
from controller import Supervisor, GPS, Motor
import json, os, math

# ENVIRONMENT CONFIGURATION #
#************************************************************#
robot = Supervisor()
timestep = int(robot.getBasicTimeStep())
robot_node = robot.getFromDef("EPUCK")
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

#GPS.....
gps = robot.getDevice("gps")
gps.enable(timestep)

# HYPER PARAMETERS #
#````````````````````#
target_x, target_y = -0.5, 0.3
threshold = 0.5
speed = 3.0
directions = {
    "F": (speed, speed),
    "L": (-speed, speed),
    "R": (speed, -speed),
    "B": (-speed, -speed)
}
#````````````````````#

# THIS FUNCTION LOADS THE LEARNED OPTIMAL PATH FROM TRAINED MODEL'S FILE
def load_learned_path(file_name):
    if os.path.exists(file_name):
        with open(file_name, "r") as f:
            return json.load(f)
    else:
        print("Learned path file not found!")
        return []
        
#THIS FUNCTION RETRIEVES THE CURRENT ROBOT POSITION
def get_position():
    pos = gps.getValues()
    return pos[0], pos[1]
    
#THIS FUNCTION RETRIEVES THE DISTANCE TO THE TARGET BY USING EUCLEDIAN DISTANCE
def distance(x, y):
    return math.sqrt((x - target_x)**2 + (y - target_y)**2)
    
#THE MOVE FUNCTION TO MOVING ROBOT AHEAD, DEPENDING ON THE CURRENT MOVE
def move(l_speed, r_speed, steps=50):
    left_motor.setVelocity(l_speed)
    right_motor.setVelocity(r_speed)
    for _ in range(steps):
        robot.step(timestep)
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
    robot.step(timestep)

#CHECKING FOR THE FILE PATH
def load_path(file_name):
    if os.path.exists(file_name):
        with open(file_name, "r") as f:
            return json.load(f)
    return []

# Load path
learned_path = load_learned_path("../training/learned_path.json")
print(f"📂 Loaded {len(learned_path)} steps.")

# ROBOT RESET FUNCTIONALITY
robot.step(timestep)
initial_pos = robot_node.getField("translation").getSFVec3f()
initial_rot = robot_node.getField("rotation").getSFRotation()
robot_node.getField("translation").setSFVec3f(initial_pos)
robot_node.getField("rotation").setSFRotation(initial_rot)

# USING THE LEARNED PATH TO REPLAY THEM
print("\n🔁 Replaying the path...")
for i, step in enumerate(learned_path):
    if step == "OBS":
        print(f"🚧 Step {i+1}: Obstacle → Left Turn")
        move(*directions["L"], steps=20)
        move(*directions["F"], steps=30)
    else:
        print(f"➡️ Step {i+1}: {step}")
        move(*directions[step])

# CONSOLE OUTPUTS
x, y = get_position()
print(f"\n📍 Final position: x={x:.3f}, y={y:.3f}")
if distance(x, y) < threshold:
    print("✅ Target reached!")
else:
    print("❌ Target NOT reached.")
