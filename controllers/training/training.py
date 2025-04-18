# IMPORTING THE LIBRARIES #
from controller import Robot, GPS, Motor, Supervisor
import math, json, random, os, logging, sys
from datetime import datetime

# THE FUNCTION THAT HANDLES THE TRAINING LOGS, TO BE WRITTEN TO THE LOG FILES#
# ----------------------------------------------------------------------------#
log_filename = f"training_output_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
logging.basicConfig(
    level=logging.INFO,  # Logging messages that are INFO level and above (INFO, WARNING, ERROR, etc.)
    format='%(asctime)s | %(message)s',
    handlers=[
        logging.FileHandler(log_filename, mode='w', encoding='utf-8'),
        logging.StreamHandler(sys.stdout)
    ]
    # This sends log output to two places: FileHandler: (Writes logs to the log_filename file), StreamHandler: Sends logs to stdout (i.e., prints them in the terminal).
)
log = logging.getLogger() # Gets the default logger instance so that we can use to save in the log file....

def flush_log():
    for handler in log.handlers:
        handler.flush() # writing all the logs immediately as per their appearance to the console...actually forcing the write function...

#-----------------------------------------------------------------------------------#

# ENVIRONMENT CONFIGURATION #
#************************************************************#
robot = Supervisor()
timestep = int(robot.getBasicTimeStep())
robot_node = robot.getFromDef("EPUCK")
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

#GPS...
gps = robot.getDevice("gps")
gps.enable(timestep)

# Distance sensors
ps_names = ["ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"]
ps = [robot.getDevice(name) for name in ps_names]
for sensor in ps:
    sensor.enable(timestep)

# Initial positions
robot.step(timestep)
initial_position = robot_node.getField("translation").getSFVec3f()
initial_rotation = robot_node.getField("rotation").getSFRotation()

#************************************************************#

# HYPER PARAMETERS #
#````````````````````#
target_x, target_y = -0.5, 0.3
speed = 3.0
threshold = 0.05
directions = {
    "F": (speed, speed),
    "L": (-speed, speed),
    "R": (speed, -speed),
    "B": (-speed, -speed)
}
# Q-learning SPECIFIED
alpha = 0.1
gamma = 0.9
epsilon = 0.2
q_table = {}

# TRAINING-SPECIFIED
path, best_path = [], []
best_path_length = float('inf')
epoch, steps_taken = 1, 0
max_steps = 5000
episode_reward = 0

#````````````````````#

###PRELIMINIRY SETUP###
if os.path.exists("q_table.json"): #maintaining a Q-Table for the robot's choice
    with open("q_table.json", "r") as f:
        q_table = json.load(f)


################################################################
################# FUNCTION DEFINITIONS #########################

# THIS FUNCTION HANDLES THE CURRENT POSITION OF THE ROBOT, AND RETURNS AS COORDINATES X, AND Y#
def get_position():
    pos = gps.getValues()
    return pos[0], pos[1] # NOT RETURNING Z COORDINATE: pos[2]

# THIS FUNCTION COMPUTES THE EUCLIDEAN DISTANCE BETWEEN THE ROBOT'S CURRENT POSITION AND THE TARGET#
def distance(x, y): # 
    return math.sqrt((x - target_x) ** 2 + (y - target_y) ** 2) #Pythagorean theorem
    # returns a single value

# THIS FUNCTION HANDLE THE MOVEMENT OF THE ROBOT, GIVEN THE LEFT AND RIGHT MOTOR SPEEC ALONG WITH A DEFAULT VALUE FOR STEPS TO MOVE, IT MOVES THE ROBOT AND RESETS THE VELOCITIES
def move(l_speed, r_speed, steps=50):
    left_motor.setVelocity(l_speed)
    right_motor.setVelocity(r_speed)
    for _ in range(steps):
        robot.step(timestep)
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
    robot.step(timestep) # syncing the robot controllers, (GPS, Sensors, all other variables), by updating the simulation environment

# THIS FUNCTION CONFIRMS IF THE FOBOT CONFRONTS THE OBSTACLE
def detect_obstacle():
    return ps[7].getValue() > 80 or ps[0].getValue() > 80 # Considering the front side for obstacle detection

# THIS FUNCTION RETRIEVES THE Q-VALUE ASSOCIATED WITH A GIVEN STATE AND ACTION, OF HOW A PARTICULAR ACTION CAN IMPACT THE REWARD
def get_q_value(state, action):
    if state not in q_table: # appending new entry if it's not already there...
        q_table[state] = {a: 0 for a in directions}
    return q_table[state][action]

# THIS FUNCTION UPDATES THE Q-VALUES
def update_q_value(state, action, reward, next_state):
    max_future_q = max(q_table.get(next_state, {a: 0 for a in directions}).values())
    current_q = get_q_value(state, action) # getting q-value
    q_table[state][action] = current_q + alpha * (reward + gamma * max_future_q - current_q)
    # using Q-learning formula to update the values: Q(s,a)=Q(s,a)+α⋅(reward+γ⋅max(Q(s ′,a ′))−Q(s,a))

# STARTING THE COMPUTATION AS SOON AS ROBOT STARTS MOVING.....
while robot.step(timestep) != -1:
    x, y = get_position()
    dist = distance(x, y)
    
    #Checking if the robot is close enough to the goal 
    if dist < threshold:
        # applying final efficiency reward
        efficiency_bonus = max(0, 1000 - (steps_taken * 5))  # Strong reward for fewer steps
        episode_reward += efficiency_bonus
        
        #Using log function to also add this to the log file.....
        log.info(f"\n🏁 Target reached at epoch {epoch}!")
        log.info(f"📍 Final position: x={x:.3f}, y={y:.3f}")
        log.info(f"🪜 Steps taken: {steps_taken}")
        log.info(f"🎯 Episode total reward: {episode_reward:.2f}")
        
        #Saving the best path as an optimal choice (for the inference part)
        if len(path) < best_path_length:
            best_path = list(path)
            best_path_length = len(path)
            with open("learned_path.json", "w") as f:
                json.dump(best_path, f, indent=2)
            log.info(f"✅ New best path saved with length {best_path_length}!")

        robot_node.getField("translation").setSFVec3f(initial_position)
        robot_node.getField("rotation").setSFRotation(initial_rotation)
        
        for _ in range(10): robot.step(timestep)
        path.clear()
        steps_taken = 0
        episode_reward = 0
        epoch += 1
        flush_log()
        continue
        
    # HANGLING THE CASE WHEN ROBOT REACHES THE HIGHEST STEPS SET
    if steps_taken >= max_steps:
        log.info(f"\n⛔ Max steps reached in epoch {epoch}.")
        break
        
    #initializing some values
    state = (round(x, 2), round(y, 2))
    exploring = random.uniform(0, 1) < epsilon
    selected_reward = 0
    best_direction = None
    best_reward = -float('inf')

    if exploring: # I AM SETTING THE LEAST VALUE FOR THIS CASE, JUST TO INCREASE THE EXPLORING DIRECTION INSTEAD OF EXPLOITING MEANS RANDOM MOVES.....
        best_direction = random.choice(list(directions))
    else:
        for direction, (l_speed, r_speed) in directions.items():
        # GETTING THE CURRENT POSITION, MOVING SINGLE STEP, CALCULATING DISTANCE WITH NEW POSITION,
        # GOING FOR OBSTACLE DETECTION CASE, REWARDING NEGATIVE WITH OBSTACLE CASE,
        # UPDATING Q-VALUES, CHECKING FOR THE BEST REWARD AND DIRECTION,
        # AND MOVING WITH THE ELSE CASE PART....
        
            original_pos = get_position()
            move(l_speed, r_speed, steps=1)
            new_x, new_y = get_position()
            new_dist = distance(new_x, new_y)
            obstacle = detect_obstacle()
            reward = -20 if obstacle else (dist - new_dist) * 100  # strong reward if approaching goal
            update_q_value(state, direction, reward, (round(new_x, 2), round(new_y, 2)))
            if reward > best_reward:
                best_reward = reward
                best_direction = direction
                selected_reward = reward
            move(-l_speed, -r_speed, steps=1)
    
    #HANGLING THE OBSTACLE CASE
    if detect_obstacle():
        log.info("🚧 Obstacle detected → turning left.")
        move(*directions["L"], steps=20)
        move(*directions["F"], steps=30)
        path.append("OBS") # AUNIQUE VALUE WAS CHOSEN TO MAKE THE FLOW SIMPLE...
        steps_taken += 1
        episode_reward -= 10  # Penalty for obstacle
        flush_log()
        continue
        
    #WHEN CALCULATED THE BEST MOVE, GO FOR IT !
    if best_direction:
        move(*directions[best_direction])
        path.append(best_direction)
        steps_taken += 1
        episode_reward += selected_reward
    else:
        break
    flush_log() # LOCKS THE LOGS TO THE LOG FILE


# SAVING THE Q-TABLE
with open("q_table.json", "w") as f:
    json.dump(q_table, f)
    
#LOGGING THE INFORMATION
log.info("\n✅ Training complete.")
flush_log()
