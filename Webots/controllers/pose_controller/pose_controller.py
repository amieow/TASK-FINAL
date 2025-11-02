from controller import Robot, Keyboard
import math
from enum import Enum,auto

robot = Robot()
timestep = int(robot.getBasicTimeStep())
keyboard = Keyboard()
keyboard.enable(timestep)
class mode(Enum):
    Single=auto()
    Pair=auto()
class action(Enum):
    Increment=auto()
    Decrement=auto()
dict_keys = {
    'W': ord('W'),
    'S': ord('S'),
    'A': ord('A'),
    'D': ord('D'),
    'E': ord('E'),
    'Q': ord('Q'),
    'C': ord('C'),
    'Z': ord('Z')
}
keys_debounce={key: False for key in dict_keys.keys()}
keys_input_step={key: 0 for key in dict_keys.keys()}
keys_input_interval=0.3

INCREMENT = 0.1
# Data containers
motors = {}
pos_sensors = {}
selection=0
current_mode=mode.Single

# Joint list (same order as provided)
motor_names = [
	# Head motors
	"Head", "Neck",
 
	"ShoulderL", "ShoulderR",
    "ArmUpperL", "ArmUpperR",
    "ArmLowerL", "ArmLowerR",
	
    # Paha Z (Belok)
    "PelvYL", "PelvYR",
    
    # Paha X (Miring)
	"PelvL", "PelvR",
    
    # Paha atas Y (gerak depan/belakang)
	"LegUpperL", "LegUpperR",
    
    # Lutut
    "LegLowerL","LegLowerR",
    
    # Tumit 
    "AnkleL","AnkleR",
    
    # Tumit X (miring) 
    "FootL", "FootR"
]

def take_input():
    key = keyboard.getKey()
    keys = set()
    while key != -1:
        keys.add(key)
        key = keyboard.getKey()
    return keys

def get_all_pos():
    pos=[round(pos_sensors[name].getValue(),3) for name in motor_names]
    print(pos)

def print_joint_list(selection):
    print(f"INCREMENT: {INCREMENT:.3f}")
    print(f"Current mode: {current_mode.name}")  
    print(f"Selection: {selection}" if current_mode==mode.Single else f"Selection: {selection} and {selection+1}")
    print("\n--- Joint list (index : name) ---")
    for i, name in enumerate(motor_names):
        if i == selection:
            if current_mode==mode.Pair:
                print(f"[{i:2d}] {name}: {round(pos_sensors[name].getValue(), 3)} <== SELECTED")
                print(f"[{i+1:2d}] {motor_names[i+1]}: {round(pos_sensors[motor_names[i+1]].getValue(), 3)} <== SELECTED")
            else:
                print(f"[{i:2d}] {name}: {round(pos_sensors[name].getValue(), 3)} <== SELECTED")
        else:
            if current_mode==mode.Pair and i==selection+1:
                continue
            else:
                print(f"[{i:2d}] {name}: {round(pos_sensors[name].getValue(), 3)}")
    get_all_pos()
            
    
    
# Initialize motors and sensors
def setup_motor():
    print(f"Initializing {len(motor_names)} motors and sensors (if present)...")
    for name in motor_names:
        device = robot.getDevice(name)
        if device is None:
            print(f"[WARN] Motor not found: {name}")
            continue
        motors[name] = device
        # try to get position sensor named <MotorName>S
        sname = name + "S"
        sensor = robot.getDevice(sname)
        if sensor:
            try:
                sensor.enable(timestep)
                pos_sensors[name] = sensor
                print(f"[OK] Sensor enabled: {sname}")
            except Exception:
                # some devices may not allow enable in this context; ignore gracefully
                print(f"[WARN] Could not enable sensor: {sname}")
        # initialize target to current sensor value if available, otherwise 0.0
        if name in pos_sensors:
            try:
                curr = pos_sensors[name].getValue()
            except Exception:
                curr = 0.0
        try:
            device.setPosition(curr)
        except Exception:
            pass
    print(motors)
    print(pos_sensors)
    
def adjust_motor(mode, action):
    if mode == mode.Single:
        if action == action.Increment:
            motors[motor_names[selection]].setPosition(pos_sensors[motor_names[selection]].getValue()+INCREMENT)
        elif action == action.Decrement:
            motors[motor_names[selection]].setPosition(pos_sensors[motor_names[selection]].getValue()-INCREMENT)
    if mode == mode.Pair:
        if action == action.Increment:
            motors[motor_names[selection]].setPosition(pos_sensors[motor_names[selection]].getValue()+INCREMENT)
            motors[motor_names[selection+1]].setPosition(pos_sensors[motor_names[selection+1]].getValue()-INCREMENT)
        elif action == action.Decrement:
            motors[motor_names[selection]].setPosition(pos_sensors[motor_names[selection]].getValue()-INCREMENT)
            motors[motor_names[selection+1]].setPosition(pos_sensors[motor_names[selection+1]].getValue()+INCREMENT)
    
setup_motor()
# Main loop
running = True
while robot.step(timestep) != -1 and running:
    print('\nControls: W/S select servo, Q/E adjust increment value, A/D increase/decrease, C change mode (Single/Pair), Z zeroing')
    print_joint_list(selection)
    keys=take_input()
    current_time =robot.getTime()
    
    if dict_keys['C'] in keys and not keys_debounce['C']:
        keys_debounce['C']=True
        current_mode=mode.Pair if current_mode==mode.Single else mode.Single
        selection=2 if current_mode==mode.Pair else selection
    elif dict_keys['C'] not in keys:
        keys_debounce['C']=False
    
    if dict_keys['Z'] in keys and not keys_debounce['Z']:
        keys_debounce['Z']=True
        if current_mode==mode.Single:
            motors[motor_names[selection]].setPosition(0.0)
        elif current_mode==mode.Pair:
            motors[motor_names[selection]].setPosition(0.0)
            motors[motor_names[selection+1]].setPosition(0.0)
    elif dict_keys['Z'] not in keys:
        keys_debounce['Z']=False
    
    if dict_keys['W'] in keys and not keys_debounce['W']:
        keys_debounce['W']=True
        selection-= 1 if current_mode==mode.Single else 2
        selection= selection if selection<len(motor_names) else 0
        if current_mode==mode.Pair and motor_names[selection] in ["Head","Neck"]:
            selection=2
    elif dict_keys['W'] not in keys:
        keys_debounce['W']=False
        
    if dict_keys["S"] in keys and not keys_debounce['S']:
        keys_debounce['S']=True
        selection+=1 if current_mode==mode.Single else 2
        selection= len(motor_names)-1 if selection<=0 else selection
        try:
            if current_mode==mode.Pair and motor_names[selection] in ["Head","Neck"]:
                selection=2
        except IndexError:
            selection=18
    elif dict_keys['S'] not in keys:
        keys_debounce['S']=False
        
    if dict_keys['Q'] in keys and (current_time-keys_input_step['Q'])>keys_input_interval:
        keys_input_step['Q']=current_time
        INCREMENT -= 0.01
    if dict_keys['E'] in keys and current_time-keys_input_step['E']>keys_input_interval:
        keys_input_step['E']=current_time
        INCREMENT += 0.01
    
    if dict_keys['A'] in keys and current_time-keys_input_step['A']>keys_input_interval:
        adjust_motor(current_mode,action.Decrement)
    if dict_keys['D'] in keys and current_time-keys_input_step['D']>keys_input_interval:
        adjust_motor(current_mode,action.Increment)
        