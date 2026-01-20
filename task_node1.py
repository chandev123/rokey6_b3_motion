import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import os
import yaml
from ament_index_python.packages import get_package_share_directory

# DSR Imports (move_basic style)
import DR_init

# Global Constants
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"
VELOCITY = 60
ACC = 60

# Global Variables
task_node = None
state = 'IDLE' # FSM State
task_queue = [] # Queue for Task Dicts: [{'cmd': 'movej', 'pos': [], 'vel': ...}]

# DR_init Settings
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def initialize_node():
    """노드 초기화 및 DSR 설정"""
    global task_node
    
    # DSR_ROBOT2 Import (move_basic style)
    from DSR_ROBOT2 import set_tool, set_tcp
    
    # DSR Settings
    task_node.get_logger().info("Initializing task_node with settings:")
    # Direct Control Node needs to set these to ensure robot knows tool/tcp
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    
    # Subscribers
    task_node.create_subscription(
        String,
        '/user/command',
        command_callback,
        10
    )
    
    task_node.get_logger().info('Task Node Started. Current State: IDLE')
    task_node.get_logger().info('Waiting for command "s" to start...')

def command_callback(msg):
    global state
    cmd = msg.data.lower()
    
    if cmd == 's':
        if state == 'IDLE':
            transition_to('DECIDE_ORDER') # Start Sequence Planning
        else:
            task_node.get_logger().warn(f'Ignored start command. Current state: {state}')
    elif cmd == 'q':
        transition_to('IDLE')
        task_node.get_logger().info('Reset command received. Returned to IDLE.')

def transition_to(new_state):
    global state
    task_node.get_logger().info(f'[FSM] State Change: {state} -> {new_state}')
    state = new_state

def perform_task():
    """FSM Logic Loop (Manual Spin)"""
    global task_node
    task_node.get_logger().info("Performing task loop...")
    
    while rclpy.ok():
        # Spin once to process callbacks (command 's', 'q')
        rclpy.spin_once(task_node, timeout_sec=0.05)
        
        # Run FSM Logic
        fsm_loop()

def load_task_queue_from_yaml():
    """Load poses from yaml and build task queue"""
    global task_queue, task_node
    
    try:
        package_share_directory = get_package_share_directory('project_hit')
        config_path = os.path.join(package_share_directory, 'config', 'pose_config.yaml')
        
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)
            
        poses = config.get('poses', {})
        sequence = ['home', 'point_1', 'point_2', 'point_3'] # Define order
        
        task_queue = []
        for point_name in sequence:
            if point_name in poses:
                task_queue.append({
                    'cmd': 'movej',
                    'pos': poses[point_name],
                    'vel': VELOCITY,
                    'acc': ACC,
                    'name': point_name
                })
        
        task_node.get_logger().info(f"Loaded {len(task_queue)} tasks from YAML: {sequence}")
        return True
        
    except Exception as e:
        task_node.get_logger().error(f"Failed to load config: {e}")
        return False

def fsm_loop():
    global state, task_queue
    
    # DSR Imports for Control
    from DSR_ROBOT2 import movej, movel
    
    if state == 'IDLE':
        pass

    elif state == 'DECIDE_ORDER':
        # Planning Phase: Decide what to do
        task_node.get_logger().info("Deciding Order (Loading YAML)...")
        if load_task_queue_from_yaml():
            transition_to('RUN')
        else:
            task_node.get_logger().error("Planning Failed. Returning to IDLE.")
            transition_to('IDLE')

    elif state == 'RUN':
        # Execution Phase: Process Queue
        if task_queue:
            current_task = task_queue.pop(0)
            task_node.get_logger().info(f"Executing Task: {current_task['name']}")
            
            if current_task['cmd'] == 'movej':
                # Execute Blocking Call (Safe in Manual Spin)
                try:
                    movej(current_task['pos'], vel=current_task['vel'], acc=current_task['acc'])
                except Exception as e:
                     task_node.get_logger().error(f"Motion Error: {e}")
            
            # Add small delay if needed or check status
            time.sleep(0.5)
        else:
            task_node.get_logger().info("All tasks completed. Returning to IDLE.")
            transition_to('IDLE')

def main(args=None):
    global task_node
    
    rclpy.init(args=args)
    task_node = rclpy.create_node('task_node', namespace=ROBOT_ID)
    
    # DR_init Node Setup
    DR_init.__dsr__node = task_node
    
    try:
        initialize_node()
        perform_task()
    except KeyboardInterrupt:
        task_node.get_logger().info("Shutting down...")
    finally:
        task_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
