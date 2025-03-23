#!/usr/bin/env python3
"""
Works by emulating keyboard_interface inputs from the svan_utils package: refer to `src/svan_utils/scripts/keyboard_interface.py` for the original script.
"""
import rospy
from std_msgs.msg import Float32MultiArray
import time

# Initialize ROS node
rospy.init_node('auto_control')

key_data = Float32MultiArray()
key_data.data = [4,0,0,0,0,0,0,0,0] # trot gait
publisher = rospy.Publisher('/svan/io_interface', Float32MultiArray, queue_size=1)
print("Publisher initialized")


default_x_vel = 0.5
default_y_vel = 0.5
default_yaw_vel = 1
default_move_time = 5
default_turn_time = 13


def constrain_velocity(vel, min_val=0.0, max_val=1.0):
    return max(min_val, min(max_val, vel))


def reset_movement():
    key_data.data[1] = 0  # y velocity
    key_data.data[2] = 0  # x velocity
    key_data.data[5] = 0  # anticlockwise yaw
    key_data.data[6] = 0  # clockwise yaw
    publisher.publish(key_data)
    print("Movement reset")


def move_forward(x_vel=default_x_vel):
    reset_movement()
    x_vel = constrain_velocity(x_vel)
    key_data.data[2] = x_vel  # Set forward velocity
    publisher.publish(key_data)
    print(f"Moving forward with velocity: {x_vel}")


def move_backward(x_vel=default_x_vel):
    reset_movement()

    x_vel = -constrain_velocity(x_vel)
    key_data.data[2] = x_vel  # Set backward velocity
    publisher.publish(key_data)
    print(f"Moving backward with velocity: {x_vel}")


def move_left(y_vel=default_y_vel):
    reset_movement()

    y_vel = -constrain_velocity(y_vel)
    key_data.data[1] = y_vel  # Set left velocit
    
    publisher.publish(key_data)
    print(f"Moving left with velocity: {y_vel}")


def move_right(y_vel=default_y_vel):
    reset_movement()

    y_vel = constrain_velocity(y_vel)
    key_data.data[1] = y_vel  # Set right velocity
    publisher.publish(key_data)
    print(f"Moving right with velocity: {y_vel}")


def turn_right(yaw_vel=default_yaw_vel):
    reset_movement()

    yaw_vel = constrain_velocity(yaw_vel)
    key_data.data[2] = 0.1
    key_data.data[5] = yaw_vel  # Set anticlockwise yaw
    publisher.publish(key_data)
    print(f"Turning left with velocity: {yaw_vel}")

def sleep():
    reset_movement()
    key_data[0] = 6
    publisher.publish(key_data)


def turn_left(yaw_vel=default_yaw_vel):
    reset_movement()

    yaw_vel = constrain_velocity(yaw_vel)
    key_data.data[6] = yaw_vel  # Set clockwise yaw
    publisher.publish(key_data)
    print(f"Turning right with velocity: {yaw_vel}")

# Function to stop
def stop():
    reset_movement()
    key_data.data[0] = 1  
    publisher.publish(key_data)
    print("Stopped")

# Function to set trot gait
def trot():
    key_data.data[0] = 4  
    publisher.publish(key_data)
    print("Trot mode activated")


def make_square(x_vel=default_x_vel, yaw_vel=default_yaw_vel, move_time=default_move_time, turn_time=default_turn_time):
    print("Starting square pattern movement")
    

    x_vel = constrain_velocity(x_vel)
    yaw_vel = constrain_velocity(yaw_vel)
    
    for i in range(4):

        move_forward(x_vel)
        time.sleep(move_time)
        

        turn_left(yaw_vel)
        time.sleep(turn_time)
    
    print("Square pattern completed")

if __name__ == '__main__':
    try:
        # First set to trot mode
        trot()
        time.sleep(2) 
        
        # Start moving in a square pattern
        while not rospy.is_shutdown():
            # Make a square with custom velocities
            make_square(x_vel=0.3, yaw_vel=1)
            

            stop()
            time.sleep(2)
            

            trot()
            time.sleep(2)
            
    except rospy.ROSInterruptException:
        pass
    finally:
        # Ensure the robot stops if the script is interrupted
        stop()