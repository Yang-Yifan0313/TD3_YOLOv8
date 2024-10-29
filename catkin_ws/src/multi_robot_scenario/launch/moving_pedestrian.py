#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SpawnModel, SetModelState, GetModelState
from tf.transformations import quaternion_from_euler
import math

def create_pedestrian_sdf():
    return """
    <?xml version="1.0" ?>
    <sdf version="1.5">
      <model name="pedestrian">
        <static>false</static>
        <link name="link">
          <collision name="collision">
            <geometry>
              <box>
                <size>0.5 0.5 0.02</size>
              </box>
            </geometry>
          </collision>
          <visual name="visual">
            <geometry>
              <mesh>
                <uri>model://person_walking/meshes/walking.dae</uri>
              </mesh>
            </geometry>
          </visual>
        </link>
      </model>
    </sdf>
    """

def spawn_pedestrian():
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    
    model_pose = Pose()
    model_pose.position.x = 0
    model_pose.position.y = 0
    model_pose.position.z = -0.9  # Start at ground level

    spawn_model('pedestrian', create_pedestrian_sdf(), '', model_pose, 'world')
    rospy.loginfo(f"Pedestrian spawned at position: x={model_pose.position.x}, y={model_pose.position.y}, z={model_pose.position.z}")


def move_pedestrian(side_length=4.0):
    pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz
    
    model_state = ModelState()
    model_state.model_name = 'pedestrian'
    
    
    speed = 0.5  # Speed of movement (m/s)
    
    start_time = rospy.get_time()
    last_log_time = start_time
    
    while not rospy.is_shutdown():
        current_time = rospy.get_time()
        t = current_time - start_time
        
        # Calculate position along the square path (counterclockwise)
        path_position = (t * speed) % (4 * side_length)
        if path_position < side_length:
            model_state.pose.position.x = 0
            model_state.pose.position.y = path_position
            yaw = math.pi / 2  # Facing positive Y direction
        elif path_position < 2 * side_length:
            model_state.pose.position.x = path_position - side_length
            model_state.pose.position.y = side_length
            yaw = 0  # Facing positive X direction
        elif path_position < 3 * side_length:
            model_state.pose.position.x = side_length
            model_state.pose.position.y = 3 * side_length - path_position
            yaw = -math.pi / 2  # Facing negative Y direction
        else:
            model_state.pose.position.x = 4 * side_length - path_position
            model_state.pose.position.y = 0
            yaw = math.pi  # Facing negative X direction
        
        model_state.pose.position.z = -0.9 
        
        # Set orientation
        quaternion = quaternion_from_euler(0, 0, yaw)
        model_state.pose.orientation.x = quaternion[0]
        model_state.pose.orientation.y = quaternion[1]
        model_state.pose.orientation.z = quaternion[2]
        model_state.pose.orientation.w = quaternion[3]
        
        pub.publish(model_state)
        
        # Log every 10 seconds
        if current_time - last_log_time >= 10:
            rospy.loginfo(f"Current position: x={model_state.pose.position.x:.2f}, y={model_state.pose.position.y:.2f}, z={model_state.pose.position.z:.2f}")
            last_log_time = current_time
        
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('moving_pedestrian')
        spawn_pedestrian()
        move_pedestrian(side_length=4.0)  # This will create a 10x10 meter square path
    except rospy.ROSInterruptException:
        pass