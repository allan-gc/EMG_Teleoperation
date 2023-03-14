from __future__ import print_function

import sys
import copy
import rospy
from moveit_msgs.msg import ExecuteTrajectoryActionResult
import geometry_msgs.msg
from sensor_msgs.msg import Joy
from std_msgs.msg import String, Int32, Float64, Header
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose
import tf


#### JOY MSG CONTROL
# ALL ARM JOINT AXES MUST HAVE CONTROLLER VALUE OF 0.0 FOR IT TO STOP MOVING
### ARM JOINT AXES: Joint 1: axes[0] for first joint left-right. CCW Rotation when looking down at joint is "left", max val of -1.0 for controller. CW Rotation is "right", max val of 1.0 for controller.
###                 Joint 2: axes[1] for second joint up-down. To rotate up, the max val is -1.0 for controller. To rotate down, max val is 1.0 for controller.
###                 Joint 3: axes[3] for third joint up-down. To rotate up, the max val is -1.0 for controller. To rotate down, max val is 1.0 for controller.
###                 Joint 4: axes[4] for fourth joint left-right. Supponation is "right", the max val is -1.0 for controller. Pronation is "left", max val is 1.0 for controller.
###
### PINCER JOINT AXES MUST HAVE VALUE OF 1.0 FOR IT TO STOP OPENING/CLOSING
## PINCER (GRIPPER) JOINT AXES: OPEN JOINT: axes[5], Controller value between [-1.0,1.0]
##                              CLOSE JOINT: axes[2], Controller value between [-1.0,1.0]


class GestureControl():

    def __init__(self):
        super(GestureControl, self).__init__()
        rospy.init_node("adroit_gesture_control")
        self.myo_sim = rospy.get_param("~sim")
        self.pub = rospy.Publisher("/joy", Joy, queue_size=10)
        self.gest_sub = rospy.Subscriber("/predicted_gestures", String, self.gest_callback)
        self.imu_sub = rospy.Subscriber("/myo_imu", Imu, self.imu_callback)
        self.connected_sub = rospy.Subscriber("myo_connected", Int32, self.connected_callback)
        self.gripper_pos_sub = rospy.Subscriber("/hdt_arm/pincer_joint_position_controller/command", Float64, self.gripper_pos_callback)
        self.execute_sub = rospy.Subscriber("/execute_trajectory/result", ExecuteTrajectoryActionResult, self.exec_callback)
        self.myo_tf = tf.TransformBroadcaster()
        self.myo_pos = Pose()
        self.myo_pos.position.x = 0.0
        self.myo_pos.position.y = -0.5
        self.myo_pos.position.z  = 0.0
        self.t0 = 0.0
        self.dt = 0.0

        self.rate = 10.0 # 10hz

        self.prev_time = Header()
        self.joy_msg = Joy()
        self.gesture_msg = String()
        self.prev_gesture = String()
        self.prev_gesture.data = "null"
        self.imu_msg = Imu()
        self.gripper_pos_msg = Float64()
        self.connected_msg = Int32()
        self.connected_msg.data = 0
        self.execute_msg = ExecuteTrajectoryActionResult()
        self.timer = rospy.Timer(rospy.Duration(1.0/10.0), self.timer_callback)
        # self.button_initialized = 0
        self.final_initialized = 0
        self.exec_initialized = 0

        self.joy_axes = [-0.0, -0.0, 1.0, -0.0, -0.0, 1.0, -0.0, -0.0]
        self.joy_buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        self.button_cntr=0
        self.open_counter = 0
        self.close_counter = 0
        self.shutdown_counter = 0
        self.rollF = 0
        self.pitchF = 0
        
        self.wrist_control = 0.0
        self.y_joy_control = 0.0  ## Corresponds to right-left movement of axes[0] (Joint 1)
        self.z_joy_control = 0.0  ## Corresponds to up-down movement of axes 
        self.shutdown_list = []

        # rospy.on_shutdown(self.arm_shutdown)

    def connected_callback(self, msg):
        self.connected_msg = msg
        # rospy.loginfo("I heard %s", self.gesture_msg.data)

    def exec_callback(self, exec_res):
        self.execute_msg = exec_res
        # rospy.loginfo("I heard %s", self.execute_msg.result.error_code.val)

    def gripper_pos_callback(self, gripper_pos):
        self.gripper_pos_msg = gripper_pos
        if self.final_initialized == 1:
            if self.gesture_msg.data == "FIST":
                # rospy.loginfo("IN FIST")
                # self.joy_msg.axes[5] = 0.5 
                # print("GRIPPER POS", self.gripper_pos_msg.data )
                self.joy_msg.axes[2] = 0.2 ## axis[2] to close,axis[5] to open
                if self.gripper_pos_msg.data < 0.35:    ## Limit the gripper joint position so that it does not continue closing when making gesture
                    # rospy.loginfo("STOP CLOSING %s", self.gripper_pos_msg.data )   ### If axis[5] has a value other than 1.0, it will still open the gripper in the next pub
                    self.joy_msg.axes[2] = 1.0
                self.pub.publish(self.joy_msg)
            else:
                self.joy_msg.axes[2] = 1.0
                self.joy_msg.axes[5] = 0.5
                if self.gripper_pos_msg.data > 0.80:    ## Limit the gripper joint position so that it does not continue closing when making gesture
                    # rospy.loginfo("STOP OPENING %s", self.gripper_pos_msg.data )   ### If axis[5] has a value other than 1.0, it will still open the gripper in the next pub
                    self.joy_msg.axes[5] = 1.0
                self.pub.publish(self.joy_msg)
        # rospy.loginfo("I heard %s", self.gripper_pos_msg.data )


    def gest_callback(self, gest_data):
        self.gesture_msg = gest_data


    def imu_callback(self, imu_data):
        self.imu_msg = imu_data

        # rospy.loginfo("IMU ACC X: %s", self.imu_msg.linear_acceleration.x)
        # rospy.loginfo("IMU ACC Y: %s", self.imu_msg.linear_acceleration.y)
        # rospy.loginfo("IMU GYRO X: %s", self.imu_msg.angular_velocity.x)
        # rospy.loginfo("IMU GRYO Y: %s", -1*self.imu_msg.angular_velocity.y / 60.0)

        #### THIS IS WHAT WORKS SO FAR
        self.wrist_control = self.imu_msg.angular_velocity.x / 60.0
        self.y_joy_control = self.imu_msg.angular_velocity.z / 60.0
        self.z_joy_control = self.imu_msg.angular_velocity.y / 60.0

        if self.wrist_control < 0.01 and self.wrist_control > -0.01:
            self.wrist_control =0.0

        if self.y_joy_control < 0.01 and self.y_joy_control > -0.01:
            self.y_joy_control =0.0

        if self.z_joy_control < 0.01 and self.z_joy_control > -0.01:
            self.z_joy_control =0.0
        
        if (self.connected_msg.data == 1 and self.final_initialized == 1):
            self.joy_msg.header.stamp = rospy.Time.now()
            self.joy_msg.axes = self.joy_axes
            self.joy_msg.buttons =  self.joy_buttons
            self.joy_msg.axes[0] = -1*self.y_joy_control 
            self.joy_msg.axes[3] = self.z_joy_control  ## The gryo values for  the y-axis of the myo are negative when moving up and positive when moving down when the armband is on
            self.joy_msg.axes[7] = self.wrist_control 
            self.pub.publish(self.joy_msg)

        ## Make Myo TF
        self.myo_tf.sendTransform((self.myo_pos.position.x, self.myo_pos.position.y, self.myo_pos.position.z), 
                                tf.transformations.quaternion_from_euler(0, self.z_joy_control, self.y_joy_control),
                                rospy.Time.now(),
                                "myo_imu",
                                "base_link")



    def press_button(self,button):
        while self.button_cntr <30:
            if self.button_cntr > 15:
                self.joy_msg.buttons[button] = 0 
            else:
                self.joy_msg.buttons[button] = 1 
            self.pub.publish(self.joy_msg)
            self.button_cntr +=1
            # rospy.sleep(0.7) When in launch file
            rospy.sleep(0.3)
        rospy.loginfo("SENDING BUTTON COMMAND %s", button)
        self.button_cntr = 0

    def init_gripper(self):
        while self.open_counter <10:
            if self.open_counter  >6:
                self.joy_msg.axes[5] = 1.0
                # self.joy_msg.axes[7] = 0.0
            else:
                self.joy_msg.axes[5] = 0.1
                # self.joy_msg.axes[7] = -1.0
            self.pub.publish(self.joy_msg)
            self.open_counter +=1
            rospy.sleep(0.3)
        rospy.loginfo("FINISHED INTIALIZING GRIPPER")
        
        self.open_counter = 0

    def sim_init(self):
        self.joy_msg.header.stamp = rospy.Time.now()
        self.joy_msg.axes = self.joy_axes
        self.joy_msg.buttons =  self.joy_buttons
        if self.exec_initialized == 0:
            self.press_button(2)
            self.init_gripper()
            self.exec_initialized = 1
            self.final_initialized =1
            rospy.loginfo("FINISHED SIM INIT: MYO CONTROL ACTIVE")

    def real_init(self):
        self.joy_msg.header.stamp = rospy.Time.now()
        self.joy_msg.axes = self.joy_axes
        self.joy_msg.buttons =  self.joy_buttons
        if self.exec_initialized == 0:
            self.press_button(2)
            self.press_button(1)
            self.press_button(3)   ###  CHECK IF REAL INIT WORKS WITH THE CHANGED NAMED SRDF TARGET
            self.exec_initialized = 1
        if (self.execute_msg.result.error_code.val == 1):
            # rospy.sleep(0.5)
            self.press_button(2)
            self.init_gripper()
            self.execute_msg.result.error_code.val = 0
            self.final_initialized =1
            rospy.loginfo("FINISHED REAL INIT: MYO CONTROL ACTIVE")

    def arm_shutdown(self):
        rospy.loginfo("ARM SHUTTING DOWN...")
        self.joy_msg.header.stamp = rospy.Time.now()
        self.joy_msg.buttons =  self.joy_buttons
        self.joy_msg.axes = [-0.0, -0.0, 0.2, -0.0, -0.0, 1.0, -0.0, -0.0]
        self.press_button(1)
        self.press_button(1)
        self.press_button(3)
        # rospy.loginfo("ARM SHUT DOWN")


    def timer_callback(self, timer):

        if self.connected_msg.data == 1:
            if self.myo_sim:
                self.sim_init()
            else:
                self.real_init()

        ## NOT FOR SHUTDOWN, ANY GESTURE CAN BE USED TO TRIGGER ANY EVENT IF REPEATED SEQEUENTIALLY
        # self.shutdown_counter+=1

        # if (self.prev_gesture.data != self.gesture_msg.data) and (self.gesture_msg.data == "FIST"):
        #     self.shutdown_counter = 0
        #     self.shutdown_list.append(1)
        #     if len(self.shutdown_list == 4):
        #         self.arm_shutdown()

        # self.prev_gesture.data = self.gesture_msg.data
        # if self.shutdown_counter > 25 and self.shutdown_list:
        #     self.shutdown_list.pop(0)

        # rospy.loginfo("SHUTDOWN LIST %s", self.shutdown_list)

        if self.rate:
            rospy.sleep(1.0/self.rate)
        else:
            rospy.sleep(1.0)


if __name__ == "__main__":
    GestureControl()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
