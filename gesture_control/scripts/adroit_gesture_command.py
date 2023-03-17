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
from enum import Enum, auto
from statistics import mode

"""
Publishes joy stick commands that control the Adroit joints. This node communicates with the generic_joystick node in the hdthdt_generic_joystick package, and essentially uses the Myo to send joystick commands 

PUBLISHERS:
    Topic name: "/joy", Type: Joy - Used to control the buttons and axes of the joy message. Buttons and axes are mapped to certain Adroit joint and state controls


SUBSCRIBERS:
    Topic name: "myo_imu",          Type: Imu - The IMU data from the Myo Armband converted into a ROS IMU message for publishing

    Topic name: "myo_connected",    Type: Int32 - Flag that lets other nodes know that connection to the Myo was successful, indicates ready to use

    Topic name: "/hdt_arm/pincer_joint_position_controller/command", Type: Float64 - Reads the current position of the pincer joint on the Adroit

    Topic name: "/hdt_arm/joint2_position_controller/command", Type: Float64 - Reads the current position of joint 2 on the Adroit

    Topic name: "/hdt_arm/joint3_position_controller/command", Type: Float64 - Reads the current position of joint 3 on the Adroit

    Topic name: "//execute_trajectory/result", Type: ExecuteTrajectoryActionResult - Checks if Adroit trajectory execution was successful 

PARAMETERS:
    ~sim: Initializes node to be used for Rviz simulation, enables simulated startup instead of real startup

"""


"""
JOY MSG CONTROL
ALL ARM JOINT AXES MUST HAVE CONTROLLER VALUE OF 0.0 FOR IT TO STOP MOVING
ARM JOINT AXES: Joint 1: axes[0] for first joint left-right. CCW Rotation when looking down at joint is "left", max val of -1.0 for controller. CW Rotation is "right", max val of 1.0 for controller.
                Joint 2: axes[1] for second joint up-down. To rotate up, the max val is -1.0 for controller. To rotate down, max val is 1.0 for controller.
                Joint 3: axes[3] for third joint up-down. To rotate up, the max val is -1.0 for controller. To rotate down, max val is 1.0 for controller.
                Joint 4: axes[7] for fourth joint left-right. Supponation is "right", the max val is -1.0 for controller. Pronation is "left", max val is 1.0 for controller.

PINCER JOINT AXES MUST HAVE VALUE OF 1.0 FOR IT TO STOP OPENING/CLOSING PINCER. JOINT AXES: OPEN JOINT: axes[5], Controller value between [-1.0,1.0], CLOSE JOINT: axes[2], Controller value between [-1.0,1.0]

JOINT 2 POSITION LIMITS: -0.9132230736114667, 0.20525960675468785

JOINT 3 JOINT POSITION LIMITS: [-0.94571528018878]

"""



class State(Enum):
    """
     Adroit Control States that determine which control messages are sent
    """
    MOTION_CONTROL = (auto(),)
    ALIGNING = (auto(),)
    IDLE = (auto(),),
    INITIALIZED = (auto(),)



class GestureControl():

    """ Initializes gesture control node"""
    def __init__(self):
        super(GestureControl, self).__init__()
        rospy.init_node("gesture_control")
        self.myo_sim = rospy.get_param("~sim")
        self.pub = rospy.Publisher("/joy", Joy, queue_size=10)
        self.gest_sub = rospy.Subscriber("/predicted_gestures", String, self.gest_callback)
        self.imu_sub = rospy.Subscriber("/myo_imu", Imu, self.imu_callback)
        self.connected_sub = rospy.Subscriber("myo_connected", Int32, self.connected_callback)
        self.gripper_pos_sub = rospy.Subscriber("/hdt_arm/pincer_joint_position_controller/command", Float64, self.gripper_pos_callback)
        self.joint2_pos_sub = rospy.Subscriber("/hdt_arm/joint2_position_controller/command", Float64, self.joint2_pos_callback)
        self.joint3_pos_sub = rospy.Subscriber("/hdt_arm/joint3_position_controller/command", Float64, self.joint3_pos_callback)
        self.execute_sub = rospy.Subscriber("/execute_trajectory/result", ExecuteTrajectoryActionResult, self.exec_callback)
        self.myo_tf = tf.TransformBroadcaster()
        self.myo_pos = Pose()
        self.myo_pos.position.x = 0.0
        self.myo_pos.position.y = -0.5
        self.myo_pos.position.z  = 0.0

        self.rate = 10.0 # 10hz
        self.joy_msg = Joy()
        self.gesture_msg = String()
        self.prev_gesture = String()
        self.prev_gesture.data = "null"
        self.imu_msg = Imu()
        self.gripper_pos_msg = Float64()
        self.joint2_pos_msg = Float64()
        self.joint3_pos_msg = Float64()
        self.connected_msg = Int32()
        self.connected_msg.data = 0
        self.execute_msg = ExecuteTrajectoryActionResult()
        self.timer = rospy.Timer(rospy.Duration(1.0/10.0), self.timer_callback)
        self.final_initialized = 0
        self.exec_initialized = 0

        # Initial axes and buttons used for joy message, sends no control to joints and presses no buttons
        self.joy_axes = [-0.0, -0.0, 1.0, -0.0, -0.0, 1.0, -0.0, -0.0]
        self.joy_buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        self.button_cntr = 0
        self.open_counter = 0
        self.close_counter = 0
        self.gest_counter = 0
     
        
        self.wrist_control = 0.0
        self.y_joy_control = 0.0  ## Corresponds to right-left movement of axes[0] (Joint 1)
        self.z_joy_control = 0.0  ## Corresponds to up-down movement of axes[3] (Joint 3)
        self.gest_list = []
        self.gesture = 'null'
        self.state = State.IDLE
        
        # Calls the shutdown functions when nodes are killed. Shutdown function puts node in Stow position 
        # rospy.on_shutdown(self.arm_shutdown)

    def connected_callback(self, msg):
        """ Callback for myo connected topic, checks if Myo connection is established """
        self.connected_msg = msg

    def exec_callback(self, exec_res):
        """ Callback for moveit execution topic, checks if Adroit has completed the startup execution and can take more commands """
        self.execute_msg = exec_res

    def gripper_pos_callback(self, gripper_pos):
        """ Callback for gripper position controller, used to set pincer joint position limits """
        self.gripper_pos_msg = gripper_pos
        if self.state == State.MOTION_CONTROL:
            if self.gesture== "PINKY":
                self.joy_msg.axes[2] = 0.2 
                if self.gripper_pos_msg.data < 0.35:   
                    # Limit the gripper joint position so that it does not continue closing when making gesture
                    self.joy_msg.axes[2] = 1.0
                    if self.joint2_pos_msg.data < -0.1:
                        # If the Adroit is extended, retract its joint while holding an item
                        self.joy_msg.axes[1] = -0.2
                self.pub.publish(self.joy_msg)

            elif self.gesture == "FIST":
                self.joy_msg.axes[2] = 0.2 
                if self.gripper_pos_msg.data < 0.03:   
                    self.joy_msg.axes[2] = 1.0
                    if self.joint2_pos_msg.data < -0.1:
                        self.joy_msg.axes[1] = -0.2

                self.pub.publish(self.joy_msg)

            else:
                # If no gripping gestures, stop gripping and beign opening
                self.joy_msg.axes[2] = 1.0
                self.joy_msg.axes[5] = 0.5
                if self.gripper_pos_msg.data > 0.80:    
                    # joint limit for opening the pincer joint
                    self.joy_msg.axes[5] = 1.0
                self.pub.publish(self.joy_msg)


    def gest_callback(self, gest_data):
        """ Callback for the predicted gestures from the gestures node. Checks the getsures being predicted, and then picks the gesture seen the most every 20 gestures to deal with small misclassifications"""
        self.gesture_msg = gest_data
        if self.gesture_msg.data == "OPEN HAND":
            self.gest_list.append(0)
        elif self.gesture_msg.data == "INDEX":
            self.gest_list.append(1)
        elif self.gesture_msg.data == "MIDDLE":
            self.gest_list.append(2)
        elif self.gesture_msg.data == "RING":
            self.gest_list.append(3)
        elif self.gesture_msg.data == "PINKY":
            self.gest_list.append(4)
        elif self.gesture_msg.data == "THUMB":
            self.gest_list.append(5)
        elif self.gesture_msg.data == "FIST":
            self.gest_list.append(6)

        if len(self.gest_list) < 20:
            gest_encoder = mode(self.gest_list)
            if gest_encoder == 0:
                self.gesture = "OPEN HAND"
            elif gest_encoder == 1:
                self.gesture = "INDEX"
            elif gest_encoder == 2:
                self.gesture = "MIDDLE"
            elif gest_encoder == 3:
                self.gesture = "RING"
            elif gest_encoder == 4:
                self.gesture = "PINKY"
            elif gest_encoder == 5:
                self.gesture = "THUMB"
            elif gest_encoder == 6:
                self.gesture = "FIST"
            self.gest_list = []

        rospy.loginfo("Predicted Gesture: %s", self.gesture)

        if self.final_initialized == 1:
            if self.gesture == "RING":
                self.state = State.ALIGNING 
                self.joy_msg.axes[0] = 0.0
                self.joy_msg.axes[3] = 0.0 
                self.joy_msg.axes[7] = 0.0
                self.pub.publish(self.joy_msg)
                rospy.loginfo("REALIGNING WITH ROBOT- CHANGE GESTURE TO RESUME CONTROL")
            else:
                self.state = State.MOTION_CONTROL
            

    def joint2_pos_callback(self, joint2_msg):
        """ Callback for Joint 2 position controller, used to set joint limits when extending the joint """
        self.joint2_pos_msg = joint2_msg
        if self.final_initialized == 1 and self.gesture == "MIDDLE":
            # rospy.loginfo("EXTENDING JOINT 2")
            self.gest_counter+=1
            if self.gest_counter >15:
                self.joy_msg.axes[1] = 0.2
                if self.joint2_pos_msg.data < -0.913:
                        self.joy_msg.axes[1] = 0.0
                        rospy.logwarn("JOINT2 POS LIMIT REACHED: %s", self.joint2_pos_msg.data)

        else:
            self.joy_msg.axes[1] = 0.0
            self.gest_counter = 0
            self.pub.publish(self.joy_msg)


    def joint3_pos_callback(self, joint3_msg):
        """ Callback for Joint 3 position controller, used to set joint limits in IMU control """
        self.joint3_pos_msg = joint3_msg


    def imu_callback(self, imu_data):
        """ Callback for IMU data from Myo. Uses the IMU data as Joy message control inputs for adroit joint control """

        self.imu_msg = imu_data
        self.wrist_control = (self.imu_msg.angular_velocity.x / 60.0) 
        self.y_joy_control = self.imu_msg.angular_velocity.z / 60.0
        self.z_joy_control = self.imu_msg.angular_velocity.y / 60.0


        # Myo Gyro Control Limits. Myo gives off signals within the +/- 0.01 range even when not moving, so just send a command of 0.0 to the Joy Commands for Adroit to ensure no joints move when Myo is not moving
        if self.wrist_control < 0.01 and self.wrist_control > -0.01:
            self.wrist_control =0.0

        if self.y_joy_control < 0.01 and self.y_joy_control > -0.01:
            self.y_joy_control =0.0

        if self.z_joy_control < 0.01 and self.z_joy_control > -0.01:
            self.z_joy_control =0.0

        
        # Adroit Joint Position Limits, if joint limit reached, send a joy stick command that moves it away from limit to prvent collision

        # Joint3 Position Limit To Avoid crashing into table 
        if self.joint3_pos_msg.data < -0.945:
            self.z_joy_control = -0.2
            rospy.logwarn("JOINT 3 POS LIMIT REACHED: %s", self.joint2_pos_msg.data)
        
        # if (self.connected_msg.data == 1 and self.final_initialized == 1):
        # if (self.state ==  State.MOTION_CONTROL):
        #     self.joy_msg.header.stamp = rospy.Time.now()
        #     self.joy_msg.axes = self.joy_axes
        #     self.joy_msg.buttons =  self.joy_buttons
        #     self.joy_msg.axes[0] = -1*self.y_joy_control #The gryo values for the y-axis of the myo are negative when moving up and positive when moving down when the armband is on, so mulitplied by -1 for easier control
        #     self.joy_msg.axes[3] = self.z_joy_control  
        #     self.joy_msg.axes[7] = self.wrist_control 
        #     self.pub.publish(self.joy_msg)

        ## Make Myo TF
        self.myo_tf.sendTransform((self.myo_pos.position.x, self.myo_pos.position.y, self.myo_pos.position.z), 
                                tf.transformations.quaternion_from_euler(0, self.z_joy_control, self.y_joy_control),
                                rospy.Time.now(),
                                "myo_imu",
                                "base_link")



    def press_button(self,button):
        """ Helper function for populating the button field of joy message. If button 2 wants to be used, call press_button (2) to publish a value in that field in the joy message"""
        while self.button_cntr <30:
            if self.button_cntr > 15:
                self.joy_msg.buttons[button] = 0 
            else:
                self.joy_msg.buttons[button] = 1 
            self.pub.publish(self.joy_msg)
            self.button_cntr +=1
            rospy.sleep(0.3)
        rospy.loginfo("SENDING BUTTON COMMAND %s", button)
        self.button_cntr = 0

    def init_gripper(self):
        """ Helper function for opening the gripper on startup. """
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
        """ Helper function for starting up the rviz if simulation only, puts Adroit in Joint by Joint control and opens the gripper. See generic_joystick node for more """
        self.joy_msg.header.stamp = rospy.Time.now()
        self.joy_msg.axes = self.joy_axes
        self.joy_msg.buttons =  self.joy_buttons
        if self.exec_initialized == 0:
            self.press_button(2)
            self.init_gripper()
            self.exec_initialized = 1
            self.final_initialized =1
            self.state = State.MOTION_CONTROL
            rospy.loginfo("FINISHED SIM INIT: MYO CONTROL ACTIVE")

    def real_init(self):
        """ Helper function for starting the real robot and rviz. Puts the robot in Ready position and in Joint by Joint control state. See generic_joystick node for more """
        self.joy_msg.header.stamp = rospy.Time.now()
        self.joy_msg.axes = self.joy_axes
        self.joy_msg.buttons =  self.joy_buttons
        if self.exec_initialized == 0:
            self.press_button(2)
            self.press_button(1)
            self.press_button(3)  
            self.exec_initialized = 1
        if (self.execute_msg.result.error_code.val == 1):
            self.press_button(2)
            self.init_gripper()
            self.execute_msg.result.error_code.val = 0
            self.final_initialized =1
            self.state = State.MOTION_CONTROL
            rospy.loginfo("FINISHED REAL INIT: MYO CONTROL ACTIVE")

    def arm_shutdown(self):
        """ Helper Function that puts adroit in Stow position on shutdown """
        rospy.loginfo("ARM SHUTTING DOWN...")
        self.joy_msg.header.stamp = rospy.Time.now()
        self.joy_msg.buttons =  self.joy_buttons
        self.joy_msg.axes = [-0.0, -0.0, 0.2, -0.0, -0.0, 1.0, -0.0, -0.0]
        self.press_button(1)
        self.press_button(1)
        self.press_button(3)


    def timer_callback(self, timer):

        """ Timer that waits for Myo to be connected so that an intialization sequence can begin """

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
