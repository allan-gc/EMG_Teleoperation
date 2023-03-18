#!/usr/bin/env python3

import rospy 
from myo_gestures.myo_raw import MyoRaw
import torch
import numpy as np
from myo_gestures.semg_network import Network_XL
from myo_gestures.train_net import Trainer, best_model_params
from statistics import mode
from myo_gestures.transfer_learn import circle_shift
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion,Vector3
from std_msgs.msg import Header,  Float64, Int32

"""
Publishes the gesture classification and IMU data stream of the Myo Armband for control of the Adroit arm

PUBLISHERS:
    Topic name: "/predicted_gestures", Type: String - The outputted gesture label from the PyTorch model 

    Topic name: "myo_imu",             Type: Imu - The IMU data from the Myo Armband converted into a ROS IMU message for publishing

    Topic name: "myo_connected",    Type: Int32 - Flag that lets other nodes know that connection to the Myo was successful, indicates ready to use

SUBSCRIBERS:
    NONE       

PARAMETERS:
    pytorch_model: The PyTorch .pt file that contains the trained model weights. Stored in the config directory
    pytorch_model-stats: The mean and standard deviation of the raw EMG data that was used for training, used to average out EMG data for classification

"""



class GestureNode():
    def __init__(self):

        """ Initializes real_time node node """

        # Initializing PyTorch model
        rospy.init_node('gestures')
        self.model = Network_XL(7)
        self.model_path = rospy.get_param("pytorch_model")  ## Best digit test was combined_transfer_data_XL3.pt
        self.model_stats = rospy.get_param("pytorch_model_stats")
        self.model.load_state_dict(torch.load(self.model_path,map_location='cpu'))
        self.model.eval()
        self.first_activity = 5 #should be 5 for DIGITS!!!!
        
        self.mean_emg,self.std_emg = np.loadtxt(str(self.model_stats),delimiter=',')  ## combined_transfer_data_argtest_stats.txt FOR ARGTEST, nina_data/combined_transfer_data_stats.txt ORIGINAL THAT WORKED
        self.emg_array = []
        self.pred_array = []
        self.last_pred = 0
        self.pred_cnt = 0
        self.start_ch = 5
        self.goal_ch = self.first_activity
        self.cal_array = []

        # Indicates Myo is not connected
        self.connected = 0

        self.gesture_pub = rospy.Publisher("/predicted_gestures", String, queue_size=10)
        self.imu_pub = rospy.Publisher("myo_imu", Imu, queue_size=10)
        self.connected_pub = rospy.Publisher("myo_connected", Int32, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(1.0/20.0), self.timer_callback)   ### TEST DIF RATES FOR TIMER
        self.gest_msg = String()

    def timer_callback(self, timer):
        # Timer callback that allows fo the connection of the Myo to be established
        while self.connected == 0:  
            # Used to wait for Adroit launch file and Rviz to finish initializing
            rospy.sleep(15.0) 
            try:
                rospy.logwarn("CONNECTING TO MYO..\n")
                myo = MyoRaw(None) 
                myo.connect()
                myo.add_emg_handler(self.proc_emg)
                myo.add_imu_handler(self.proc_imu)
                self.connected =1
            except:
                rospy.sleep(0.3)
                pass
    
        if self.connected == 1:
            connect_msg = Int32()
            connect_msg.data = 1
            self.connected_pub.publish(connect_msg)
            rospy.loginfo("CONNECTED\n")

        try:
            while not rospy.is_shutdown():
                myo.run(1)

        except KeyboardInterrupt:
            # print("KILLING NODE")
            # print('\n')
            self.myo.disconnect()
            print("Disconnected")
            pass

    def proc_emg(self,emg, moving, times=[]):
        '''Takes sampled emg, shifts the channels per the calibration, and builds a 260ms (52 samples)
        input array that is passed through the trained model to produce the predicted gesture. Prediciton
        is only reported when it differs from the previously reported prediction a sufficent number of times.
        ARGS: emg: list containing sample from each of the 8 channels
              moving and times may not be needed'''
        self.emg_array.append(list((emg - self.mean_emg)/(self.std_emg))) #normalization
        self.emg_array = circle_shift(np.array(self.emg_array),self.start_ch,self.goal_ch).tolist() #calibration
        if len(self.emg_array) == 52:
            input = torch.Tensor(self.emg_array)
            input = input.view(1,1,input.shape[0],input.shape[1])
            pred = torch.argmax(self.model(input)).item() #prediction
            self.pred_array.append(pred)
            if len(self.pred_array) == 10:
                try:
                    if mode(self.pred_array) != self.last_pred:
                        self.pred_cnt += 1
                    if self.pred_cnt > 15:
                        # Makes prediction and publishes to topic
                        pred  = self.display_gest(mode(self.pred_array))
                        self.gest_msg.data = pred
                        self.gesture_pub.publish(self.gest_msg)
                        self.last_pred = mode(self.pred_array)
                        self.pred_cnt = 0
                    self.pred_array.clear()
                except:
                    self.pred_array.clear()
            self.emg_array.pop(0)


    def display_gest(self,pred):
        '''Not really necessary, can just use the mode of the pred_array, but can be kept to only test this node by adding loginfo at the end of this function'''
        if pred == 0:
            prediction = "OPEN HAND"
        elif pred == 1:
            prediction = "INDEX"
        elif pred == 2:
            prediction = "MIDDLE"
        elif pred == 3:
            prediction = "RING"
        elif pred == 4:
            prediction = "PINKY"
        elif pred == 5:
            prediction = "THUMB"
        elif pred == 6:
            prediction = "FIST"

        return prediction


    def proc_imu(self, quat,acc,gyro):
        # Myo IMU prcoessing function. Takes in IMU data stream and publishes it to myo_imu topic
        
        quats = Quaternion()
        quats.x = quat[0]/16384.0
        quats.y = quat[1]/16384.0
        quats.z = quat[2]/16384.0
        quats.w = quat[3]/16384.0

        gyro_vec = Vector3()
        gyro_vec.x = 1.5*gyro[0]/16.0
        gyro_vec.y = 1.5*gyro[1]/16.0
        gyro_vec.z = 1.52*gyro[2]/16.0

        acc_vec = Vector3()
        acc_vec.x = acc[0]/2048.0
        acc_vec.y = acc[1]/2048.0
        acc_vec.z = acc[2]/2048.0

        covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        imu = Imu()
        imu.header.stamp = rospy.Time.now()
        imu.header.frame_id = "myo"
        imu.angular_velocity = gyro_vec
        imu.linear_acceleration = acc_vec
        imu.orientation = quats 
        imu.linear_acceleration_covariance = covariance
        imu.angular_velocity_covariance = covariance
        imu.orientation_covariance = covariance
        self.imu_pub.publish(imu)

if __name__ == '__main__':
    rospy.init_node('gestures')
    try:
        GestureNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("KILLING NODE")
        pass



