#!/usr/bin/env python3

import rospy 
from myo_gestures.myo_raw import MyoRaw
import torch
import numpy as np
from myo_gestures.semg_network import Network_XL, Network_enhanced
from myo_gestures.train_net import Trainer, best_model_params
import time, sys, copy, csv, argparse
from statistics import mode
from myo_gestures.transfer_learn import circle_shift, most_active
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion,Vector3
from std_msgs.msg import Header,  Float64, Int32


class GestureNode():
    def __init__(self):
        rospy.init_node('real_time_pred')
        # self.gesture_pub = rospy.Publisher("predicted_gestures", String, queue_size=10)
        # self.test_pub = rospy.Publisher("test", String, queue_size=10)
        # self.timer = rospy.Timer(rospy.Duration(1.0/100.0), self.timer_callback)
        # gest_msg = String()

        self.model = Network_XL(7)
        self.model_path = rospy.get_param("pytorch_model")  ## Best digit test was combined_transfer_data_XL3.pt
        self.model_stats = rospy.get_param("pytorch_model_stats")
        self.model.load_state_dict(torch.load(self.model_path,map_location='cpu'))
        self.model.eval()
        self.first_activity = 4 #should be 4 for DIGITS!!!!
        
        # self.myo = MyoRaw(None)
        self.mean_emg,self.std_emg = np.loadtxt(str(self.model_stats),delimiter=',')  ## combined_transfer_data_argtest_stats.txt FOR ARGTEST, nina_data/combined_transfer_data_stats.txt ORIGINAL THAT WORKED
        self.emg_array = []
        self.pred_array = []
        self.last_pred = 0
        self.pred_cnt = 0
        self.start_ch = 0
        self.goal_ch = self.first_activity
        self.cal_array = []

        # self.myo.add_emg_handler(self.proc_emg)
        # print("CONNECTING")    
        # self.myo.connect()
        # print("CONNECTED")
        self.connected = 0
        # self.myo.add_emg_handler(self.proc_emg)
        # self.myo.add_imu_handler(self.proc_imu)

        self.gesture_pub = rospy.Publisher("/predicted_gestures", String, queue_size=10)
        self.imu_pub = rospy.Publisher("myo_imu", Imu, queue_size=10)
        self.connected_pub = rospy.Publisher("myo_connected", Int32, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(1.0/20.0), self.timer_callback)   ### TEST DIF RATES FOR TIMER
        self.gest_msg = String()

    def timer_callback(self, timer):
        # self.myo.add_emg_handler(self.proc_emg)
        # print("CONNECTING")    
        # self.myo.connect()
        # print("CONNECTED")
        while self.connected == 0:  
            # self.myo.add_emg_handler(self.proc_emg)
            try:
                print("CONNECTING")   
                myo = MyoRaw(None) 
                myo.connect()
                myo.add_emg_handler(self.proc_emg)
                myo.add_imu_handler(self.proc_imu)
                self.connected =1
                print("CONNECTED")
            except:
                rospy.sleep(0.3)
                pass
    
        if self.connected == 1:
            connect_msg = Int32()
            connect_msg.data = 1
            self.connected_pub.publish(connect_msg)

        try:
            while not rospy.is_shutdown():
                myo.run(1)
                # self.gest_msg.data = "null"
                # self.gesture_pub.publish(self.gest_msg)

        except KeyboardInterrupt:
            print("KILLING NODE")
            print('\n')
            self.myo.disconnect()
            print("Disconnected")
            pass

    def proc_emg(self,emg, moving, times=[]):
        '''Takes sampled emg, shifts the channels per the calibration, and builds a 260ms (52 samples)
        input array that is passed through the trained model to produce the predicted gesture. Prediciton
        is only reported when it differs from the previously reported prediction a sufficent number of times.
        ARGS: emg: list containing sample from each of the 8 channels
              moving and times may not be needed'''
        # since = time.time()
        self.emg_array.append(list((emg - self.mean_emg)/(self.std_emg))) #normalization
        self.emg_array = circle_shift(np.array(self.emg_array),self.start_ch,self.goal_ch).tolist() #calibration
        if len(self.emg_array) == 52:
            input = torch.Tensor(self.emg_array)
            input = input.view(1,1,input.shape[0],input.shape[1])
            pred = torch.argmax(self.model(input)).item() #prediction
            # print(pred)
            self.pred_array.append(pred)
            if len(self.pred_array) == 10:
                try:
                    if mode(self.pred_array) != self.last_pred:
                        self.pred_cnt += 1
                    if self.pred_cnt > 15:
                        # print('Predicted Gesture: {}'.format(mode(self.pred_array)))
                        pred  = self.display_gest(mode(self.pred_array))
                        # gest_msg = String()
                        self.gest_msg.data = pred
                        self.gesture_pub.publish(self.gest_msg)
                        self.last_pred = mode(self.pred_array)
                        self.pred_cnt = 0
                    self.pred_array.clear()
                except:
                    self.pred_array.clear()
            self.emg_array.pop(0)
        # else:

        #     self.gest_msg.data = "null"
        #     self.gesture_pub.publish(self.gest_msg)

    def display_gest(self,pred):
        '''Prints text corresponding to passed prediction (pred)'''
        if pred == 0:
            prediction = "OPEN HAND"
            # rospy.loginfo('\rPredicted Gesture: OPEN HAND     ',end='')
        elif pred == 1:
            prediction = "INDEX"
            # rospy.loginfo('\rPredicted Gesture: INDEX FINGER  ',end='')
        elif pred == 2:
            prediction = "MIDDLE"
            # rospy.loginfo('\rPredicted Gesture: MIDDLE FINGER ',end='')
        elif pred == 3:
            prediction = "RING"
            # rospy.loginfo('\rPredicted Gesture: RING FINGER   ',end='')
        elif pred == 4:
            prediction = "PINKY"
            # rospy.loginfo('\rPredicted Gesture: PINKY FINGER  ',end='')
        elif pred == 5:
            prediction = "THUMB"
            # rospy.loginfo('\rPredicted Gesture: THUMB         ',end='')
        elif pred == 6:
            prediction = "FIST"
            # rospy.loginfo('\rPredicted Gesture: FIST          ',end='')

        rospy.loginfo("Predicted Gesture: %s", prediction)
        return prediction


    def proc_imu(self, quat,acc,gyro):
        
        quats = Quaternion()
        quats.x = quat[0]/16384.0
        quats.y = quat[1]/16384.0
        quats.z = quat[2]/16384.0
        quats.w = quat[3]/16384.0

        gyro_vec = Vector3()
        gyro_vec.x = gyro[0]/16.0
        gyro_vec.y = gyro[1]/16.0
        gyro_vec.z = gyro[2]/16.0

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

        # self.pub = rospy.Publisher("predicted_gestures", String, queue_size=10)
        # cal = input('Begin calibration recording? ')
        # if cal == 'y':
        #     rt.calibrate()

        # start = input('Begin real time prediction? ')
        # if start == 'y':
        #     try:
        #         # while True:
        #         rt.start_pred()
        #         msg.data = self.display_gest
        #         print("GEST",msg.data)
        #         pub.publish(msg)
        #     except KeyboardInterrupt:
        #         print('\n')
        #         self.m.disconnect()
        #         print("Disconnected")




if __name__ == '__main__':
    rospy.init_node('real_time_pred')
    try:
        GestureNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("KILLING NODE")
        pass



