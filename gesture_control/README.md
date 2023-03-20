# Gesture Control Package

The `gesture_control` package allows for control of an Adroit robotic arm using EMG and IMU data from a Myo Armband. The EMG signals from the Myo are passed to a gesture classification machine learning model that then outputs a predicted hand gesture. The machine learning model I used was developed by Robert Schloen, so I will link his Github [here](https://github.com/rschloen/semg_control).

A note about that package: While using his package I took it upon myself to make some changes to the ways in which data is passed in to certain files. The overall functionality and model structure do not change, but I found it easier to use certain aspects of the package after implementing these changes. Those changes are not reflected on Robert's Github for the package, so if you want to see or use the package with the changes I made I have the my fork [here](https://github.com/allan-gc/myo_gestures). 

# Launch Files
The package contains the following launch files:

* `hdt_arm_bringup_1.launch`: this launches the necessary Adroit controllers, communication protocols, MoveIt and Rviz. It launches the `hdt_arm_bringup_2.launch` and the `adroit_gesture_control.launch` launch file described below.
   * Use `roslaunch gesture_control hdt_arm_bringup_1.launch simulation:=false controller_type:=myo` to start up the Adroit and establish the Myo armband as the controller. The Adroit must be connected to your computer for the communication protocols to be established. 
      * Set `simulation` to `false` to control the robot through a simulated rviz environment. `controller_type` can be set to `xbox` to control the Adroit with an Xbox controller. 
      
* `adroit_gesture_control.launch`: this launches the `real_time` gesture node and the `adroit_gesture_command` node. It takes a `myo_sim` argument that determines whether to only launch rviz or to also communicate with the real robot. It also passes in the necessary PyTorch files into the `real_time` node This launch file on its own does not control the real Adroit arm. It must either be included in the Adroit control launch files or launched after the Adroit control launch files have been launched. 
    * Use `adroit_gesture_control.launch myo_sim:=true` to control the Adroit in simulation with predicted gestures and IMU measurments from the Myo. 
    * The launch file loads in two parameters: `pytorch_model_stats` parameter loads in the mean and standard deviation of the model to the `real_time` node, and `pytorch_model` loads in the model file.
     

# Using the Package

This package is intended to work alongisde the other Adroit control packages that I cannot share. This package must be in the same directory as the other Adroit control packages to ensure proper setup. Below I describe the necessary steps for making sure the `gesture_control` package works as intended alongside the other Adroit control packages assuming you have access to them and they are in the same directory. 

1). You will first need the proper set up for connecting to and running the Myo Armband. I suggest looking at this [link](http://www.fernandocosentino.net/pyoconnect/) for getting started. 

2). Once you are able to connect with the Myo, you should be able to run `roslaunch gesture_control hdt_arm_bringup_1.launch simulation:=true controller_type:=myo`. This will start up an Rviz environment with the Adroit robot model loaded in. With the Myo on your forearm you should be able to control the Adroit by moving your arm and making gestures. If you find that the model is not correctly predicting your gestures, I suggest looking at Robert's package and gathering your own data. If you do train your own data, make sure to add the .pt model file and the stats .txt file to the config directory of this package and changing the params accordingly in the `adroit_gesture_control.launch` file.  Doing so is what allows the `real_time` node to use the trained weights for prediction. 

3). If the running the above steps allows you to control the Adroit successfully in simulation, run `roslaunch gesture_control hdt_arm_bringup_1.launch simulation:=false controller_type:=myo` to control the real Adroit. 

Here is a demo of how the IMU control looks in Rviz. The TF frame is that of the Myo's IMU.

https://user-images.githubusercontent.com/103614797/226111274-473e9105-e919-47c1-94e6-75de98c42efa.mp4




<br>
<br/>

And here is a video of real time prediction with the node:


[REALTIME_NODE.webm](https://user-images.githubusercontent.com/103614797/226111660-7e2f8179-795a-4b84-aff8-ab8f2482efb0.webm)

<br>
<br/>

For more detailed videos involving control of the real Adroit and more information, check out my portfolio post [here](https://allan-gc.github.io/Adroit.html).


