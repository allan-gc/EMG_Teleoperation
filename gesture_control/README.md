# Gesture Control Package

The `gesture_control` package allows for control of an Adroit robotic arm using EMG and IMU data from a Myo Armband. The EMG signals from the Myo are passed to a gesture classification machine learning model that then outputs a predicted hand gesture. The machine learning model I used was developed by Robert Schloen, so I will link his Github [here](https://github.com/rschloen/semg_control).

A note about that package: While using his package I took it upon myself to make some changes to the ways in which data is passed in to certain files. The overall functionality and model structure do not change, but I found it easier to use certain aspects of the package after implementing these changes. Those changes are not reflected on Robert's Github for the package, so if you want to see or use the package with the changes I made I have the my fork [here](https://github.com/allan-gc/myo_gestures). 

# Launch Files
The package contains the following launch files:

* `adroit_gesture_control.launch`: this launches the `real_time` gesture node and the `adroit_gesture_command` node. It takes a `myo_sim` argument that determines whether to only launch rviz or to also communicate with the real robot. This launch file on its own does not control the real Adroit arm. 
    * Use `adroit_gesture_control.launch myo_sim:=true` to control the Adroit in simulation with predicted gestures and IMU measurments from the Myo. 

# Using the Package

This package is intended to work alongisde the other Adroit control packages that I cannot share. However, if you do have access to those files and are interested in running this package with it, you will need the proper set up for connecting to and running the Myo Armband. I suggest looking this [link](http://www.fernandocosentino.net/pyoconnect/) for getting started. Once you are able to connect with the Myo, you should be able to run the launch file above with `myo_sim` set to  true. With the Myo on your forearm you should be able to control the Adroit by moving your arm and making gestures.

Here is a demo of how the control looks in Rviz: 
