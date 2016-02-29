# Data collection for human-human and human-robot handover tasks

This software was developed during a group project, it's aim is to collect and synchronize data coming from different source during a human-human handover task. The equipment includes:

1. Motion Capture system
2. Microsoft Kinect
3. inertial sensor embedded in smartwatches

## Installation

For what regards packages "mocap_optitrack", "syn" and "wearable_client" is sufficient to copy them in a catkin workspace and compile them using catkin_make. While for "openni_tracker" before you could compile it you have to install openNi library (you can find installation instruction here https://github.com/OpenNI/OpenNI).

All the ROS package were developed for ROS Indigo.

To install the android applications you have to use Android Studio.
## Usage
Here follows instructions to use the code <br><br>
<b>On the smartwatches</b>:<br>
execute the "prova" application and pres the button "start", this application cames from https://github.com/bbbruno/WearAmI and was developed by Andrea Galea.<br>
<b>On the smartphone</b>:<br>
execute the "prova" application, if the smarthpone is properly receiving data from the smartwatches it appears "sending..."<br>
<b>On the mocap pc</b>:<br>
  open the streaming menu of motive set the streaming IP with the IP of your computer and tick "Brodcast Frame Data"<br> 
<b>On your pc</b>:<br>
  execute "openni_tracker", you can check if the kinect is working properly executing in another terminal "rviz"<br>
  execute "mocap_node", if inside the mocap area there are some markers it will display the number of markers<br>
  execute "wearable_client_node", when the client is receiving correctly the data in the terminal is written "starting" <br>
  finally execute "sync_node", it immediately asks for the experiment name once you press enter it starts recording and it stops   when you terminate the node with "ctrl + c".<br><br>
All the ROS nodes were developped and used under ROS Indigo. Let's see some additional information on these nodes:<br><br>
<b>openni_tracker</b><br>
It was taken from the openNI library and it uses the "tf" package in order to make the transformations of some body link frames available (official wiki:http://wiki.ros.org/openni_tracker)<br><br>
<b>mocap_node</b><br><br>
It comes from the "mocap_optitrack" package (https://github.com/ros-drivers/mocap_optitrack) but was modified. The original one in fact was able to publish only the data coming from rigid body, this version publishes position for all the markers. The markers coordinates are pushed in an array and published in the topic "/markers_coo".<br><br>
<b>wearable_client_node</b><br><br>
This node implements a client that reads from a socket the angular velocities and linear acceleration arriving from two smartwatches through the smartphone, this data are divided depending on the source and published on two different topics "wearS_1" and "wearS_2" using UMI messages.<br><br>
<b>sync_node</b><br><br>
It subscribes to the topics of "wearable_client_node" and "mocap_node" and it uses a listener to read kinect data. Once we have all the data source available it saves in a bag file all the synchronized data. The name of the bag file is asked to the user before starting the record.



## Credits

Author: Carfì Alessandro<br>
Library: OpenNI (openni_tracker), NatNet(mocap_optitrack)
## License

GPL



