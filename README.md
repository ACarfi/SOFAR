# Data collection for human-human and human-robot handover tasks

This software was developped during a group project, it's aim is to collect and synchronize data coming from different source during a human-human handover task. The equipment includes:

1. motion capture system
2. kinect
3. inertial sensor embedded in smartwatches

## Installation

For what regards packages "mocap_optitrack", "syn" and "wearable_client" is sufficient to copy them in a catkin workespace and compile them using catkin_make. While for "openni_tracker" before you could compile it you have to install openNi library (you can find installation instruction here https://github.com/OpenNI/OpenNI).

All the ROS package were developed for ROS Indigo.

To install the android applications you have to use Android Studio.
## Usage

<b>On the smartwatches</b>:<br>
execute the "prova" application and pres the button "start".<br>
<b>On the smartphone pc</b>:<br>
execute the "prova" application, if the smarthpone is properly reciving data from the smartwatches it apears "sending..."<br>
<b>On the mocap pc</b>:<br>
  open the streaming menu of motive set the streaming IP with the IP of your computer and tick "Brodcast Frame Data"<br> 
<b>On your pc</b>:<br>
  execute "openni_tracker", you can check if the kinect is working properly executin in another terminal "rviz"<br>
  execute "mocap_node", if inside the mocap area there are some markers it will display the number of markers<br>
  execute "wearable_client_node", when the client is reciving correctly the data in the terminal il written "starting" <br>
  finally execute "sync_node", it immediately asks for the experiment name once you pres enter it starts recording and it stops   when you terminate the node with "ctrl + c".<br><br>
All the ROS node was developped and used under ROS Indigo let's see some additional information on this nodes:<br><br>
<b>openni_tracker</b><br>
Was taken from the openNI library and it uses the "tf" package in order to make available the transformations of some body link frames (official wiki:http://wiki.ros.org/openni_tracker)<br><br>
<b>mocap_node</b><br>



## Credits

TODO: Write credits (Authors, libraries used, etc.)

## License

TODO: Write license (GPL, CC, MIT)

## History (optional)

TODO: Write version history

