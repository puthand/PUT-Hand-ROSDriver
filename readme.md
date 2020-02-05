# Install
Clone puthand repository to the src folder in your workspace:

    $ cd catkin_ws/src
    $ git clone https://bitbucket.org/romanelo/puthand_driver

Clone puthand communication library to the puthan_driver node:

    $ cd puthand_driver/puthand_driver
    $ git clone https://bitbucket.org/romanelo/liderhand-pc-commprotocol

Compile the sources:

    $ cd ../../
    $ catkin_make
    
# Run PUThand simulation

In the first terminal start the Gazebo simulation:

    $ roslaunch puthand_gazebo puthand.launch
    
In the second terminal run MoveIt:

    $ roslaunch puthand_moveit_config puthand_moveit_planning_execution.launch simulation:=true limited:=true
    
In the third terminal start the PUThand controller:

    $ roslaunch puthand_driver puthand_controller.launch standalone:=true simulation:=true autostart:=false gui:=false
    
In the fourth terminal send the order:

    $ rostopic pub /puthand_controller/order std_msgs/String "movePinch"
    or
    $ rostopic pub /puthand_controller/order std_msgs/String "moveInit"
    or 
    $ rostopic pub /puthand_controller/order std_msgs/String "closeHand"
    or 
    $ rostopic pub /puthand_controller/order std_msgs/String "openHand"
    
# Run driver with the real hand

Add your user to dialout group (do it only once!):

    $ sudo adduser second_user dialout
    
Connect the device to your computer and set serial port baudrate:

    $ stty -F /dev/ttyUSB0 460800
    
Run driver:

    $ roslaunch puthand_driver puthand_driver.launch with_optoforce:=false
    
Then you can use predefined commands:

    $ rostopic pub /puthand_controller/order std_msgs/String "movePinch"
    or
    $ rostopic pub /puthand_controller/order std_msgs/String "moveInit"
    or 
    $ rostopic pub /puthand_controller/order std_msgs/String "closeHand"
    or 
    $ rostopic pub /puthand_controller/order std_msgs/String "openHand"

     

License

Creative Commons License

Unless stated otherwise, PUT-Hand project elements are licensed under a Creative Commons Attribution-NonCommercial 4.0 International (CC BY-NC 4.0). Accompanying firmware and software are licensed under a MIT License.

When using PUT-Hand design files, firmware, software, or utilising project as a whole please cite:

TBD
