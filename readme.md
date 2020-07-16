# Install
Clone puthand repository to the src folder in your workspace:

    $ cd catkin_ws/src
    $ git clone https://github.com/puthand/PUT-Hand-ROSDriver

Clone puthand communication library to the puthan_driver node:

    $ cd puthand_driver/puthand_driver
    $ git clone https://github.com/puthand/PUT-Hand-PC-CommProtocol

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

## License

<a rel="license" href="http://creativecommons.org/licenses/by-nc/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by-nc/4.0/88x31.png" /></a>

Unless stated otherwise, PUT-Hand project elements are licensed under a [Creative Commons Attribution-NonCommercial 4.0 International](https://creativecommons.org/licenses/by-nc/4.0/) (CC BY-NC 4.0). Accompanying firmware and software are licensed under a [MIT License](https://opensource.org/licenses/MIT).

When using PUT-Hand design files, firmware, software, or utilising project as a whole please cite the following article: **PUT-Hand—Hybrid Industrial and Biomimetic Gripper for Elastic Object Manipulation**

```plaintext
Mańkowski, T.; Tomczyński, J.; Walas, K.; Belter, D. PUT-Hand—Hybrid Industrial and Biomimetic Gripper for Elastic Object Manipulation. Electronics 2020, 9, 1147. 

@article{putHandMankowski2020,
   author = {Mańkowski, Tomasz and Tomczyński, Jakub and Walas, Krzysztof and Belter, Dominik},
   title = {PUT-Hand—Hybrid Industrial and Biomimetic Gripper for Elastic Object Manipulation},
   journal = {Electronics},
   volume = {9},
   year = {2020},
   number = {7},
   article-number = {1147},
   url = {https://www.mdpi.com/2079-9292/9/7/1147},
   issn = {2079-9292},
   doi = {10.3390/electronics9071147}
}
```
