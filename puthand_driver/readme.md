
rosrun puthand_driver create_udev_rules

sudo stty -F /dev/ttyUSB0 460800

roslaunch puthand_driver puthand_controller.launch standalone:=false simulation:=false autostart:=false gui:=false serial_device:=/dev/ttyUSB0

Simple joint gui controller:
roslaunch puthand_driver puthand_controller.launch standalone:=false simulation:=false autostart:=false gui:=true serial_device:=/dev/ttyUSB0

Moveit:
roslaunch puthand_moveit_config puthand_moveit_planning_execution.launch simulation:=false limited:=true

Predefined configurations:
rostopic pub /puthand_controller/order std_msgs/String "moveInit"
rostopic pub /puthand_controller/order std_msgs/String "movePinch"