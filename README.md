# BWO
Code for my robot, BWO.

## Hardware
- The body is a [DFRobot HCR](https://www.dfrobot.com/product-63.html).
- The drive motor controller is a [DFRobot 2×15A DC Motor Driver](https://www.dfrobot.com/product-796.html).
- The drive motor controller and encoders, as well as the front bumpers, are connected to a
  [Seeeduino XIAO](https://www.seeedstudio.com/Seeeduino-XIAO-Arduino-Microcontroller-SAMD21-Cortex-M0+-p-4426.html)
  microcontroller, which controls the motor velocities, and prevents forward motion if a bumper is triggered.
  The code running on this microcontroller is in `src/arduino/motor_control/motor_control.ino`.
- The main computer is an on-board [Nvidia Jetson Nano](https://developer.nvidia.com/embedded/jetson-nano-developer-kit).
- The camera is an [Intel RealSense Depth Camera D435i](https://www.intelrealsense.com/depth-camera-d435i/).

## Software
- Ubuntu 18.04 Bionic Beaver
- [ROS 2 Dashing Diademata](https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Debians/) --
  This is (at the time of this writing) the latest LTS version of ROS 2 that is available for Ubuntu 18.04.
- Python 3.6 (unfortunately Nvidia's DNN libraries do not yet support newer versions)
