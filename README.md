# BWO
Code for my robot, BWO.

## Hardware
- Chassis: [DFRobot HCR](https://www.dfrobot.com/product-63.html)
- Computer: [Nvidia Jetson Nano](https://developer.nvidia.com/embedded/jetson-nano-developer-kit)
- Camera: [Intel RealSense Depth Camera D435i](https://www.intelrealsense.com/depth-camera-d435i/)
- Drive motor controller: [DFRobot 2×15A DC Motor Driver](https://www.dfrobot.com/product-796.html)
- The drive motor controller and encoders, as well as the front bumpers, are connected to a
  [Seeeduino XIAO](https://www.seeedstudio.com/Seeeduino-XIAO-Arduino-Microcontroller-SAMD21-Cortex-M0+-p-4426.html)
  microcontroller, which controls the motor velocities, and prevents forward motion if a bumper is triggered.
  The code running on this microcontroller is in `src/arduino/motor_control/motor_control.ino`.
- Power:
  - One [7.4V 5200mAh LiPo](https://www.amazon.com/gp/product/B06ZYRCPS3)
    for computer and peripheral power.
  - One [11.1V 5000mAh LiPo](https://www.amazon.com/gp/product/B06XNTHQRZ)
    for drive motor and servo motor power.

## Software
- Ubuntu 18.04 Bionic Beaver
- [ROS 2 Foxy Fitzroy](https://docs.ros.org/en/foxy/)
- Python 3.6 (unfortunately Nvidia's DNN libraries do not yet support newer versions)
