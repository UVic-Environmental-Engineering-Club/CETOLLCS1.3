# Sea Puppy Low Level Control System V1.3
### Made by UVic Environmental Engineering Club ([UVEEC](https://uveec.ca/)) at [University of Victoria](https://www.uvic.ca/)
[![stmicroelectronics](https://img.shields.io/badge/stmicroelectronics-03234B.svg?&logo=stmicroelectronics)](https://www.st.com/content/st_com/en.html)
[![c](https://img.shields.io/badge/c-A8B9CC.svg?&logo=c)](https://en.cppreference.com/w/c/language)
[![cplusplus](https://img.shields.io/badge/c++-00599C.svg?&logo=cplusplus)](https://www.st.com/content/st_com/en.html)
[![ros](https://img.shields.io/badge/ros-22314E.svg?&logo=ros)](https://docs.ros.org/en/humble/index.html)

## Description
This repo is part of Sea Glider project (SeaPuppy) by UVEEC. Sea glider is an unmanned underwater vehicle (UUV) for ocean research. Compare to conventional submarine, glider is more efficient because it utilizes buoyance and gavity as driving power. 

This is a low level control system for the sea glider written in C/C++. This repo is responsible for STM32 on UVEEC's PCB "Megamind". "Megamind" is responsible for reading sensors and actuating engines on the sea glider. Full documentation about Low Level Control System can be found [here](https://docs.google.com/document/d/1BvOxKdqG76WM1FazrVoIZYhodaNmmSBSupSmKFFQtNc/edit?tab=t.0) (only available for UVEEC member).

CUBE IDE development environment was used on Windows 10. MicroROS for ROS2 Humble has been deployed. MicroROS is to be used to communication with ROS2 Humble on Raspberry Pi with Blue OS from Blue Robotics. If you need access to this documentation, please create [issue]((https://github.com/UVic-Environmental-Engineering-Club/SeaPuppy1.3/issues)) on this project.

## Getting Started (If you are UVEEC member)
1. Go to uveec club room and log into the desktop.
2. Open STM32 Cube IDE.
3. Select SeaPuppy1.3 on Project Explorer.
4. main.c can be found at SeaPuppy1.3 -> Core -> Src -> main.c
5. Enjoy!

## Getting Started (If you are not UVEEC member)
1. Install **V1.18.0** [STM32 CUBE IDE](https://www.st.com/en/development-tools/stm32cubeide.html) on Windows 10. Remember your workspace.
2. Git clone this repository in the workspace you have defined.
3. Open CubeIDE. File -> Import -> General -> Existing Projects into Workspace -> Select the clone of this project. You do not have to change anything on ioc.
4. Connect your STM32 with ST-Link.
5. Connect your ST-Link to your Windows machine.
6. Build. (CubeIDE might prompt to install ST-Link-Server. Follow the pop-up's instruction)
7. Run.
8. You can not unplug your STM32. (If there was any issue before this process, please use [git issues](https://github.com/UVic-Environmental-Engineering-Club/SeaPuppy1.3/issues) to report)
9. Plug in your STM32 to Raspberry Pi with NAvigator Flight Controller.
10. You are done with this part. Follow instruction on Raspberry Pi documentation. (To be developed and documented)

## Relevent documentations
* [micro_ros_stm32cubemx_utils](https://github.com/micro-ROS/micro_ros_stm32cubemx_utils)
* [First micro-ROS Application on Linux](https://micro.ros.org/docs/tutorials/core/first_application_linux/)
* [ROS2 humble](https://docs.ros.org/en/humble/index.html)

