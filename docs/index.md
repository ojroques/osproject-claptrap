# Claptrap, a room mapping robot

## Table of contents

1. [Presentation](#presentation)
2. [Links](#links)
3. [Source code and instructions](#instructions)
4. [Claptrap architecture](#architecture)
5. [Algorithms](#algorithms)
6. [Work division](#division)
7. [Videos and Pictures](#videos)

## <a name="presentation"></a>Presentation
_Claptrap_ is a robot based on the LEGO MINDSTORMS EV3, whose purpose is to map a previously unknown room. It has been designed by Nathan Biette, Erwan Culerier and Olivier Roques for the OS fall course 2017 at [EURECOM](http://www.eurecom.fr/en).

## <a name="links"></a>Links

#### General information
* [Project website](http://soc.eurecom.fr/OS/projects_fall2017.html)
* [Project repository](https://gitlab.eurecom.fr/ludovic.apvrille/OS_Robot_Project_Fall2017)
* [Claptrap repository](https://github.com/ojroques/OSproject-Claptrap)
* [Robot set-up](http://www.ev3dev.org/docs/getting-started/)

#### Documentation
* [EV3DEV Hardware Documentation](http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-jessie/index.html)
* [EV3DEV C Library](http://in4lio.github.io/ev3dev-c/index.html)

## <a name="instructions"></a>Source code and instructions

Source code is available on [our repository](https://github.com/ojroques/OSproject-Claptrap). Follow these steps to run our algorithm on your Lego Mindstorms EV3:

1. [Set-up the robot](http://www.ev3dev.org/docs/getting-started/) and connect to it via ssh.
2. Update the robot with the latest packages:
``` Shell
sudo apt-get update && sudo apt-get upgrade
```
3. Install extra packages you will need for the project:
``` Shell
sudo apt-get install gcc make libbluetooth-dev
```
4. Install the development environment on the robot:
``` Shell
GIT_SSL_NO_VERIFY=true git clone https://github.com/in4lio/ev3dev-c.git
cd ~/ev3dev-c/source/ev3
make
sudo make install
```
5. Clone our repository on the robot and compile the source:
``` Shell
cd ~
git clone https://github.com/ojroques/OSproject-Claptrap.git
cd OSproject-Claptrap/src/
make main
```
6. Install the server **on a computer**:
``` Shell
git clone https://gitlab.eurecom.fr/ludovic.apvrille/OS_Robot_Project_Fall2017.git
cd OS_Robot_Project_Fall2017/server
make server
```
7. On the robot, change `SERV_ADDR` in `src/client.h` with your server bluetooth adress.
8. On your server, create a file with 14 lines. Each line of this file must correspond to a robot following this format:
```
1 BLUETOOTH_ADDR ROBOT_NAME
```
You can copy-paste this example in a file called `claptrap` to use only Claptrap as robot (the team ID of Claptrap is 7, you can change it in `config.h`):
```
1 00:00:00:00:00:00 Team01
1 00:00:00:00:00:00 Team02
1 00:00:00:00:00:00 Team03
1 00:00:00:00:00:00 Team04
1 00:00:00:00:00:00 Team05
1 00:00:00:00:00:00 Team06
1 a0:e6:f8:dc:96:79 Claptrap  <-- Use your own bluetooth address here
1 00:00:00:00:00:00 Team07
1 00:00:00:00:00:00 Team08
1 00:00:00:00:00:00 Team09
1 00:00:00:00:00:00 Team10
1 00:00:00:00:00:00 Team11
1 00:00:00:00:00:00 Team12
1 00:00:00:00:00:00 Team13
```
9. Launch the server with the previously created file:
``` Shell
./server claptrap
```
10. You can now run the program on your robot:
``` Shell
cd OSproject-Claptrap/src/
make run
```

## <a name="architecture"></a>Claptrap architecture

#### Sensors
Claptrap uses 3 sensors:
* [Ultrasonic Sensor](https://shop.lego.com/en-CA/EV3-Ultrasonic-Sensor-45504)
* [Color Sensor](https://shop.lego.com/en-CA/EV3-Color-Sensor-45506)
* [Gyro Sensor](https://shop.lego.com/en-CA/EV3-Color-Sensor-45506)

#### Tachos
Claptrap uses 4 tachos:
* Two [large tachos](https://shop.lego.com/en-CA/EV3-Large-Servo-Motor-45502) for both wheels
* One large tacho in its back for the obstacle carrier
* One [medium tacho](https://shop.lego.com/en-CA/EV3-Medium-Servo-Motor-45503) to control the rotation of the ultrasonic sensor

## <a name="algorithms"></a>Algorithms

Coming soon

## <a name="division"></a>Work division

#### Olivier Roques
* Main algorithm: procedures for room exploration and mapping
* Client / Server communications
* Map and image
* Website
* Integration

#### Nathan Biette
* Robot architecture: building the LEGO robot
* Sensors: tests, initialization, getters
* Robot movements: release of an obstacle, controlling the ultrasonic tacho
* Robot configuration in `config.c`

#### Erwan Culerier
* Robot movements: rotation, translation
* Positioning: calculation, thread
* Robot configuration in `config.c`

## <a name="videos"></a>Videos and Pictures

Coming soon
