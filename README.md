# Robocon-2019
Design documents and codes related to robots which participated at Robocon 2019 representing IIT Delhi: https://drive.google.com/drive/folders/1nrrfucBYuTk6GnWyP7ws8S1Cq5NcwrkV?usp=sharing

# Quadruped-Robot-MR2
It’s a 4 legged autonomous robot.
It autonomously traces a defined path encountering terrains such as an inclined plane,  horizontal plane and even steps. 
Mechanical Design and Fabrication- Each of the 4 leg is based on the 5R-5bar mechanism. The degree of freedom of robot is 8(2 in each leg)
The legs are made of carbon fiber tubes and are mounted on an aluminium chassis. This makes the robot very light as well as robust.
Electrical Design- 8 Dynamixel MX-64T(2 for each leg) are used as actuators for the robot. The robot is controlled using a RaspberryPi. There is a wireless communication between RaspberryPi(Model 3B+) and computer over Wi-fi.
The mathematical modelling of the toe trajectory of the robot was done in MatLab and programming was done in Python using the Matlab data.
This mechanism has got huge upside potential with an opportunity for numerous applications in different areas. The robot can be used to scan an unknown area with unknown terrain with the help of various sensors like laser scan, computer vision etc .Various other mechanisms like gripping can be installed for specific purposes.

# 4-wheeled-mecanum-holonomic-drive
It is a 4 wheeled Holonomic drive capable of omni-directional movement. The drive is semi-autonomous and can be controlled via a PS4 controller, or programmed to run autonomously. The sensors include advanced line following sensor LSA08, Ultrasonic Distance Sensors HC-SR04 and optical encoders for linear distance feedback. 4 Banebot RS770 DC motors are used to drive the four mecanum holonomic wheels and sabertooth motor driver are used for the control. Arduino Mega ADK is the micro controller used.
The drive is attached with a shagai (a curvi linear large object) throwing mechanism. A ramp descends from the bot, picks up the "Shagai" and is capable of throwing it up to 2 meters.
