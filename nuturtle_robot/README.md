# NUTURTLE_ROBOT package
## Package use to interacts with the turtlebot hardware

Launch file:
* basic_remote.laucnch: Run roslaunch nuturtle_robot basic_remote.launch robot:=x. If x=1, turtlebot will run on the remote PC directly(localhost).

* odom_teleop.launch: Run roslaunch nurturle_robot odom_teleop.launch circle:=true/false. Argument circle default is set to true. When circle is true, it will launch follow_circle node; when circle is false, it will launch turtlebot3_teleop_key node. This launch file also launches odmeter and turtleinterface node remotely on turtlebot3 and launches rviz on the pc. 

Service:
* control.srv: After launching the odom_teleop.launch file, run rosservice call /control direction:=x(x=0: Clockwise x=1: Counter-clockewise x=3:stop the motion) to let the turtlebot to move. 


Task F.008(Physical Testing):
* Experiment1: Drive the robot forward and backward in a straight line several times, and then stopping when the turtlebot is in its initial configuration.
  
  x3 Speed Demo:
  
  ![1](https://user-images.githubusercontent.com/70287453/107980841-928dcf00-6f86-11eb-8a3e-b2804e49e68c.gif)
  
  Position in odom:
  x: -0.172933
  y: 0.055919
  z: 0.0

* Experiment2: Rotate the robot clockwise and counter clockwise several times, stopping when the turtlebot is in its initial configuration.
  
  x3 Speed Demo:
  
  ![2](https://user-images.githubusercontent.com/70287453/107981136-25c70480-6f87-11eb-9226-5defdeaddb3a.gif)

  Position in odom:
  x: -0.00014527
  y: -0.00014829
  z: 0.0

* Experiment3: Drive the robot in a circle, clockwise and counter clockwise several times, stopping when the turtlebot is its initial configuration.
  
  x4 Speed demo:
  
  ![3](https://user-images.githubusercontent.com/70287453/107981762-578c9b00-6f88-11eb-9486-e954e4d33547.gif)
  
  Position in odom:
  x: 0.1418113
  y: -0.206308
  z: 0.0

* Experiment4: Try one of the previous experiments again, but try to get either a significantly better or significantly worse result. (Repeat Experiment 3)
  
  x4 Speed demo:
  
  ![4](https://user-images.githubusercontent.com/70287453/107982128-1d6fc900-6f89-11eb-8247-7b2a4db226cb.gif)
 
  Position in odom:
  x: -0.0655977
  y: -0.0330638
  z: 0.0
  
  Reflection: In this repeat experiment3, my driving is more consistent and stable compared to the previous test so the odom gives a closer result to the origin. I did not change the linear.x and angular.z value of twist message but I pressed the key faster than the previous test which certainly reduces odom position errors. 
