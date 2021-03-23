# NUSLAM

## EKF for known data association
### Demo (X3 speed)
![ezgif com-gif-maker (7)](https://user-images.githubusercontent.com/70287453/111915711-a1641780-8a45-11eb-8195-0a614c478679.gif)

#### Paths:

Red path: EKF slam path

Green path: Odom path

Blue path: Real path from nurtlesim simulator

#### Circles:

Blue: Tubes in simulator 

Red: Sensed tube in range

Green: Estimated locations of tubes in range 

#### Error
![Screenshot from 2021-03-22 03-10-01](https://user-images.githubusercontent.com/70287453/111959198-2558e680-8abc-11eb-98a3-f813facea48c.png)


#### Usage
Run

      roslaunch nuslam slam.launch robot:=1(localhost)

## Landmark Detection

Simulation:

![ezgif com-gif-maker (5)](https://user-images.githubusercontent.com/70287453/111893611-fb2afa00-89d1-11eb-964a-2e410d8270f7.gif)

Turtlebot3:

![Screenshot from 2021-03-20 20-23-26](https://user-images.githubusercontent.com/70287453/111894383-55c75480-89d8-11eb-847b-00391daf88ad.png)

#### Usage
Run
      
      Simulation:
      
      roslaunch nuslam landmark_detect.launch simulate:=true
      
      Turtlelbot3:
      
      roslaunch nuslam landmark_detect.launch simulate:=false 
      
#### Note: If running on turtlebot, user need to change header.frame_id from "turtle" to "base_scan" in landmarks(Line 85) and display node.(Line 59)

## EKF for unknown data association
### Demo (X3 speed)
![ezgif com-gif-maker (10)](https://user-images.githubusercontent.com/70287453/111963676-c8602f00-8ac1-11eb-87e3-ce65009a0f91.gif)

#### Paths:

Red path: EKF slam path

Green path: Odom path

Blue path: Real path from nurtlesim simulator

#### Circles:

Blue: Tubes in simulator 

Red: Fake sensed tube in range

Green: Sensed tube from laserscan

Yellow: Estimated locations of tubes in range 

#### Error
![Screenshot from 2021-03-22 03-48-11](https://user-images.githubusercontent.com/70287453/111963671-c5653e80-8ac1-11eb-96bc-45a320799c3c.png)


#### Usage
Run

      roslaunch nuslam unknown_data_assoc.launch robot:=1(localhost)
      
## EKF SLAM IN REAL WORLD
#### Demo
![ezgif com-gif-maker (11)](https://user-images.githubusercontent.com/70287453/112034580-a17b1a80-8b0c-11eb-8bb2-c97970b239b8.gif)

![real_slam](https://user-images.githubusercontent.com/70287453/112031962-e5205500-8b09-11eb-8aa8-795c64451696.png)

#### Paths:

Red path: EKF slam path

Green path: Odom path
#### Circles:
Green: Sensed tube from laserscan

Yellow: Estimated locations of tubes in range 
#### Error

![Screenshot from 2021-03-22 12-34-55](https://user-images.githubusercontent.com/70287453/112033292-45fc5d00-8b0b-11eb-9aa6-6cc7d1691d68.png)

#### Usage
Run

      roslaunch nuslam unknown_data_assoc.launch robot:=0(Turtlebot3)
