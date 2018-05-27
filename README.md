# EECS106B Robotic Manipulation and Interaction - Spring 2018 - UC Berkeley


## Lab 1 Trajectory Tracking with Baxter
Implemented closed-loop control with baxter including workspace, joint velocity, and joint torque control. Compared the speed and accuracy of the trajectory following with each.

## Lab 2 Grasp Planning with Baxter and Sawyer
Planned and executed a grasp with Baxter consisting of detecting the object to grasp, planning a stable grasp, moving into position, closing the grippers, and lifting.

## Lab 3 Constrained Control with Turtlebots
Developed controllers using motion contstraint to control a robot subject to constraints such as
1. Onstacle Avoidance
2. Non-holonomic dynamics (e.g. cars)

## Lab 4 Soft Robotics
Used the soft robotics circuit and finger to 
1. Calibrate the ﬂex sensor and pressure sensor.
2. Create two separate PID controllers for the finger angle with the following sensors flex sensor that measures bending and pressure sensor as feedback.
3. Compare the two controllers in terms of performance and usefulness.

## Final Project

### Research background
Catching a thrown ball is not easy neither for humans nor for robots, which demands highly for a tight interaction of skills in mechanics, control, planning and visual sensing to reach the necessary precision in space and time. In general, a stereo vision system tracks the ball and predicts the balls trajectory, then the point and time, where and in which orientation the robot should intercept the ball on its trajectory, is determined. Then, the robot conﬁguration to reach the catch point is computed and finally a path is generated, which brings the robot from its start conguration to the desired catch conguration.

### Project overview

We build a robotic ball-catching system from Rethink Robotics Baxter Robot arm. The project is decomposed as follows:
​
1. Design a stereo vision system to track the ball and predict the balls trajectory. 
2. Determine the point, time and orientation that the robot should intercept the ball on its trajectory. 
3. Compute the robot conﬁguration to reach the catch point is and generate a path, which brings the robot from its start conﬁguration to the desired catch conﬁguration. 
4. Write a fast controller to move the robot.​

### Our contributions

We use a humanoid robot Baxter with a basket (only the arm is moving), a usb webcam and a standard PC with Ubuntu. In our work, we not only implement the physical model to predict the ball trajectory but also using machine learning model. Then, we compare the results of this two models and analyze their performances. It turns out that the machine learning model can get a lower error rate and a better result.

The poster and final report of our project is in the final_project/materials folder. 

The website of our project is https://baxter-catching-ball.weebly.com.

Labs and Project Codes done by Yichi Zhang, Jingjun Liu and Jiarong Li.



