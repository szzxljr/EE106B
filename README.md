# EECS106B Robotic Manipulation and Interaction - Spring 2018 - UC Berkeley
## Labs and Project Code 

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

The website of our project is https://baxter-catching-ball.weebly.com.