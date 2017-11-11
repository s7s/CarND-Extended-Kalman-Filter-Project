**Extended Kalman Filter**
---

**Extended Kalman Filter Project**

The goals of this project are the following:

* Implement the Extended kalman filter algorithm
* Run the implemented algorithm on the simulator
* Summarize the results with a written report



## Project Basics
In this project, I used C++ to write a program taking in radar and lidar data to track position using Extended Kalman Filters.

The code will make a prediction based on the sensor measurement and then update the expected position. See files in the 'src' folder for the primary C++ files making up this project.
---
### The results

#### 1. Running the algorithm on dataset1

| Input |   RMSE   |
| ----- | ------- |
|  px   | 0.0964 |
|  py   | 0.0853 |
|  vx   | 0.4154 |
|  vy   | 0.4316 |

<img src="./writeup_images/EKF.PNG" alt="dataset1_result"  width=700>

#### 2. Running the algorithm on dataset1

| Input |   RMSE   |
| ----- | ------- |
|  px   | 0.0744 |
|  py   | 0.1059 |
|  vx   | 0.5457 |
|  vy   | 0.8256 |

<img src="./writeup_images/EKF2.JPG" alt="dataset2_result" width=700>
