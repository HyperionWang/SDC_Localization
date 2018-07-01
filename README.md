# Self-Driving Car Project: Localization through Partical Filter

[//]: # (Image References)
[image1]: ./Doc/ParticleFilterFlowchart.png
[image2]: ./Doc/TestPass.png
[image3]: ./Doc/ParticleFilter_Result.gif

## Overview
This project is the design and implementation of the particle filter for localizing the car's position with known map info using the car's state prediction based on the motion model and Bayesian Model. 

The following figure is the work flow of the particle filter for localization:

![MPC Control Loop][image1]

#### Initialization
After deciding the number of the initial particles (500 used in the project), the particle filter initialization would generate the positions of all particles following the Gaussion distribution based on the initial position from GPS and uncertainties and set each particle's weigh being 1. 

#### Prediction
If the particle filter has been initialized (having the information about velocity and yaw rate from previous state), the particle filter would predict the position of the each particle based on the info of current state as well as velocity and yaw rate. 

#### Update Weights of particles
After the prediction, the system would request the new measurement (observation) on the next state, in order to compare with the prediction of each particle, and update the weigh of the particle based on the difference between the prediction and the nearest landmark observations.


#### Resample

Once the weight of each particle has been updated, the particles would be resampled based on their weights.

#### Next Cycle of Particle Filter
After resampling, the system will go into the next cycle of the particle filter work flow. As the cycle goes, the particle with less weights would be filtered out, and the particle left are more and more condense to the car's actual location.


##Particle Filter Model

The number of the particles in the filter is set to 500 based on the system running time and coverage (localization position accuracy) requirement. The more particles, the higher accuracy and larger map coverage, but the longer computatino time required. 

The process of particle filter is the process of the Bayesian estimations based on the Bicycle Model prediction and Map Landmark observations.

a. The prediction of each particle's next state is based on the current state (position and yaw) and motion (velocity and yaw rate). 

The formula to predict the next state is as follow:

```c++

    if (abs(yaw_rate) > 0.0001) {

      predict_x = pre_x + velocity / yaw_rate * (sin(pre_theta + yaw_rate * delta_t) - sin(pre_theta));
      predict_y = pre_y + velocity / yaw_rate * (cos(pre_theta) - cos(pre_theta + yaw_rate * delta_t));
      predict_theta = pre_theta + yaw_rate * delta_t;
    } else {
      predict_x = pre_x + velocity * delta_t * cos(yaw_rate);
      predict_y = pre_y + velocity * delta_t * sin(yaw_rate);
      predict_theta = pre_theta + yaw_rate * delta_t;
    }

```

b. The process to update the weigh of each particle is 
1. Get the observations data from the system 

2. For each particle, go through the data association process using the nearest neighborhood method by finding the observation's associated landmark with minimum distance.

3. Calculate the particle's weigh by multiplying the number in reverse of the difference between estimated observation's location and landmark's location (location error). The less the observation estimation error, the larger the weigh of the particle would have.

The code of data association and weigh update could be find from line 125 to line 166 in particle_filter.cpp. 

The formula to calculate the weigh factor for each observation of each particle is as follow:

```c++

ParticleWeights *= WeightCal(x_map, y_map, near_x, near_y, std_landmark[0], std_landmark[1]);


inline double WeightCal(double x1, double y1, double x2, double y2, double std_x, double std_y) {
	double Weight;
	Weight = 1/(2*M_PI*std_x*std_y)*exp(-1*((x1-x2)*(x1-x2)/2/std_x/std_x+(y1-y2)*(y1-y2)/2/std_y/std_y));

	return Weight;

}

```

x_map and y_map are estimated map location of observation for current particle to update the weigh. And near_x and near_y are the map location of the landmark consider to be the associated landmark to the observation.

###Resample

The resample is realized through a standard discrete distribution library in c++. This random number generator would genrate the new samples based on the weigh of the previous particles. The larger the weigh of the particle, the higher chance the particle would be picked into the next round.

## Result and Summary

After tuning the particle numbers and particle update procedures. The implemented particle filter could successfully localize the car's position as well as associate the observations to the right landmark in the map. It demonstrates the particle filter's localization is accurate. Meanwhile, the whole process of localization could be finished within each update delta time, and the whole process could be finished within the required time. And therefore, the designed particle filter passed the test. 

![MPC Control Loop][image2]
![MPC Control Loop][image3]

