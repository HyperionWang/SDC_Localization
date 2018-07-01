/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  // TODO: -- Finished
  // Set the number of particles. Initialize all particles to first position (based on estimates of
  //   x, y, theta and their uncertainties from GPS) and all weights to 1.
  // Add random Gaussian noise to each particle.
  // NOTE: Consult particle_filter.h for more information about this method (and others in this file).
  if (is_initialized == false) {
//    std::cout << "Start to init..." << endl;
    default_random_engine gen;
    double std_x = std[0];
    double std_y = std[1];
    double std_theta = std[2];

    normal_distribution<double> dist_x(x, std_x);
    normal_distribution<double> dist_y(y, std_y);
    normal_distribution<double> dist_theta(theta, std_theta);
    num_particles = 500;
    double sample_x, sample_y, sample_theta;
    for (int i = 0; i < num_particles; i++) {

      sample_x = dist_x(gen);
      sample_y = dist_x(gen);
      sample_theta = dist_theta(gen);
      Particle NewParticle;
      NewParticle.id = i;
      NewParticle.x = sample_x;
      NewParticle.y = sample_y;
      NewParticle.theta = sample_theta;
      NewParticle.weight = 1;
      particles.push_back(NewParticle);
      weights.push_back(1);
    }
    is_initialized = true;
//    std::cout << "End to init..." << endl;
  }

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
  // TODO: -- Finished
  // Add measurements to each particle and add random Gaussian noise.
  // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
  //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
  //  http://www.cplusplus.com/reference/random/default_random_engine/
//  std::cout << "Start to predict..." << endl;
  default_random_engine gen;
  double std_x = std_pos[0];
  double std_y = std_pos[1];
  double std_theta = std_pos[2];


  double predict_x, predict_y, predict_theta;
  double pre_x, pre_y, pre_theta;
  for (int i = 0; i < num_particles; i++) {
    pre_x = particles[i].x;
    pre_y = particles[i].y;
    pre_theta = particles[i].theta;

    if (abs(yaw_rate) > 0.0001) {

      predict_x = pre_x + velocity / yaw_rate * (sin(pre_theta + yaw_rate * delta_t) - sin(pre_theta));
      predict_y = pre_y + velocity / yaw_rate * (cos(pre_theta) - cos(pre_theta + yaw_rate * delta_t));
      predict_theta = pre_theta + yaw_rate * delta_t;
    } else {
      predict_x = pre_x + velocity * delta_t * cos(yaw_rate);
      predict_y = pre_y + velocity * delta_t * sin(yaw_rate);
      predict_theta = pre_theta + yaw_rate * delta_t;
    }

    normal_distribution<double> dist_x(predict_x, std_x);
    normal_distribution<double> dist_y(predict_y, std_y);
    normal_distribution<double> dist_theta(predict_theta, std_theta);

    particles[i].x = dist_x(gen);
    particles[i].y = dist_y(gen);
    particles[i].theta = dist_theta(gen);

  }
//  std::cout << "End to predict..." << endl;
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs> &observations) {
  // No need this function since the dataAssociation is implemented in the updateWeights function
  int i = 0;
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
  // TODO: --Finished
  // Update the weights of each particle using a mult-variate Gaussian distribution. You can read
  //   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
  // NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
  //   according to the MAP'S coordinate system. You will need to transform between the two systems.
  //   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
  //   The following is a good resource for the theory:
  //   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
  //   and the following is a good resource for the actual equation to implement (look at equation
  //   3.33
  //   http://planning.cs.uiuc.edu/node99.html
//  std::cout << "Start to Update..." << endl;
  double x_part, y_part, x_obs, y_obs, x_map, y_map, theta;

  // First to transfer the observation from Vehicle's coordinate to map's coordinate
  for (int particle_index = 0; particle_index < num_particles; particle_index++) {
//    std::cout << "Sweeping Particles @" << particle_index << "out of " << num_particles << endl;
    x_part = particles[particle_index].x;
    y_part = particles[particle_index].y;
    theta = particles[particle_index].theta;
    std::vector<LandmarkObs> Mapobservations;
    std::vector<int> associations;
    std::vector<double> sense_x;
    std::vector<double> sense_y;
    double ParticleWeights = 1;
    for (int i = 0; i < observations.size(); i++) {
//      std::cout << "Sweeping Observations @" << i << "out of " << observations.size() << endl;
      LandmarkObs LMObs;
      x_obs = observations[i].x;
      y_obs = observations[i].y;
      x_map = x_part + (cos(theta) * x_obs) - (sin(theta) * y_obs);
      y_map = y_part + (sin(theta) * x_obs) + (cos(theta) * y_obs);
      double min_dist = sensor_range * 2;
      int LM_Index = 0;
      double near_x = 0;
      double near_y = 0;
      for (int j = 0; j < map_landmarks.landmark_list.size(); j++) {
//        std::cout << "Sweeping landmarks @" << j << endl;
        double LM_dist;
        LM_dist = dist(x_map, y_map, map_landmarks.landmark_list[j].x_f, map_landmarks.landmark_list[j].y_f);
        if (LM_dist <= min_dist) {
          LM_Index = map_landmarks.landmark_list[j].id_i;
          near_x = map_landmarks.landmark_list[j].x_f;
          near_y = map_landmarks.landmark_list[j].y_f;
          min_dist = LM_dist;
        }
      }
//      std::cout << "End of landmark sweeping" << endl;
      LMObs.x = x_map;
      sense_x.push_back(x_map);
      LMObs.y = y_map;
      sense_y.push_back(y_map);
      LMObs.id = LM_Index;
      associations.push_back(LM_Index);
      Mapobservations.push_back(LMObs);

      ParticleWeights *= WeightCal(x_map, y_map, near_x, near_y, std_landmark[0], std_landmark[1]);
    }
//    std::cout << "End of observation sweeping" << endl;
    particles[particle_index].weight = ParticleWeights;
    weights[particle_index] = ParticleWeights;
//    std::cout << "Next particle @" << particle_index << "out of " << num_particles << endl;
//    std::cout <<"particle address is " << &particles[particle_index] <<endl;
//    std::cout <<"Associations address is " << &associations <<endl;
//    std::cout <<"Sense x address is " << &sense_x <<endl;
//    std::cout <<"Sense y address is " << &sense_y <<endl;

    ParticleFilter::SetAssociations(particles[particle_index], associations, sense_x, sense_y);
//    std::cout << "Next particle @" << particle_index << "out of " << num_particles << endl;
  }
//  std::cout << "End of Particle sweeping" << endl;
//  std::cout << "End to Update..." << endl;
}

void ParticleFilter::resample() {
  // TODO: -- Finished
  // Resample particles with replacement with probability proportional to their weight.
  // NOTE: You may find std::discrete_distribution helpful here.
  //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
//  std::cout << "Start to Resample..." << endl;
  std::random_device rd;
  std::mt19937 gen(rd());
  std::discrete_distribution<> d(weights.begin(), weights.end());
  std::vector<double> new_weights;
  std::vector<Particle> new_particles;
  for (int i = 0; i < num_particles; ++i) {
    Particle temp_particle = particles[d(gen)];
    new_particles.push_back(temp_particle);
    new_weights.push_back(temp_particle.weight);
  }
  particles = new_particles;
  weights = new_weights;
//  std::cout << "End to Resample..." << endl;
}


Particle ParticleFilter::SetAssociations(Particle &particle, const std::vector<int> &associations,
                                         const std::vector<double> &sense_x, const std::vector<double> &sense_y) {
  //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
//  std::cout << "Start to assign association" << endl;
  particle.associations = associations;
//  std::cout << "Start to assign sense x" << endl;
  particle.sense_x = sense_x;
//  std::cout << "Start to assign sense y" << endl;
  particle.sense_y = sense_y;
//  std::cout << "End of SetAssociations" << endl;
  return particle;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  stringstream ss;
  copy(v.begin(), v.end(), ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseX(Particle best) {
  vector<double> v = best.sense_x;
  stringstream ss;
  copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseY(Particle best) {
  vector<double> v = best.sense_y;
  stringstream ss;
  copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}
