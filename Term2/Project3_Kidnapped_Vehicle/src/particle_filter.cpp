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
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	default_random_engine gen;
	int num_particles = 1000;
	
	// Creates a normal (gaussian) distribution for x, y, theta
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> angle_theta(theta, std[2]);

	// Resize the vector(weights & particles)
	weights.resize(num_particles);
	particles.resize(num_particles);

	// Initial weight value
	float weight_0 = 1.0 / num_particles;

	// Initialization of weights and particles
	for (int i = 0; i < num_particles; ++i)
	{
		particles[i].id = 1;
		particles[i].x  = dist_x(gen);
		particles[i].y  = dist_y(gen);
		particles[i].theta  = angle_theta(gen);
		particles[i].weight = weight_0; 
	}

	// Initialization done
	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	default_random_engine gen;

	// Create a normal (gaussian) distribution for x, y, theta
	normal_distribution<double> dist_x(0.0, std_pos[0]);
	normal_distribution<double> dist_y(0.0, std_pos[1]);
	normal_distribution<double> angle_theta(0.0, std_pos[2]);

	// Prediction
	for (int i = 0; i < num_particles; ++i)
	{
		if (fabs(yaw_rate) > 0.01) // If yaw rate in not zero
		{
			float x_ = particles[i].x;
			float y_ = particles[i].y;
			float theta_ = particles[i].theta;

			particles[i].x = x_ + (velocity / yaw_rate) * (sin(theta_ + yaw_rate * delta_t) - sin(theta_));
			particles[i].y = y_ + (velocity / yaw_rate) * (cos(theta_) - cos(theta_ + yaw_rate * delta_t));
			particles[i].theta = theta_ + yaw_rate * delta_t;
		}
		else // If yaw rate is close to zero
		{
			float x_ = particles[i].x;
			float y_ = particles[i].y;
			float theta_ = particles[i].theta;

			particles[i].x = x_ + velocity * delta_t * cos(theta_);
			particles[i].y = y_ + velocity * delta_t * sin(theta_);
		}

		// Add Noise
		particles[i].x += dist_x(gen);
		particles[i].y += dist_y(gen);
		particles[i].theta += angle_theta(gen);
	}

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	for (int i = 0; i < observations.size(); ++i)
	{
		float min_dist = 10000000.0;
		float observation_x = observations[i].x;
		float observation_y = observations[i].y;

		for (int j = 0; j < predicted.size(); j++)
		{
			float prediction_x = predicted[j].x;
			float prediction_y = predicted[j].y; 

			float x_2 = (observation_x - prediction_x) * (observation_x - prediction_x);
			float y_2 = (observation_y - prediction_y) * (observation_y - prediction_y);
			
			float dist = sqrt(x_2 + y_2);
			
			// If there is new minimum dist set observation id to predicted id 
			if (dist < min_dist)
			{
				min_dist = dist;
				observations[i].id = predicted[j].id;
			}
		}
	}


}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
	
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	
	default_random_engine gen;
	uniform_real_distribution<float> distribution(0.0, 1.0);
	
	float beta = 0.0;

	// Find max weight
	float w_max = 0;
	for (int i = 0; i < num_particles; ++i)
	{
		if (particles[i].weight > w_max) 
		{
			w_max = particles[i].weight;
		}
	}
	
	int index = int(distribution(gen) * num_particles);

	// Resampling wheel
	for (int i = 0; i < num_particles; ++i)
	{
		beta = beta + distribution(gen) * 2.0 * w_max;
		while (particles[index].weight < beta) 
		{
			beta = beta - particles[index].weight;
			index = (index + 1) % num_particles;
		}
		particles[i] = particles[index];
	}

}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
