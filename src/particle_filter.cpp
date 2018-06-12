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

#define EPS 0.00001

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
if (!is_initialized) {
    /* Setting up a number of particles */
    num_particles = 100;

    /* Standard deviations */
    double std_x, std_y, std_theta;
    std_x = std[0];
    std_y = std[1];
    std_theta = std[2];

    /* Making normal distributions */
    normal_distribution<double> n_dist_x(x, std_x);
    normal_distribution<double> n_dist_y(y, std_y);
    normal_distribution<double> n_dist_theta(theta, std_theta);

    /* Generate a normally distributed particle cloud with mean on GPS values */
    for (int i = 0; i < num_particles; i++) {
        Particle particle;
        particle.id = i;
        particle.x = n_dist_x(gen);
        particle.y = n_dist_y(gen);
        particle.theta = n_dist_theta(gen);
        particle.weight = 1.0;

        particles.push_back(particle);
	}
    
    /* Setting up an initialization flag */
    is_initialized = true;
} else {
    return;
}

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
/* Standard deviations */
double std_x, std_y, std_theta;
std_x = std_pos[0];
std_y = std_pos[1];
std_theta = std_pos[2];

/* Making normal distributions */
normal_distribution<double> n_dist_x(0, std_x);
normal_distribution<double> n_dist_y(0, std_y);
normal_distribution<double> n_dist_theta(0, std_theta);

/* Predict a new state */
for (int i = 0; i < num_particles; i++){
    double theta = particles[i].theta;

    if (fabs(yaw_rate) > EPS) {
        particles[i].x += velocity / yaw_rate * ( sin( theta + yaw_rate * delta_t ) - sin( theta ) );
        particles[i].y += velocity / yaw_rate * ( cos( theta ) - cos( theta + yaw_rate * delta_t ) );
        particles[i].theta += yaw_rate * delta_t;      
    } else {
        particles[i].x += velocity * delta_t * cos( theta );
        particles[i].y += velocity * delta_t * sin( theta );      
    }

    /* Adding up a noise */
    particles[i].x += n_dist_x(gen);
    particles[i].y += n_dist_y(gen);
    particles[i].theta += n_dist_theta(gen);
  }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
int nObservations, nPredictions;
double minDistance, xDistance, yDistance, distance;

nObservations = observations.size();
nPredictions = predicted.size();

for (int i = 0; i < nObservations; i++){
    /* Initialize a min distance */
    minDistance = numeric_limits<double>::max();

    int mapId = -1;

    for (int j = 0; j < nPredictions; j++ ){
        xDistance = observations[i].x - predicted[j].x;
        yDistance = observations[i].y - predicted[j].y;
        distance = xDistance * xDistance + yDistance * yDistance;
        if (distance < minDistance){
            minDistance = distance;
            mapId = predicted[j].id;
        }
    }
    observations[i].id = mapId;
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
double stdLandmarkRange, stdLandmarkBearing;
double x, y, theta;
double sensor_range_2;
float landmarkX, landmarkY;
int id;
double dX, dY;

stdLandmarkRange = std_landmark[0];
stdLandmarkBearing = std_landmark[1];

for(int i = 0; i < num_particles; i++){
    x = particles[i].x;
    y = particles[i].y;
    theta = particles[i].theta;
    /* Find landmarks */
    sensor_range_2 = sensor_range * sensor_range;
    vector<LandmarkObs> inRangeLandmarks;
    for(unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++) {
        landmarkX = map_landmarks.landmark_list[j].x_f;
        landmarkY = map_landmarks.landmark_list[j].y_f;
        id = map_landmarks.landmark_list[j].id_i;
        dX = x - landmarkX;
        dY = y - landmarkY;
        if(dX*dX + dY*dY <= sensor_range_2){
            inRangeLandmarks.push_back(LandmarkObs{id, landmarkX, landmarkY});
        }
    }

    // Transform observation coordinates.
    vector<LandmarkObs> mappedObservations;
    for(unsigned int j = 0; j < observations.size(); j++) {
      double xx = cos(theta)*observations[j].x - sin(theta)*observations[j].y + x;
      double yy = sin(theta)*observations[j].x + cos(theta)*observations[j].y + y;
      mappedObservations.push_back(LandmarkObs{ observations[j].id, xx, yy });
    }

    // Observation association to landmark.
    dataAssociation(inRangeLandmarks, mappedObservations);

    /* Reset weights */
    particles[i].weight = 1.0;
    /* Calculate weights */
    for(unsigned int j = 0; j < mappedObservations.size(); j++){
        double observationX = mappedObservations[j].x;
        double observationY = mappedObservations[j].y;

        int landmarkId = mappedObservations[j].id;

        double landmarkX, landmarkY;
        int k = 0;
        int nLandmarks = inRangeLandmarks.size();
        bool found = false;
        while( !found && k < nLandmarks ) {
            if ( inRangeLandmarks[k].id == landmarkId) {
                found = true;
                landmarkX = inRangeLandmarks[k].x;
                landmarkY = inRangeLandmarks[k].y;
            }
            k++;
        }

        /* Weights */
        double dX = observationX - landmarkX;
        double dY = observationY - landmarkY;

        double weight = ( 1/(2*M_PI*stdLandmarkRange*stdLandmarkBearing)) * exp( -( dX*dX/(2*stdLandmarkRange*stdLandmarkRange) + (dY*dY/(2*stdLandmarkBearing*stdLandmarkBearing)) ) );
        if (weight == 0) {
            particles[i].weight *= EPS;
        } else {
            particles[i].weight *= weight;
        } 
    } /* End of the last j FOR loop */
  } /* End of the i FOR loop */
} /* End of the method */

void ParticleFilter::resample() {
vector<double> weights;
double maxWeight = numeric_limits<double>::min();
for(int i = 0; i < num_particles; i++) {
    weights.push_back(particles[i].weight);
    if ( particles[i].weight > maxWeight ) {
        maxWeight = particles[i].weight;
    }
  }

uniform_real_distribution<double> distDouble(0.0, maxWeight);
uniform_int_distribution<int> distInt(0, num_particles - 1);

int index = distInt(gen);
double beta = 0.0;

/* The wheel */
vector<Particle> resampledParticles;
for(int i = 0; i < num_particles; i++) {
    beta += distDouble(gen) * 2.0;
    while( beta > weights[index]) {
        beta -= weights[index];
        index = (index + 1) % num_particles;
    }
    resampledParticles.push_back(particles[index]);
}
particles = resampledParticles;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
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
