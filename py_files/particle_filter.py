import numpy as np
from scipy.stats import norm
# import scipy.special
import math


class Particle:
    def __init__(self, x, y, theta, ln_p, lastX, lastY):
        self.ln_p = ln_p
        self.x = x
        self.y = y
        self.lastX = lastX
        self.lastY = lastY
        self.theta = theta
        # self.lastTheta = theta
        self.prob_previous = ln_p
        # self.d = 0

    def move_by(self, x, y, theta, theta_var, d_var):
        theta_variance = theta_var
        distance_variance = d_var
        self.theta = theta + np.random.normal(0, theta_variance)
        d = ((x-self.lastX)**2 + (y-self.lastY)**2)**(1/2) + \
            np.random.normal(0, distance_variance)
        self.x = max(min(3, (self.x + d*np.cos(self.theta))), 0)
        self.y = max(min(3, self.y + d*np.sin(self.theta)), 0)
        self.lastX = x
        self.lastY = y
       # self.lastTheta

    def sensor_reading(self, sonar_reading, map, sense_var):
        sensor_variance = sense_var  # measured in meters, so 10cm
        sensor_mean = map.closest_distance(
            (self.x, self.y), self.theta)  # takes origin, theta

        if sensor_mean == None:
            self.ln_p = -math.inf
            print('\nInfinity Value Set\n')
        else:
            # print(norm.pdf(sonar_reading, sensor_mean, variance))
            probSensorGivenLoc = np.log(norm.pdf(
                sonar_reading, sensor_mean, sensor_variance))
            # probRobotAtLoc = self.ln_p
            self.ln_p += probSensorGivenLoc


class ParticleFilter:
    def __init__(self, map, num_particles, theta_var, d_var, sense_var):
        self._particles = []
        self._map = map
        self.theta_var = theta_var
        self.d_var = d_var
        self.sense_var = sense_var
        for i in range(num_particles):
            x = np.random.uniform(0, 3)
            y = np.random.uniform(0, 3)
            theta = np.random.uniform(-1 * np.pi, np.pi)
            ln_p = np.log(1/num_particles)
            self._particles.append(Particle(x, y, theta, ln_p, x, y))

    def move_by(self, x, y, theta):
        # for real theta:
        for particle in self._particles:
            particle.move_by(x, y, theta, self.theta_var, self.d_var)

    def measure(self, sonar_reading):

        avg_prob = 0
        for particle in self._particles:
            particle.sensor_reading(sonar_reading, self._map, self.sense_var)
            avg_prob += math.exp(particle.ln_p)
        avg_prob /= len(self._particles)
        print(avg_prob)

        # sum of prob of robot location given each sensor reading

        totVal = np.log(np.sum(np.exp(
            [particle.ln_p for particle in self._particles])))

        # print probabilities

        for particle in self._particles:
            particle.ln_p -= totVal

        probabilitites = [math.exp(particle.ln_p)
                          for particle in self._particles]
        # if .9 < sum(probabilitites) < 1.1:
        temp_arr = np.random.choice(a=[i for i in range(0, len(self._particles))], size=len(
            self._particles), replace=True, p=([math.exp(particle.ln_p) for particle in self._particles]))

        copy_arr = []
        for i in temp_arr:
            copy_arr.append(Particle(
                self._particles[i].x, self._particles[i].y, self._particles[i].theta, self._particles[i].ln_p, self._particles[i].lastX, self._particles[i].lastY))
        self._particles = copy_arr

    def estimate(self):
        # return max prob.
        # sself._particles = np.random.choice(a=self._particles, size=len( self._particles), replace=True, p=([math.exp(particle.ln_p) for particle in self._particles]))
        maxParticle = max(self._particles, key=lambda particle: particle.ln_p)
        maxParticle = max(self._particles, key=lambda particle: particle.ln_p)
        return maxParticle

    def foundLocation(self):
        avgX = sum([particle.x for particle in self._particles]) / \
            len(self._particles)
        avgY = sum([particle.y for particle in self._particles]) / \
            len(self._particles)
        for particle in self._particles:
            if (abs(particle.x - avgX) > .2 or abs(particle.y - avgY) > .2):
                return False

        return True


# import scipy.special


class Particle:
    def __init__(self, x, y, theta, ln_p, lastX, lastY):
        self.ln_p = ln_p
        self.x = x
        self.y = y
        self.lastX = lastX
        self.lastY = lastY
        self.theta = theta
        # self.lastTheta = theta
        self.prob_previous = ln_p
        # self.d = 0

    def move_by(self, x, y, theta, theta_var, d_var):
        theta_variance = theta_var
        distance_variance = d_var
        self.theta = theta + np.random.normal(0, theta_variance)
        d = ((x-self.lastX)**2 + (y-self.lastY)**2)**(1/2) + \
            np.random.normal(0, distance_variance)
        self.x = max(min(3, (self.x + d*np.cos(self.theta))), 0)
        self.y = max(min(3, self.y + d*np.sin(self.theta)), 0)
        self.lastX = x
        self.lastY = y
       # self.lastTheta

    def sensor_reading(self, sonar_reading, map, sense_var):
        sensor_variance = sense_var  # measured in meters, so 10cm
        sensor_mean = map.closest_distance(
            (self.x, self.y), self.theta)  # takes origin, theta

        if sensor_mean == None:
            self.ln_p = -math.inf
            print('\nInfinity Value Set\n')
        else:
            # print(norm.pdf(sonar_reading, sensor_mean, variance))
            probSensorGivenLoc = np.log(norm.pdf(
                sonar_reading, sensor_mean, sensor_variance))
            # probRobotAtLoc = self.ln_p
            self.ln_p += probSensorGivenLoc


class ParticleFilter:
    def __init__(self, map, num_particles, theta_var, d_var, sense_var):
        self._particles = []
        self._map = map
        self.theta_var = theta_var
        self.d_var = d_var
        self.sense_var = sense_var
        for i in range(num_particles):
            x = np.random.uniform(0, 3)
            y = np.random.uniform(0, 3)
            theta = np.random.uniform(-1 * np.pi, np.pi)
            ln_p = np.log(1/num_particles)
            self._particles.append(Particle(x, y, theta, ln_p, x, y))

    def move_by(self, x, y, theta):
        # for real theta:
        for particle in self._particles:
            particle.move_by(x, y, theta, self.theta_var, self.d_var)

    def measure(self, sonar_reading):

        avg_prob = 0
        for particle in self._particles:
            particle.sensor_reading(sonar_reading, self._map, self.sense_var)
            avg_prob += math.exp(particle.ln_p)
        avg_prob /= len(self._particles)
        print(avg_prob)

        # sum of prob of robot location given each sensor reading

        totVal = np.log(np.sum(np.exp(
            [particle.ln_p for particle in self._particles])))

        # print probabilities

        for particle in self._particles:
            particle.ln_p -= totVal

        probabilitites = [math.exp(particle.ln_p)
                          for particle in self._particles]
        # if .9 < sum(probabilitites) < 1.1:
        temp_arr = np.random.choice(a=[i for i in range(0, len(self._particles))], size=len(
            self._particles), replace=True, p=([math.exp(particle.ln_p) for particle in self._particles]))

        copy_arr = []
        for i in temp_arr:
            copy_arr.append(Particle(
                self._particles[i].x, self._particles[i].y, self._particles[i].theta, self._particles[i].ln_p, self._particles[i].lastX, self._particles[i].lastY))
        self._particles = copy_arr

    def estimate(self):
        # return max prob.
        # sself._particles = np.random.choice(a=self._particles, size=len( self._particles), replace=True, p=([math.exp(particle.ln_p) for particle in self._particles]))
        maxParticle = max(self._particles, key=lambda particle: particle.ln_p)
        maxParticle = max(self._particles, key=lambda particle: particle.ln_p)
        return maxParticle

    def foundLocation(self):
        avgX = sum([particle.x for particle in self._particles]) / \
            len(self._particles)
        avgY = sum([particle.y for particle in self._particles]) / \
            len(self._particles)
        for particle in self._particles:
            if (abs(particle.x - avgX) > .2 or abs(particle.y - avgY) > .2):
                return False

        return True
