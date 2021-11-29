import math
import numpy as np
from scipy.stats import norm
import copy

class Particle:
    def __init__(self, x, y, theta, ln_p, map):
        self.x = x
        self.y = y
        self.theta = theta
        self.ln_p = ln_p
        self.map = map

    def holdInMap(value, min_val, max_val):
        return min(max(value, min_val), max_val)

    def get_updated_prob(self, new_prob):
        return self.ln_p + math.log(new_prob)

    def update_pose(self, noisy_dir, noisy_dst):
        self.theta += noisy_dir
        self.theta %= 2*math.pi

        # check walls also
        self.x = Particle.holdInMap(self.x + noisy_dst * math.cos(self.theta), self.map.bottom_left[0], self.map.top_right[0])
        self.y = Particle.holdInMap(self.y + noisy_dst * math.sin(self.theta), self.map.bottom_left[1], self.map.top_right[1])


    def get_formatted_data(self):
        # Assume z is 0.1, theta in radians
        return [self.x, self.y, 0, self.theta]

class ParticleFilter:
    def __init__(self, world_map, sd_dst, sd_dir, sd_son, num_particles, init_x, init_y, init_theta):

        # init Standard Deviation
        self.sd_dst = sd_dst
        self.sd_dir = sd_dir
        self.sd_son = sd_son

        print(sd_dst, sd_dir, sd_son)

        # Store the map
        self.map = world_map

        # Store the particles here
        self._particles = []

        # When we initialise, the probability of each particle
        # is equal
        p = 1.0 / num_particles
        log_p = math.log(p) # base e (natural log)

        self.probabilities = np.empty([num_particles])


        # Initialise
        for _ in range(num_particles):

            # Randomly distribute around the map
            x = np.random.uniform(self.map.bottom_left[0], self.map.top_right[0])
            y = np.random.uniform(self.map.bottom_left[1], self.map.top_right[1])
            theta = np.random.uniform(0, 2 * math.pi)

            # Create and store the particle
            p = Particle(x, y, theta, log_p, world_map)
            self._particles.append(p)

    def move_by(self, dist, theta):
        for i in range(len(self._particles)):
            noisy_dir = theta + np.random.normal(0, self.sd_dir)
            noisy_dst = dist + np.random.normal(0, self.sd_dst)

            self._particles[i].update_pose(noisy_dir, noisy_dst)

    def measure(self, sonar_reading):

        for i in range(len(self._particles)):
            actual_dist_virt = self.map.closest_distance((self._particles[i].x, self._particles[i].y), self._particles[i].theta)
            p_sensor_given_rob_at_loc = norm.pdf(sonar_reading, actual_dist_virt, self.sd_son)
            self.probabilities[i] = self._particles[i].get_updated_prob(p_sensor_given_rob_at_loc)

        exp_arr = np.exp(self.probabilities)
        sum = np.sum(exp_arr)
        self.probabilities -= math.log(sum)
        for i in range(len(self._particles)):
            self._particles[i].ln_p = self.probabilities[i]


#         p_sonar = 0
#         print("************SONAR_READING: " + str(sonar_reading))
#         for i in range(len(self._particles)):
#             p_sonar += self._particles[i].ln_p
#
#         post_prob_total = 0
#         for i in range(len(self._particles)):
#             print(self._particles[i].x)
#             print(self._particles[i].y)
#             print(self._particles[i].theta)
#             print(self._particles[i].ln_p)
#             print("dist: " + str(self.map.closest_distance((self._particles[i].x, self._particles[i].y), self._particles[i].theta)))
#
# #a          # calculate p(sensor|robot@location) = N(sensor reading; mean = dist to wall, sd_son)
#             dist_to_wall = self.map.closest_distance((self._particles[i].x, self._particles[i].y), self._particles[i].theta)
#             # p_s_given_rl = norm.pdf(sonar_reading, dist_to_wall, self.sd_son)
#             p_s_given_rl = norm(dist_to_wall,self.sd_son).pdf(sonar_reading)
#             print("p(sensor|robot): " + str(p_s_given_rl))
#
# #b          # p(robot@location)
# #             print("p(robot): " + str(self._particles[i].ln_p))
#
# # #c          # p(sensor reading)
# #             print("p(sensor): " + str(p_sonar))
#
#             # posterior_prob = p_s_given_rl*self._particles[i].ln_p/p_sonar
#             posterior_prob = p_s_given_rl
#             print("post prob: " + str(posterior_prob))
#             # post_prob_total += posterior_prob
#             self._particles[i].update_prob(posterior_prob)
#             print(self._particles[i].ln_p)
#             print("")
#
#         print("totalProb: " + str(post_prob_total))
#         self.resample()




    def estimate(self):
        # Return the best estimate of our position - this is just
        # the average of the particles, scaled by the probability
        x_ag      = [ p.x for p in self._particles ]
        y_ag    = [ p.y for p in self._particles ]
        theta_ag  = [ p.theta for p in self._particles ]

        # Weighted by probabilities
        x = np.average(x_ag)
        y = np.average(y_ag)
        theta = np.average(theta_ag)

        return x, y, theta

    def resample(self):
        index = np.random.choice(np.arange(0, len(self._particles)), 
                                 len(self._particles), 
                                 True, 
                                 np.exp(self.probabilities))
        # print(index)
        new_particles = []
        for i in index:
            new_particles.append(Particle(self._particles[i].x,
                                          self._particles[i].y,
                                          self._particles[i].theta,
                                          self._particles[i].ln_p,
                                          self._particles[i].map))

        self._particles = new_particles

        # probs = [ p.ln_p for p in self._particles ]
        # total = sum(probs)
        # probs = np.array(probs) / total
        #
        # probTotal = 0
        #
        # ideal_ind = -1
        # threshold = 0.01
        # for i in range(len(self._particles)):
        #     print(probs[i])
        #     probTotal += probs[i]
        #     if(ideal_ind == -1):
        #         ideal_ind = i
        #     else:
        #         if(probs[i] < probs[ideal_ind]):
        #             ideal_ind = i
        #
        # print("ideal_ind: " + str(ideal_ind))
        # for i in range(len(self._particles)):
        #     if(probs[i] < threshold):
        #
        #         print("change(" + str(i)+ "): " + str(self._particles[i].x) + ", " + str(self._particles[i].y) + ", " + str(self._particles[i].theta))
        #         print("to: " + str(self._particles[ideal_ind].x) + ", " + str(self._particles[ideal_ind].y) + ", " + str(self._particles[ideal_ind].theta))
        #         self._particles[i].x = self._particles[ideal_ind].x
        #         self._particles[i].y = self._particles[ideal_ind].y
        #         self._particles[i].theta = self._particles[ideal_ind].theta
        #         self._particles[i].ln_p = self._particles[ideal_ind].ln_p
        #
        #
        # print("probtotal: " + str(probTotal))
        # print(ideal_ind)
        # Where do we think we are?

    def get_formatted_particle_data(self):
        # Returns particle data in the desired format for display in
        # coppelia sim
        data = []
        for i in range(len(self._particles)):
            #print(self._particles[i].get_formatted_data())
            data.extend(self._particles[i].get_formatted_data())
        return data
