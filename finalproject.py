from pyCreate2 import create2
import math
from py_files import odometry, pid_controller, lab8_map, rrt, particle_filter, rrt_map
import numpy as np


class Run:
    def __init__(self, factory):
        """Constructor.
        Args:
            factory (factory.FactoryCreate)
        """
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.servo = factory.create_servo()
        self.sonar = factory.create_sonar()
        self.arm = factory.create_kuka_lbr4p()
        self.virtual_create = factory.create_virtual_create()
        # self.virtual_create = factory.create_virtual_create("192.168.1.XXX")
        self.odometry = odometry.Odometry()
        self.mapJ = lab8_map.Map("lab8_map.json")
        self.map = rrt_map.Map("configuration_space.png")
        self.rrt = rrt.RRT(self.map)

        # TODO identify good PID controller gains
        self.pidDistance = pid_controller.PIDController(
            1000, 0, 50, [0, 0], [-200, 200], is_angle=False)
        self.pidTheta = pid_controller.PIDController(
            300, 5, 50, [-10, 10], [-200, 200], is_angle=True)
        pos_init = self.create.sim_get_position()
        # TODO identify good particle filter parameters
        self.pf = particle_filter.ParticleFilter(
            self.mapJ, 3000, 0.1, 0.1, 0.05)

        self.joint_angles = np.zeros(7)
        self.J_ONE_LEN = 0.4
        self.J_TWO_LEN = 0.673

    def sleep(self, time_in_sec):
        """Sleeps for the specified amount of time while keeping odometry up-to-date
        Args:
            time_in_sec (float): time to sleep in seconds
        """
        start = self.time.time()
        while True:
            state = self.create.update()
            if state is not None:
                self.odometry.update(
                    state.leftEncoderCounts, state.rightEncoderCounts)
                # print("[{},{},{}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))
            t = self.time.time()
            if start + time_in_sec <= t:
                break

    def go_to_angle(self, goal_theta):
        old_x = self.odometry.x
        old_y = self.odometry.y
        old_theta = self.odometry.theta
        while math.fabs(math.atan2(
                math.sin(goal_theta - self.odometry.theta),
                math.cos(goal_theta - self.odometry.theta))) > 0.05:
            output_theta = self.pidTheta.update(
                self.odometry.theta, goal_theta, self.time.time())
            self.create.drive_direct(int(+output_theta), int(-output_theta))
            self.sleep(0.01)
        self.create.drive_direct(0, 0)
        self.pf.move_by(self.odometry.x - old_x, self.odometry.y -
                        old_y, self.odometry.theta - old_theta)

    def forward(self):
        old_x = self.odometry.x
        old_y = self.odometry.y
        old_theta = self.odometry.theta
        base_speed = 100
        distance = 0.5
        goal_x = self.odometry.x + math.cos(self.odometry.theta) * distance
        goal_y = self.odometry.y + math.sin(self.odometry.theta) * distance
        while True:
            goal_theta = math.atan2(
                goal_y - self.odometry.y, goal_x - self.odometry.x)
            output_theta = self.pidTheta.update(
                self.odometry.theta, goal_theta, self.time.time())
            self.create.drive_direct(
                int(base_speed+output_theta), int(base_speed-output_theta))

            # stop if close enough to goal
            distance = math.sqrt(
                math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
            if distance < 0.05:
                self.create.drive_direct(0, 0)
                break
            self.sleep(0.01)
        self.pf.move_by(self.odometry.x - old_x, self.odometry.y -
                        old_y, self.odometry.theta - old_theta)

    def visualize(self):
        best_particle = self.pf.estimate()
        self.virtual_create.set_pose(
            (best_particle.x, best_particle.y, 0.1), best_particle.theta)
        data = []
        for particle in self.pf._particles:
            data.extend([particle.x, particle.y, 0.1, particle.theta])
        self.virtual_create.set_point_cloud(data)

    def inv_kinematics(l_1, l_2, y, z):
        z = z - 0.31
        y = -y + 0.00087
        r = math.sqrt(y*y+z*z)
        alpha = math.acos((l_1*l_1+l_2*l_2-r*r)/(2*l_1*l_2))
        beta = math.acos((r*r+l_1*l_1-l_2*l_2)/(2*l_1*r))
        theta2 = math.pi + alpha
        theta1 = math.atan(y/z) + beta
        if math.degrees(theta1) < -90 and math.degrees(theta1) >= -95:  # buffer
            theta1 = math.radians(-90)
        elif math.degrees(theta1) < -95:  # bring in range
            theta1 = math.radians(math.degrees(theta1)+180)
        elif math.degrees(theta1) > 90 and math.degrees(theta1) <= 95:  # buffer
            theta1 = math.radians(90)
        elif math.degrees(theta1) > 95:  # bring in range
            theta1 = math.radians(math.degrees(theta1)-180)

        if math.degrees(theta2) < -120 and math.degrees(theta2) >= -125:
            theta2 = math.radians(-120)
        elif math.degrees(theta2) < -125:
            theta2 = math.radians(math.degrees(theta2)+360)
        elif math.degrees(theta2) > 120 and math.degrees(theta2) <= 125:
            theta2 = math.radians(120)
        elif math.degrees(theta2) > 125:
            theta2 = math.radians(math.degrees(theta2)-360)
        print("Go to ", (y-0.00087)*-1, ", ", z+0.31,
              ", IK[", math.degrees(theta1), ", ", math.degrees(theta2), "]")
        return [theta1, theta2]

    def pick_up(self):
        self.arm.open_gripper()
        self.arm.go_to(1, math.pi/2)
        self.time.sleep(1)
        self.arm.go_to(5, -math.pi/12)
        self.time.sleep(1)
        self.arm.go_to(3, math.pi/9)
        self.time.sleep(1)
        self.arm.close_gripper()
        self.time.sleep(6)
        self.arm.go_to(1, 0)
        self.arm.go_to(5, math.pi/2)
        self.time.sleep(1)

    def set_down(self, shelf):

        l_1 = 0.4  # j2-j4
        l_2 = 0.39  # j4-j6

        if shelf == 3:
            theta = Run.inv_kinematics(l_1, l_2, -0.1, 1)
            self.arm.go_to(1, theta[0])
            self.arm.go_to(3, theta[1])
            self.time.sleep(10)
        elif shelf == 2:
            theta = Run.inv_kinematics(l_1, l_2, -0.4, 0.9)
            self.arm.go_to(1, theta[0])
            self.arm.go_to(3, theta[1])
            self.time.sleep(10)
        elif shelf == 1:

            theta = Run.inv_kinematics(l_1, l_2, -0.6, 0.7)
            self.arm.go_to(1, theta[0])
            self.arm.go_to(3, theta[1])
            self.time.sleep(10)
        else:
            print("error")
            return 0
        self.arm.go_to(0, -math.pi)
        self.time.sleep(2)
        self.arm.go_to(5, -math.pi/3)
        self.time.sleep(2)
        self.arm.open_gripper()
        self.time.sleep(3)
        self.arm.go_to(1, 0)
        self.arm.go_to(3, 0)
        self.arm.go_to(5, 0)

    def localization(self):
        init_x = 1.5
        init_y = 1.5
        init_theta = 0

        # Particle Filter params
        self.variance_dst = 0.01  # in M
        self.variance_dir = 0.05  # in radians
        self.variance_son = 0.01

        # Set up the particle filter with N particles
        num_particles = 1000
        pf = ParticleFilter(self.map, math.sqrt(self.variance_dst), math.sqrt(self.variance_dir), math.sqrt(self.variance_son),
                            num_particles, init_x, init_y, init_theta)

        # This is an example on how to estimate the distance to a wall for the given
        # map, assuming the robot is at (0, 0) and has heading math.pi
        # print(self.map.closest_distance((0.5,0.5), 0))

        # Drive parameters
        t_forward = 1       # How long it drives forward for
        t_turn = 1.8
        mag_forward = 150   # mm /sec
        wall_buffer = mag_forward + 150
        mag_turn = 100      # mm /sec
        var_dir = 0  # 0.1     # radians
        var_dst = 0  # 100    # mms

        # Initialise where we start. These variables will take note of
        # our current position
        self.x = init_x
        self.y = init_y
        self.theta = init_theta

        x_hat, y_hat, theta_hat = pf.estimate()
        self.virtual_create.set_pose((x_hat, y_hat, 0.1), theta_hat)
        self.virtual_create.set_point_cloud(pf.get_formatted_particle_data())

        self.servo.go_to(0)
        self.time.sleep(1)
        step_count = 0

        # This is an example on how to detect that a button was pressed in V-REP
        while True:

            # Stop the previous movement
            # self.forward(0,0,0)

            # What is the true sonar distance that we're getting
            sonar_dist = self.sonar.get_distance()
            rand_numb = random.randint(0, 5)
            print("*************random number: " + str(rand_numb))
            step_count += 1

            # Read the buttons
            b = self.virtual_create.get_last_button()
            # if b == self.virtual_create.Button.MoveForward:
            if rand_numb < 4:
                print("Forward pressed!")
                if(sonar_dist > (wall_buffer/1000)):
                    pf.move_by(mag_forward/1000, 0)

                    self.create.drive_direct(mag_forward, mag_forward)
                    self.time.sleep(t_forward)
                else:
                    print("skip")

            # elif b == self.virtual_create.Button.TurnLeft:
            elif rand_numb == 4:
                print("Turn left pressed!")
                pf.move_by(0, math.pi/2)

                self.turn(math.pi/2)

            # elif b == self.virtual_create.Button.TurnRight:
            elif rand_numb == 5:
                print("Turn right pressed!")
                pf.move_by(0, -math.pi/2)

                self.turn(-math.pi/2)
                # self.theta -=

            # elif b == self.virtual_create.Button.Sense:
            print("sense!")
            sonar_dist = self.sonar.get_distance()
            pf.measure(sonar_dist)

            # self.servo.go_to(-30)
            # self.time.sleep(0.5)
            # sonar_dist = self.sonar.get_distance()
            # pf.measure(sonar_dist)

            # self.servo.go_to(30)
            # self.time.sleep(0.5)
            # sonar_dist = self.sonar.get_distance()
            # pf.measure(sonar_dist)

            # self.servo.go_to(0)
            # self.time.sleep(0.5)

            if(step_count % 3 == 0):

                pf.resample()
                x_hat, y_hat, theta_hat = pf.estimate()
                # Visualize the estimated pose
                self.virtual_create.set_pose((x_hat, y_hat, 0.1), theta_hat)

            self.create.drive_direct(0, 0)  # stop moving

            # Update the particle filter visualisation
            self.virtual_create.set_point_cloud(
                pf.get_formatted_particle_data())

            # Slight delay required
            self.time.sleep(0.01)

    def go_to_point(self, x_init, z_init, output=True):

        # change frame of reference
        x = -x_init + 1.6
        z = -z_init + 3.4

        # precalculate r
        r = np.sqrt(np.power(x, 2) + np.power(z, 2))

        # Calculate angle 1
        beta_num = np.power(r, 2) + self.J_ONE_LEN**2 - self.J_TWO_LEN**2
        beta_den = 2 * self.J_ONE_LEN * r
        beta = np.arccos(beta_num / beta_den)
        if(x >= 0):
            theta_1 = np.arctan2(z, x) + beta
        else:
            theta_1 = np.arctan2(z, x) - beta

        # Calculate angle 2
        alpha_num = self.J_ONE_LEN**2 + self.J_TWO_LEN**2 - np.power(r, 2)
        alpha_den = 2 * self.J_ONE_LEN * self.J_TWO_LEN
        alpha = np.arccos(alpha_num / alpha_den)
        if(x <= 0):
            theta_2 = - np.pi + alpha
        else:
            theta_2 = np.pi - alpha

        # Display information
        if (output):
            print("Go to [{:.1f},{:.1f}] , IK: [{:.2f} deg, {:.2f} deg]".format(
                x_init, z_init, np.degrees(theta_1), np.degrees(theta_2)))

        # Move robotic arm
        self.go_to_angle(np.degrees(theta_1), np.degrees(theta_2), False)

    def go_to_angle(self, angle_1, angle_2, output=True):

        # Convert angles to radians
        angle_1_rad = np.deg2rad(angle_1)
        angle_2_rad = np.deg2rad(angle_2)

        # Transform absolute angles to robot's frame of reference angles
        if(angle_1 >= 0):
            adjusted_angle_1_rad = angle_1_rad - (np.pi / 2)
        else:
            adjusted_angle_1_rad = -1 * angle_1_rad - (np.pi / 2)

        # Send movement commands to robot
        self.arm.go_to(0, adjusted_angle_1_rad)
        self.arm.go_to(3, angle_2_rad)
        self.time.sleep(1)

        # Calculate what point we should end at
        x = self.J_ONE_LEN * \
            np.cos(angle_1_rad) + self.J_TWO_LEN * \
            np.cos(angle_1_rad + angle_2_rad)
        z = self.J_ONE_LEN * \
            np.sin(angle_1_rad) + self.J_TWO_LEN * \
            np.sin(angle_1_rad + angle_2_rad) + 0.30  # Ground offset

        # Display check data
        if(output):
            print("Go to {}, {} deg, FK: [{:.2f},{:.2f},{:.2f}]".format(
                angle_1, angle_2, x, z, 0.32))

    def run(self):
        self.create.start()
        self.create.safe()

        self.create.drive_direct(0, 0)

        # self.arm.open_gripper()

        # self.time.sleep(4)

        # self.arm.close_gripper()

        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])
        # self.visualize()
        # self.virtual_create.enable_buttons()
        # self.visualize()

        # self.arm.go_to(4, math.radians(-90))
        # self.time.sleep(4)

        while True:
            b = self.virtual_create.get_last_button()
            if b == self.virtual_create.Button.MoveForward:
                self.forward()
                self.visualize()
            elif b == self.virtual_create.Button.TurnLeft:
                self.go_to_angle(self.odometry.theta + math.pi / 2)
                self.visualize()
            elif b == self.virtual_create.Button.TurnRight:
                self.go_to_angle(self.odometry.theta - math.pi / 2)
                self.visualize()
            elif b == self.virtual_create.Button.Sense:
                distance = self.sonar.get_distance()
                print(distance)
                self.pf.measure(distance, 0)
                self.visualize()

            # posC = self.create.sim_get_position()

            # print(posC)

            # get actual starting position
            x_i, y_i = 100 * \
                self.create.sim_get_position(
                )[0], 100*self.create.sim_get_position()[1]
            # account for map frame of reference
            x_init = (x_i, 300-y_i)

            #############################
            ## Map and Path Generation ##
            #############################

            # Generate Map
            print("Generating Map...")
            # TODO: get actual goal position (1.0738 m away from robot)
            K = 5000
            delta = 5
            self.rrt.build(x_init, K, delta)            # build map
            x_goal = [165, 67]
            # Generate shortest path with Dijkstra's
            path = self.rrt.shortest_path(x_goal)

            # View Map
            for v in self.rrt.T:                # Display full map
                for u in v.neighbors:
                    self.map.draw_line(
                        (v.state[0], v.state[1]), (u.state[0], u.state[1]), (255, 0, 0))
            for idx in range(0, len(path)-1):   # Display map with shortest path
                self.map.draw_line((path[idx].state[0], path[idx].state[1]), (
                    path[idx+1].state[0], path[idx+1].state[1]), (0, 255, 0))

            # Save Map
            self.map.save("f_rrt.png")

            print("Map Generated!")

            ###############################
            ## Waypoint following of rrt ##
            ###############################

            # TODO these need to be adjusted based on starting position
            self.odometry.x = x_init[0] / 100.0
            self.odometry.y = x_init[1] / 100.0 - 2
            self.odometry.theta = 0
            base_speed = 100
            pf_ind = 0
            best_estimate_x = 1.0
            best_estimate_y = 0.5

            for p in path:
                goal_x = p.state[0] / 100.0
                goal_y = (300.0 - p.state[1]) / 100.0
                print("*************NEW GOAL*************")
                while True:
                    print("Goal:", goal_x, goal_y, "   Current:",
                          best_estimate_x, best_estimate_y)
                    state = self.create.update()
                    if state is not None:
                        sonar_d = self.sonar.get_distance()  # get sonar reading

                        # update and send odometry to pf and assess particles
                        self.odometry.update(
                            state.leftEncoderCounts, state.rightEncoderCounts)

                        # get particle with highest probability
                        pf_ind += 1
                        if (pf_ind % 3 == 0):
                            self.pf.move_by(self.odometry.x,
                                            self.odometry.y, self.odometry.theta)
                            self.pf.measure(sonar_d)
                            pf_estimate = self.pf.estimate()
                            self.virtual_create.set_pose(
                                (pf_estimate.x, pf_estimate.y, 0.1), pf_estimate.theta)
                            point_cloud = []
                            for par in self.pf._particles:
                                point_cloud.append(par.x)
                                point_cloud.append(par.y)
                                point_cloud.append(0.1)
                                point_cloud.append(par.theta)
                            self.virtual_create.set_point_cloud(point_cloud)

                        if pf_ind > 100:
                            print("\nSwitching to Particle Filter\n")
                            best_estimate_x = pf_estimate.x
                            best_estimate_y = pf_estimate.y
                            best_estimate_theta = pf_estimate.theta
                        else:
                            best_estimate_x = self.odometry.x
                            best_estimate_y = self.odometry.y
                            best_estimate_theta = self.odometry.theta

                        goal_theta = math.atan2(
                            goal_y - best_estimate_y, goal_x - best_estimate_x)
                        output_theta = self.pidTheta.update(
                            best_estimate_theta, goal_theta, self.time.time())

                        distance = math.sqrt(
                            math.pow(goal_x - best_estimate_x, 2) + math.pow(goal_y - best_estimate_y, 2))
                        output_distance = self.pidDistance.update(
                            0, distance, self.time.time())

                        # self.create.drive_direct(int(base_speed), int(base_speed))
                        self.create.drive_direct(
                            int(output_distance+output_theta), int(output_distance-output_theta))
                        # print("[{},{},{}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))

                        # distance = math.sqrt(math.pow(goal_x - best_estimate_x, 2) + math.pow(goal_y - best_estimate_y, 2))
                        if distance < 0.1 and p.state[0] != path[-1].state[0]:
                            break
                        if distance < 0.1 and p.state[0] == path[-1].state[0]:
                            print("Distance: ", distance)
                            self.create.drive_direct(0, 0)
                            self.time.sleep(1)
                            break

            # Set orientation to theta=0
            goal_theta = 0
            while (abs(goal_theta - self.odometry.theta) > 0.01):
                state = self.create.update()
                if state is not None:
                    self.odometry.update(
                        state.leftEncoderCounts, state.rightEncoderCounts)
                    output_theta = self.pidTheta.update(
                        self.odometry.theta, goal_theta, self.time.time())
                    self.create.drive_direct(output_theta, -output_theta)

            self.create.drive_direct(0, 0)
            self.time.sleep(1)

            ##########################
            ## Place Glass on Shelf ##
            ##########################

            # Leave room so arm doesn't knock over cup
            self.arm.go_to(1, math.radians(80))
            self.time.sleep(3)
            self.arm.go_to(2, math.radians(-90))
            self.time.sleep(3)
            self.arm.go_to(6, math.radians(90))
            self.time.sleep(3)
            print("Moved to Start Location")

            self.go_to_point(x_goal[0] / 100.0, (300 - x_goal[1]) / 100.0)
            self.time.sleep(3)
            # Bring grippers down around cup
            self.arm.go_to(1, math.radians(90))
            self.time.sleep(3)
            self.arm.close_gripper()
            self.time.sleep(6)

            # self.pick_up()
            # self.time.sleep(1)
            # self.set_down(2)

            while(1):
                pass
