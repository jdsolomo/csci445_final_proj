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
        self.pidTheta = pid_controller.PIDController(
            200, 0, 100, [-10, 10], [-50, 50], is_angle=True)
        # TODO identify good particle filter parameters
        self.pf = particle_filter.ParticleFilter(
            self.mapJ, 1000, 0.06, 0.15, 0.2)

        self.joint_angles = np.zeros(7)

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
        x, y, theta = self.pf.get_estimate()
        self.virtual_create.set_pose((x, y, 0.1), theta)
        data = []
        for particle in self.pf._particles:
            data.extend([particle.x, particle.y, 0.1, particle.theta])
        self.virtual_create.set_point_cloud(data)

    def inverse_kinematics(self, y_i, z_i):
        L1 = 0.4  # estimated using V-REP (joint2 - joint4)
        L2 = 0.39  # estimated using V-REP (joint4 - joint6)
        # Corrections for our coordinate system
        z = z_i - 0.3105
        y = -y_i
        # compute inverse kinematics
        r = math.sqrt(y*y + z*z)
        alpha = math.acos((L1*L1 + L2*L2 - r*r) / (2*L1*L2))
        theta2 = math.pi - alpha

        beta = math.acos((r*r + L1*L1 - L2*L2) / (2*L1*r))
        theta1 = math.atan2(y, z) - beta
        if theta2 < -math.pi / 2.0 or theta2 > math.pi / 2.0 or theta1 < -math.pi / 2.0 or theta1 > math.pi / 2.0:
            theta2 = math.pi + alpha
            theta1 = math.atan2(y, z) + beta
        if theta2 < -math.pi / 2.0 or theta2 > math.pi / 2.0 or theta1 < -math.pi / 2.0 or theta1 > math.pi / 2.0:
            print("Not possible")
            return

        self.arm.go_to(3, theta1)
        self.arm.go_to(5, theta2)
        print("Go to [{},{}], IK: [{} deg, {} deg]".format(
            y_i, z_i, math.degrees(theta1), math.degrees(theta2)))

     def pick_up(self):
        self.arm.open_gripper(80)
        self.arm.go_to(1, math.pi/2)
        self.time.sleep(1)
        self.arm.close_gripper(30)
        self.arm.go_to(1, 0)
        self.arm.go_to(5, math.pi/2)
        self.time.sleep(1)
        self.arm.go_to(2, math.pi)
        self.time.sleep(2)

     def set_down(self, shelf):

        if shelf == 3:
            self.inverse_kinematics(3.7, 1.16)
        elif shelf == 2:
            self.inverse_kinematics(3.69, 0.928)
        elif shelf == 1:
            self.inverse_kinematics(3.68, 0.92)
        else:
            print("error")
            return 0
        self.arm.open_gripper(80)
        self.arm.go_to(1, 0)
        self.arm.go_to(3, 0)
        self.arm.go_to(5, 0)

        
    def localization(self):
        init_x = 1.5
        init_y = 1.5
        init_theta = 0

        # Particle Filter params
        self.variance_dst = 0.01 #in M
        self.variance_dir = 0.05 #in radians
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
        var_dir = 0#0.1     # radians
        var_dst = 0#100    # mms

        # Initialise where we start. These variables will take note of
        # our current position
        self.x = init_x
        self.y = init_y
        self.theta = init_theta

        x_hat, y_hat, theta_hat = pf.estimate()
        self.virtual_create.set_pose(( x_hat, y_hat, 0.1 ), theta_hat)
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
            rand_numb = random.randint(0,5)
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
                self.virtual_create.set_pose(( x_hat, y_hat, 0.1 ), theta_hat)

            self.create.drive_direct(0,0) #stop moving

            # Update the particle filter visualisation
            self.virtual_create.set_point_cloud(pf.get_formatted_particle_data())

            # Slight delay required
            self.time.sleep(0.01)

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
        self.visualize()
        self.virtual_create.enable_buttons()
        self.visualize()

        self.arm.go_to(4, math.radians(-90))
        self.time.sleep(4)

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

            x_i, y_i = 100*self.create.sim_get_position()   # get actual starting position
            x_init = (x_i, 335-y_i)                         # account for map frame of reference

            #############################
            ## Map and Path Generation ##
            #############################

            # Generate Map
            print("Generating Map...")
            # TODO: get actual goal position (1.0738 m away from robot)
            x_goal = (150, 91)
            K = 3000
            delta = 10
            self.rrt.build(x_init, K, delta)            # build map
            # add goal position to map
            self.rrt.T.append(rrt.Vertex(x_goal))
            x_near = self.rrt.nearest_neighbor(x_goal)  #
            x_near.neighbors.append(self.rrt.T[-1])     #
            # Generate shortest path with Dijkstra's
            path = self.rrt.shortest_path(x_goal)

            # View Map
            for v in self.rrt.T:                # Display full map
                for u in v.neighbors:
                    self.map.draw_line(
                        (v.state[0], v.state[1]), (u.state[0], u.state[1]), (0, 0, 0))
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
            self.odometry.y = x_init[1] / 100.0
            self.odometry.theta = math.pi / 2
            base_speed = 100

            for p in path:
                goal_x = p.state[0] / 100.0
                goal_y = 3.35 - p.state[1] / 100.0
                print(goal_x, goal_y)
                while True:
                    state = self.create.update()
                    if state is not None:
                        self.odometry.update(
                            state.leftEncoderCounts, state.rightEncoderCounts)
                        goal_theta = math.atan2(
                            goal_y - self.odometry.y, goal_x - self.odometry.x)
                        theta = math.atan2(
                            math.sin(self.odometry.theta), math.cos(self.odometry.theta))
                        output_theta = self.pidTheta.update(
                            self.odometry.theta, goal_theta, self.time.time())
                        self.create.drive_direct(
                            int(base_speed+output_theta), int(base_speed-output_theta))
                        # print("[{},{},{}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))

                        distance = math.sqrt(
                            math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
                        if distance < 0.05:
                            break

            ##########################
            ## Place Glass on Shelf ##
            ##########################

            self.arm.go_to(4, math.radians(-90))
            self.arm.go_to(5, math.radians(90))
            self.time.sleep(100)

            self.time.sleep(0.01)

            self.pick_up
            self.set_down(shelf_num)
