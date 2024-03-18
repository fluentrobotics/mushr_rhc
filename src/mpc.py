import numpy as np
import rospy
import utils
import time
import itertools
from controller import BaseController

from nav_msgs.srv import GetMap


class ModelPredictiveController(BaseController):
    def __init__(self):
        super(ModelPredictiveController, self).__init__()

        self.reset_params()
        self.reset_state()

    def get_reference_index(self, pose):
        '''
        get_reference_index finds the index i in the controller's path
            to compute the next control control against
        input:
            pose - current pose of the car, represented as [x, y, heading]
        output:2
            i - referencence index
        '''
        with self.path_lock:
            pose = np.array(pose)
            diff = self.path[:, :3] - pose
            dist = np.linalg.norm(diff[:, :2], axis=1)
            index = dist.argmin()
            #print("path length: " + str(len(self.path)))
            #print('init ind: ' + str(index))
            
            while(dist[index] < self.waypoint_lookahead and index <= len(self.path) - 2):
                index += 1
                index = min(index, len(self.path)-1)
             
            if(len(self.path)==1):
                self.index = 0
                return 0  # handle special case of a simple go-to pose
            self.index = index
            #print('returning ind: ' + str(index))
            return index

    def get_control(self, pose, index):
        '''
        get_control - computes the control action given an index into the
            reference trajectory, and the current pose of the car.
            Note: the output velocity is given in the reference point.
        input:
            pose - the vehicle's current pose [x, y, heading]
            index - an integer corresponding to the reference index into the
                reference path to control against
        output:
            control - [velocity, steering angle]
        '''
        assert len(pose) == 3

        # For each K trial, the first position is at the current position
        rollouts = np.zeros((self.K, self.T, 3))
        rollouts[:, 0, :] = np.array(pose)

        speed_sign = np.array([-1*self.speed, 0 ,1*self.speed])
        #speed_sign = np.array([self.speed])  # we got 3 speeds, forward V, 0, reverse V, where V is the desired speed from the xyhv waypoint
        min_cost = 1000000000   # very large initial cost because we are looking for the minimum.
        #min_cost_ctrl = np.zeros(2)  # default controls are no steering and no throttle
        min_cost_ctrl=np.array([0,0])
        for sign in range(len(speed_sign)):
            #self.trajs[:, :, 0] = self.path[index, 3] * speed_sign[sign]  # multiply magnitude with sign
            self.trajs[:,:,0] = speed_sign[sign]
        
            # perform rollouts for each control trajectory
            for t in range(1, self.T):
                cur_x = rollouts[:, t - 1]
                xdot, ydot, thetadot = self.apply_kinematics(cur_x, self.trajs[:, t - 1])
                #print("xdot: " + str(xdot))
                rollouts[:, t, 0] = cur_x[:, 0] + xdot
                rollouts[:, t, 1] = cur_x[:, 1] + ydot
                rollouts[:, t, 2] = cur_x[:, 2] + thetadot

            costs = self.apply_cost(rollouts, index)  # get the cost for each roll out

            min_control = np.argmin(costs)  # find the min
            if(min_cost > costs[min_control]):  # if the min is less than global min,
                min_cost = costs[min_control]  # reset global min
                min_cost_ctrl = np.copy(self.trajs[min_control][0])  # save the last best control set.

        #print(min_cost)
        return min_cost_ctrl

    def reset_state(self):
        '''
        Utility function for resetting internal states.
        '''
        with self.path_lock:
            self.trajs = self.get_control_trajectories()
            assert self.trajs.shape == (self.K, self.T, 2)
            self.scaled = np.zeros((self.K * self.T, 3))
            self.bbox_map = np.zeros((self.K * self.T, 2, 4))
            self.perm = np.zeros(self.K * self.T).astype(int)
            self.map = self.get_map()
            self.perm_reg = self.load_permissible_region(self.map)
            self.map_x = self.map.info.origin.position.x
            self.map_y = self.map.info.origin.position.y
            self.map_angle = utils.rosquaternion_to_angle(self.map.info.origin.orientation)
            self.map_c = np.cos(self.map_angle)
            self.map_s = np.sin(self.map_angle)
            self.index = 0


    def reset_params(self):
        '''
        Utility function for updating parameters which depend on the ros parameter
            server. Setting parameters, such as gains, can be useful for interative
            testing.
        '''
        with self.path_lock:
            self.wheelbase = float(rospy.get_param("trajgen/wheelbase", 0.33))
            #self.min_delta = float(rospy.get_param("trajgen/min_delta", -0.34))
            #self.max_delta = float(rospy.get_param("trajgen/max_delta", 0.34))
            self.min_delta = float(rospy.get_param("trajgen/min_delta", -0.354))
            self.max_delta = float(rospy.get_param("trajgen/max_delta", 0.354))

            self.K = int(rospy.get_param("mpc/K", 62))
            self.T = int(rospy.get_param("mpc/T", 8))

            self.speed = float(rospy.get_param("mpc/speed", 0.4))
            
            #self.finish_threshold = float(rospy.get_param("mpc/finish_threshold", 0.5))
            self.finish_threshold = float(rospy.get_param("mpc/finish_threshold", 0.25))
            #self.exceed_threshold = float(rospy.get_param("mpc/exceed_threshold", 100.0))
            self.exceed_threshold = float(rospy.get_param("mpc/exceed_threshold", 50.0))
            # Average distance from the current reference pose to lookahed.
            #self.waypoint_lookahead = float(rospy.get_param("mpc/waypoint_lookahead", 0.5))
            self.waypoint_lookahead = float(rospy.get_param("mpc/waypoint_lookahead", self.speed*0.55))
            print("== MPC Reference Speed: " + str(self.speed) + " ==")
            print("== MPC Look Ahead: " + str(self.waypoint_lookahead) + " ==")
            print("== MPC Goal Tolerance: " + str(self.finish_threshold) + " ==")
            
            self.collision_w = float(rospy.get_param("mpc/collision_w", 1e5))
            #self.error_w = float(rospy.get_param("mpc/error_w", 10.0))
            self.error_w = float(rospy.get_param("mpc/error_w", 10.0))
            #Orientation error
            self.error_th = float(rospy.get_param("mpc/error_th", 0.1))

            self.car_length = float(rospy.get_param("mpc/car_length", 0.7))
            self.car_width = float(rospy.get_param("mpc/car_width", 0.4))

    def get_control_trajectories(self):
        '''
        get_control_trajectories computes K control trajectories to be
            rolled out on each update step. You should only execute this
            function when initializing the state of the controller.

            various methods can be used for generating a sufficient set
            of control trajectories, but we suggest stepping over the
            control space (of steering angles) to create the initial
            set of control trajectories.
        output:
            ctrls - a (K x T x 2) vector which lists K control trajectories
                of length T
        '''
        ctrls = np.zeros((self.K, self.T, 2))
        step_size = (self.max_delta - self.min_delta) / (self.K - 1)
        ctrls[:, :, 0] = self.speed
        for t in range(self.T):
            ctrls[:, t, 1] = np.arange(self.min_delta, self.max_delta + step_size, step_size)
        return ctrls

    def apply_kinematics(self, cur_x, control):
        '''
        apply_kinematics 'steps' forward the pose of the car using
            the kinematic car model for a given set of K controls.
        input:
            cur_x   (K x 3) - current K "poses" of the car
            control (K x 2) - current controls to step forward
        output:
            (x_dot, y_dot, theta_dot) - where each *_dot is a list
                of k deltas computed by the kinematic car model.
        '''
        dt = 0.1
        speed = control[:, 0]
        steering_angle = control[:, 1]
        x_dot = speed * np.cos(cur_x[:, 2]) * dt
        y_dot = speed * np.sin(cur_x[:, 2]) * dt
        theta_dot = ((speed * np.tan(steering_angle)) / (self.wheelbase)) * dt
        return (x_dot, y_dot, theta_dot)

    def apply_cost(self, poses, index):
        '''
        rollouts (K,T,3) - poses of each rollout
        index    (int)   - reference index in path
        '''
        all_poses = poses.copy()
        all_poses.resize(self.K * self.T, 3)
        collisions = self.check_collisions_in_map(all_poses)
        collisions.resize(self.K, self.T)
        collision_cost = collisions.sum(axis=1) * self.collision_w
        error_cost = np.linalg.norm(poses[:, self.T - 1, :2] - self.path[index, :2], axis = 1) * self.error_w
        #add orientation error cost
        r_index = index - 4
        if r_index <0:
            r_index=0
        error_cost_rot = np.abs(np.sin(poses[:, self.T - 1, 2] - self.path[r_index, 2])) * self.error_th
        return collision_cost + error_cost + error_cost_rot

    def check_collisions_in_map(self, poses):
        '''
        check_collisions_in_map is a collision checker that determines whether a set of K * T poses
            are in collision with occupied pixels on the map.
        input:
            poses (K * T x 3) - poses to check for collisions on
        output:
            collisions - a (K * T x 1) float vector where 1.0 signifies collision and 0.0 signifies
                no collision for the input pose with corresponding index.
        '''

        self.world2map(poses, out=self.scaled)

        L = self.car_length
        W = self.car_width

        # Specify specs of bounding box
        bbox = np.array([
            [L / 2.0, W / 2.0],
            [L / 2.0, -W / 2.0],
            [-L / 2.0, W / 2.0],
            [-L / 2.0, -W / 2.0]
        ]) / (self.map.info.resolution)

        x = np.tile(bbox[:, 0], (len(poses), 1))
        y = np.tile(bbox[:, 1], (len(poses), 1))

        xs = self.scaled[:, 0]
        ys = self.scaled[:, 1]
        thetas = self.scaled[:, 2]

        c = np.resize(np.cos(thetas), (len(thetas), 1))
        s = np.resize(np.sin(thetas), (len(thetas), 1))

        self.bbox_map[:, 0] = (x * c - y * s) + np.tile(np.resize(xs, (len(xs), 1)), 4)
        self.bbox_map[:, 1] = (x * s + y * c) + np.tile(np.resize(ys, (len(ys), 1)), 4)

        bbox_idx = self.bbox_map.astype(int)

        self.perm[:] = 0
        self.perm = np.logical_or(self.perm, self.perm_reg[bbox_idx[:, 1, 0], bbox_idx[:, 0, 0]])
        self.perm = np.logical_or(self.perm, self.perm_reg[bbox_idx[:, 1, 1], bbox_idx[:, 0, 1]])
        self.perm = np.logical_or(self.perm, self.perm_reg[bbox_idx[:, 1, 2], bbox_idx[:, 0, 2]])
        self.perm = np.logical_or(self.perm, self.perm_reg[bbox_idx[:, 1, 3], bbox_idx[:, 0, 3]])

        return self.perm.astype(float)

    def get_map(self):
        '''
        get_map is a utility function which fetches a map from the map_server
        output:
            map_msg - a GetMap message returned by the mapserver
        '''
        srv_name = rospy.get_param("static_map", default="/static_map")
        rospy.logdebug("Waiting for map service")
        rospy.wait_for_service(srv_name)
        rospy.logdebug("Map service started")

        map_msg = rospy.ServiceProxy(srv_name, GetMap)().map
        return map_msg

    def load_permissible_region(self, map):
        '''
        load_permissible_region uses map data to compute a 'permissible_region'
            matrix given the map information. In this matrix, 0 is permissible,
            1 is not.
        input:
            map - GetMap message
        output:
            pr - permissible region matrix
        '''
        map_data = np.array(map.data)
        array_255 = map_data.reshape((map.info.height, map.info.width))
        pr = np.zeros_like(array_255, dtype=bool)

        # Numpy array of dimension (map_msg.info.height, map_msg.info.width),
        # With values 0: not permissible, 1: permissible
        pr[array_255 == 0] = 1
        pr = np.logical_not(pr)  # 0 is permissible, 1 is not

        return pr

    def world2map(self, poses, out):
        '''
        world2map is a utility function which converts poses from global
            'world' coordinates (ROS world frame coordinates) to 'map'
            coordinates, that is pixel frame.
        input:
            poses - set of X input poses
            out - output buffer to load converted poses into
        '''
        out[:] = poses
        # translation
        out[:, 0] -= self.map_x
        out[:, 1] -= self.map_y

        # scale
        out[:, :2] *= (1.0 / float(self.map.info.resolution))

        # we need to store the x coordinates since they will be overwritten
        temp = np.copy(out[:, 0])
        out[:, 0] = self.map_c * out[:, 0] - self.map_s * out[:, 1]
        out[:, 1] = self.map_s * temp + self.map_c * out[:, 1]
        out[:, 2] += self.map_angle

    def update_mpc_params_tuning(self, params_dict):
        '''
        updates params in accordance to the params dictionary. If param is not in dictionary, it is not changed.
        '''
        self.min_delta = float(params_dict.get('min_delta', self.min_delta))
        self.max_delta = float(params_dict.get('max_delta', self.max_delta))
        self.K = int(params_dict.get('K', self.K))
        self.T = int(params_dict.get('T', self.T))
        self.speed = float(params_dict.get('speed', self.speed))
        self.finish_threshold = float(params_dict.get('finish_threshold', self.finish_threshold))
        self.exceed_threshold = float(params_dict.get('exceed_threshold', self.exceed_threshold))
        self.waypoint_lookahead = float(params_dict.get('waypoint_lookahead', self.waypoint_lookahead))
        self.collision_w = float(params_dict.get('collision_w', self.collision_w))
        self.error_w = float(params_dict.get('error_w', self.error_w))
        self.error_th = float(params_dict.get('error_th', self.error_th))


    def mpc_params_generator(self, **kwargs):
        '''
        mpc_params_generator iterates through param space to find best values 
            that result in most accurate path tracking. Each time this function is called,
            it yields the next possible parameter combination. By default, this only tunes
            K through np.arange(51,85,2) and T through np.arange(5,25)}.
        input:
            kwargs - Keyword arguments corresponding to MPC parameters.
                Each argument's key is the parameter name as a string,
                and the value is the range of values to iterate the controller 
                through. The value type is provided below, but keep in mind that 
                it is a list of these numbers that should by inputted.
                Supported parameters include:
                - 'min_delta': float, the minimum steering angle in radians.
                - 'max_delta': float, the maximum steering angle in radians.
                - 'K': int, the number of control trajectories. Will overwrite default.
                - 'T': int, the time horizon for predictions. Will overwrite default.
                - 'speed': float, the reference speed in meters per second.
                - 'finish_threshold': float, the distance within which the target is considered reached.
                - 'exceed_threshold': float, the threshold distance at which the path is considered exceeded.
                - 'waypoint_lookahead': float, the lookahead distance for waypoint tracking.
                - 'collision_w': float, the weight of the collision cost in the cost function.
                - 'error_w': float, the weight of the tracking error cost in the cost function.
                - 'error_th': float, the weight of the heading error cost in the cost function.

                Example usage with default values: mpc_params_generator()
                Example usage with additional params: mpc_params_generator(speed = np.linspace(0.1, 0.6, 4), K = np.arange(40,50))
        output:
            params_dict - Dictionary of param combo of most recent call
        
        '''
        default_param_space ={'K': np.arange(51,85,2),
                              'T': np.arange(5,25)}
        
        param_space = {**default_param_space, **kwargs}

        for combo in itertools.product(*(param_space[key] for key in param_space)):
            params_dict = dict(zip(param_space.keys(), combo))
            self.update_mpc_params_tuning(params_dict)
            self.reset_state()
            yield params_dict

