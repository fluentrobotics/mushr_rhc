import numpy as np
import rospy
import utils
import time
import math
from controller import BaseController
from geometry_msgs.msg import PoseStamped #todo: find a way to bring it out of this file
from nav_msgs.msg import Path
from nav_msgs.srv import GetMap

class timed_pose2d:
    def __init__(self) -> None:
        """
        Pose object with time
        time_abs is the abosolute time (i.e. ROS time)
        time_rel is the relative time (from a ref timestamp)
        """
        self.x=0
        self.y=0
        self.th=0
        self.time_abs=-1
        self.time_rel=-1

    def from_poseStamped(self, ps_in:PoseStamped, ref_time=0):
        
        self.x = ps_in.pose.position.x
        self.y = ps_in.pose.position.y
        self.th = utils.rosquaternion_to_angle(ps_in.pose.orientation)
        self.time_abs = ps_in.header.stamp.to_sec()
        self.time_rel = self.time_abs - ref_time

    def from_numpy(self, np_in:np.ndarray):
        self.x = np_in[0]
        self.y = np_in[1]
        self.th = np_in[2]

    def to_numpy(self):
        out_np = np.array([self.x, self.y, self.th])
        return out_np


class jeeho_traj:
    def __init__(self) -> None:
        self.traj = []
        self.ref_time = 0
        self.frame = ""

    def from_nav_path(self, nav_path_msg:Path):
        """
        Parse nav_msgs/path
        Each entry is a geometry_msgs/poseStamped
        Use the highest header as the source of ref timestamp
        """

        self.traj.clear()

        self.ref_time = nav_path_msg.header.stamp.to_sec()
        self.frame = nav_path_msg.header.frame_id

        skip_first = 2 #todo: make this responsive
        
        for ind in range(skip_first,len(nav_path_msg.poses)):
            #temporary manipulation of timestamp
            pp = timed_pose2d()
            pp.from_poseStamped(nav_path_msg.poses[ind],self.ref_time)
            #pp.time_rel = ind*0.265
            self.traj.append(pp)

            #print reference vel
            if(ind > skip_first):
                piv_ind = ind-skip_first
                #dl = math.sqrt( (self.traj[piv_ind].x - self.traj[piv_ind-1].x)**2 + (self.traj[piv_ind].y - self.traj[piv_ind-1].y)**2 )
                #dt = self.traj[piv_ind].time_rel - self.traj[piv_ind-1].time_rel
                #print("ref_vel " + str(ind) + str(": ") + str(dl/dt))

    def to_numpy(self) -> np.array:
        traj_len = len(self.traj)
        out_np = np.zeros((traj_len,3))
        
        for ind in range(traj_len):
            out_np[ind] = np.array([self.traj[ind].x, self.traj[ind].y, self.traj[ind].th])

        return out_np


class ModelPredictiveController(BaseController):
    def __init__(self):
        super(ModelPredictiveController, self).__init__()

        self.reset_params()
        self.reset_state()

    def get_reference_index(self, pose, use_lookahead = True):
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
            
            if(use_lookahead):
                while(dist[index] < self.waypoint_lookahead and index <= len(self.path) - 2):
                    index += 1
                    index = min(index, len(self.path)-1)
             
            if(len(self.path)==1):
                self.index = 0
                return 0  # handle special case of a simple go-to pose
            self.index = index
            #print('returning ind: ' + str(index))
            return index

    def get_reference_index_by_time(self, cur_time_abs, start_ind_in:int=0):
        """
        get reference point by timestamp
        find the closest pose among ones come after current timestamp      
        input time is relateive to the reference timestamp of the msg

        return the last index if all of the pose time are behind the current time
        """
        cur_time_rel = cur_time_abs - self.trajectory.ref_time
        traj_length = len(self.trajectory.traj)
        out_ind = traj_length -1 #last index
        start_ind = 0
        #is_found = False

        if(start_ind_in < traj_length):
            start_ind = start_ind_in #use start ind only when it's in the valid range
        with self.path_lock:
            #find the first index where time stamp is larger than the current
            #currently a linear serach with starting info
            for ind in range(start_ind,traj_length):
                if(self.trajectory.traj[ind].time_rel > cur_time_rel):
                    out_ind = ind
                    break
                else:
                    pass
            #end of loop
            

            return out_ind


    def get_control(self, pose, index, jeeho_mode=False):
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

        # change tracking velocity based on error on x-axis i.r.t. robot
        ref_pose = self.trajectory.traj[index]
        error_x, _ = self.error_xy(pose,ref_pose)
        error_th = ref_pose.th - pose[2]

        #arbitrary Kx
        Kx = 0.5
        #keeping distance
        kd = 0.05
        tracking_speed = self.speed*math.cos(error_th) + Kx * (error_x-kd) #kanayama linear velocity
        #print(tracking_speed)
        #for analysis
        #measure delta time
        #time_now = time.time()
        #self.time_analyze = time_now
        #print("time:" +str(self.time_analyze) + ";e_x:"+str(error_x)+";error_th:"+str(error_th)+";track_vel:"+str(tracking_speed))


        speed_sign = np.array([-1*tracking_speed, 0 ,1*tracking_speed])
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
            
            if(not jeeho_mode):
                costs = self.apply_cost(rollouts, index)  # get the cost for each roll out

            else:
                costs = self.apply_cost_jeeho(rollouts, index, pose)  # get the cost for each roll out

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
            self.min_delta = float(rospy.get_param("trajgen/min_delta", -0.384))
            self.max_delta = float(rospy.get_param("trajgen/max_delta", 0.384))

            self.K = int(rospy.get_param("mpc/K", 89))
            self.T = int(rospy.get_param("mpc/T", 12))

            self.speed = float(rospy.get_param("mpc/speed", 0.4))
            
            #self.finish_threshold = float(rospy.get_param("mpc/finish_threshold", 0.5))
            self.finish_threshold = float(rospy.get_param("mpc/finish_threshold", 0.1))
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

            #x error (might be a good idea to have this value varying by dist error)
            self.w_x_err = float(rospy.get_param("mpc/w_x_err", 1.0))
            #y error
            self.w_y_err = float(rospy.get_param("mpc/w_y_err", 5.0))

            self.car_length = float(rospy.get_param("mpc/car_length", 0.7))
            self.car_width = float(rospy.get_param("mpc/car_width", 0.4))

            #for analyze
            self.time_analyze = 0

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
    

    def error_xy(self, cur_pose, ref_pose, use_manual_th = False, manual_th = 0):

        #took 21.5 ns to check time on my PC
        if isinstance(ref_pose,np.ndarray):
            temp_obj = timed_pose2d()
            temp_obj.from_numpy(ref_pose)
            ref_pose = temp_obj

        theta = 0
        if(not use_manual_th):
            theta = cur_pose[2]
        else:
            theta = manual_th

        c, s = np.cos(theta), np.sin(theta)
        #R = np.array([(c, s), (-s, c)])

        R = np.array([[c,s],
                    [-s,c]])

        delta_xy_irt_world = np.array([ref_pose.x - cur_pose[0],ref_pose.y-cur_pose[1]])
        delta_xy_irt_robot = np.matmul(R,delta_xy_irt_world.T)

        return abs(delta_xy_irt_robot[0]), abs(delta_xy_irt_robot[1])
    
    def apply_cost_jeeho(self, poses, index, cur_pose):
        '''
        rollouts (K,T,3) - poses of each rollout
        index    (int)   - reference index in path
        '''
        all_poses = poses.copy()
        all_poses.resize(self.K * self.T, 3)
        collisions = self.check_collisions_in_map(all_poses)
        collisions.resize(self.K, self.T)
        collision_cost = collisions.sum(axis=1) * self.collision_w

        x_mat = poses[:,self.T-1, 0]
        y_mat = poses[:,self.T-1, 1]
        th_mat = poses[:,self.T-1, 2]

        ref_pose = self.trajectory.traj[index]

        x_err_mat = np.zeros((self.K))
        y_err_mat = np.zeros((self.K))
        y_path_err_mat = np.zeros((self.K))

        #current error from the cloeset point on path i.r.t. robot        
        closest_index = self.get_reference_index(cur_pose,use_lookahead = False)
        closest_pose = self.get_reference_pose(closest_index)

        #rotate all by robot angle
        for row in range(self.K):
            pivot_robot_pose2d = np.array([x_mat[row],y_mat[row],th_mat[row]]).T
            piv_err_x, piv_err_y = self.error_xy(pivot_robot_pose2d, ref_pose, use_manual_th = True, manual_th = cur_pose[2])
            
            #_, err_y_path_irt_robot = self.error_xy(pivot_robot_pose2d, closest_pose)

            x_err_mat[row] = piv_err_x
            y_err_mat[row] = piv_err_y
            #y_path_err_mat[row] = err_y_path_irt_robot

        # varying x_error weight
        ## L2 norm to referece pose
        # ref_pose_np = ref_pose.to_numpy()
        # l2_norm = np.linalg.norm(ref_pose_np, cur_pose)
        # current error from reference pose in y i.r.t. robot
        err_x_irt_robot, err_y_irt_robot = self.error_xy(cur_pose, ref_pose)

        self.w_x_err = err_y_irt_robot/(err_x_irt_robot)
        #print(self.w_x_err)

        x_err_cost = x_err_mat * self.w_x_err
        y_err_cost = y_err_mat * self.w_y_err       
        #y_path_err_cost = y_path_err_mat * self.w_y_err

        #distance error
        error_dist = np.linalg.norm(poses[:, self.T - 1, :2] - self.path[index, :2], axis = 1) * self.w_x_err
        #orientation error
        error_cost_rot = np.abs(np.sin(poses[:, self.T - 1, 2] - self.path[closest_index, 2])) * self.error_th


        return collision_cost + y_err_cost + error_cost_rot + error_dist

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
