import rospy
import threading

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, PointStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from std_msgs.msg import Header, Float32, String
from std_srvs.srv import Empty as SrvEmpty
from mushr_rhc.msg import XYHVPath, XYHV
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
import hexNfloat

import mpc
import nonlinear
import pid
import purepursuit
import utils

import numpy as np
from mpc import timed_pose2d
from trajectory_class import jeeho_traj, interpolate_pose

import time

controllers = {
    "PID": pid.PIDController,
    "PP": purepursuit.PurePursuitController,
    "NL": nonlinear.NonLinearController,
    "MPC": mpc.ModelPredictiveController,
}

class ExcThread(threading.Thread):

    def __init__(self, bucket):
        threading.Thread.__init__(self)
        self.bucket = bucket

    def run(self):
        try:
            raise Exception('An error occured here.')
        except Exception:
            self.bucket.put(sys.exc_info())

class ControlNode:
    def __init__(self, name):
        self.ackermann_msg_id = 0        
        self.path_event = threading.Event()
        self.reset_lock = threading.Lock()
        self.ready_event = threading.Event()
        self.start(name)

        # for execution time
        self.exec_time = 0
        

    def start(self, name):
        rospy.init_node(name, anonymous=True, disable_signals=True)

        self.setup_pub_sub()
        self.load_controller()
        self.ready_event.set()

        rate = rospy.Rate(50)
        self.inferred_pose = None
        self.inferred_pose_time = None
        
        print("Control Node Initialized") #when map is not present, it waits indefinately. probably need to handle callback triggered before this

        while not rospy.is_shutdown():
            #start_time = time.time()

            self.path_event.wait()
            self.reset_lock.acquire()
            ip = self.inferred_pose
            ip_time = self.inferred_pose_time #ROS time
            
            try:
                if ip is not None and self.controller.ready():
                    if(not self.controller.is_traj): #original tracking
                        index = self.controller.get_reference_index(ip)
                        pose = self.controller.get_reference_pose(index)
                        error = self.controller.get_error(ip, index)
                        cte = error[1]

                        self.publish_selected_pose(pose)
                        self.publish_cte(cte)

                        next_ctrl, next_est = self.controller.get_control(ip, index)
                        if next_ctrl is not None:
                            self.publish_ctrl(next_ctrl)
                        if self.controller.path_complete(ip, error):
                            print("Goal reached")
                            self.path_event.clear()
                            print(ip, error)
                            self.controller._ready = False

                        self.last_est = next_est

                    else: #use timed path
                        #choose index by time. (i.e. choose the closest pose by time among ones comes after current time)
                        ip_time = ip_time.to_sec()
                        index = self.controller.get_reference_index_by_time(ip_time) #current time from ref timestamp

                        #set steering limit by mode
                        if self.controller.mode_list[index] == 'p' and self.controller.is_pushing == False:
                            self.controller.min_delta = self.controller.min_steer_push
                            self.controller.max_delta = self.controller.max_steer_push
                            self.controller.trajs = self.controller.get_control_trajectories()
                            self.controller.is_pushing = True
                            print("Switch to Pushing")
                        elif self.controller.mode_list[index] == 'n' and self.controller.is_pushing == True:
                            self.controller.min_delta = self.controller.min_steer_nonpush
                            self.controller.max_delta = self.controller.max_steer_nonpush
                            self.controller.trajs = self.controller.get_control_trajectories()
                            self.controller.is_pushing = False
                            print("Switch to Non-Pushing")

                        ref_pose = self.controller.get_reference_pose_traj(index)
                        error = self.controller.get_error_traj(ip, index)
                        self.publish_selected_pose2d(ref_pose)
                        
                        # interpolated pose
                        i_pose = ref_pose
                        
                        if(index != 0):
                            cur_time_rel = ip_time - self.controller.trajectory.ref_time # delta time from header timestamp
                            if(cur_time_rel < self.controller.trajectory.traj[-1].time_rel):
                                #print("traj: " + str(self.controller.trajectory.ref_time))
                                #print("cur: " + str(cur_time_rel))
                                i_pose = interpolate_pose(self.controller.trajectory.traj[index-1],ref_pose, cur_time_rel)


                        #print(i_pose.x)
                        #temp
                        #print(ip, error)
                        
                        next_ctrl,min_est = self.controller.get_control(ip, index,True,i_pose)
                        

                        global last_cmd
                        last_cmd = next_ctrl #for future use
                        
                        if next_ctrl is not None:
                            self.publish_ctrl(next_ctrl)
                        if self.controller.path_complete_traj(index, error):
                            # stop measuring exec time
                            time_exec = (time.time() - self.exec_time) # in seconds
                            print("Goal reached")
                            self.path_event.clear()
                            print(ip, error)
                            # publish exec_time
                            self.pub_execution_time.publish(time_exec)

                            self.controller._ready = False
                        #publish interpolated point
                        
                        self.publish_interpolated_pose2d(i_pose)

            except:
                pass

            self.reset_lock.release()
            #end_time = time.time()
            #print(f'{(end_time-start_time)*1000:.3f} ms')
            rate.sleep()

    def shutdown(self):
        rospy.signal_shutdown("shutting down from signal")
        self.path_event.clear()
        self.ready_event.clear()
        exit(0)

    def load_controller(self):
        self.controller_type = rospy.get_param("~controller/type", default="MPC")
        print(self.controller_type)
        self.controller = controllers[self.controller_type]()

    def setup_pub_sub(self):
        rospy.Service("~reset/hard", SrvEmpty, self.srv_reset_hard)
        rospy.Service("~reset/state", SrvEmpty,  self.srv_reset_state)
        rospy.Service("~reset/params", SrvEmpty, self.srv_reset_params)
        
        # namespace for topic/param names
        robot_prefix = rospy.get_param("~car_name",default='/mushr2')

        rospy.Subscriber("/initialpose",
                PoseWithCovarianceStamped, self.cb_init_pose, queue_size=1)

        rospy.Subscriber(
            "/clicked_point",
            PointStamped,
            self.clicked_point_cb,
            queue_size=1
        )

        rospy.Subscriber(
            #"/car/goal", PoseStamped, self.cb_goal, queue_size=1
            (robot_prefix + "/goal"), PoseStamped, self.cb_goal, queue_size=1
        )

        #rospy.Subscriber("/car/global_planner/path",
        rospy.Subscriber((robot_prefix + "/global_planner/path"),
                Path, self.cb_path, queue_size=1)

        rospy.Subscriber((robot_prefix + "/planned_trajectory"), Path, self.cb_trajectory, queue_size=1)

        # Subscriber for path info string
        rospy.Subscriber((robot_prefix + "/planned_path_serialized"), String, self.cb_path_str, queue_size=1)

        #rospy.Subscriber(rospy.get_param("~pose_cb",default=robot_prefix+'/particle_filter/inferred_pose'),
        rospy.Subscriber(rospy.get_param("~pose_cb",default='/natnet_ros/mushr2/pose'),
                         PoseStamped, self.cb_pose, queue_size=10)

        self.rp_ctrls = rospy.Publisher(
            #"/car/mux/ackermann_cmd_mux/input/navigation",
            (robot_prefix + "/mux/ackermann_cmd_mux/input/navigation"),
            AckermannDriveStamped, queue_size=2
        )

        self.rp_cte = rospy.Publisher(
            #rospy.get_param(
            #    "~cte_viz_topic",
            #    default="/controller/cte"
            #),
            (robot_prefix + rospy.get_param("~cte_viz_topic", default="/controller/cte")),
            Float32, queue_size=2
        )

        self.rp_waypoints = rospy.Publisher(
            #"/controller/path/waypoints",
            (robot_prefix + "/controller/path/waypoints"),
            Marker, queue_size=2
        )

        self.rp_waypoint = rospy.Publisher(
            #"/controller/path/selected_waypoint",
            (robot_prefix + "/controller/path/selected_waypoint"),
            PoseStamped, queue_size=2
        )

        self.rp_ipoint = rospy.Publisher(
            #"/controller/path/selected_waypoint",
            (robot_prefix + "/controller/path/interpolated_waypoint"),
            PoseStamped, queue_size=2
        )


        self.rp_path_viz = rospy.Publisher(
            #"/controller/path/poses",
            (robot_prefix + "/controller/path/poses"),
            PoseArray, queue_size=2
        )

        self.nav_path_viz = rospy.Publisher(
            #"/controller/path/poses",
            (robot_prefix + "/controller/path_unpacked"),
            Path, queue_size=2
        )

        self.pub_execution_time = rospy.Publisher(
            (robot_prefix + "/execution_time"),
            Float32, queue_size=1 #may change to serialized str
        )


    def srv_reset_hard(self, msg):
        '''
        Hard reset does a complete reload of the controller.
        '''
        rospy.loginfo("Start hard reset")
        self.reset_lock.acquire()
        self.load_controller()
        self.reset_lock.release()
        rospy.loginfo("End hard reset")
        return []

    def srv_reset_params(self, msg):
        '''
        Param reset resets parameters of the controller. Useful for iterative tuning.
        '''
        rospy.loginfo("Start param reset")
        self.reset_lock.acquire()
        self.controller.reset_params()
        self.reset_lock.release()
        rospy.loginfo("End param reset")
        return []

    def srv_reset_state(self, msg):
        '''
        State reset resets state dependent variables, such as accumulators in PID control.
        '''
        rospy.loginfo("Start state reset")
        self.reset_lock.acquire()
        self.controller.reset_state()
        self.reset_lock.release()
        rospy.loginfo("End state reset")
        return []

    def cb_odom(self, msg):
        self.inferred_pose = utils.rospose_to_posetup(msg.pose.pose)

    def cb_path(self, msg):
        
        print("Got path!")
        trajectory = XYHVPath()
        for i in range(len(msg.poses)):
            point = XYHV()
            point.x = msg.poses[i].pose.position.x
            point.y = msg.poses[i].pose.position.y
            point.h = utils.rosquaternion_to_angle(msg.poses[i].pose.orientation)
            if(i != len(msg.poses) - 1):
                point.v = 1
            else:
                point.v = 0
            trajectory.waypoints.append(point)
        path = trajectory.waypoints
        # self.visualize_path(path)
        self.controller.set_path(path)
        self.path_event.set()
        print("Path set")
        return True
    
        """
        print("Timed path received")
        trajectory = jeeho_traj()
        trajectory.from_nav_path(msg)
        self.controller.set_trajectory(trajectory)
        self.path_event.set()
        print("Timed Path set")
        return True
        """
    
    def cb_trajectory(self, msg):
        print("Trajectory received")
        trajectory = jeeho_traj()
        trajectory.from_nav_path(msg)
        self.controller.set_trajectory(trajectory)
        self.path_event.set()
        # start measuring execution time
        self.exec_time = time.time()
        print("Trajectory set")
        self.controller.min_delta = self.controller.min_steer_nonpush
        self.controller.max_delta = self.controller.max_steer_nonpush
        self.controller.trajs = self.controller.get_control_trajectories()
        self.controller.is_pushing = False
        print("Switch to Non-Pushing")
        return True
    

    def path_str_to_nav_path(self, path_str_in:str, timing_str_in:str , data_delim:str, elem_delim:str)->Path:
        # into list
        poses_str_list = path_str_in.split(data_delim)
        timing_str_list = timing_str_in.split(data_delim)

        #gen nav path
        nPath = Path()
        time_ref = rospy.Time.now()
        nPath.header.stamp = time_ref
        for n in range(len(poses_str_list)):
            p = poses_str_list[n]
            pose_sp = p.split(elem_delim)
            temp_ps = PoseStamped()
            temp_ps.pose.position.x = hexNfloat.hex_to_float(pose_sp[0])
            temp_ps.pose.position.y = hexNfloat.hex_to_float(pose_sp[1])

            yaw = hexNfloat.hex_to_float(pose_sp[2])
            #to quaternion
            temp_ps.pose.orientation = utils.angle_to_rosquaternion(yaw)

            #timing
            time_stamp = hexNfloat.hex_to_float(timing_str_list[n])
            temp_ps.header.stamp = time_ref + rospy.Duration(time_stamp)

            nPath.poses.append(temp_ps)

        return nPath


    def cb_path_str(self, msg:String):
        group_delim = "!";
        data_delim = ";";
        elem_delim = ",";
        print("Serialized Path Info Received")
        # split path and mode
        path_str_sp = msg.data.split(group_delim)

        # reconstruct nav_msgs/Path
        nPath = self.path_str_to_nav_path(path_str_sp[0],path_str_sp[2],data_delim,elem_delim)
        nPath.header.frame_id = path_str_sp[3]

        # get mode list
        mode_list = path_str_sp[1].split(data_delim)
        self.controller.set_pushing_status(mode_list)

        # trigger run
        self.cb_trajectory(nPath)

        #vis
        self.nav_path_viz.publish(nPath)


    def cb_goal(self, msg):
        self.path = None
        goal = utils.rospose_to_posetup(msg.pose)
        self.controller.set_goal(goal)
        self.path_event.set()
        print("goal set", goal)

    def clicked_point_cb(self, msg):
        self.path = None
        goal = utils.rospoint_to_posetup(msg)
        self.controller.set_goal(goal)
        self.path_event.set()
        print("goal set", goal)

    def cb_pose(self, msg):
        self.inferred_pose = [
            msg.pose.position.x,
            msg.pose.position.y,
            utils.rosquaternion_to_angle(msg.pose.orientation)]
        

        """
        if not hasattr(self, 'last_est'):
            self.last_est = self.inferred_pose

        if not hasattr(self, 'last_ts'):
            self.last_ts = msg.header.stamp
        
        if 'last_cmd' not in globals():
            global last_cmd
            last_cmd = [0,0]

        dTime = (msg.header.stamp - self.last_ts).to_sec()
        max_disp = dTime * 0.4 * 5 # margin
        #print(max_disp)

        speed = last_cmd [0]
        steering_angle = last_cmd [1]
        x_dot = speed * np.cos(self.last_est [2]) * dTime
        y_dot = speed * np.sin(self.last_est [2]) * dTime
        theta_dot = ((speed * np.tan(steering_angle)) / (0.33)) * dTime

        kin_est = [self.last_est[0] + x_dot, self.last_est[1] + y_dot, self.last_est[2] + theta_dot]
        

        if(np.sqrt(pow(self.inferred_pose[0]-kin_est[0],2) + pow(self.inferred_pose[1]-kin_est[1],2))>max_disp):
            print(np.sqrt(pow(self.inferred_pose[0]-kin_est[0],2) + pow(self.inferred_pose[1]-kin_est[1],2)))
            # estimate by kinematics
            self.inferred_pose = kin_est
            self.last_est = kin_est    

        else:
            self.last_est = self.inferred_pose           
            

        # #handle occasional popping
        #if abs(self.last_est[2] - self.inferred_pose[2])>4:
        #    print('trigger')
        #    self.inferred_pose = self.last_est

        #else:
        #    print("lift")
        """            

        self.inferred_pose_time = msg.header.stamp
        self.last_ts = msg.header.stamp

    def publish_ctrl(self, ctrl):
        assert len(ctrl) == 2
        ctrlmsg = AckermannDriveStamped()
        ctrlmsg.header.stamp = rospy.Time.now()
        ctrlmsg.header.seq = self.ackermann_msg_id
        ctrlmsg.drive.speed = ctrl[0]
        ctrlmsg.drive.steering_angle = ctrl[1]
        self.rp_ctrls.publish(ctrlmsg)
        self.ackermann_msg_id += 1

    def visualize_path(self, path):
        marker = self.make_marker(path[0], 0, "start")
        self.rp_waypoints.publish(marker)
        poses = []
        for i in range(1, len(path)):
            p = Pose()
            p.position.x = path[i].x
            p.position.y = path[i].y
            p.orientation = utils.angle_to_rosquaternion(path[i].h)
            poses.append(p)
        pa = PoseArray()
        pa.header = Header()
        pa.header.stamp = rospy.Time.now()
        pa.header.frame_id = "map"
        pa.poses = poses
        self.rp_path_viz.publish(pa)

    def make_marker(self, config, i, point_type):
        marker = Marker()
        marker.header = Header()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "map"
        marker.ns = str(config)
        marker.id = i
        marker.type = Marker.CUBE
        marker.pose.position.x = config.x
        marker.pose.position.y = config.y
        marker.pose.orientation.w = 1
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        # marker.lifetime = { sec: 0, nsec: 0 }
        if point_type == "waypoint":
            marker.color.b = 1.0
        else:
            marker.color.g = 1.0

        return marker

    def publish_selected_pose(self, pose):
        p = PoseStamped()
        p.header = Header()
        p.header.stamp = rospy.Time.now() - rospy.Duration(0.1) # set to in the past to visualize longer
        #p.header.frame_id = "map_mocap"
        p.header.frame_id = "map"
        p.pose.position.x = pose[0]
        p.pose.position.y = pose[1]
        p.pose.orientation = utils.angle_to_rosquaternion(pose[2])
        self.rp_waypoint.publish(p)

    #todo: integrate common parse of timed_pose2d
    def publish_selected_pose2d(self, pose):
        p = PoseStamped()
        p.header = Header()
        p.header.stamp = rospy.Time.now() - rospy.Duration(0.1) # set to in the past to visualize longer
        p.header.frame_id = "map_mocap"
        #p.header.frame_id = "map"
        p.pose.position.x = pose.x
        p.pose.position.y = pose.y
        p.pose.orientation = utils.angle_to_rosquaternion(pose.th)
        self.rp_waypoint.publish(p)

    def publish_interpolated_pose2d(self, pose):
        p = PoseStamped()
        p.header = Header()
        p.header.stamp = rospy.Time.now() - rospy.Duration(0.1) # set to in the past to visualize longer
        p.header.frame_id = "map_mocap"
        #p.header.frame_id = "map"
        p.pose.position.x = pose.x
        p.pose.position.y = pose.y
        p.pose.orientation = utils.angle_to_rosquaternion(pose.th)
        self.rp_ipoint.publish(p)

    def publish_cte(self, cte):
        self.rp_cte.publish(Float32(cte))

    def cb_init_pose(self, pose):
        self.path_event.set()
