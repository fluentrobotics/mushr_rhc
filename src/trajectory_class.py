import numpy as np

from geometry_msgs.msg import PoseStamped #todo: find a way to bring it out of this file
from nav_msgs.msg import Path
from nav_msgs.srv import GetMap
import utils


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
        self.time_abs = ps_in.header.stamp.to_sec() # ROS time
        self.time_rel = self.time_abs - ref_time # delta time from header timestamp

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
        self.ref_time = 0 #header timestamp
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

        skip_first = 0 #todo: make this responsive
        
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
    


# Function to interpolate between two angles (yaw)
def interpolate_th(th1, th2, t):
    angle_diff = np.arctan2(np.sin(th2 - th1), np.cos(th2 - th1))
    return th1 + t * angle_diff

def interpolate_pose(from_pose:timed_pose2d, to_pose:timed_pose2d, target_time) -> timed_pose2d:
    #if not (from_pose.time_abs <= target_time <= to_pose.time_abs):
    #    raise ValueError("Target time is outside the interval of the provided poses.")

    #print("target: " + str(target_time))
    #print("from_time: " + str(from_pose.time_rel))
    #print("to_time: " + str(to_pose.time_rel))

    # Compute the interpolation factor
    t_factor = (target_time - from_pose.time_rel) / (to_pose.time_rel - from_pose.time_rel)

    #print("from x: " + str(from_pose.x))
    #print("to x: " + str(to_pose.x) + "\n")
    #print("to_time: " + str(to_pose.time_rel) + "\n")

    # Interpolate x and y
    x = np.interp(target_time, [from_pose.time_rel, to_pose.time_rel], [from_pose.x, to_pose.x])
    y = np.interp(target_time, [from_pose.time_rel, to_pose.time_rel], [from_pose.y, to_pose.y])

    #print("x: " + str(x))
    #print("y: " + str(y))

    # Interpolate th (yaw)
    th = interpolate_th(from_pose.th, to_pose.th, t_factor)

    # Create and return the interpolated pose
    ipose = timed_pose2d()
    ipose.x = x
    ipose.y = y
    ipose.th = th
    ipose.time_abs = from_pose.time_abs - from_pose.time_rel + target_time
    ipose.time_rel = target_time
    return ipose