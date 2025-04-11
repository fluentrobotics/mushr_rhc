import struct
import numpy as np

def float2binarystr(f_in: float) -> bytes:
    """
    Convert a float to its 4-byte binary representation.
    """
    return struct.pack('f', f_in)

def binarystr2float(b_in: bytes) -> float:
    """
    Convert a 4-byte binary representation back to a float.
    """
    return struct.unpack('f', b_in)[0]

def bool2binarystr(b_in: bool) -> bytes:
    """
    Convert a boolean to its binary representation.
    't' if True, 'f' if False.
    """
    return b"t" if b_in else b"f"

def binarystr2bool(b_in: bytes) -> bool:
    """
    Convert the binary representation back to a boolean.
    Expects b"t" for True and b"f" for False.
    """
    if b_in == b"t":
        return True
    elif b_in == b"f":
        return False
    else:
        raise ValueError("Invalid boolean binary string")

class trajectory_elem:
    def __init__(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0,
                 ref_vel: float = 0.0, time: float = -1.0, is_pushing: bool = False):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.ref_vel = ref_vel
        self.time = time
        self.is_pushing = is_pushing

    def __array__(self, dtype=None):
        return np.array([self.x, self.y, self.yaw], dtype=dtype)

    def print(self):
        """
        Prints the trajectory element. (Note: is_pushing is now part of the data,
        but for consistency with the C++ print function we do not display it.)
        """
        print(f"(x={self.x}, y={self.y}, yaw={self.yaw}, ref_vel={self.ref_vel}, time={self.time}, is_pushing={self.is_pushing})", end="")

class ReloPush_trajectory:
    def __init__(self, serialized_trajectory: bytes = None):
        # Define delimiters and header as bytes
        self.header_delim = b"!!!"
        self.elem_delim = b";;;"
        self.var_delim = b",,,"
        self.header = b"t"  # header for trajectory
        self.time_zero = 0.0
        self.trajectory_points = []  # list of trajectory_elem
        
        # If a serialized trajectory is provided, deserialize it.
        if serialized_trajectory is not None:
            self.deserialize(serialized_trajectory)

    def append_waypoint(self, wpt: trajectory_elem):
        self.trajectory_points.append(wpt)

    def serialize(self) -> bytes:
        """
        Serializes the trajectory into a bytes string.
        Format: header!time_zero;x,y,yaw,ref_vel,time,is_pushing;...;
        """
        # Start with header, header delimiter, time_zero and element delimiter.
        out_bytes = self.header + self.header_delim + float2binarystr(self.time_zero) + self.elem_delim

        for i, point in enumerate(self.trajectory_points):
            temp_bytes = (
                float2binarystr(point.x) + self.var_delim +
                float2binarystr(point.y) + self.var_delim +
                float2binarystr(point.yaw) + self.var_delim +
                float2binarystr(point.ref_vel) + self.var_delim +
                float2binarystr(point.time) + self.var_delim +
                bool2binarystr(point.is_pushing)
            )
            out_bytes += temp_bytes
            if i != len(self.trajectory_points) - 1:
                out_bytes += self.elem_delim

        return out_bytes

    def deserialize(self, serialized_traj: bytes):
        """
        Deserializes the given bytes string into the trajectory.
        Expects format: header!time_zero;x,y,yaw,ref_vel,time,is_pushing;...;
        """
        # Split using the header delimiter.
        header_sp = serialized_traj.split(self.header_delim, 1)
        if header_sp[0] == self.header:
            # Split the rest into elements.
            elems = header_sp[1].split(self.elem_delim)
            if len(elems) < 1:
                raise ValueError("Serialized data does not contain time_zero")
            # First element is time_zero.
            self.time_zero = binarystr2float(elems[0])
            # Process each trajectory element from index 1 onward.
            for elem in elems[1:]:
                var_sp = elem.split(self.var_delim)
                if len(var_sp) < 6:
                    raise ValueError("Serialized element does not have enough fields")
                x_in = binarystr2float(var_sp[0])
                y_in = binarystr2float(var_sp[1])
                yaw_in = binarystr2float(var_sp[2])
                ref_vel_in = binarystr2float(var_sp[3])
                time_in = binarystr2float(var_sp[4])
                is_pushing_in = binarystr2bool(var_sp[5])
                temp_elem = trajectory_elem(x_in, y_in, yaw_in, ref_vel_in, time_in, is_pushing_in)
                self.append_waypoint(temp_elem)
        else:
            raise ValueError("Unknown header in serialized trajectory.")

    def print(self):
        """
        Prints the entire trajectory.
        """
        print("Trajectory:")
        print("Time Zero:", self.time_zero)
        print("Waypoints:")
        for i, wpt in enumerate(self.trajectory_points):
            print(f"Waypoint {i}:", end=" ")
            wpt.print()
            print()  # Newline after each waypoint

# Function to interpolate between two angles (yaw)
def interpolate_yaw(th1, th2, t):
    angle_diff = np.arctan2(np.sin(th2 - th1), np.cos(th2 - th1))
    return th1 + t * angle_diff

def interpolate_pose_relopush(from_pose:trajectory_elem, to_pose:trajectory_elem, target_time) -> trajectory_elem:

    # Compute the interpolation factor
    t_factor = (target_time - from_pose.time) / (to_pose.time - from_pose.time)

    # Interpolate x and y
    x = np.interp(target_time, [from_pose.time, to_pose.time], [from_pose.x, to_pose.x])
    y = np.interp(target_time, [from_pose.time, to_pose.time], [from_pose.y, to_pose.y])

    # Interpolate th (yaw)
    yaw = interpolate_yaw(from_pose.yaw, to_pose.yaw, t_factor)

    # Create and return the interpolated pose
    ipose = trajectory_elem()
    ipose.x = x
    ipose.y = y
    ipose.yaw = yaw
    ipose.ref_vel = from_pose.ref_vel
    ipose.is_pushing = from_pose.is_pushing
    #ipose.time_abs = from_pose.time_abs - from_pose.time + target_time
    ipose.time = target_time # assume the same time_zero
    return ipose
