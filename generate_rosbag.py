# rosbags libraries
from rosbags.highlevel import AnyReader
from rosbags.rosbag1 import Writer
from rosbags.typesys import Stores, get_typestore

import numpy as np
from pathlib import Path
import json

# load external files
import spoofing_sim

def binary_to_xyz(binary):
    x = binary[:, 0:4].view(dtype=np.float32)
    y = binary[:, 4:8].view(dtype=np.float32)
    z = binary[:, 8:12].view(dtype=np.float32)
    return x.flatten(), y.flatten(), z.flatten()

def compare_reference(rosbag_time, dataframe_reference):
    reference_time = dataframe_reference['timestamp']
    x, y = np.array(dataframe_reference['x']), np.array(dataframe_reference['y'])

    # find corresponding timestamp
    time_diff = np.abs(reference_time - rosbag_time)
    corresponding_index = np.argmin(time_diff)
    return x[corresponding_index], y[corresponding_index]

def check_spoofing_condition(odom_x, odom_y, spoofer_x, spoofer_y, distance_threshold):
    dist_spoofer_to_robot = ((odom_x - spoofer_x) ** 2 + (odom_y - spoofer_y) ** 2) ** 0.5

    if dist_spoofer_to_robot <= distance_threshold:
        return True
    else:
        return False
    
def decide_spoofing_param(odom_x, odom_y, spoofer_x, spoofer_y):
    spoofing_angle = np.degrees(np.arctan2(spoofer_y - odom_y, spoofer_x - odom_x)) 
    return spoofing_angle

def create_pointcloud2(points, seq, stamp_ns, frame_id, typestore):
    blob = points.astype(np.float32).tobytes()
    data_array = np.frombuffer(blob, dtype=np.uint8)
    PointField = typestore.types['sensor_msgs/msg/PointField']
    fields = [
        PointField(name='x', offset=0, datatype=7, count=1),
        PointField(name='y', offset=4, datatype=7, count=1),
        PointField(name='z', offset=8, datatype=7, count=1),
    ]
    Header = typestore.types['std_msgs/msg/Header']
    Timestamp = typestore.types['builtin_interfaces/msg/Time']
    ros_time = Timestamp(sec=int(stamp_ns // 1e9), nanosec=int(stamp_ns % 1e9))
    header = Header(seq=seq, stamp=ros_time, frame_id=frame_id)
    PointCloud2 = typestore.types['sensor_msgs/msg/PointCloud2']
    return PointCloud2(header=header, height=1, width=points.shape[0], fields=fields,
                       is_bigendian=False, point_step=12, row_step=12 * points.shape[0],
                       data=data_array, is_dense=True)

def generate_main(spoofer_x, spoofer_y, reference_dataframe):
        
    with open('config_temp.json', 'r') as f:
        config = json.load(f)

    bag_path = Path(config['rosbag']['input_bag'])
    output_bag_path = Path(config['rosbag']['output_bag'])
    spoofing_mode = config['rosbag']['spoofing_mode']

    lidar_topic = config['rosbag']['lidar_topic']
    imu_topic = config['rosbag']['imu_topic']
    topic_length = int(config['rosbag']['topic_length'])
    lidar_freq = float(config['rosbag']['topic_freq'])
    distance_threshold = float(config['rosbag']['distance_threshold'])

    if output_bag_path.exists():
        output_bag_path.unlink()

    typestore = get_typestore(Stores.ROS1_NOETIC)

    start_time = None
    cnt = 0

    with AnyReader([bag_path], default_typestore=typestore) as reader:
        with Writer(output_bag_path) as writer:
            lidar_conn_out = writer.add_connection(lidar_topic, 'sensor_msgs/msg/PointCloud2', typestore=typestore)
            imu_conn_out = writer.add_connection(imu_topic, 'sensor_msgs/msg/Imu', typestore=typestore)

            connections = [x for x in reader.connections if x.topic == lidar_topic or x.topic == imu_topic]

            for connection, timestamp, rawdata in reader.messages(connections=connections):

                msg = reader.deserialize(rawdata, connection.msgtype)
                msg_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

                if connection.topic == imu_topic:
                    writer.write(imu_conn_out, msg_ns, rawdata)
                    continue

                elif connection.topic == lidar_topic:

                    msg = reader.deserialize(rawdata, connection.msgtype)
                    msg_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

                    now_time = timestamp/1e9

                    if start_time == None:
                        start_time = now_time

                    rosbag_time = now_time - start_time

                    odom_x, odom_y = compare_reference(rosbag_time, reference_dataframe)
                    is_spoofing = check_spoofing_condition(odom_x, odom_y, spoofer_x, spoofer_y, distance_threshold)

                    iteration = int(msg.data.shape[0]/topic_length)
                    bin_points = np.frombuffer(msg.data, dtype=np.uint8).reshape(iteration, topic_length)
                    x, y, z = binary_to_xyz(bin_points)

                    raw_cloud = np.vstack((x, y, z)).T

                    if is_spoofing and spoofing_mode == "removal":
                        spoofing_angle = decide_spoofing_param(odom_x, odom_y, spoofer_x, spoofer_y)
                        x_remaining, y_remaining, z_remaining, x_spoofed, y_spoofed, z_spoofed = spoofing_sim.spoof_main(raw_cloud, spoofing_angle, config['spoofing_simulation']['spoofing_range'])

                    elif is_spoofing and spoofing_mode == "static_injection":
                        spoofing_angle = decide_spoofing_param(odom_x, odom_y, spoofer_x, spoofer_y)
                        x_remaining, y_remaining, z_remaining, x_spoofed, y_spoofed, z_spoofed = spoofing_sim.injection_main(raw_cloud, spoofing_angle, config['spoofing_simulation']['spoofing_range'], config['spoofing_simulation']['static_wall_dist'])

                    elif is_spoofing and spoofing_mode == "dynamic_injection":
                        spoofing_angle = decide_spoofing_param(odom_x, odom_y, spoofer_x, spoofer_y)
                        x_remaining, y_remaining, z_remaining, x_spoofed, y_spoofed, z_spoofed = spoofing_sim.dynamic_injection_main(raw_cloud, now_time, spoofing_angle, config['spoofing_simulation']['spoofing_range'])
                        
                    else:
                        x_remaining, y_remaining, z_remaining, x_spoofed, y_spoofed, z_spoofed = x, y, z, np.array([]), np.array([]), np.array([])

                    x_simulated = np.concatenate((x_remaining, x_spoofed))
                    y_simulated = np.concatenate((y_remaining, y_spoofed))
                    z_simulated = np.concatenate((z_remaining, z_spoofed))
                    simulated_cloud =  np.vstack((x_simulated.astype(np.float32), y_simulated.astype(np.float32), z_simulated.astype(np.float32))).T
                    
                    out_msg = create_pointcloud2(simulated_cloud, cnt, msg_ns, msg.header.frame_id, typestore)
                    serialized_msg = typestore.serialize_ros1(out_msg, lidar_conn_out.msgtype)
                    writer.write(lidar_conn_out, msg_ns, serialized_msg)
                    cnt += 1