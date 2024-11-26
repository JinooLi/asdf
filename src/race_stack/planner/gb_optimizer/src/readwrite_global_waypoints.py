'''
Shared functions to read and write map information (global waypoints)

Previously, the global waypoints obtained during the mapping phase were saved in a rosbag.

Now, this is done using binary files.
'''

import rospkg
import os
import json
import csv
from rospy_message_converter import message_converter

from visualization_msgs.msg import MarkerArray
from f110_msgs.msg import WpntArray
from std_msgs.msg import String, Float32
from typing import Tuple, List, Dict

def write_global_waypoints(map_name:str,
    map_info_str:str,    
    est_lap_time:Float32,
    centerline_markers:MarkerArray,
    centerline_waypoints:WpntArray,
    global_traj_markers_iqp:MarkerArray,
    global_traj_wpnts_iqp:WpntArray,
    global_traj_markers_sp:MarkerArray,
    global_traj_wpnts_sp:WpntArray,
    trackbounds_markers:MarkerArray,
    map_editor_bool = False
                           )->None:
    '''
    Writes map information to a JSON file with map name specified by `map_name`.
    - map_info_str
        - from topic /map_infos: python str
    - est_lap_time
        - from topic /estimated_lap_time: Float32
    - centerline_markers
        - from topic /centerline_waypoints/markers: MarkerArray
    - centerline_waypoints
        - from topic /centerline_waypoints: WpntArray
    - global_traj_markers_iqp
        - from topic /global_waypoints: MarkerArray
    - global_traj_wpnts_iqp
        - from topic /global_waypoints/markers: WpntArray
    - global_traj_markers_sp
        - from topic /global_waypoints/shortest_path: MarkerArray
    - global_traj_wpnts_sp
        - from topic /global_waypoints/shortest_path/markers: WpntArray
    - trackbounds_markers
        - from topic /trackbounds/markers: MarkerArray
    '''

    # Get path of stack_master package to get the map waypoint path
    r = rospkg.RosPack()
    if not map_editor_bool:
        base_path = os.path.join(r.get_path('stack_master'), 'maps', map_name)
    else:
        base_path = os.path.join(r.get_path('map_editor'), 'maps', map_name)
    os.makedirs(base_path, exist_ok=True)
    path = os.path.join(base_path, 'global_waypoints.json')
    print(f"[INFO] WRITE_GLOBAL_WAYPOINTS: Writing global waypoints to {path}")

    # Dictionary will be converted into a JSON for serialization
    d: Dict[str, Dict] = {}
    d['map_info_str'] = {'data': map_info_str}
    d['est_lap_time'] = {'data': est_lap_time}
    d['centerline_markers'] = message_converter.convert_ros_message_to_dictionary(centerline_markers)
    d['centerline_waypoints'] = message_converter.convert_ros_message_to_dictionary(centerline_waypoints)
    d['global_traj_markers_iqp'] = message_converter.convert_ros_message_to_dictionary(global_traj_markers_iqp)
    d['global_traj_wpnts_iqp'] = message_converter.convert_ros_message_to_dictionary(global_traj_wpnts_iqp)
    d['global_traj_markers_sp'] = message_converter.convert_ros_message_to_dictionary(global_traj_markers_sp)
    d['global_traj_wpnts_sp'] = message_converter.convert_ros_message_to_dictionary(global_traj_wpnts_sp)
    d['trackbounds_markers'] = message_converter.convert_ros_message_to_dictionary(trackbounds_markers)

    # Serialize JSON
    with open(path, 'w') as f:
        json.dump(d, f)

    # Save to detailed CSV
    csv_path = os.path.join(base_path, 'global_waypoints.csv')
    with open(csv_path, 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(['opt_x', 'opt_y', 'outer_width', 'inner_width', 'center_x', 'center_y',
                             'outer_x', 'outer_y', 'inner_x', 'inner_y', 'curvature', 'ref_v'])

        # Write data from global_traj_wpnts_iqp
        for wp in global_traj_wpnts_iqp.wpnts:
            opt_x = wp.x_m
            opt_y = wp.y_m
            outer_width = wp.d_right
            inner_width = wp.d_left
            center_x = (wp.x_m + wp.d_left - wp.d_right) / 2  # Center x approximation
            center_y = (wp.y_m + wp.d_left - wp.d_right) / 2  # Center y approximation
            outer_x = wp.x_m + wp.d_right
            outer_y = wp.y_m + wp.d_right
            inner_x = wp.x_m - wp.d_left
            inner_y = wp.y_m - wp.d_left
            curvature = getattr(wp, 'kappa_radpm', None)  # Safely access curvature
            ref_v = getattr(wp, 'vx_mps', None)  # Safely access reference velocity

            csv_writer.writerow([opt_x, opt_y, outer_width, inner_width, center_x, center_y,
                                 outer_x, outer_y, inner_x, inner_y, curvature, ref_v])

    # Save simplified CSV for MPPI with x, y, velocity
    simplified_csv_path = os.path.join('/home/turtleship/turtleship_ws/outputs', f'{map_name}.csv')
    os.makedirs(os.path.dirname(simplified_csv_path), exist_ok=True)
    with open(simplified_csv_path, 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(['x', 'y', 'velocity'])  # Simplified format for MPPI

        # Write simplified data from global_traj_wpnts_iqp
        for wp in global_traj_wpnts_iqp.wpnts:
            x = wp.x_m
            y = wp.y_m
            velocity = getattr(wp, 'vx_mps', None)  # Safely access velocity
            csv_writer.writerow([x, y, velocity])

    print(f"[INFO] WRITE_GLOBAL_WAYPOINTS: Simplified CSV saved to {simplified_csv_path}")

def read_global_waypoints(map_name:str)->Tuple[
    String, Float32, MarkerArray, WpntArray, MarkerArray, WpntArray, MarkerArray, WpntArray, MarkerArray
]:
    '''
    Reads map information from a JSON file with map name specified by `map_name`.

    Outputs Message objects as follows:
    - map_info_str
        - from topic /map_infos: String
    - est_lap_time
        - from topic /estimated_lap_time: Float32
    - centerline_markers
        - from topic /centerline_waypoints/markers: MarkerArray
    - centerline_waypoints
        - from topic /centerline_waypoints: WpntArray
    - global_traj_markers_iqp
        - from topic /global_waypoints: MarkerArray
    - global_traj_wpnts_iqp
        - from topic /global_waypoints/markers: WpntArray
    - global_traj_markers_sp
        - from topic /global_waypoints/shortest_path: MarkerArray
    - global_traj_wpnts_sp
        - from topic /global_waypoints/shortest_path/markers: WpntArray
    - trackbounds_markers
        - from topic /trackbounds/markers: MarkerArray
    '''

    # Get path of stack_master package to get the map waypoint path
    r = rospkg.RosPack()
    path = os.path.join(r.get_path('stack_master'), 'maps', map_name, 'global_waypoints.json')

    print(f"[INFO] READ_GLOBAL_WAYPOINTS: Reading global waypoints from {path}")
    # Deserialize JSON and Reconstruct the maps elements
    with open(path, 'r') as f:
        d: Dict[str, List] = json.load(f)
    map_info_str = message_converter.convert_dictionary_to_ros_message('std_msgs/String', d['map_info_str'])
    est_lap_time = message_converter.convert_dictionary_to_ros_message('std_msgs/Float32', d['est_lap_time'])
    centerline_markers = message_converter.convert_dictionary_to_ros_message(
                                            'visualization_msgs/MarkerArray',
                                            d['centerline_markers'])
    centerline_waypoints = message_converter.convert_dictionary_to_ros_message(
                                            'f110_msgs/WpntArray',
                                            d['centerline_waypoints'])
    global_traj_markers_iqp = message_converter.convert_dictionary_to_ros_message(
                                            'visualization_msgs/MarkerArray',
                                            d['global_traj_markers_iqp'])
    global_traj_wpnts_iqp = message_converter.convert_dictionary_to_ros_message(
                                            'f110_msgs/WpntArray',
                                            d['global_traj_wpnts_iqp'])
    global_traj_markers_sp = message_converter.convert_dictionary_to_ros_message(
                                            'visualization_msgs/MarkerArray',
                                            d['global_traj_markers_sp'])
    global_traj_wpnts_sp = message_converter.convert_dictionary_to_ros_message(
                                            'f110_msgs/WpntArray',
                                            d['global_traj_wpnts_sp'])
    trackbounds_markers = message_converter.convert_dictionary_to_ros_message(
                                            'visualization_msgs/MarkerArray',
                                            d['trackbounds_markers'])


    return map_info_str, est_lap_time,\
            centerline_markers, centerline_waypoints, \
            global_traj_markers_iqp, global_traj_wpnts_iqp, \
            global_traj_markers_sp, global_traj_wpnts_sp, trackbounds_markers
