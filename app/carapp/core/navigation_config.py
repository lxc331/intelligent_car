"""Navigation parameters derived from the car vendor's installed Nav2 files.

This module is also copied into the session's temporary directory on the car.
It has no ROS or GUI imports and never edits the vendor's configuration.
"""
import copy
import math


def navigation_parameters(vendor, controller, map_path, speed):
    speed = float(speed)
    if not math.isfinite(speed) or not .02 <= speed <= .15:
        raise ValueError('导航速度应在 0.02–0.15 m/s')
    params = copy.deepcopy(vendor)
    for key, value in controller.items():
        params[key] = copy.deepcopy(value)
    def node(name):
        return params.setdefault(name, {}).setdefault('ros__parameters', {})
    node('map_server').update(yaml_filename=map_path, use_sim_time=False, frame_id='map')
    node('amcl').update(set_initial_pose=False, scan_topic='/scan_raw',
                        update_min_d=0., update_min_a=0.,
                        robot_model_type='nav2_amcl::OmniMotionModel',
                        base_frame_id='base_footprint', global_frame_id='map', odom_frame_id='odom')
    planner = node('planner_server')
    planner['GridBased'] = dict(plugin='nav2_navfn_planner/NavfnPlanner',
                               tolerance=.05, use_astar=True, allow_unknown=False)
    planner['planner_plugins'] = ['GridBased']
    control = node('controller_server')
    control.update(controller_frequency=10., min_x_velocity_threshold=.001,
                   min_y_velocity_threshold=.001, min_theta_velocity_threshold=.001)
    control.setdefault('progress_checker', {}).update(required_movement_radius=.04,
                                                       movement_time_allowance=20.)
    control.setdefault('general_goal_checker', {}).update(xy_goal_tolerance=.06,
                                                          yaw_goal_tolerance=.20)
    follow = control.setdefault('FollowPath', {})
    follow.update(min_vel_x=0., min_vel_y=0., max_vel_x=speed, max_vel_y=0.,
                  max_vel_theta=.35, max_speed_xy=speed, min_speed_xy=0.,
                  min_speed_theta=0., trans_stopped_velocity=.01,
                  acc_lim_x=.20, decel_lim_x=-.30, acc_lim_theta=.50,
                  decel_lim_theta=-.50, vx_samples=12, vy_samples=1,
                  vtheta_samples=20, sim_time=1.7, xy_goal_tolerance=.06)
    critics = list(follow.get('critics',[]))
    if 'ObstacleFootprint' not in critics:
        critics.append('ObstacleFootprint')
    follow['critics'] = critics
    follow['ObstacleFootprint.scale'] = 1.
    # A conservative rectangle measured relative to the laser. It includes the
    # claw and wheels; small padding also covers base/laser mounting offsets.
    footprint = '[[0.24,0.105],[0.24,-0.105],[-0.11,-0.105],[-0.11,0.105]]'
    for name in ('global_costmap', 'local_costmap'):
        cost = params.setdefault(name, {}).setdefault(name, {}).setdefault('ros__parameters', {})
        cost.update(robot_base_frame='base_footprint', footprint=footprint,
                    footprint_padding=.01, resolution=.025,
                    always_send_full_costmap=True, use_sim_time=False)
        cost.pop('robot_radius', None)
        cost.setdefault('inflation_layer', {}).update(inflation_radius=.20, cost_scaling_factor=5.)
        for layer in ('obstacle_layer', 'voxel_layer'):
            if layer in cost:
                cost[layer].setdefault('scan', {}).update(topic='/scan_raw')
    smooth = node('velocity_smoother')
    smooth.update(max_velocity=[speed, 0., .35], min_velocity=[0., 0., -.35],
                  max_accel=[.20, 0., .50], max_decel=[-.30, 0., -.50],
                  velocity_timeout=.35, smoothing_frequency=10.)
    node('behavior_server').update(max_rotational_vel=.35, min_rotational_vel=.05,
                                    rotational_acc_lim=.50)
    return params


LAUNCH_SOURCE = r'''
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
 root=os.path.dirname(__file__)
 token=os.path.basename(root).removeprefix('carapp_nav_')
 config=root+'/nav2.yaml'
 raw='/carapp_nav_'+token+'/raw_cmd_vel'
 smooth='/carapp_nav_'+token+'/smoothed_cmd_vel'
 specs=[('nav2_map_server','map_server','map_server',[]),
        ('nav2_amcl','amcl','amcl',[]),
        ('nav2_controller','controller_server','controller_server',[('cmd_vel',raw)]),
        ('nav2_planner','planner_server','planner_server',[]),
        ('nav2_smoother','smoother_server','smoother_server',[]),
        ('nav2_behaviors','behavior_server','behavior_server',[('cmd_vel',raw)]),
        ('nav2_bt_navigator','bt_navigator','bt_navigator',[]),
        ('nav2_velocity_smoother','velocity_smoother','velocity_smoother',
          [('cmd_vel',raw),('cmd_vel_smoothed',smooth)])]
 if os.path.isfile(root+'/keepout.yaml'):
  specs[0:0]=[('nav2_map_server','map_server','sandbox_mask_server',[]),
              ('nav2_map_server','costmap_filter_info_server','sandbox_filter_info_server',[])]
 nodes=[Node(package=package,executable=executable,name=name,
             parameters=[config],remappings=remaps,
             arguments=['--ros-args','--log-level','warn','--disable-external-lib-logs'],
             output='screen') for package,executable,name,remaps in specs]
 nodes.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',
                   name='carapp_navigation_lifecycle',output='screen',
                   parameters=[{'autostart':True,'use_sim_time':False,
                                'node_names':[spec[2] for spec in specs],
                                'bond_timeout':2.0}]))
 return LaunchDescription(nodes)
'''
