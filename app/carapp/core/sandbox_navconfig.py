"""Road-following profile: RPP + independent keepout, applied to a temporary copy."""
def road_parameters(params,root,speed):
    control=params['controller_server']['ros__parameters']
    control['goal_checker_plugins']=['general_goal_checker']
    control['FollowPath']=dict(plugin='nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
        desired_linear_vel=speed,lookahead_dist=.12,min_lookahead_dist=.08,max_lookahead_dist=.18,
        lookahead_time=1.2,use_velocity_scaled_lookahead_dist=True,
        min_approach_linear_velocity=min(.02,speed),approach_velocity_scaling_dist=.15,
        use_collision_detection=True,max_allowed_time_to_collision_up_to_carrot=1.,
        use_regulated_linear_velocity_scaling=True,regulated_linear_scaling_min_radius=.2,
        regulated_linear_scaling_min_speed=min(.02,speed),use_cost_regulated_linear_velocity_scaling=False,
        use_rotate_to_heading=True,rotate_to_heading_angular_vel=.25,rotate_to_heading_min_angle=.35,
        max_angular_accel=.5,allow_reversing=False,transform_tolerance=.2)
    for name in ('global_costmap','local_costmap'):
        cost=params[name][name]['ros__parameters'];cost['filters']=['sandbox_keepout']
        cost['sandbox_keepout']=dict(plugin='nav2_costmap_2d::KeepoutFilter',enabled=True,
                                    filter_info_topic='/sandbox_filter_info')
    params['sandbox_mask_server']={'ros__parameters':dict(use_sim_time=False,yaml_filename=root+'/keepout.yaml',
        topic_name='/sandbox_keepout_mask',frame_id='map')}
    params['sandbox_filter_info_server']={'ros__parameters':dict(use_sim_time=False,type=0,base=0.,multiplier=1.,
        filter_info_topic='/sandbox_filter_info',mask_topic='/sandbox_keepout_mask')}
    return params
