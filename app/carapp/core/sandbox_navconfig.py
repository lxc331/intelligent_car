"""Road-following profile: RPP + independent keepout, applied to a temporary copy."""
def road_parameters(params,root,speed):
    control=params['controller_server']['ros__parameters']
    control['goal_checker_plugins']=['general_goal_checker']
    control['FollowPath']=dict(plugin='nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
        desired_linear_vel=speed,lookahead_dist=.18,min_lookahead_dist=.12,max_lookahead_dist=.25,
        lookahead_time=1.0,use_velocity_scaled_lookahead_dist=False,
        min_approach_linear_velocity=min(.02,speed),approach_velocity_scaling_dist=.12,
        use_collision_detection=True,max_allowed_time_to_collision_up_to_carrot=1.,
        use_regulated_linear_velocity_scaling=True,regulated_linear_scaling_min_radius=.15,
        regulated_linear_scaling_min_speed=min(.02,speed),use_cost_regulated_linear_velocity_scaling=False,
        use_rotate_to_heading=True,rotate_to_heading_angular_vel=.45,rotate_to_heading_min_angle=.15,
        max_angular_accel=.9,allow_reversing=False,transform_tolerance=.2)
    # The keepout mask only restricts planning. Keeping it in the local costmap
    # marks every cell off the 25-27 cm sandbox road lethal, so the controller
    # rejects its own footprint as a collision and aborts on "Failed to make
    # progress" whenever tracking error is a few centimetres. The controller
    # still sees real obstacles from the laser; the route itself was already
    # checked against the mask and the live global costmap before starting.
    cost=params['global_costmap']['global_costmap']['ros__parameters']
    cost['filters']=['sandbox_keepout']
    cost['sandbox_keepout']=dict(plugin='nav2_costmap_2d::KeepoutFilter',enabled=True,
                                filter_info_topic='/sandbox_filter_info')
    # 25-27 cm roads leave ~11 cm on each side of the 0.21 m wide body. A 0.20 m
    # inflation ring turns that entire corridor into 253-cost cells, which the
    # controller reports as a collision once tracking error reaches a few cm.
    local=params['local_costmap']['local_costmap']['ros__parameters']
    local.setdefault('inflation_layer',{})['inflation_radius']=.15
    params['sandbox_mask_server']={'ros__parameters':dict(use_sim_time=False,yaml_filename=root+'/keepout.yaml',
        topic_name='/sandbox_keepout_mask',frame_id='map')}
    params['sandbox_filter_info_server']={'ros__parameters':dict(use_sim_time=False,type=0,base=0.,multiplier=1.,
        filter_info_topic='/sandbox_filter_info',mask_topic='/sandbox_keepout_mask')}
    return params
