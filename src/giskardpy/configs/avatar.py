from typing import Optional

from giskardpy.configs.data_types import ControlModes, SupportedQPSolver
from giskardpy.configs.default_giskard import Giskard
from giskardpy.my_types import Derivatives

class Avatar_Base(Giskard):
    def __init__(self, root_link_name: Optional[str] = None):
        super().__init__(root_link_name=root_link_name)
        self.set_qp_solver(SupportedQPSolver.qpalm)
        self.configure_MaxTrajectoryLength(length=60)
        # Add self collision matrix and external collision settings as needed
        

class Avatar_StandAlone(Avatar_Base):
    def __init__(self):
        self.add_robot_from_parameter_server(add_drive_joint_to_group=False)
        super().__init__('map')
        self.set_default_visualization_marker_color(1, 1, 1, 1)
        self.set_control_mode(ControlModes.stand_alone)
        self.publish_all_tf()
        self.configure_VisualizationBehavior(in_planning_loop=True)
        self.configure_CollisionMarker(in_planning_loop=True)
        self.add_fixed_joint(parent_link='map', child_link='odom_combined')
        self.register_controlled_joints([
            # 'root_to_pelvis_yaw',
            'pelvis_yaw_to_pelvis_pitch',
            'pelvis_pitch_to_pelvis',
            'pelvis_to_spine_01_yaw',
            'spine_01_to_spine_02',
            'spine_02_to_spine_03',
            'spine_03_to_spine_04_yaw',
            'spine_04_yaw_to_spine_04_pitch',
            'spine_04_pitch_to_spine_04',
            'spine_04_to_spine_05',
            'spine_05_to_neck_01',
            'neck_01_to_neck_02',
            'neck_02_to_head_yaw',
            'head_yaw_to_head_pitch',
            'head_pitch_to_head',
            'spine_05_to_clavicle_l',
            'clavicle_l_to_upperarm_l_yaw',
            'upperarm_l_yaw_to_upperarm_l_pitch',
            'upperarm_l_pitch_to_upperarm_l',
            'upperarm_l_to_lowerarm_l_yaw',
            'lowerarm_l_yaw_to_lowerarm_l_pitch',
            'lowerarm_l_pitch_to_lowerarm_l',
            'lowerarm_l_to_hand_l_yaw',
            'hand_l_yaw_to_hand_l_pitch',
            'hand_l_pitch_to_hand_l',
            'spine_05_to_clavicle_r',
            'clavicle_r_to_upperarm_r_yaw',
            'upperarm_r_yaw_to_upperarm_r_pitch',
            'upperarm_r_pitch_to_upperarm_r',
            'upperarm_r_to_lowerarm_r_yaw',
            'lowerarm_r_yaw_to_lowerarm_r_pitch',
            'lowerarm_r_pitch_to_lowerarm_r',
            'lowerarm_r_to_hand_r_yaw',
            'hand_r_yaw_to_hand_r_pitch',
            'hand_r_pitch_to_hand_r',
            'pelvis_to_thigh_l_yaw',
            'thigh_l_yaw_to_thigh_l_pitch',
            'thigh_l_pitch_to_thigh_l',
            'thigh_l_to_calf_l_yaw',
            'calf_l_yaw_to_calf_l_pitch',
            'calf_l_pitch_to_calf_l',
            'calf_l_to_foot_l_yaw',
            'foot_l_yaw_to_foot_l_pitch',
            'foot_l_pitch_to_foot_l',
            'pelvis_to_thigh_r_yaw',
            'thigh_r_yaw_to_thigh_r_pitch',
            'thigh_r_pitch_to_thigh_r',
            'thigh_r_to_calf_r_yaw',
            'calf_r_yaw_to_calf_r_pitch',
            'calf_r_pitch_to_calf_r',
            'calf_r_to_foot_r_yaw',
            'foot_r_yaw_to_foot_r_pitch',
            'foot_r_pitch_to_foot_r',
            # 'brumbrum'
        ])
        self.add_omni_drive_joint(parent_link_name='odom_combined',
                                  child_link_name='root',
                                  name='brumbrum',
                                  translation_limits={
                                      Derivatives.velocity: 0.4,
                                      Derivatives.acceleration: 1,
                                      Derivatives.jerk: 5,
                                  },
                                  rotation_limits={
                                      Derivatives.velocity: 0.2,
                                      Derivatives.acceleration: 1,
                                      Derivatives.jerk: 5
                                  })  

""" class Avatar_Unreal(Giskard):
    def __init__(self, root_link_name: Optional[str] = None):
        super().__init__(root_link_name=root_link_name)
        # self.set_collision_checker(CollisionCheckerLib.none)
        self.set_qp_solver(SupportedQPSolver.qpalm)
        self.add_robot_from_parameter_server()
        self.add_sync_tf_frame('map', 'odom_combined')
        self.add_omni_drive_joint(name='brumbrum',
                                  parent_link_name='odom_combined',
                                  child_link_name='root',
                                  translation_limits={
                                      Derivatives.velocity: 0.4,
                                      Derivatives.acceleration: 1,
                                      Derivatives.jerk: 5,
                                  },
                                  rotation_limits={
                                      Derivatives.velocity: 0.2,
                                      Derivatives.acceleration: 1,
                                      Derivatives.jerk: 5
                                  },
                                  odometry_topic='/base_odometry/odom')
        fill_velocity_values = False
        self.add_follow_joint_trajectory_server(namespace='/whole_body_controller/follow_joint_trajectory',
                                                state_topic='/whole_body_controller/state',
                                                fill_velocity_values=fill_velocity_values)
        self.add_base_cmd_velocity(cmd_vel_topic='/base_controller/command',
                                   track_only_velocity=True)
        self.overwrite_external_collision_avoidance('brumbrum',
                                                    number_of_repeller=2,
                                                    soft_threshold=0.2,
                                                    hard_threshold=0.1) """