from giskardpy.configs.data_types import ControlModes
from giskardpy.configs.default_giskard import Giskard
from giskardpy.my_types import Derivatives


class Avatar_StandAlone(Giskard):
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
            #'root_to_pelvis_yaw',
            'pelvis_yaw_to_pelvis_pitch',
            'pelvis_pitch_to_pelvis',
            'pelvis_to_spine_04_yaw',
            'spine_04_yaw_to_spine_04_pitch',
            'spine_04_pitch_to_spine_04',
            'spine_04_to_head_yaw',
            'head_yaw_to_head_pitch',
            'head_pitch_to_head',
            'spine_04_to_clavicle_l',
            'clavicle_l_to_upperarm_l_yaw',
            'upperarm_l_yaw_to_upperarm_l_pitch',
            'upperarm_l_pitch_to_upperarm_l',
            'upperarm_l_to_lowerarm_l_yaw',
            'lowerarm_l_yaw_to_lowerarm_l_pitch',
            'lowerarm_l_pitch_to_lowerarm_l',
            'lowerarm_l_to_hand_l_yaw',
            'hand_l_yaw_to_hand_l_pitch',
            'hand_l_pitch_to_hand_l',
            #'brumbrum'
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
                             }
                             )
    