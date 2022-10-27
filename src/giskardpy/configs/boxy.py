from std_msgs.msg import ColorRGBA

from giskardpy.configs.data_types import ControlModes
from giskardpy.configs.default_giskard import Giskard


class Boxy_Base(Giskard):
    def __init__(self):
        super().__init__()


class Boxy_StandAlone(Boxy_Base):
    def __init__(self):
        self.add_robot_from_parameter_server()
        super().__init__()
        self.set_default_visualization_marker_color(1, 1, 1, 1)
        self._general_config.control_mode = ControlModes.stand_alone
        self.publish_all_tf()
        self.configure_VisualizationBehavior(in_planning_loop=True)
        self.configure_CollisionMarker(in_planning_loop=True)
        self.root_link_name = 'map'
        self.add_fixed_joint(parent_link='map', child_link='boxy_description/odom')
        self.register_controlled_joints([
            'neck_shoulder_pan_joint',
            'neck_shoulder_lift_joint',
            'neck_elbow_joint',
            'neck_wrist_1_joint',
            'neck_wrist_2_joint',
            'neck_wrist_3_joint',
            'triangle_base_joint',
            'left_arm_0_joint',
            'left_arm_1_joint',
            'left_arm_2_joint',
            'left_arm_3_joint',
            'left_arm_4_joint',
            'left_arm_5_joint',
            'left_arm_6_joint',
            'right_arm_0_joint',
            'right_arm_1_joint',
            'right_arm_2_joint',
            'right_arm_3_joint',
            'right_arm_4_joint',
            'right_arm_5_joint',
            'right_arm_6_joint',
            'odom_x_joint',
            'odom_y_joint',
            'odom_z_joint',
        ])
        self.overwrite_external_collision_avoidance('odom_z_joint',
                                                    number_of_repeller=2,
                                                    soft_threshold=0.2,
                                                    hard_threshold=0.1)
