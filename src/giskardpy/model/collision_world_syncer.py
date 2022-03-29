from itertools import product, combinations_with_replacement
from time import time
from typing import List, Dict, Tuple, Union

import numpy as np

from giskard_msgs.msg import CollisionEntry
from giskardpy import RobotName, identifier
from giskardpy.data_types import Collisions, JointStates, PrefixName
from giskardpy.exceptions import UnknownGroupException
from giskardpy.model.world import SubWorldTree
from giskardpy.model.world import WorldTree
from giskardpy.utils import logging


class CollisionWorldSynchronizer(object):
    def __init__(self, world):
        self.world = world  # type: WorldTree
        self.black_list = set()
        try:
            self.ignored_pairs = self.god_map.get_data(identifier.ignored_self_collisions)
        except KeyError as e:
            self.ignored_pairs = set()
        try:
            self.white_list_pairs = self.god_map.get_data(identifier.added_self_collisions)
            self.white_list_pairs = set(
                x if self.world.link_order(*x) else tuple(reversed(x)) for x in self.white_list_pairs)
        except KeyError as e:
            self.white_list_pairs = set()

        self.world_version = -1

    def has_world_changed(self):
        if self.world_version != self.world.model_version:
            self.world_version = self.world.model_version
            return True
        return False

    @property
    def robot(self):
        """
        :rtype: SubWorldTree
        """
        return self.world.groups[RobotName]

    @property
    def god_map(self):
        """
        :rtype: giskardpy.god_map.GodMap
        """
        return self.world.god_map

    def add_object(self, link):
        """
        :type link: giskardpy.model.world.Link
        """
        pass

    def reset_collision_blacklist(self):
        self.black_list = set()
        self.update_collision_blacklist(white_list_combinations=self.white_list_pairs)

    @profile
    def update_collision_blacklist(self, link_combinations: set = None, white_list_combinations: set = None,
                                   distance_threshold_zero: float = 0.05, distance_threshold_rnd: float = 0.0,
                                   non_controlled: bool = False, steps: int = 10):
        np.random.seed(1337)
        if link_combinations is None:
            link_combinations = set(combinations_with_replacement(self.world.link_names_with_collisions, 2))
        # logging.loginfo('calculating self collision matrix')
        joint_state_tmp = self.world.state
        t = time()

        # find meaningless collisions
        for link_a, link_b in link_combinations:
            link_combination = self.world.sort_links(link_a, link_b)
            if link_combination in self.black_list:
                continue
            if link_a == link_b \
                    or link_a in self.ignored_pairs \
                    or link_b in self.ignored_pairs \
                    or (link_a, link_b) in self.ignored_pairs \
                    or (link_b, link_a) in self.ignored_pairs \
                    or self.world.are_linked(link_a, link_b, non_controlled=non_controlled) \
                    or (not self.world.is_link_controlled(link_a) and not self.world.is_link_controlled(link_b)):
                self.black_list.add(link_combination)

        unknown = link_combinations.difference(self.black_list)
        self.set_joint_state_to_zero()
        for link_a, link_b in self.check_collisions2(unknown, distance_threshold_zero):
            link_combination = self.world.sort_links(link_a, link_b)
            self.black_list.add(link_combination)
        unknown = unknown.difference(self.black_list)

        # Remove combinations which can never touch
        # by checking combinations which a single joint can influence
        for joint_name in self.world.controlled_joints:
            parent_links = self.world.get_siblings_with_collisions(joint_name)
            if not parent_links:
                continue
            child_links = self.world.get_directly_controlled_child_links_with_collisions(joint_name)
            if self.world.is_joint_continuous(joint_name):
                min_position = -np.pi
                max_position = np.pi
            else:
                min_position, max_position = self.world.get_joint_position_limits(joint_name)

            # joint_name can make these links touch.
            current_combinations = set(product(parent_links, child_links))
            # Filter for combinations which are still unknown
            # and make sure the link_a, link_b order is same as in unknown
            subset_of_unknown = [x for x in unknown if
                                 x in current_combinations or (x[1], x[0]) in current_combinations]
            if not subset_of_unknown:
                continue
            sometimes = set()
            for position in np.linspace(min_position, max_position, steps):
                self.world.state[joint_name].position = position
                self.sync()
                for link_a, link_b in subset_of_unknown:
                    if self.in_collision(link_a, link_b, distance_threshold_rnd):
                        sometimes.add(self.world.sort_links(link_a, link_b))
            never = set(subset_of_unknown).difference(sometimes)
            unknown = unknown.difference(never)
            self.black_list.update(never)

        logging.logdebug(f'Calculated self collision matrix in {time() - t:.3f}s')
        self.world.state = joint_state_tmp
        # unknown.update(self.white_list_pairs)
        if white_list_combinations is not None:
            self.black_list.difference_update(white_list_combinations)
        # self.black_list[group_name] = unknown
        # return self.collision_matrices[group_name]

    def get_pose(self, link_name):
        return self.world.compute_fk_pose_with_collision_offset(self.world.root_link_name, link_name)

    def set_joint_state_to_zero(self):
        self.world.state = JointStates()

    def set_max_joint_state(self, group):
        def f(joint_name):
            _, upper_limit = group.get_joint_position_limits(joint_name)
            if upper_limit is None:
                return np.pi * 2
            return upper_limit

        group.state = self.generate_joint_state(group, f)

    def set_min_joint_state(self, group):
        def f(joint_name):
            lower_limit, _ = group.get_joint_position_limits(joint_name)
            if lower_limit is None:
                return -np.pi * 2
            return lower_limit

        group.state = self.generate_joint_state(group, f)

    def set_rnd_joint_state(self, group):
        def f(joint_name):
            lower_limit, upper_limit = group.get_joint_position_limits(joint_name)
            if lower_limit is None:
                return np.random.random() * np.pi * 2
            lower_limit = max(lower_limit, -10)
            upper_limit = min(upper_limit, 10)
            return (np.random.random() * (upper_limit - lower_limit)) + lower_limit

        group.state = self.generate_joint_state(group, f)

    def generate_joint_state(self, group, f):
        """
        :param f: lambda joint_info: float
        :return:
        """
        js = JointStates()
        for joint_name in sorted(group.movable_joints):
            if group.search_downwards_for_links(joint_name):
                js[joint_name].position = f(joint_name)
            else:
                js[joint_name].position = 0
        return js

    def check_collisions2(self, link_combinations, distance):
        in_collision = set()
        self.sync()
        for link_a, link_b in link_combinations:
            if self.in_collision(link_a, link_b, distance):
                in_collision.add((link_a, link_b))
        return in_collision

    def check_collisions(self, cut_off_distances, collision_list_size=15):
        """
        :param cut_off_distances: (robot_link, body_b, link_b) -> cut off distance. Contacts between objects not in this
                                    dict or further away than the cut off distance will be ignored.
        :type cut_off_distances: dict
        :param self_collision_d: distances grater than this value will be ignored
        :type self_collision_d: float
        :type enable_self_collision: bool
        :return: (robot_link, body_b, link_b) -> Collision
        :rtype: Collisions
        """
        pass

    def in_collision(self, link_a, link_b, distance):
        """
        :type link_a: str
        :type link_b: str
        :type distance: float
        :rtype: bool
        """
        return False

    def sync(self):
        """
        :type world: giskardpy.model.world.WorldTree
        """
        pass

    def collision_goals_to_collision_matrix(self,
                                            collision_goals: List[CollisionEntry],
                                            min_dist: dict,
                                            added_checks: Dict[
                                                Tuple[Union[str, PrefixName], Union[str, PrefixName]], float]):
        """
        :param collision_goals: list of CollisionEntry
        :type collision_goals: list
        :return: dict mapping (robot_link, body_b, link_b) -> min allowed distance
        :rtype: dict
        """
        collision_goals = self.verify_collision_entries(collision_goals)
        min_allowed_distance = {}
        for collision_entry in collision_goals:  # type: CollisionEntry
            if collision_entry.group1 == collision_entry.ALL:
                group1_links = self.world.link_names_with_collisions
            else:
                group1_links = self.world.groups[collision_entry.group1].link_names_with_collisions
            if collision_entry.group2 == collision_entry.ALL:
                group2_links = self.world.link_names_with_collisions
            else:
                group2_links = self.world.groups[collision_entry.group2].link_names_with_collisions
            for link1 in group1_links:
                for link2 in group2_links:
                    key = self.world.sort_links(link1, link2)
                    r_key = (key[1], key[0])
                    if self.is_allow_collision(collision_entry):
                        if key in min_allowed_distance:
                            del min_allowed_distance[key]
                        elif r_key in min_allowed_distance:
                            del min_allowed_distance[r_key]
                    elif self.is_avoid_collision(collision_entry):
                        if key not in self.black_list:
                            min_allowed_distance[key] = min_dist[key[0]]
                    else:
                        raise AttributeError(f'Invalid collision entry type: {collision_entry.type}')
        for (link1, link2), distance in added_checks.items():
            key = self.world.sort_links(link1, link2)
            if key in min_allowed_distance:
                min_allowed_distance[key] = max(distance, min_allowed_distance[key])
            else:
                min_allowed_distance[key] = distance
        for key in self.black_list:
            if key in min_allowed_distance:
                del min_allowed_distance[key]
        return min_allowed_distance

    def verify_collision_entries(self, collision_goals: List[CollisionEntry]) -> List[CollisionEntry]:
        for collision_entry in collision_goals:
            if collision_entry.group1 != collision_entry.ALL and collision_entry.group1 not in self.world.groups:
                raise UnknownGroupException(f'group1 \'{collision_entry.group1}\' unknown.')
            if collision_entry.group2 != collision_entry.ALL and collision_entry.group2 not in self.world.groups:
                raise UnknownGroupException(f'group2 \'{collision_entry.group1}\' unknown.')

        for i, ce in enumerate(reversed(collision_goals)):
            if self.is_avoid_all_collision(ce):
                # remove everything before the avoid all
                collision_goals = collision_goals[len(collision_goals) - i - 1:]
                break
            if self.is_allow_all_collision(ce):
                # remove everything before the allow all, including the allow all
                collision_goals = collision_goals[len(collision_goals) - i:]
                break
        else:
            # put an avoid all at the front
            ce = CollisionEntry()
            ce.type = CollisionEntry.AVOID_COLLISION
            ce.distance = -1
            collision_goals.insert(0, ce)

        return collision_goals

    def get_possible_collisions(self, link):
        # TODO speed up by saving this
        possible_collisions = set()
        for link1, link2 in self.collision_matrices[RobotName]:
            if link == link1:
                possible_collisions.add(link2)
            elif link == link2:
                possible_collisions.add(link1)
        return possible_collisions

    def is_avoid_collision(self, collision_entry: CollisionEntry) -> bool:
        return collision_entry.type == CollisionEntry.AVOID_COLLISION

    def is_allow_collision(self, collision_entry: CollisionEntry) -> bool:
        return collision_entry.type == CollisionEntry.ALLOW_COLLISION

    def is_avoid_all_self_collision(self, collision_entry: CollisionEntry) -> bool:
        return self.is_avoid_collision(collision_entry) \
               and collision_entry.group1 == RobotName and collision_entry.group2 == RobotName

    def is_allow_all_self_collision(self, collision_entry: CollisionEntry) -> bool:
        return self.is_allow_collision(collision_entry) \
               and collision_entry.group1 == RobotName and collision_entry.group2 == RobotName

    def is_avoid_all_collision(self, collision_entry: CollisionEntry) -> bool:
        """
        :type collision_entry: CollisionEntry
        :return: bool
        """
        return self.is_avoid_collision(collision_entry) \
               and collision_entry.group1 == collision_entry.ALL and collision_entry.group2 == collision_entry.ALL

    def is_allow_all_collision(self, collision_entry: CollisionEntry) -> bool:
        """
        :type collision_entry: CollisionEntry
        :return: bool
        """
        return self.is_allow_collision(collision_entry) \
               and collision_entry.group1 == collision_entry.ALL and collision_entry.group2 == collision_entry.ALL

    def reset_cache(self):
        pass
