"""
Armer Class

.. codeauthor:: Gavin Suddreys
.. codeauthor:: Dasun Gunasinghe
"""
from __future__ import annotations
from typing import List, Dict, Any, Tuple

import timeit
import importlib

import rospy
import tf2_ros
import yaml

import numpy as np
import roboticstoolbox as rtb
import spatialgeometry as sg
import spatialmath as sm
from roboticstoolbox.backends.swift import Swift

from spatialmath.base.argcheck import getvector

from armer.utils import populate_transform_stamped
from armer.models import URDFRobot
from armer.robots import ROSRobot
from armer.timer import Timer
# Add cython global checker
from armer.cython import collision_handler

class Armer:
    """
    The Armer Driver.

    :param robot: [description], List of robots to be managed by the driver
    :type robots: List[rtb.robot.Robot], optional
    :param backend: [description], defaults to None
    :type backend: rtb.backends.Connector, optional

    .. codeauthor:: Gavin Suddrey
    .. sectionauthor:: Gavin Suddrey
    """

    # pylint: disable=too-many-instance-attributes

    def __init__(
            self,
            robots: List[rtb.robot.Robot] = None,
            backend: rtb.backends.Connector = None,
            backend_args: Dict[str, Any] = None,
            readonly_backends: List[Tuple[rtb.backends.Connector, Dict[str, Any]]] = None,
            publish_transforms: bool = False,
            logging: dict[str, bool] = None) -> None:

        self.robots: List[ROSRobot] = robots
        self.backend: rtb.backends.Connector = backend
        self.readonly_backends : List[rtb.backends.Connector] = readonly_backends \
            if readonly_backends else []
        self.backend_args = backend_args

        if not self.robots:
            self.robots = [ROSRobot(self, rtb.models.URDF.UR5())]

        if not self.backend:
            self.backend = Swift()

        self.is_publishing_transforms = publish_transforms

        self.broadcaster: tf2_ros.TransformBroadcaster = None

        if self.is_publishing_transforms:
            self.broadcaster = tf2_ros.TransformBroadcaster()

        self.frequency = min([r.frequency for r in self.robots])
        self.rate = rospy.Rate(self.frequency)

        self.last_tick = rospy.get_time()

        # This is a global dictionary of dictionaries (per robot) for multi robot scenarios
        self.global_collision_dict = dict()

        # Launch backend
        self.backend.launch(**(backend_args if backend_args else dict()))

        # print(f"init links:")
        for robot in self.robots:
            # Add robot to the backend
            # NOTE: collision alpha useful for Swift simulation (ignored in ROS backend)
            self.backend.add(robot, collision_alpha=0.2)
            
            # Resolve robot links for collision checking
            # NOTE: must be done after adding to backend
            # NOTE: this gets an understanding of the current robot's overlapped self collisions
            robot.characterise_collision_overlaps()

            # This method extracts all captured collision objects (dict) from each robot
            # Needed to conduct global collision checking (if more than one robot instance is in play)
            # NOTE: all robot instances read the same robot description param, so all robots will get the same description
            #       this may not be the preferred implementation for future use cases.
            self.global_collision_dict[robot.name] = robot.get_link_collision_dict()
            # print(f"Current robot [{robot.name}] has collision dictionary of: {self.global_collision_dict[robot.name]}")

            # # TESTING
            # # Add dummy object for testing
            # s0 = sg.Sphere(radius=0.05, pose=sm.SE3(0.5, 0, 0.5))
            # s1 = sg.Sphere(radius=0.05, pose=sm.SE3(0.5, 0, 0.1))
            # robot.add_collision_obj(s0)
            # robot.add_collision_obj(s1)
            # self.backend.add(s0)
            # self.backend.add(s1)

        for readonly, args in self.readonly_backends:
            readonly.launch(**args)

            for robot in self.robots:
                readonly.add(robot, readonly=True)

        # Logging
        self.log_frequency = logging and 'frequency' in logging and logging['frequency']

    # def reset_backend(self):
    #     """
    #     Resets the backend correctly
    #     """
    #     # Check for error
    #     for robot in self.robots:
    #         self.backend.remove(robot)

    #     # for robot in self.robots:
    #     #     self.backend.add(robot)

    def close(self):
        """
        Close backend and stop action servers
        """
        self.backend.close()

        for robot in self.robots:
            robot.close()

    def publish_transforms(self) -> None:
        """[summary]
        """
        if not self.is_publishing_transforms:
            return

        transforms = []

        for robot in self.robots:
            joint_positions = getvector(robot.q, robot.n)

            for link in robot.links:
                if link.parent is None:
                    continue

                if link.isjoint:
                    transform = link._Ts @ link._ets[-1].A(joint_positions[link.jindex])
                else:
                    transform = link._Ts

                transforms.append(populate_transform_stamped(
                    link.parent.name,
                    link.name,
                    transform
                ))

            for gripper in robot.grippers:
                joint_positions = getvector(gripper.q, gripper.n)

                for link in gripper.links:
                    if link.parent is None:
                        continue

                    if link.isjoint:
                        transform = link._Ts @ link._ets[-1].A(joint_positions[link.jindex])
                    else:
                        transform = link._Ts

                    transforms.append(populate_transform_stamped(
                        link.parent.name,
                        link.name,
                        transform
                    ))
        
        self.broadcaster.sendTransform(transforms)

    @staticmethod
    def load(path: str) -> Armer:
        """
        Generates an Armer Driver instance from the configuration file at path

        :param path: The path to the configuration file
        :type path: str
        :return: An Armer driver instance
        :rtype: Armer
        """
        with open(path, 'r') as handle:
            config = yaml.load(handle, Loader=yaml.SafeLoader)

        robots: List[rtb.robot.Robot] = []

        for spec in config['robots']:
            robot_cls = URDFRobot
            wrapper = ROSRobot

            model_spec = {}
            
            if 'model' in spec:
              model_type = spec['model'] if isinstance(spec['model'], str) else spec['model']['type'] if 'type' in spec['model'] else None
              model_spec = spec['model'] if isinstance(spec['model'], dict) else {}
              
              if model_type: 
                module_name, model_name = model_type.rsplit('.', maxsplit=1)            
                robot_cls = getattr(importlib.import_module(module_name), model_name)
                
              if 'type' in model_spec:
                del model_spec['type']     

              del spec['model']

            if 'type' in spec:
                module_name, model_name = spec['type'].rsplit('.', maxsplit=1)
                wrapper = getattr(importlib.import_module(module_name), model_name)
                del spec['type']

            robots.append(wrapper(robot_cls(**model_spec), **spec))

        backend = None
        backend_args = dict()

        if 'backend' in config:
            module_name, model_name = config['backend']['type'].rsplit('.', maxsplit=1)
            backend_cls = getattr(importlib.import_module(module_name), model_name)

            backend = backend_cls()
            backend_args = config['args'] if 'args' in config else dict()

        readonly_backends = []

        if 'readonly_backends' in config:
            for spec in config['readonly_backends']:
                module_name, model_name = spec['type'].rsplit('.', maxsplit=1)
                backend_cls = getattr(importlib.import_module(module_name), model_name)

                readonly_backends.append((backend_cls(), spec['args'] if 'args' in spec else dict()))

        logging = config['logging'] if 'logging' in config else {}
        publish_transforms = config['publish_transforms'] if 'publish_transforms' in config else False

        return Armer(
            robots=robots,
            backend=backend,
            backend_args=backend_args,
            readonly_backends=readonly_backends,
            publish_transforms=publish_transforms,
            logging=logging
        )

    def global_collision_check(self, robot: ROSRobot):
        """
        Conducts a full check for collisions
        NOTE: takes a given robot object and runs its collision check (of its own dictionary) against the global dictionary
                the global dictionary may have collision data from multiple robots (with different link data)
        TODO: currently each robot is checked against its own link data. This is needed for self collision checking
            but could be possibly optimised in some way as to not be overloaded with multiple instances
        NOTE: [2023-10-31] Identified that this component is very inefficient for the panda (real test). Implemented 
                a start and stop link (e.g., terminating search from end-effector to panda_link8, rather than full tree)
        """
        # Error handling on gripper name
        if robot.gripper == None or robot.gripper == "":
            rospy.logerr(f"Global Collision Check -> gripper name is invalid: {robot.gripper}")
            return False
        
        # Error handling on empty lick dictionary (should never happen but just in case)
        if robot.link_dict == dict() or robot.link_dict == None:
            rospy.logerr(f"Global Collision Check -> link dictionary is invalid: {robot.link_dict}")
            return False

        # Error handling on collision object dict and overlap dict
        if robot.overlapped_link_dict == dict() or robot.overlapped_link_dict == None or \
            robot.collision_dict == dict() or robot.collision_dict == None:
            rospy.logerr(f"Global Collision Check -> collision or overlap dictionaries invalid: [{robot.collision_dict}] | [{robot.overlapped_link_dict}]")
            return False
        
        if robot.collision_sliced_links == None:
            rospy.logerr(f"Global Collision Check -> could not get collision sliced links: [{robot.collision_sliced_links}]")
            return False

        # Debugging
        # print(f"sliced links: {[link.name for link in robot.collision_sliced_links]}")
        # print(f"col dict -> robots to check: {[robot for robot in self.global_collision_dict.keys()]}")
        # print(f"col dict -> links to check as a dict: {[link for link in self.global_collision_dict.values()]}")

        # Alternative Method
        # NOTE: this has between 1-6% increase in speed of execution
        with Timer("NEW GLOBAL CHECK", enabled=False):
            col_link_id = collision_handler.global_check(
                robot_name = robot.name,
                robot_names = list(self.global_collision_dict.keys()),
                len_robots = len(self.global_collision_dict.keys()),
                robot_links = robot.collision_sliced_links,
                len_links = len(robot.collision_sliced_links),
                global_dict = self.global_collision_dict,
                overlap_dict = robot.overlapped_link_dict
            )
    
        if col_link_id >= 0:
            rospy.logwarn(f"Global Collision Check -> Robot [{robot.name}] in collision with link {robot.collision_sliced_links[col_link_id].name}")
            return True
        else:
            # No collisions found with no errors identified.
            return False
    
    def archived_global_collision_check(self, robot: ROSRobot):
        """
        Conducts a full check for collisions
        NOTE: [2023-11-03] archieved as cython implementation is being used
            kept for debugging purposes
        """
        # Error handling on gripper name
        if robot.gripper == None or robot.gripper == "":
            rospy.logerr(f"Global Collision Check -> gripper name is invalid: {robot.gripper}")
            return False
        
        # Error handling on empty lick dictionary (should never happen but just in case)
        if robot.link_dict == dict() or robot.link_dict == None:
            rospy.logerr(f"Global Collision Check -> link dictionary is invalid: {robot.link_dict}")
            return False

        # Error handling on collision object dict and overlap dict
        if robot.overlapped_link_dict == dict() or robot.overlapped_link_dict == None or \
            robot.collision_dict == dict() or robot.collision_dict == None:
            rospy.logerr(f"Global Collision Check -> collision or overlap dictionaries invalid: [{robot.collision_dict}] | [{robot.overlapped_link_dict}]")
            return False

        with Timer("Link Slicing Check", enabled=False):
            # Prepare sliced link based on a defined stop link 
            # TODO: this could be update-able for interesting collision checks based on runtime requirements
            col_start_link_idx = [i for i, link in enumerate(robot.sorted_links) if link.name == robot.collision_start_link]
            col_stop_link_idx = [i for i, link in enumerate(robot.sorted_links) if link.name == robot.collision_stop_link]
            # NOTE: the assumption here is that each link is unique (which is handled low level by rtb) so we take the first element if found
            # NOTE: sorted links is from base link upwards tree. We want to slice from stop link to end
            # print(f"start_idx: {col_start_link_idx} | stop_idx: {col_stop_link_idx}")
            if len(col_start_link_idx) > 0 and len(col_stop_link_idx) > 0:
                if col_start_link_idx[0] == len(robot.sorted_links):
                    sliced_links = robot.sorted_links[col_stop_link_idx[0]:None]
                else:
                    sliced_links = robot.sorted_links[col_stop_link_idx[0]:col_start_link_idx[0] + 1]
            else:
                sliced_links = robot.sorted_links

            # # Debugging
            # print(f"sliced links: {[link.name for link in sliced_links]}")
            # print(f"first 5 test: {[link.name for link in sliced_links[:5]]}")
            # print(f"last 5 test: {[link.name for link in sliced_links[5:]]}")
            # print(f"col dict -> robots to check: {[robot for robot in self.global_collision_dict.keys()]}")
            # print(f"col dict -> links to check as a dict: {[link for link in self.global_collision_dict.values()]}")
        
        # Iterate through global dictionary and check current robot for collisions
        with Timer("OLD GLOBAL CHECK", enabled=False):
            for robot_name in self.global_collision_dict.keys():
                # print(f"Checking {robot.name} against robot in dict: {robot_name}")
                for link_name in self.global_collision_dict[robot_name]:
                    # Handle Self Checking with known Overlaps
                    if robot.name == robot_name and link_name in robot.overlapped_link_dict.keys():
                        ignore_list = robot.overlapped_link_dict[link_name]
                    else:
                        ignore_list = []

                    # Get out check robot (in dictionary) details
                    collision_shape_list = self.global_collision_dict[robot_name][link_name]
                    # print(f"[{robot.name}] checking against [{robot_name}] with link name: {link_name}")
                    # This is a reverse search from top (ee) to bottom (base). 
                    # The rationale is to configure our stop point from the start of the tree to its root
                    # NOTE: the longer we traverse, the more of the robot's links are checked and the longer this will take
                    #       optimising our tree like this is based on the assumption that the 
                    #       leading tree links will be most likely in contact with the environment
                    # NOTE: defaults stop link to base_link of robot. TODO: add a config param for updating this
                    col_link, collision = robot.check_link_collision(
                        target_link=link_name, 
                        sliced_links=sliced_links, 
                        ignore_list=ignore_list,
                        check_list=collision_shape_list
                    )
                    
                    if collision:
                        rospy.logwarn(f"Global Collision Check -> Robot [{robot.name}] in collision with robot [{robot_name}] link {link_name}")
                        return True

        # No collisions found with no errors identified.
        return False
    
    def update_dynamic_objects(self, robot: ROSRobot) -> None:
        """
        method to handle the addition and removal of dynamic objects per robot instance
        """
        # Check if the current robot has any objects that need removal
        if robot.dynamic_collision_removal_dict:
            for d_obj_name in list(robot.dynamic_collision_removal_dict.copy().keys()):
                rospy.logwarn(f"Removal of Dynamic Objects in Progress")
                # remove from backend
                # NOTE: there is a noted bug in the swift backend that sets the object 
                #       (in a separate dictionary called swift_objects) to None. In the self.backend.step()
                #       method below, this attempts to run some methods that belong to the shape but cannot do so
                #       as it is a NoneType.
                shape_to_remove = robot.dynamic_collision_removal_dict[d_obj_name].shape
                rospy.loginfo(f"Remove object is: {shape_to_remove}")
                # TODO: add this feature in once swift side is fixed 
                #       should still work for ROS backend
                # self.backend.remove(obj_to_remove)
                # remove from robot dict
                robot.dynamic_collision_removal_dict.pop(d_obj_name)
                rospy.loginfo(f"Removed successfully")
        else:
            # Check if the current robot has any newly added objects to add to the backend
            # NOTE: this loop is run everytime at the moment (not an issue with limited shapes but needs better optimisation for scale)
            for dynamic_obj in robot.dynamic_collision_dict.values():
                if dynamic_obj.is_added == False:
                    rospy.loginfo(f"Adding Dynamic Object: {dynamic_obj}")
                    dynamic_obj.id = self.backend.add(dynamic_obj.shape)
                    dynamic_obj.is_added = True
                    rospy.loginfo(f"Added Successfully")


    def run(self) -> None:
        """
        Runs the driver. This is a blocking call.
        """
        self.last_tick = rospy.get_time()
        rospy.loginfo(f"ARMer Node Running...")
        added = False
        
        while not rospy.is_shutdown():
            with Timer('ROS', self.log_frequency):
                current_time = rospy.get_time()
                dt = current_time - self.last_tick
                backend_reset = False

                # Step the robot(s)
                for robot in self.robots:
                    # NOTE: this is currently in ideation
                    # Should the collision checking occur at this level? Makes sense for global setups and would also work for single setups
                    # TODO: Each robot performs its own self check (possibly saves some time?)
                    # Each robot is then checked with a global collision check (environment and other robots)
                    # If collisions are found (self or with global) preemption signal sent to each robot object.
                    if self.global_collision_check(robot=robot) and robot.preempted == False:
                        # Current robot found to be in collision so preempt
                        robot.collision_approached = True
                        robot.preempt()
                    
                    # Check if the current robot has any  dynamic objects that need backend update
                    self.update_dynamic_objects(robot=robot)
                
                    # Step the current robot (based on user input via ROS)
                    robot.step(dt=dt)

                    # Check if requested (resets overall for all robots in scene)
                    if robot.backend_reset: backend_reset = True

                # with Timer('step'):
                self.backend.step(dt=dt)

                for backend, args in self.readonly_backends:
                    backend.step(dt=dt)

                self.publish_transforms()

                self.rate.sleep()

                self.last_tick = current_time
