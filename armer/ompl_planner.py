#!/usr/bin/env python3
"""
Planner implementations to be used by ARMer
NOTE: testing ompl (implementation is based off: https://github.com/lyfkyle/pybullet_ompl)
.. codeauthor:: Dasun Gunasinghe
"""
# OMPL binding imports
from ompl import util as ou
from ompl import base as ob
from ompl import geometric as og

# Other required imports
import time
import copy
import rospy
import numpy as np
import spatialmath as sm
import roboticstoolbox as rtb

# Define a state space object (called by ompl)
# Based on implementation here: https://github.com/lyfkyle/pybullet_ompl
class StateSpace(ob.RealVectorStateSpace):
    def __init__(self, robot_dim) -> None:
        super().__init__(robot_dim)
        self.robot_dim = robot_dim
        self.state_sampler = None

    def alloc_state_sampler(self):
        """
        Called by ompl internally
        """
        rospy.logwarn(f"This will cause problems if the underlying planner is multi-threaded")
        if self.state_sampler:
            return self.state_sampler
        
        # when ompl planner calls this, we will return our sampler
        return self.allocDefaultStateSampler()
    
    def set_state_sampler(self, state_sampler):
        """
        Optional method to set a custom state sampler
        """
        self.state_sampler = state_sampler

# Based on implementation here: https://github.com/lyfkyle/pybullet_ompl
# Adapted for use with ARMer
class OMPLInterface():
    def __init__(self, 
        robot: rtb.Robot, 
        obstacles: list = [],
        iterations: int = 500) -> None:
        """
        OMPL python binding interface for a toolbox derived robot
        """
        if not isinstance(robot, rtb.Robot):
            rospy.logwarn(f"robot type incompatible -> expecting type: {type(rtb.Robot)}")
            return
        
        self.robot = robot
        self.iterations = iterations
        self.robot_dim = len(robot.q)

        print(f"robot dim: {self.robot_dim}")
        if self.robot_dim <= 0:
            rospy.logwarn(f"robot dim is invalid: {self.robot_dim}")
            return
        
        # Create a state space object
        self.ss = StateSpace(robot_dim=self.robot_dim)

        # Set the bounds of the robot
        robot_bounds = ob.RealVectorBounds(self.robot_dim)
        # This is a 2xN array
        # TODO: add error checking on size
        lower_lim = self.robot.qlim[0,:]
        upper_lim = self.robot.qlim[1,:]
        # Apply joint bounds to robot bounds
        for idx in range(self.robot_dim):
            robot_bounds.setLow(idx, lower_lim[idx])
            robot_bounds.setHigh(idx, upper_lim[idx])
        # Set bounds
        self.ss.setBounds(robot_bounds)

        # Create a simple setup with our state space
        self.simple_setup = og.SimpleSetup(self.ss)
        # Define the validity checking function
        self.simple_setup.setStateValidityChecker(
            ob.StateValidityCheckerFn(self.is_state_valid)
        )
        # Get space information
        self.si = self.simple_setup.getSpaceInformation()

        # Setup our obstacles with method
        self.set_obstacles(obstacles)
        # Setup our planner (RRT by default)
        self.set_planner("RRT")
    
    def set_obstacles(self, obstacles: list = []):
        self.obstacles = obstacles
        
        # Update collision detection
        # TODO: implement this
        self.setup_collision_detection(self.robot, self.obstacles)
        

    def add_obstacles(self, obstacle):
        self.obstacles.append(obstacle)
    
    def remove_obstacles(self, obstacle):
        self.obstacles.remove(obstacle)

    def is_state_valid(self, state):
        # TODO: need to implement this

        return True
            
    def setup_collision_detection(self, robot, obstacles):
        # TODO: need to implement this
        pass

    def set_state_sampler(self, state_sampler):
        self.ss.set_state_sampler(state_sampler)

    def state_to_list(self, state):
        return [state[i] for i in range(self.robot_dim)]
    
    def set_planner(self, planner_name):
        """
        Selects the required plan - add as needed
        NOTE: based on https://github.com/lyfkyle/pybullet_ompl
        """
        if planner_name == "PRM":
            self.planner = og.PRM(self.simple_setup.getSpaceInformation())
        elif planner_name == "RRT":
            self.planner = og.RRT(self.simple_setup.getSpaceInformation())
        elif planner_name == "RRTConnect":
            self.planner = og.RRTConnect(self.simple_setup.getSpaceInformation())
        elif planner_name == "RRTstar":
            self.planner = og.RRTstar(self.simple_setup.getSpaceInformation())
        elif planner_name == "EST":
            self.planner = og.EST(self.simple_setup.getSpaceInformation())
        elif planner_name == "FMT":
            self.planner = og.FMT(self.simple_setup.getSpaceInformation())
        elif planner_name == "BITstar":
            self.planner = og.BITstar(self.simple_setup.getSpaceInformation())
        else:
            rospy.logwarn(f"{planner_name} not recognized, please add it first")
            return

        self.simple_setup.setPlanner(self.planner)

    def plan(self, start, goal, allowed_time=5):
        """
        Determines a plan from provided start and goal states
        NOTE: default allowed time is 5 (seconds)
        """
        rospy.loginfo(f"TEST: ompl planner beginning...")
        rospy.loginfo(f"OMPL planner -> Current planner params: [{self.planner.params()}]")

        # Convert provided start and goal (joint states)
        # into ompl.base.State objects
        start_ob = ob.State(self.ss)
        goal_ob = ob.State(self.ss)
        for i in range(len(start)):
            start_ob[i] = start[i]
            goal_ob[i] = goal[i]
        self.simple_setup.setStartAndGoalStates(start_ob, goal_ob)

        # Attempt to solve within allocated time
        solved = self.simple_setup.solve(allowed_time)
        result = False
        sol_path_list = []
        if solved:
            rospy.loginfo(f"OMPL planner -> Found solution: interpolating into {self.iterations} segments")
            # print the path to screen
            sol_path_geometric = self.simple_setup.getSolutionPath()
            sol_path_geometric.interpolate(self.iterations)
            sol_path_states = sol_path_geometric.getStates()
            sol_path_list = [self.state_to_list(state) for state in sol_path_states]
            rospy.loginfo(f"OMPL planner -> iterations calculated: {len(sol_path_list)}")
            # print(sol_path_list)

            # TODO: Check if state is valid
            for sol_path in sol_path_list:
                self.is_state_valid(sol_path)
            result = True
        else:
            rospy.logwarn(f"OMPL planner -> No solution found")

        return result, sol_path_list