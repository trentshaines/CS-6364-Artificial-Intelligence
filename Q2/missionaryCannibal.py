# references code from https://github.com/aimacode/aima-python/blob/master/search.py
import sys
from collections import deque

from utils import *

import math

class Problem:
    """The abstract class for a formal problem. You should subclass
    this and implement the methods actions and result, and possibly
    __init__, goal_test, and path_cost. Then you will create instances
    of your subclass and solve them with the various search functions."""

    def __init__(self, initial, goal=None):
        """The constructor specifies the initial state, and possibly a goal
        state, if there is a unique goal. Your subclass's constructor can add
        other arguments."""
        self.initial = initial
        self.goal = goal

    def actions(self, state):
        """Return the actions that can be executed in the given
        state. The result would typically be a list, but if there are
        many actions, consider yielding them one at a time in an
        iterator, rather than building them all at once."""
        raise NotImplementedError

    def result(self, state, action):
        """Return the state that results from executing the given
        action in the given state. The action must be one of
        self.actions(state)."""
        raise NotImplementedError

    def goal_test(self, state):
        """Return True if the state is a goal. The default method compares the
        state to self.goal or checks for state in self.goal if it is a
        list, as specified in the constructor. Override this method if
        checking against a single self.goal is not enough."""
        if isinstance(self.goal, list):
            return is_in(state, self.goal)
        else:
            return state == self.goal

    def path_cost(self, c, state1, action, state2):
        """Return the cost of a solution path that arrives at state2 from
        state1 via action, assuming cost c to get up to state1. If the problem
        is such that the path doesn't matter, this function will only look at
        state2. If the path does matter, it will consider c and maybe state1
        and action. The default method costs 1 for every step in the path."""
        return c + 1

    def value(self, state):
        """For optimization problems, each state has a value. Hill Climbing
        and related algorithms try to maximize this value."""
        raise NotImplementedError

# ______________________________________________________________________________



# My Code:
class MissionaryCannibal(Problem):# Creates an instance of the missionary cannibal problem in Q2
    def __init__(self, initial=(3, 3, 0, 0, 0), goal=(0, 0, 3, 3, 1)):
        super().__init__(initial, goal)

    def checkMissionaryCannibalValid(self, newState, action):
        """Checks validity of a possible action on the current state by performing the action."""
        if action == 'LR2C':
            newState[1] -= 2
            newState[3] += 2
            newState[4] += 1
        elif action == 'RL2C':
            newState[1] += 2
            newState[3] -= 2       
            newState[4] -= 1
        elif action == 'LR2M':
            newState[0] -= 2
            newState[2] += 2
            newState[4] += 1
        elif action == 'RL2M':
            newState[0] += 2
            newState[2] -= 2
            newState[4] -= 1
        elif action == 'LR1C':
            newState[1] -= 1
            newState[3] += 1
            newState[4] += 1
        elif action == 'RL1C':
            newState[1] += 1
            newState[3] -= 1 
            newState[4] -= 1
        elif action == 'LR1M':
            newState[0] -= 1
            newState[2] += 1
            newState[4] += 1
        elif action == 'RL1M':
            newState[0] += 1
            newState[2] -= 1
            newState[4] -= 1
        elif action == 'LR1C1M':
            newState[0] -= 1
            newState[1] -= 1
            newState[2] += 1
            newState[3] += 1
            newState[4] += 1
        elif action == 'RL1C1M':
            newState[0] += 1
            newState[1] += 1
            newState[2] -= 1
            newState[3] -= 1
            newState[4] -= 1
            
        # Check for Validitiy
        if (newState[0] != 0 and newState[0] < newState[1]) or (newState[2] != 0 and newState[2] < newState[3]): # Less Missionaries Left Alone
            return False
        if newState[0] > 3 or newState[0] < 0 or newState[1] > 3 or newState[1] < 0 or newState[2] > 3 or newState[2] < 0 or newState[3] > 3 or newState[3] < 0: # Check if we have enough M/C to move
            return False
        if newState[4] < 0 or newState[4] > 1: # Check of boat on correct side
            return False
        return True

    def actions(self, state):
      possible_actions = []
      stateArr = list(state)
      if state[4] == 0:
          possible_actions = ['LR1M', 'LR1C', 'LR2M', 'LR2C', 'LR1C1M']
          possible_actions = [x for x in possible_actions if self.checkMissionaryCannibalValid(stateArr.copy(), x)]
      else:
          possible_actions = ['RL1M', 'RL1C', 'RL2M', 'RL2C', 'RL1C1M']
          possible_actions = [x for x in possible_actions if self.checkMissionaryCannibalValid(stateArr.copy(), x)]
      return possible_actions

    def result(self, lastState, action):# Returns a new state based off the current/last state and the desired action
        state = list(lastState)
        if action == 'LR2C':
            state[1] -= 2
            state[3] += 2
            state[4] += 1
        elif action == 'RL2C':
            state[1] += 2
            state[3] -= 2       
            state[4] -= 1
        elif action == 'LR2M':
            state[0] -= 2
            state[2] += 2
            state[4] += 1
        elif action == 'RL2M':
            state[0] += 2
            state[2] -= 2
            state[4] -= 1
        elif action == 'LR1C':
            state[1] -= 1
            state[3] += 1
            state[4] += 1
        elif action == 'RL1C':
            state[1] += 1
            state[3] -= 1 
            state[4] -= 1
        elif action == 'LR1M':
            state[0] -= 1
            state[2] += 1
            state[4] += 1
        elif action == 'RL1M':
            state[0] += 1
            state[2] -= 1
            state[4] -= 1
        elif action == 'LR1C1M':
            state[0] -= 1
            state[1] -= 1
            state[2] += 1
            state[3] += 1
            state[4] += 1
        elif action == 'RL1C1M':
            state[0] += 1
            state[1] += 1
            state[2] -= 1
            state[3] -= 1
            state[4] -= 1

        return tuple(state)

    def goal_test(self, state):
        return state == self.goal

    def path_cost(self, c, state1, action, state2):
        return c + 1

    def h(self, node):
        """ Returns the heuristic value for a state. h = 0 for the goal. h = 7 for initial. h(n) is defined as follows: 
            h(n) = 7 - #Cannibals on Right - #Missionaries on Right - 1 if boat on right side"""
        stateArr = list(node.state)
        hval = 7
        hval -= stateArr[2]
        hval -= stateArr[3]
        hval -= stateArr[4]
        return hval
