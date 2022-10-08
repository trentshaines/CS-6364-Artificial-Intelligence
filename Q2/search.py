# Funciton Implementation Details From: https://github.com/aimacode/aima-python/blob/master/search.py
import sys
from collections import deque

from utils import *

import math

class Node:
    """Class for a Node in the search tree, which contains state information, 
    the parent, a pointer to the node/state that preceded it, and the action that
    the parent took to arrive at the state, as well as the current total path cost, or g"""
    def __init__(self, state, parent=None, action=None, path_cost=0):
        """Create a search tree Node, derived from a parent by an action."""
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost
        self.depth = 0
        if parent:
            self.depth = parent.depth + 1

    def __repr__(self):
        return "<Node {}>".format(self.state)

    def __lt__(self, node):
        return self.state < node.state

    def expand(self, problem):
        """List the nodes reachable in one step from this node."""
        return [self.child_node(problem, action)
                for action in problem.actions(self.state)]

    def child_node(self, problem, action):
        """[Figure 3.10]"""
        next_state = problem.result(self.state, action)
        next_node = Node(next_state, self, action, problem.path_cost(self.path_cost, self.state, action, next_state))
        return next_node

    def solution(self):
        """Return the sequence of actions to go from the root to this node."""
        return [node.action for node in self.path()[1:]]

    def path(self):
        """Return a list of nodes forming the path from the root to this node."""
        node, path_back = self, []
        while node:
            path_back.append(node)
            node = node.parent
        return list(reversed(path_back))

    # We want for a queue of nodes in breadth_first_graph_search or
    # astar_search to have no duplicated states, so we treat nodes
    # with the same state as equal. [Problem: this may not be what you
    # want in other contexts.]

    def __eq__(self, other):
        return isinstance(other, Node) and self.state == other.state

    def __hash__(self):
        # We use the hash value of the state
        # stored in the node instead of the node
        # object itself to quickly search a node
        # with the same state in a Hash Table
        return hash(self.state)

    def expand2(self, problem): # Modified version of expand used for listing nodes
        return [self.child_node(problem, action).state for action in problem.actions(self.state)]

# Algorithms

def best_first_graph_search(problem, f, display=False):
    """Search the nodes with the lowest f scores first.
    You specify the function f(node) that you want to minimize; for example,
    if f is a heuristic estimate to the goal, then we have greedy best
    first search; if f is node.depth then we have breadth-first search.
    There is a subtlety: the line "f = memoize(f, 'f')" means that the f
    values will be cached on the nodes as they are computed. So after doing
    a best first search you can examine the f values of the path returned."""
    f = memoize(f, 'f')
    node = Node(problem.initial)
    frontier = PriorityQueue('min', f)
    frontier.append(node)
    explored = set()
    steps = 0
    while frontier:
        node = frontier.pop()
        if problem.goal_test(node.state):
            if steps < 5:
                print("\n(1) current: " + str(node.state) + "\n(2) frontier: " + str(frontier.heap) + "\n(3) explored/expanded: " + str(explored) + "\n(4) children: " + str(node.expand2(problem)))
            else:
                print("\n(1) current: " + str(node.state))
            steps += 1
            if display:
                print(len(explored), "paths have been expanded and",
                      len(frontier), "paths remain in the frontier")
            return node
        explored.add(node.state)
        for child in node.expand(problem):
            if child.state not in explored and child not in frontier:
                frontier.append(child)
            elif child in frontier:
                if f(child) < frontier[child]:
                    del frontier[child]
                    frontier.append(child)
        if steps < 5:
            print("\n(1) current: " + str(node.state) + "\n(2) frontier: " + str(frontier.heap) + "\n(3) explored/expanded: " + str(explored) + "\n(4) children: " + str(node.expand2(problem)))
        else:
            print("\n(1) current: " + str(node.state))
        steps += 1
    return None


def uniform_cost_search(problem, display=False):
    return best_first_graph_search(problem, lambda node: node.path_cost, display)


def depth_limited_search(problem, limit=50):
    frontier = []
    explored = set()
    i = 0

    def recursive_dls(node, problem, limit, i):
        frontier.pop()
        if problem.goal_test(node.state):
            if i < 5:
                print("\n(1) current: " + str(node.state) + "\n(2) frontier: " + str(frontier) + "\n(3) explored/expanded: " + str(explored) + "\n(4) children: " + str(node.expand2(problem)))
                i += 1
            else:
                print("\n(1) current: " + str(node.state))
            return node
        elif limit == 0:
            if i < 5:
                print("\n(1) current: " + str(node.state) + "\n(2) frontier: " + str(frontier) + "\n(3) explored/expanded: " + str(explored) + "\n(4) children: " + str(node.expand2(problem)))
                i += 1
            else:
                print("\n(1) current: " + str(node.state))
            return 'cutoff'
        else:
            explored.add(node.state)
            cutoff_occurred = False
            
            for child in node.expand(problem):
                frontier.append(child)
            
            if i < 5:
                print("\n(1) current: " + str(node.state) + "\n(2) frontier: " + str(frontier) + "\n(3) explored/expanded: " + str(explored) + "\n(4) children: " + str(node.expand2(problem)))
                i += 1
                
            else:
                print("\n(1) current: " + str(node.state))
            for child in node.expand(problem):
                result = recursive_dls(child, problem, limit - 1, i)
                if result == 'cutoff':
                    cutoff_occurred = True
                elif result is not None:
                    return result
            return 'cutoff' if cutoff_occurred else None

    # Body of depth_limited_search:
    frontier.append(problem.initial)
    return recursive_dls(Node(problem.initial), problem, limit, i)


def iterative_deepening_search(problem):
    for depth in range(sys.maxsize):
        result = depth_limited_search(problem, depth)
        if result != 'cutoff':
            return result


greedy_best_first_graph_search = best_first_graph_search


def astar_search(problem, h=None, display=False):
    """A* search is best-first graph search with f(n) = g(n)+h(n).
    You need to specify the h function when you call astar_search, or
    else in your Problem subclass."""
    h = memoize(h or problem.h, 'h')
    return best_first_graph_search(problem, lambda n: n.path_cost + h(n), display)


def recursive_best_first_search(problem, h=None):
    h = memoize(h or problem.h, 'h')
    i = 0
    
    frontier = []
    explored = set()

    def RBFS(problem, node, flimit, i):
        frontier.pop()
        if problem.goal_test(node.state):
            if i < 5:
                print("\n(1) current: " + str(node.state) + "\n(2) frontier: " + str(frontier) + "\n(3) explored/expanded: " + str(explored) + "\n(4) children: " + str(node.expand2(problem)))
                i += 1
            else:
                print("\n(1) current: " + str(node.state))
            return node, 0  # (The second value is immaterial)
        explored.add(node.state)
        successors = node.expand(problem)
        if len(successors) == 0:
            if i < 5:
                print("\n(1) current: " + str(node.state) + "\n(2) frontier: " + str(frontier) + "\n(3) explored/expanded: " + str(explored) + "\n(4) children: " + str(node.expand2(problem)))
                i += 1
            else:
                print("\n(1) current: " + str(node.state))
            return None, math.inf
        for s in successors:
            frontier.append(s)
            s.f = max(s.path_cost + h(s), node.f)
        while True:
            # Order by lowest f value
            successors.sort(key=lambda x: x.f)
            best = successors[0]
            if len(successors) > 1:
                alternative = successors[1].f
            else:
                alternative = math.inf
            if i < 5:
                print("\n(1) current: " + str(node.state) + "\n(2) frontier: " + str(frontier) + "\n(3) explored/expanded: " + str(explored) + "\n(4) children: " + str(node.expand2(problem)))
                i += 1
            else:
                print("\n(1) current: " + str(node.state))
            if best.f > flimit:
                return None, best.f
            result, best.f = RBFS(problem, best, min(flimit, alternative), i)
            if result is not None:
                return result, best.f

    node = Node(problem.initial)
    node.f = h(node)
    
    frontier.append(problem.initial)
    result, bestf = RBFS(problem, node, math.inf, i)
    return result
