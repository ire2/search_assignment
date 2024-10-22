# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
#
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util


class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem: SearchProblem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** BEGIN YOUR CODE HERE ***"
    start = problem.getStartState()
    fronteir = util.Stack()
    fronteir.push((start, []))
    visited = {start}
    # i = 0

    while fronteir:
        n, path = fronteir.pop()
        if problem.isGoalState(n):
            # print(f"Actions: {path}")
            return path
        visited.add(n)
        # print(f"Visited {i}: {visited}")
        # print(f"Fronteir at {i} : {fronteir.list}")
        for s, action, _ in problem.getSuccessors(n):
            # print(f"Successors at {ii} : {s} , {action}")
            if s not in visited:
                fronteir.push((s, path + [action]))
    return []

    "*** END YOUR CODE HERE ***"


def breadthFirstSearch(problem: SearchProblem):
    """Search the shallowest nodes in the search tree first."""
    "*** BEGIN YOUR CODE HERE ***"
    start = problem.getStartState()
    frontier = util.Queue()
    frontier.push((start, []))
    visited = set()

    while frontier:
        n, path = frontier.pop()
        if problem.isGoalState(n):
            return path
        if n not in visited:
            visited.add(n)
            for s, action, _ in problem.getSuccessors(n):
                if s not in visited:
                    new_path = path + [action]
                    frontier.push((s, new_path))
    return []
    "*** END YOUR CODE HERE ***"


def uniformCostSearch(problem: SearchProblem):
    """Search the node of least total cost first."""
    "*** BEGIN YOUR CODE HERE ***"
    # Adapted From:https://www.datacamp.com/tutorial/dijkstra-algorithm-in-python
    start = problem.getStartState()
    distances = {start: 0}
    frontier = util.PriorityQueue()
    frontier.push((start, []), 0)
    visited = set()

    while frontier:
        n, path = frontier.pop()
        print(f"n: {n}")
        if problem.isGoalState(n):
            # print(f"Distances: {distances}")
            # print(f"Path: {path}")
            return path
        if n not in visited:
            visited.add(n)
            for s, action, cost in problem.getSuccessors(n):
                new_cost = distances[n] + cost
                if s not in distances or new_cost < distances[s]:
                    distances[s] = new_cost
                    new_path = path + [action]
                    frontier.update((s, new_path), new_cost)
    return []
    "*** END YOUR CODE HERE ***"


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** BEGIN YOUR CODE HERE ***"

    start = problem.getStartState()
    g_cost = {start: 0}
    frontier = util.PriorityQueue()
    frontier.push((start, []), 0)
    visited = set()

    while frontier:
        n, path = frontier.pop()
        if problem.isGoalState(n):
            return path
        if n not in visited:
            visited.add(n)
            for successor, action, step_cost in problem.getSuccessors(n):
                new_g = g_cost[n] + step_cost
                f_cost = new_g + heuristic(successor, problem)
                if successor not in g_cost or new_g < g_cost[successor]:
                    g_cost[successor] = new_g
                    new_path = path + [action]
                    frontier.update((successor, new_path), f_cost)

    return []
    "*** END YOUR CODE HERE ***"


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
