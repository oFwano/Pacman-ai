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

#####################################################
#####################################################
# Please enter the number of hours you spent on this
# assignment here
"""
num_hours_i_spent_on_this_assignment = 0
"""
#
#####################################################
#####################################################

#####################################################
#####################################################
# Give one short piece of feedback about the course so far. What
# have you found most interesting? Is there a topic that you had trouble
# understanding? Are there any changes that could improve the value of the
# course to you? (We will anonymize these before reading them.)
"""
<Your feedback goes here>

"""
#####################################################
#####################################################



"""

In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util
import heapq

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
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Q1.1
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print ( problem.getStartState() )
    You will get (5,5)

    print (problem.isGoalState(problem.getStartState()) )
    You will get True

    print ( problem.getSuccessors(problem.getStartState()) )
    You will get [((x1,y1),'South',1),((x2,y2),'West',1)]
    """
    "*** YOUR CODE HERE ***"
    #initialize frontier using the initial state of problem
    frontier = util.Stack()
    explored = []
    frontier.push( (problem.getStartState(), []) )

    while not frontier.isEmpty(): #loop do. if the frontier is empty return failure
        node = frontier.pop()     #choose leaf node and remove it from the frontier
        explored.append(node[0])
        if problem.isGoalState(node[0]): #if the node contains a goal state return the solutionv
            return node[1]

        for nextnode in problem.getSuccessors(node[0]): #expand the node, add successors to frontier
            if nextnode[0] not in explored:
                frontier.push((nextnode[0], node[1] + [nextnode[1]]))
    return None
def breadthFirstSearch(problem):
    """
    Q1.2
    Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"

    frontier = util.Queue()
    frontier.push( (problem.getStartState(), []) )
    explored = []
    explored.append(problem.getStartState())

    while not frontier.isEmpty():
        node = frontier.pop()
        if problem.isGoalState(node[0]):
            return node[1]

        for nextnode in problem.getSuccessors(node[0]):
            if nextnode[0] not in explored:
                explored.append(nextnode[0])
                frontier.push((nextnode[0], node[1] + [nextnode[1]] ))
    return None

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """
    Q1.3
    Search the node that has the lowest combined cost and heuristic first."""
    """Call heuristic(s,problem) to get h(s) value."""
    "*** YOUR CODE HERE ***"

    frontier = util.PriorityQueue()
    explored = []
    frontier.push((problem.getStartState(),[],0),heuristic(problem.getStartState(),problem))

    while not frontier.isEmpty():
        node = frontier.pop()
        if problem.isGoalState(node[0]):
            return node[1]
        if node[0] not in explored:
            explored.append(node[0])
            for nextnode in problem.getSuccessors(node[0]):
                cost = node[2] + nextnode[2]
                path = node[1] + [nextnode[1]]
                frontier.push((nextnode[0],path,cost),heuristic(nextnode[0],problem)+cost)

    return []

def priorityQueueDepthFirstSearch(problem):
    """
    Q1.4a.
    Reimplement DFS using a priority queue.
    """
    "*** YOUR CODE HERE ***"

    explored = []
    frontier = util.PriorityQueue()
    frontier.push((problem.getStartState(),[]),0)

    depth = 0
    while not frontier.isEmpty():
        node = frontier.pop()
        if problem.isGoalState(node[0]):
            return node[1]
        explored.append(node[0])

        for nextnode in problem.getSuccessors(node[0]):
            if nextnode[0] not in explored:
                path = node[1] + [nextnode[1]]
                depth = depth - 1
                frontier.push((nextnode[0],path),depth)


    return None



def priorityQueueBreadthFirstSearch(problem):
    """
    Q1.4b.
    Reimplement BFS using a priority queue.
    """
    "*** YOUR CODE HERE ***"
    frontier = util.PriorityQueue()
    frontier.push((problem.getStartState(),[]),0)
    explored = []
    explored.append(problem.getStartState)

    while not frontier.isEmpty():
        node = frontier.pop()
        if problem.isGoalState(node[0]):
            return node[1]

        for nextnode in problem.getSuccessors(node[0]):
            if nextnode[0] not in explored:
                explored.append(node[0])
                path = node[1] + [nextnode[1]]
                depth = len(node[1])
                frontier.push((nextnode[0],path),depth)

    return None

#####################################################
#####################################################
# Discuss the results of comparing the priority-queue
# based implementations of BFS and DFS with your original
# implementations.

"""
<Your discussion goes here>

<Answer to written questions>

Question 1.1: the exploration order is expected for depth-frist.
The search algorithm goes to the deepest node possible first before order paths.
Although we have explored the nodes, the Pac-man does not actually visit every node
on his path towards the goal. My resulting length of 130 is not a least cost solution
because DFS is an algorithm that returns the first solution which may or may not be the
shortest solution. in this case, it is not the shortest solution possible.


Question 1.4:
medium maze dfs node explored 146, cost 130
medium maze dfs2 nodes explored 146, cost 130
medium maze bfs nodes explored 269, cost 68
medium maze bfs2 node explored 275, cost 68
medium maze A* nodes explored 269, cost 68

Under comparison of these results bfs,bfs2 and A* all appear to find the least
cost solution of 68. On the other hand, the DFS algorithm found a solution of
the highest cost but with the least nodes explored. This is because the DFS algorithm
is not an optimal algorithm, it will return the first solution found if it is found.
Using DFS is a good idea if the user is only intereseted in looking for a solution
without care of the path being the most optimal. On the otherhand, BFS will always return
the shortest possible solution if a solution exisits. This is usually at the expense of more nodes
explored. Additionally, BFS is only optimal when actions are unweighted or uniformaly weighted.
If actions are weighted then it is better to use A*

"""



#####################################################
#####################################################



# Abbreviations (please DO NOT change these.)
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
bfs2 = priorityQueueBreadthFirstSearch
dfs2 = priorityQueueDepthFirstSearch
