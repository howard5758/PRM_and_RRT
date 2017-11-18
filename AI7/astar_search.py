# Author: Ping-Jung Liu
# Date: September 26th 2017
# COSC 76 Assignment 2: Mazeworld
# Acknowledgement: Professor Devin Balkom for providing the general structure 
from SearchSolution import SearchSolution
from collections import deque
from heapq import heappush, heappop

class AstarNode:
    # each search node except the root has a parent node
    # and all search nodes wrap a state object

    def __init__(self, state, heuristic, parent=None, transition_cost=0):
        self.state = state
        self.heuristic = heuristic
        self.transition_cost = transition_cost
        self.parent = parent

    def priority(self):
        return self.heuristic + self.transition_cost
        # you write this part

    # comparison operator,
    # needed for heappush and heappop to work with AstarNodes:
    def __lt__(self, other):

        # I modified this part for extra credit
        # When the two priorities are identical, compare the heuristic
        # This modification saves decent amount of runtime
        if self.priority() < other.priority():
            return True
        elif self.priority() > other.priority():
            return False
        else:
            return self.heuristic < other.heuristic
        #return self.priority() < other.priority()


# take the current node, and follow its parents back
#  as far as possible. Grab the states from the nodes,
#  and reverse the resulting list of states.
def backchain(node):
    result = []
    current = node
    while current:
        result.append(current.state)
        current = current.parent

    result.reverse()
    return result


def astar_search(search_problem, heuristic_fn):
    # I'll get you started: 
    s_p = search_problem
    start_node = AstarNode(search_problem.start_state, heuristic_fn(search_problem.start_state))
    pqueue = []
    heappush(pqueue, start_node)
    solution = SearchSolution(search_problem, "Astar with heuristic " + heuristic_fn.__name__)

    visited_cost = {}
    visited_cost[start_node.state] = start_node.priority()

    # you write the rest:
    while len(pqueue) > 0:

        # pop the first element in the pqueue
        current = heappop(pqueue)
        
        if s_p.goal_test(current.state):
            solution.path = backchain(current)
            solution.cost = current.transition_cost
            return solution

        # if current has already been visited and its priority is larger, ignore 
        elif current.state in visited_cost and current.priority() > visited_cost[current.state]:
            continue

        solution.nodes_visited = solution.nodes_visited + 1

        successors = s_p.get_successors(current.state)

        # note the second element of child indicates whether the cost should be increased
        for child in successors:
            
            # initialize a new child node
            new_node = AstarNode(child[0], heuristic_fn(child[0])) 
            new_node.heuristic = heuristic_fn(child[0])
            new_node.parent = current

            # change the cost
            new_node.transition_cost = current.transition_cost + child[1]
            
            # determine whether to add the child or not
            #(not child[0] in visited_cost)
            if (not child[0] in visited_cost) or (visited_cost[child[0]] > new_node.priority()):
                heappush(pqueue, new_node)
                visited_cost[child[0]] = new_node.priority()

    solution.path = []

    return solution
    
##############################################################################################
# this part is merely for testing the functionality of model
def bfs_search(search_problem):
    # create solution
    solution = SearchSolution(search_problem, "BFS")
    s_p = search_problem

    # create frontier que
    que = deque()
    # the visited dict is a mapping between child and parent
    visited = {}

    # put start_state into que and visited
    que.append(s_p.start_state)
    visited[s_p.start_state] = None

    visited_nodes = 0
    while(not len(que) == 0):

        # get the first element in que
        toVisit = que.popleft()

        # if toVisit state is the goal, backchain and return solution
        if(s_p.goal_test(toVisit)):
            solution.path = bfs_backchain(toVisit, visited)
            solution.nodes_visited = visited_nodes
            return solution

        # if not the goal, obtain its successors
        successors = s_p.get_successors(toVisit)

        # loop through the successors, if not visited, put to the back of que and save into visited
        for child in successors:
            if(not child[0] in visited):
                que.append(child[0])
                visited[child[0]] = toVisit
                visited_nodes = visited_nodes + 1

    # if the search never reach goal state, return [] as path            
    solution.path = []
    solution.nodes_visited = visited_nodes

    return solution

def bfs_backchain(state, visited):

    # use the child and parent pairs in visited to track solution path
    result = [state] 
    prev = visited[state]

    # if prev = none, then it must be the start state --> break the loop
    while(not prev == None):
        result.append(prev)
        prev = visited[prev]

    # reverse the result for correct representation
    result.reverse()
    return result
