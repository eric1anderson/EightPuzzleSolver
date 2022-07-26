import sys
import puzz
import pdqpq


MAX_SEARCH_ITERS = 100000
GOAL_STATE = puzz.EightPuzzleBoard("012345678")

def is_goal(n):
    """

    Args:
        'n' - a state

    Returns:
        boolean checking if this state is the goal state

    """
    return n == GOAL_STATE

def addCost(state):
    """
    Counts the number of misplaced tiles from the current state to the goal state

    Args:
        'state' - current state

    Returns:
        number of misplaced tiles


    """
    total = 0
    for i in range(len(str(state))):

        if str(state)[i] != str(GOAL_STATE)[i] and str(state)[i] != '0':
            total += 1
    return total

def manhattanCost(state):
    """
    Distance to goal state by making up, down, left, right moves regardless of other tiles on board

    Args:
        'state' - current state

    Returns:
        Manhattan distance from current state to goal state

    """
    manCost = 0
    if state == GOAL_STATE:
        manCost = 1
    else:
        for i in range(len(str(state))):

            correctTile = GOAL_STATE.find(str(GOAL_STATE)[i])

            if str(str(GOAL_STATE)[i]) != "0":
                tile = state.find(str(GOAL_STATE)[i])
                manCost += abs(tile[0] - correctTile[0]) + abs(tile[1] - correctTile[1])
    return manCost

def weightedAddCost(state):
    """
    Weighted manhattan cost. Weight is equal to squared value of tile times Manhattan distance from goal state
    Args:
        'state' - current state

    Returns:
        weighted distance to goal state

    """
    manCost = 0
    total = 0
    if state == GOAL_STATE:
        total = 1
    else:
        for i in range(len(str(state))):

            correctTile = GOAL_STATE.find(str(GOAL_STATE)[i])

            if str(str(GOAL_STATE)[i]) != "0":
                tile = state.find(str(GOAL_STATE)[i])

                manCost = abs(tile[0] - correctTile[0]) + abs(tile[1] - correctTile[1])

            total += int(str(GOAL_STATE)[i])**2 * manCost
    return total


def generic_search(cost_function, start_state, search_type):
    """
        Generic search structure used as a skeleton for uniform cost, greedy, and A* search variants

        Args:
            'cost_function' - cost function to be used in search algorithm - one of: misplaced tile count heuristic,
                              Manhattan distance heuristic, weighted Manhattan distance heuristic
            'start_state' - initial position of tiles
            'search_type' - search algorithm type

        Returns:
            'path' -  a list of 2-tuples representing the path from the start state to the goal state
                (both should be included), with each entry being a (str, EightPuzzleBoard) pair
                indicating the move and resulting state for each action.  Omitted if the search
                fails.
            'path_cost' - the total cost of the path, taking into account the costs associated
                with each state transition.  Omitted if the search fails.
            'frontier_count' - the number of unique states added to the search frontier at any
                point during the search.
            'expanded_count' - the number of unique states removed from the frontier and expanded
                (successors generated).


    """
    explored = pdqpq.PriorityQueue()
    frontier = pdqpq.PriorityQueue()
    if search_type == "astar" or search_type == "ucost":
        frontier.add(start_state, 0)
    else:
        frontier.add(start_state, cost_function(start_state))
    parents = {}
    moves = {}
    path = []
    path_cost = 0
    frontier_count = 1
    expanded_count = 0
    costs = {start_state: cost_function(start_state)}
    trueCosts = {start_state: 0}

    while not frontier.empty():
        node = frontier.pop()

        current = node
        if is_goal(node):

            while parents.get(current):
                move = moves[current]
                path = [(move, current)] + path  # move is key of value current
                current = parents[current]
            if search_type == 'ucost':
                path_cost = costs[node]
            else:
                path_cost = trueCosts[node]
            path = [('start', start_state)] + path
            return path, path_cost, frontier_count, expanded_count

        else:
            expanded_count += 1
            explored.add(node)

            for k, n in node.successors().items():
                if search_type == 'ucost':
                    cost = int(str(node)[int(str(n).index("0"))]) ** 2 + costs[node]
                else:
                    trueCostValue = int(str(node)[int(str(n).index("0"))]) ** 2 + trueCosts[node]
                    if search_type == 'astar':
                        cost = cost_function(n) + trueCostValue
                    if search_type =='greedy':
                        cost = cost_function(n)


                if (n not in frontier) and (n not in explored):
                    frontier.add(n, cost)
                    frontier_count += 1
                    parents[n] = node
                    moves[n] = k
                    costs[n] = cost
                    if search_type != 'ucost':
                        trueCosts[n] = trueCostValue

                elif (n in frontier) and (frontier.get(n) > cost):
                    frontier.add(n, cost)
                    moves[n] = k
                    costs[n] = cost
                    parents[n] = node
                    if search_type != 'ucost':
                        trueCosts[n] = trueCostValue

    return path, path_cost, frontier_count, expanded_count


def greedyh1(start_state):
    # Greedy search that counts number of misplaced tiles
    return generic_search(addCost, start_state, 'greedy')


def greedyh2(start_state):
    # Greedy Manhattan cost search
    return generic_search(manhattanCost, start_state, 'greedy')

def greedyh3(start_state):
    # Greedy weighted Manhattan cost search
    return generic_search(weightedAddCost, start_state, 'greedy')

def astarh1(start_state):
    # A* search that counts number of misplaced tiles
    return generic_search(addCost, start_state, 'astar')


def astarh2(start_state):
    # A* Manhattan cost search
    return generic_search(manhattanCost, start_state, 'astar')


def astarh3(start_state):
    # A* weighted Manhattan cost search
    return generic_search(weightedAddCost, start_state, 'astar')



def solve_puzzle(start_state, strategy):
    """Perform a search to find a solution to a puzzle.
    
    Args:
        start_state: an EightPuzzleBoard object indicating the start state for the search
        flavor: a string indicating which type of search to run.  Can be one of the following:
            'ucost' - uniform-cost search
            'greedy-h1' - Greedy best-first search using a misplaced tile count heuristic - Number of misplaced tiles
            'greedy-h2' - Greedy best-first search using a Manhattan distance heuristic
            'greedy-h3' - Greedy best-first search using a weighted Manhattan distance heuristic
            'astar-h1' - A* search using a misplaced tile count heuristic
            'astar-h2' - A* search using a Manhattan distance heuristic
            'astar-h3' - A* search using a weighted Manhattan distance heuristic
    
    Returns: 
        A dictionary containing describing the search performed, containing the following entries:
            'path' -  a list of 2-tuples representing the path from the start state to the goal state
                (both should be included), with each entry being a (str, EightPuzzleBoard) pair
                indicating the move and resulting state for each action.  Omitted if the search
                fails.
            'path_cost' - the total cost of the path, taking into account the costs associated 
                with each state transition.  Omitted if the search fails.
            'frontier_count' - the number of unique states added to the search frontier at any
                point during the search.
            'expanded_count' - the number of unique states removed from the frontier and expanded 
                (successors generated).
    """
    results = {
        'path': [],
        'path_cost': 0,
        'frontier_count': 0,
        'expanded_count': 0,
    }

    returned = eval(strategy)(start_state)

    results['path'] = returned[0]
    results['path_cost'] = returned[1]
    results['frontier_count'] = returned[2]
    results['expanded_count'] = returned[3]

    return results


def print_summary(results):
    if 'path' in results:
        print("found solution of length {}, cost {}".format(len(results['path']), 
                                                            results['path_cost']))
        for move, state in results['path']:
            print("  {:5} {}".format(move, state))
    else:
        print("no solution found")
    print("{} states placed on frontier, {} states expanded".format(results['frontier_count'], 
                                                                    results['expanded_count']))


############################################

if __name__ == '__main__':

    start = puzz.EightPuzzleBoard(sys.argv[1])
    method = sys.argv[2]

    print("solving puzzle {} -> {}".format(start, GOAL_STATE))
    results = solve_puzzle(start, method)
    print_summary(results)
