import sys
import puzz
import pdqpq


MAX_SEARCH_ITERS = 100000
GOAL_STATE = puzz.EightPuzzleBoard("012345678")


def bfs(start_state):
    explored = pdqpq.PriorityQueue()
    frontier = pdqpq.PriorityQueue()
    frontier.add(start_state)
    parents = {}
    moves = {}
    path = []
    path_cost = 0
    frontier_count = 1
    expanded_count = 0
    if start_state == GOAL_STATE:
        frontier_count = 0
        expanded_count = 0
        path = [("start", start_state)]
    else:
        while not frontier.empty():
            node = frontier.pop()
            expanded_count += 1

            current = node
            explored.add(node)
            tempCurr = current
            trueCostValue = 0
            for k, n in node.successors().items():
                move = k
                if (n not in frontier) and (n not in explored):

                    if is_goal(n):
                        path = [(k, n)] + path
                        prev = n
                        while parents.get(current):

                            path_cost += int(str(prev)[int(str(current).index("0"))]) ** 2
                            move = moves[current]
                            path = [(move, current)] + path # move is key of value current
                            prev = current
                            current = parents[current]

                        path_cost += int(str(prev)[int(str(current).index("0"))]) ** 2
                        path = [('start', start_state)] + path

                        return path, path_cost, frontier_count, expanded_count
                    else:

                        frontier.add(n)
                        frontier_count += 1
                        parents[n] = node
                        moves[n] = k

    return path, path_cost, frontier_count, expanded_count


def is_goal(n):
    return n == GOAL_STATE


def ucost(start_state):
    explored = pdqpq.PriorityQueue()
    frontier = pdqpq.PriorityQueue()
    frontier.add(start_state, 0)
    parents = {}
    moves = {}
    path = []
    path_cost = 0
    frontier_count = 1
    expanded_count = 0
    costs = {start_state: 0}

    while not frontier.empty():
        node = frontier.pop()

        current = node
        if is_goal(node):

            while parents.get(current):

                move = moves[current]
                path = [(move, current)] + path  # move is key of value current

                current = parents[current]
            path_cost = costs[node]

            path = [('start', start_state)] + path
            return path, path_cost, frontier_count, expanded_count
        else:
            expanded_count += 1
            explored.add(node)

            for k, n in node.successors().items():

                cost = int(str(node)[int(str(n).index("0"))]) ** 2 + costs[node]

                if (n not in frontier) and (n not in explored):

                    frontier.add(n, cost)
                    frontier_count += 1
                    parents[n] = node
                    moves[n] = k
                    costs[n] = cost

                elif (n in frontier) and (frontier.get(n) > cost):
                    frontier.add(n, cost)
                    moves[n] = k
                    costs[n] = cost
                    parents[n] = node

    return path, path_cost, frontier_count, expanded_count


def greedyh1(start_state):
    explored = pdqpq.PriorityQueue()
    frontier = pdqpq.PriorityQueue()
    frontier.add(start_state, addCost(start_state, GOAL_STATE))
    parents = {}
    moves = {}
    path = []
    path_cost = 0
    frontier_count = 1
    expanded_count = 0
    costs = {start_state: addCost(start_state, GOAL_STATE)}
    trueCosts = {start_state: 0}

    while not frontier.empty():
        node = frontier.pop()

        current = node
        if is_goal(node):
           # print(costs)
            while parents.get(current):
                move = moves[current]
                path = [(move, current)] + path  # move is key of value current
                current = parents[current]
            path_cost = trueCosts[node]
            path = [('start', start_state)] + path
            return path, path_cost, frontier_count, expanded_count

        else:
            expanded_count += 1
            explored.add(node)

            for k, n in node.successors().items():

                cost = addCost(n, GOAL_STATE)
                trueCostValue = int(str(node)[int(str(n).index("0"))]) ** 2 + trueCosts[node]

                if (n not in frontier) and (n not in explored):

                    frontier.add(n, cost)
                    frontier_count += 1
                    parents[n] = node
                    moves[n] = k
                    costs[n] = cost
                    trueCosts[n] = trueCostValue

                elif (n in frontier) and (frontier.get(n) > cost):
                    frontier.add(n, cost)
                    moves[n] = k
                    costs[n] = cost
                    parents[n] = node
                    trueCosts[n] = trueCostValue

    return path, path_cost, frontier_count, expanded_count


def addCost(state, end):

    total = 0
    for i in range(len(str(state))):

        if str(state)[i] != str(end)[i] and str(state)[i] != '0':
            total += 1
    return total


def greedyh2(start_state): # Number of misplaced tiles
    explored = pdqpq.PriorityQueue()
    frontier = pdqpq.PriorityQueue()
    frontier.add(start_state, manhattanCost(start_state, GOAL_STATE))
    parents = {}
    moves = {}
    path = []
    path_cost = 0
    frontier_count = 1
    expanded_count = 0
    costs = {start_state: manhattanCost(start_state, GOAL_STATE)}
    trueCosts = {start_state: 0}

    while not frontier.empty():
        node = frontier.pop()

        current = node
        if is_goal(node):
            # print(costs)
            while parents.get(current):
                move = moves[current]
                path = [(move, current)] + path  # move is key of value current
                current = parents[current]
            path_cost = trueCosts[node]
            path = [('start', start_state)] + path
            return path, path_cost, frontier_count, expanded_count

        else:
            expanded_count += 1
            explored.add(node)

            for k, n in node.successors().items():

                cost = manhattanCost(n, GOAL_STATE)
                trueCostValue = int(str(node)[int(str(n).index("0"))]) ** 2 + trueCosts[node]

                if (n not in frontier) and (n not in explored):

                    frontier.add(n, cost)
                    frontier_count += 1
                    parents[n] = node
                    moves[n] = k
                    costs[n] = cost
                    trueCosts[n] = trueCostValue

                elif (n in frontier) and (frontier.get(n) > cost):
                    frontier.add(n, cost)
                    moves[n] = k
                    costs[n] = cost
                    parents[n] = node
                    trueCosts[n] = trueCostValue

    return path, path_cost, frontier_count, expanded_count


def manhattanCost(state, end):
    manCost = 0
    if state == end:
        manCost = 1
    else:
      #  print(state.pretty())
        for i in range(len(str(state))):

            correctTile = end.find(str(end)[i])

            if str(str(end)[i]) != "0":

                tile = state.find(str(end)[i])
              #  print("Tile 0: ", tile[0], correctTile[0])

                manCost += abs(tile[0] - correctTile[0]) + abs(tile[1] - correctTile[1])

        #print("Cost: ", manCost)

    return manCost


def greedyh3(start_state):

    explored = pdqpq.PriorityQueue()
    frontier = pdqpq.PriorityQueue()
    frontier.add(start_state, weightedAddCost(start_state, GOAL_STATE))
    parents = {}
    moves = {}
    path = []
    path_cost = 0
    frontier_count = 1
    expanded_count = 0
    costs = {start_state: weightedAddCost(start_state, GOAL_STATE)}
    trueCosts = {start_state: 0}

    while not frontier.empty():
        node = frontier.pop()

        current = node
        if is_goal(node):
            # print(costs)
            while parents.get(current):
                move = moves[current]
                path = [(move, current)] + path  # move is key of value current
                current = parents[current]
            path_cost = trueCosts[node]
            path = [('start', start_state)] + path
            return path, path_cost, frontier_count, expanded_count

        else:
            expanded_count += 1
            explored.add(node)

            for k, n in node.successors().items():

                cost = weightedAddCost(n, GOAL_STATE)
                trueCostValue = int(str(node)[int(str(n).index("0"))]) ** 2 + trueCosts[node]

                if (n not in frontier) and (n not in explored):

                    frontier.add(n, cost)
                    frontier_count += 1
                    parents[n] = node
                    moves[n] = k
                    costs[n] = cost
                    trueCosts[n] = trueCostValue

                elif (n in frontier) and (frontier.get(n) > cost):
                    frontier.add(n, cost)
                    moves[n] = k
                    costs[n] = cost
                    parents[n] = node
                    trueCosts[n] = trueCostValue

    return path, path_cost, frontier_count, expanded_count


def weightedAddCost(state, end):
 # print(state.pretty())
    manCost = 0
    costAdd = 0
    total = 0
    if state == end:
        manCost = 1
        costAdd = 1
        total = 1
    else:
        #  print(state.pretty())
      #  print("-------------------")
        for i in range(len(str(state))):

            correctTile = end.find(str(end)[i])
           # print("Tile: ", str(end)[i])
            if str(state)[i] != str(end)[i]:
                costAdd = 1

            if str(str(end)[i]) != "0":
                tile = state.find(str(end)[i])

                manCost = abs(tile[0] - correctTile[0]) + abs(tile[1] - correctTile[1])

            total += int(str(end)[i])**2 * manCost
    return total


def astarh1(start_state):

    explored = pdqpq.PriorityQueue()
    frontier = pdqpq.PriorityQueue()
    frontier.add(start_state, 0)
    parents = {}
    moves = {}
    path = []
    path_cost = 0
    frontier_count = 1
    expanded_count = 0
    costs = {start_state: addCost(start_state, GOAL_STATE)}
    trueCosts = {start_state: 0}

    while not frontier.empty():
        node = frontier.pop()

        current = node
        if is_goal(node):
            # print(costs)
            while parents.get(current):
                move = moves[current]
                path = [(move, current)] + path  # move is key of value current
                current = parents[current]
            path_cost = trueCosts[node]
            path = [('start', start_state)] + path
            return path, path_cost, frontier_count, expanded_count

        else:
            expanded_count += 1
            explored.add(node)

            for k, n in node.successors().items():
                trueCostValue = int(str(node)[int(str(n).index("0"))]) ** 2 + trueCosts[node]
                cost = addCost(n, GOAL_STATE) + trueCostValue

                if (n not in frontier) and (n not in explored):
                    frontier.add(n, cost)
                    frontier_count += 1
                    parents[n] = node
                    moves[n] = k
                    costs[n] = cost
                    trueCosts[n] = trueCostValue

                elif (n in frontier) and (frontier.get(n) > cost):
                    frontier.add(n, cost)
                    moves[n] = k
                    costs[n] = cost
                    parents[n] = node
                    trueCosts[n] = trueCostValue

    return path, path_cost, frontier_count, expanded_count


def astarh2(start_state):

    explored = pdqpq.PriorityQueue()
    frontier = pdqpq.PriorityQueue()
    frontier.add(start_state, 0)
    parents = {}
    moves = {}
    path = []
    path_cost = 0
    frontier_count = 1
    expanded_count = 0
    costs = {start_state: manhattanCost(start_state, GOAL_STATE)}
    trueCosts = {start_state: 0}

    while not frontier.empty():
        node = frontier.pop()

        current = node
        if is_goal(node):
            # print(costs)
            while parents.get(current):
                move = moves[current]
                path = [(move, current)] + path  # move is key of value current
                current = parents[current]
            path_cost = trueCosts[node]
            path = [('start', start_state)] + path
            return path, path_cost, frontier_count, expanded_count

        else:
            expanded_count += 1
            explored.add(node)

            for k, n in node.successors().items():
                trueCostValue = int(str(node)[int(str(n).index("0"))]) ** 2 + trueCosts[node]
                cost = manhattanCost(n, GOAL_STATE) + trueCostValue

                if (n not in frontier) and (n not in explored):
                    frontier.add(n, cost)
                    frontier_count += 1
                    parents[n] = node
                    moves[n] = k
                    costs[n] = cost
                    trueCosts[n] = trueCostValue

                elif (n in frontier) and (frontier.get(n) > cost):
                    frontier.add(n, cost)
                    moves[n] = k
                    costs[n] = cost
                    parents[n] = node
                    trueCosts[n] = trueCostValue

    return path, path_cost, frontier_count, expanded_count


def astarh3(start_state):

    explored = pdqpq.PriorityQueue()
    frontier = pdqpq.PriorityQueue()
    frontier.add(start_state, 0)
    parents = {}
    moves = {}
    path = []
    path_cost = 0
    frontier_count = 1
    expanded_count = 0
    costs = {start_state: weightedAddCost(start_state, GOAL_STATE)}
    trueCosts = {start_state: 0}

    while not frontier.empty():
        node = frontier.pop()

        current = node
        if is_goal(node):
            # print(costs)
            while parents.get(current):
                move = moves[current]
                path = [(move, current)] + path  # move is key of value current
                current = parents[current]
            path_cost = trueCosts[node]
            path = [('start', start_state)] + path
            return path, path_cost, frontier_count, expanded_count

        else:
            expanded_count += 1
            explored.add(node)

            for k, n in node.successors().items():
                trueCostValue = int(str(node)[int(str(n).index("0"))]) ** 2 + trueCosts[node]
                cost = weightedAddCost(n, GOAL_STATE) + trueCostValue

                if (n not in frontier) and (n not in explored):
                    frontier.add(n, cost)
                    frontier_count += 1
                    parents[n] = node
                    moves[n] = k
                    costs[n] = cost
                    trueCosts[n] = trueCostValue

                elif (n in frontier) and (frontier.get(n) > cost):
                    frontier.add(n, cost)
                    moves[n] = k
                    costs[n] = cost
                    parents[n] = node
                    trueCosts[n] = trueCostValue

    return path, path_cost, frontier_count, expanded_count


def solve_puzzle(start_state, strategy):
    """Perform a search to find a solution to a puzzle.
    
    Args:
        start_state: an EightPuzzleBoard object indicating the start state for the search
        flavor: a string indicating which type of search to run.  Can be one of the following:
            'bfs' - breadth-first search
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
    if strategy == "bfs":
        returned = bfs(start_state)
    elif strategy == "ucost":
        returned = ucost(start_state)

    elif strategy == "greedy-h1":
        returned = greedyh1(start_state)

    elif strategy == "greedy-h2":
        returned = greedyh2(start_state)

    elif strategy == "greedy-h3":
        returned = greedyh3(start_state)

    elif strategy == "astar-h1":
        returned = astarh1(start_state)

    elif strategy == "astar-h2":
        returned = astarh2(start_state)

    elif strategy == "astar-h3":
        returned = astarh3(start_state)

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
