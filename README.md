# EightPuzzleSolver

Implementation of Uniform Cost, A*, and Greedy Search Algorithms to solve any Eight Puzzle and output steps taken

Run using the following command: python solver.py 'starting board state' 'search algorithm' 

Where: 
  'Starting board state' is any valid starting Eight Puzzle board with 0 as blank tile
  'Search algorithm' is one of ucost, astarh1, astarh2, astarh3, greedyh1, greedyh2, greedyh3
  
example starting board representation:
[8] [0] [2]
[3] [5] [6]
[1] [7] [4]  
example command: python solver.py 802356174 astar-h2 

Outputs the direction and resulting state of steps taken by search algorithm to find a valid solution in at most 100,000 steps

A* and Greedy Search Algorithms are each implemented with the following heuristics:
h1: misplaced tile count 
h2: Manhattan distance 
h3: weighted Manhattan distance -> Weight is equal to squared value of tile times Manhattan distance from goal state
