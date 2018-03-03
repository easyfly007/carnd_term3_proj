# ----------
# User Instructions:
# 
# Create a function compute_value which returns
# a grid of values. The value of a cell is the minimum
# number of moves required to get from the cell to the goal. 
#
# If a cell is a wall or it is impossible to reach the goal from a cell,
# assign that cell a value of 99.
# ----------
import copy
grid = [[0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0]]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1 # the cost associated with moving from a cell to an adjacent one

delta = [[-1, 0 ], # go up
         [ 0, -1], # go left
         [ 1, 0 ], # go down
         [ 0, 1 ]] # go right

delta_name = ['^', '<', 'v', '>']

def get_prev_list(curr, value, delta):
    x_range = list(range(len(value)))
    y_range = list(range(len(value[0])))
    prev_list = []
    curr_x, curr_y = curr
    for move in delta:
        dx, dy = move
        prev = [curr_x - dx, curr_y - dy]
        if prev[0] in x_range and prev[1] in y_range:
            if value[prev[0]][prev[1]] == -1:
                prev_list.append(prev)
    return prev_list

def compute_value(grid,goal,cost):
    # ----------------------------------------
    # insert code below
    # ----------------------------------------
    
    # initialize
    x_range, y_range = list(range(len(grid))), list(range(len(grid[0])))
    value = [[-1,] * len(y_range) for i in x_range]
    for i in range(len(grid)):
        for j in range(len(grid[i])):
            if grid[i][j] == 1:
                value[i][j] = 99
    value[goal[0]][goal[1]] = 0

    open = [goal,]
    while len(open):
        open.reverse()
        curr = open.pop()
        prev_list = get_prev_list(curr, value, delta)
        open += prev_list
        for prev in prev_list:
            value[prev[0]][prev[1]] = value[curr[0]][curr[1]] + 1

    movement = copy.deepcopy(grid)
    for x1 in x_range:
        for y1 in y_range:
            min_val = 999
            if grid[x1][y1] == 1:
                continue
            for k in range(len(delta)):
                dx, dy = delta[k]
                x2, y2 = x1 + dx, y1 + dy
                if x2 in x_range and y2 in y_range:
                    if value[x2][y2] < min_val :
                        movement[x1][y1] = delta_name[k]
                        min_val = value[x2][y2]
    return movement

value = compute_value(grid, goal, cost)
for i in value:
    print(i)