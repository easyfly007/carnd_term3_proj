# ----------
# User Instructions:
# 
# Define a function, search() that returns a list
# in the form of [optimal path length, row, col]. For
# the grid shown below, your function should output
# [11, 4, 5].
#
# If there is no valid path from the start point
# to the goal, your function should return the string
# 'fail'
# ----------

# Grid format:
#   0 = Navigable space
#   1 = Occupied space

grid = [[0, 0, 1, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 1, 1, 1, 0],
        [0, 0, 0, 0, 1, 0]]
init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
print(goal)
cost = 1

delta = [[-1, 0], # go up
         [ 0,-1], # go left
         [ 1, 0], # go down
         [ 0, 1]] # go right

delta_name = ['^', '<', 'v', '>']

def find_min_state(openlist):
    if len(openlist) == 0:
        return None
    min_state = openlist[0]
    min_g = openlist[0][0]
    for state in openlist:
        if state[0] < min_g:
            min_g = state[0]
            min_state = state
    return min_state

def search(grid,init,goal,cost):
    init_state = [0, ] + init 
    openlist = [init_state,]

    path = None
    while len(openlist) > 0:
        print('now the openlist = ', openlist)
        min_state = find_min_state(openlist)
        if min_state == None:
            break
        openlist.remove(min_state)
        grid[min_state[1]][min_state[2]] = 1

        # check movement to next state
        for move in delta:
            next_state = min_state[:] # copy it
            next_state[0] += cost
            next_state[1] += move[0]
            next_state[2] += move[1]
            if next_state[1:] == goal:
                path = next_state
                break
            # check if out of boundry
            if next_state[1] < 0 or next_state[2] < 0:
                continue
            if next_state[1] >= len(grid) or next_state[2] >= len(grid[0]) :
                continue
            if grid[next_state[1]][next_state[2]] == 1:
                continue
            openlist.append(next_state)
        if path != None:
            break
    return path
path = search(grid,init,goal,cost)
print(path)