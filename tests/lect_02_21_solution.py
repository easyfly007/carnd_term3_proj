# ----------
# User Instructions:
# 
# Implement the function optimum_policy2D below.
#
# You are given a car in grid with initial state
# init. Your task is to compute and return the car's 
# optimal path to the position specified in goal; 
# the costs for each motion are as defined in cost.
#
# There are four motion directions: up, left, down, and right.
# Increasing the index in this array corresponds to making a
# a left turn, and decreasing the index corresponds to making a 
# right turn.

forward = [[-1,  0], # go up
           [ 0, -1], # go left
           [ 1,  0], # go down
           [ 0,  1]] # go right
forward_name = ['up', 'left', 'down', 'right']

# action has 3 values: right turn, no turn, left turn
action = [-1, 0, 1]
action_name = ['R', '#', 'L']

# EXAMPLE INPUTS:
# grid format:
#     0 = navigable space
#     1 = unnavigable space 
grid = [[1, 1, 1, 0, 0, 0],
        [1, 1, 1, 0, 1, 0],
        [0, 0, 0, 0, 0, 0],
        [1, 1, 1, 0, 1, 1],
        [1, 1, 1, 0, 1, 1]]

init = [4, 3, 0] # given in the form [row,col,direction]
                 # direction = 0: up
                 #             1: left
                 #             2: down
                 #             3: right
                
goal = [2, 0] # given in the form [row,col]

cost = [2, 1, 20] # cost has 3 values, corresponding to making 
                  # a right turn, no turn, and a left turn

# EXAMPLE OUTPUT:
# calling optimum_policy2D with the given parameters should return 
# [[' ', ' ', ' ', 'R', '#', 'R'],
#  [' ', ' ', ' ', '#', ' ', '#'],
#  ['*', '#', '#', '#', '#', 'R'],
#  [' ', ' ', ' ', '#', ' ', ' '],
#  [' ', ' ', ' ', '#', ' ', ' ']]
# ----------

# ----------------------------------------
# modify code below
# ----------------------------------------

def optimum_policy2D(grid,init,goal,cost):
  x_size = len(grid)
  y_size = len(grid[0])
  d_size = 4
  value = [[[9999 for d in range(d_size)] for y in range(y_size)] for x in range(x_size)]
  policy3D = [[[' ' for d in range(d_size)] for y in range(y_size)] for x in range(x_size)]
  # policy3D indicate the direction movement

  x_goal = goal[0]
  y_goal = goal[1]
  for d in range(d_size):
    value[x_goal][y_goal][d] = 0
  
  changed = True
  while changed:
    changed = False
    for x in range(x_size):
      for y in range(y_size):
        if x == goal[0] and y == goal[1]:
          continue
        if grid[x][y] == 1:
          continue
        for d in range(d_size):
          min_val = value[x][y][d]
          for i in range(len(action)):
            next_d = d + action[i]
            if next_d < 0:
              next_d += 4
            if next_d >= 4:
              next_d -= 4
            next_x = x + forward[next_d][0]
            next_y = y + forward[next_d][1]
            if next_x < x_size and next_x >=0 and next_y >=0 and next_y < y_size:
              if grid[next_x][next_y] == 0:
                val = cost[i] + value[next_x][next_y][next_d]
                if val < min_val:
                  min_val = val
                  changed = True
                  value[x][y][d] = val
                  policy3D[x][y][d] = i
  # print('\npolicy3D = ')
  # for i in policy3D:
  #   print(i)

  # print('\nvalue = ')
  # for i in value:
  #   print(i)

  # now value is ready, we will try to build the policy2D
  policy2D = [[' ' for y in range(y_size)] for x in range(x_size)]
  policy2D[x_goal][y_goal] = '*'
  x1, y1, d1 = init[0], init[1], init[2]
  while (x1, y1) != (goal[0], goal[1]):
    a1 = policy3D[x1][y1][d1]
    policy2D[x1][y1] = action_name[a1]

    d1 = d1 + action[a1]
    if d1 >= 4:
      d1 -= 4
    if d1 < 0:
      d1 += 4
    x1 = x1 + forward[d1][0]
    y1 = y1 + forward[d1][1]

  return policy2D

policy2D =  optimum_policy2D(grid,init,goal,cost)
for i in policy2D:
  print(i)

