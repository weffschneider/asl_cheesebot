import numpy as np

# Represents a motion planning problem to be solved using A*
class AStar(object):

    def __init__(self, statespace_lo, statespace_hi, x_init, x_goal, occupancy, resolution):
        self.statespace_lo = statespace_lo         # state space lower bound (e.g., (-5, -5))
        self.statespace_hi = statespace_hi         # state space upper bound (e.g., (5, 5))
        self.occupancy = occupancy                 # occupancy grid
        self.resolution = resolution               # resolution of the discretization of state space (cell/m)
        self.x_init = self.snap_to_grid(x_init)    # initial state
        self.x_goal = self.snap_to_grid(x_goal)    # goal state

        self.closed_set = []    # the set containing the states that have been visited
        self.open_set = []      # the set containing the states that are candidate for future expension

        self.f_score = {}       # dictionary of the f score (estimated cost from start to goal passing through state)
        self.g_score = {}       # dictionary of the g score (cost-to-go from start to state)
        self.came_from = {}     # dictionary keeping track of each state's parent to reconstruct the path

        self.open_set.append(x_init)
        self.g_score[x_init] = 0
        self.f_score[x_init] = self.distance(x_init,x_goal)

        self.path = None        # the final path as a list of states

        self.dist_from_wall = 1.0 # distance from wall to keep

    # Checks if a give state is free, meaning it is inside the bounds of the map and
    # is not inside any obstacle
    # INPUT: (x)
    #          x - tuple state
    # OUTPUT: Boolean True/False
    def is_free(self, x):
        if x==self.x_init or x==self.x_goal:
            return True
        for dim in range(len(x)):
            if x[dim] < self.statespace_lo[dim]:
                return False
            if x[dim] >= self.statespace_hi[dim]:
                return False
        if not self.occupancy.is_free(x):
            return False
        return True

    # computes the euclidean distance between two states
    # INPUT: (x1, x2)
    #          x1 - first state tuple
    #          x2 - second state tuple
    # OUTPUT: Float euclidean distance
    def distance(self, x1, x2):
        return np.linalg.norm(np.array(x1)-np.array(x2))

    # returns the closest point on a discrete state grid
    # INPUT: (x)
    #          x - tuple state
    # OUTPUT: A tuple that represents the closest point to x on the discrete state grid
    def snap_to_grid(self, x):
        return (self.resolution*round(x[0]/self.resolution), self.resolution*round(x[1]/self.resolution))

    # gets the FREE neighbor states of a given state. Assumes a motion model
    # where we can move up, down, left, right, or along the diagonals by an
    # amount equal to self.resolution.
    # Use self.is_free in order to check if any given state is indeed free.
    # Use self.snap_to_grid (see above) to ensure that the neighbors you compute
    # are actually on the discrete grid, i.e., if you were to compute neighbors by
    # simply adding/subtracting self.resolution from x, numerical error could
    # creep in over the course of many additions and cause grid point equality
    # checks to fail. To remedy this, you should make sure that every neighbor is
    # snapped to the grid as it is computed.
    # INPUT: (x)
    #           x - tuple state
    # OUTPUT: List of neighbors that are free, as a list of TUPLES
    def get_neighbors(self, x):
        (xpos, ypos) = x
        moves = [[xpos-self.resolution, ypos],
                 [xpos+self.resolution, ypos],
                 [xpos, ypos+self.resolution],
                 [xpos, ypos-self.resolution],
                 [xpos-self.resolution*np.sqrt(2), ypos+self.resolution*np.sqrt(2)],
                 [xpos-self.resolution*np.sqrt(2), ypos-self.resolution*np.sqrt(2)],
                 [xpos+self.resolution*np.sqrt(2), ypos+self.resolution*np.sqrt(2)],
                 [xpos+self.resolution*np.sqrt(2), ypos-self.resolution*np.sqrt(2)]];

        neighbors = [tuple(self.snap_to_grid(el)) for el in moves if self.is_free(self.snap_to_grid(el))]

        return neighbors

    # Gets the state in open_set that has the lowest f_score
    # INPUT: None
    # OUTPUT: A tuple, the state found in open_set that has the lowest f_score
    def find_best_f_score(self):
        return min(self.open_set, key=lambda x: self.f_score[x])

    # Use the came_from map to reconstruct a path from the initial location
    # to the goal location
    # INPUT: None
    # OUTPUT: A list of tuples, which is a list of the states that go from start to goal
    def reconstruct_path(self):
        path = [self.x_goal]
        current = path[-1]
        while current != self.x_init:
            path.append(self.came_from[current])
            current = path[-1]
        return list(reversed(path))


    # def fix_animal_position(self, eps):

    #     goal = self.x_goal

    #     # find the wall closest to the goal position
    #     closest_line = None
    #     min_dist = 1000

    #     for obs in self.occupancy.obstacles:

    #         x1, y1 = obs[0]
    #         x2, y2 = obs[1]

    #         x0, y0 = goal
    #         dist = abs((y2-y1)*x0 - (x2-x1)*y0 + x2*y1 - y2*x1)/np.sqrt((y2-y1)**2 + (x2-x1)**2)

    #         if dist < min_dist:
    #             min_dist = dist
    #             closest_line = obs

    #     # Find slope of wall and a line perpendicular to the wall
    #     x1, y1 = obs[0]
    #     x2, y2 = obs[1]

    #     slope_wall = (y2-y1)/(x2-x1)
    #     slope_line = (1.,-1./slope_wall)

    #     # normalize slope_line, and add the line scaled by
    #     # (min_dist + eps), which gives new goal
    #     norm_line = np.norm(slope_line)


    #     new_goal = goal + slope_line/norm_line*(min_dist + eps)
    #     return new_goal

    # Solves the planning problem using the A* search algorithm. It places
    # the solution as a list of of tuples (each representing a state) that go
    # from self.x_init to self.x_goal inside the variable self.path
    # INPUT: None
    # OUTPUT: Boolean, True if a solution from x_init to x_goal was found
    def solve(self):

        # animal position is most likely beyond the wall
        # if not self.is_free(self.x_goal):
        #     self.x_goal = self.fix_animal_position(self.dist_from_wall)

        while len(self.open_set)>0:
            x_curr = self.find_best_f_score()

            if x_curr == self.x_goal:
                self.path = self.reconstruct_path()
                return True

            self.open_set.remove(x_curr)
            self.closed_set.append(x_curr)

            for x_neigh in self.get_neighbors(x_curr):
                if x_neigh in self.closed_set:
                    continue

                tentative_g_score = self.g_score[x_curr] + self.distance(x_curr, x_neigh)

                if x_neigh not in self.open_set:
                    self.open_set.append(x_neigh)
                elif (tentative_g_score > self.g_score[x_neigh]):
                    continue

                self.came_from[x_neigh] = x_curr
                self.g_score[x_neigh]   = tentative_g_score
                self.f_score[x_neigh]   = tentative_g_score + self.distance(x_neigh, self.x_goal)

        return False

# A 2D state space grid with a set of rectangular obstacles. The grid is fully deterministic
class DetOccupancyGrid2D(object):
    def __init__(self, width, height, obstacles):
        self.width = width
        self.height = height
        self.obstacles = obstacles

    def is_free(self, x):
        for obs in self.obstacles:
            inside = True
            for dim in range(len(x)):
                if x[dim] < obs[0][dim] or x[dim] > obs[1][dim]:
                    inside = False
                    break
            if inside:
                return False
        return True
