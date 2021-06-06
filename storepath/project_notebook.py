
from helpers import Map, load_map_10, show_map
import math
from tkinter import *
import tkinter as tk



map_10 = load_map_10()







class PathPlanner():
    """Construct a PathPlanner Object"""
    def __init__(self, M, start=None, goal=None):
        """ """
        self.map = M
        self.start= start
        self.goal = goal
        self.closedSet = self.create_closedSet() if goal != None and start != None else None
        self.openSet = self.create_openSet() if goal != None and start != None else None
        self.cameFrom = self.create_cameFrom() if goal != None and start != None else None
        self.gScore = self.create_gScore() if goal != None and start != None else None
        self.fScore = self.create_fScore() if goal != None and start != None else None
        self.path = self.run_search() if self.map and self.start != None and self.goal != None else None
    
    def reconstruct_path(self, current):
        """ Reconstructs path after search """
        total_path = [current]
        while current in self.cameFrom.keys():
            current = self.cameFrom[current]
            total_path.append(current)
        return total_path
    
    def _reset(self):

        self.closedSet = None
        self.openSet = None
        self.cameFrom = None
        self.gScore = None
        self.fScore = None
        self.path = self.run_search() if self.map and self.start and self.goal else None

    def run_search(self):
        """ """
        if self.map == None:
            raise(ValueError, "Must create map before running search. Try running PathPlanner.set_map(start_node)")
        if self.goal == None:
            raise(ValueError, "Must create goal node before running search. Try running PathPlanner.set_goal(start_node)")
        if self.start == None:
            raise(ValueError, "Must create start node before running search. Try running PathPlanner.set_start(start_node)")

        self.closedSet = self.closedSet if self.closedSet != None else self.create_closedSet()
        self.openSet = self.openSet if self.openSet != None else  self.create_openSet()
        self.cameFrom = self.cameFrom if self.cameFrom != None else  self.create_cameFrom()
        self.gScore = self.gScore if self.gScore != None else  self.create_gScore()
        self.fScore = self.fScore if self.fScore != None else  self.create_fScore()

        while not self.is_open_empty():
            current = self.get_current_node()

            if current == self.goal:
                self.path = [x for x in reversed(self.reconstruct_path(current))]
                return self.path
            else:
                self.openSet.remove(current)
                self.closedSet.add(current)

            for neighbor in self.get_neighbors(current):
                if neighbor in self.closedSet:
                    continue    # Ignore the neighbor which is already evaluated.

                if not neighbor in self.openSet:    # Discover a new node
                    self.openSet.add(neighbor)
                
                # The distance from start to a neighbor
                #the "dist_between" function may vary as per the solution requirements.
                if self.get_tentative_gScore(current, neighbor) >= self.get_gScore(neighbor):
                    continue        # This is not a better path.

                # This path is the best until now. Record it!
                self.record_best_path_to(current, neighbor)
        print("No Path Found")
        self.path = None
        return False


# In[9]:


def create_closedSet(self):

    return set()


# In[10]:


def create_openSet(self):

    if self.start != None:
        # TODO: return a data structure suitable to hold the set of currently discovered nodes 
        # that are not evaluated yet. Make sure to include the start node.
        openSet = set()
        openSet.add(self.start)
        return openSet
    
    raise(ValueError, "Must create start node before creating an open set. Try running PathPlanner.set_start(start_node)")


# In[11]:


def create_cameFrom(self):

    cameFrom = {}
    return cameFrom


# In[12]:


def create_gScore(self):

    if self.start != None:
        gScore = {}
        for node in self.map.intersections.keys():
            if node == self.start:
                gScore[node] = 0
            else: gScore[node] = float('inf')
        return gScore
    raise(ValueError, "Must create start node before creating gScore. Try running PathPlanner.set_start(start_node)")


# In[13]:


def create_fScore(self):

    if self.start != None:
        fScore = {}
        for node in self.map.intersections.keys():
            if node == self.start:
                fScore[node] = self.heuristic_cost_estimate(self.start)
            else: fScore[node] = float('inf')
        return fScore
    raise(ValueError, "Must create start node before creating fScore. Try running PathPlanner.set_start(start_node)")


# In[14]:


def set_map(self, M):
    """Method used to set map attribute """
    self._reset(self)
    self.start = None
    self.goal = None
    # TODO: Set map to new value. 
    self.map = M


# In[15]:


def set_start(self, start):

    self._reset(self)

    self.start = start
    self.goal = None
    


# In[16]:


def set_goal(self, goal):

    self._reset(self)

    self.goal = goal


# In[17]:


def is_open_empty(self):

    return not bool(self.openSet)


# In[18]:


def get_current_node(self):

    current = None
    minim = float('inf')
    for node in self.openSet:
        if self.fScore[node] < minim:
            minim = self.fScore[node]
            current = node
    return current        


# In[19]:


def get_neighbors(self, node):

    return set(self.map.roads[node]) 


# In[20]:


def get_gScore(self, node):

    return self.gScore[node]


# In[21]:


def distance(self, node_1, node_2):

    x1 = self.map.intersections[node_1][0]
    y1 = self.map.intersections[node_1][1]
    x2 = self.map.intersections[node_2][0]
    y2 = self.map.intersections[node_2][1]
    dist = math.sqrt( (x2-x1)**2 + (y2-y1)**2 )
    return dist


# In[22]:


def get_tentative_gScore(self, current, neighbor):

    g_score_current = self.get_gScore(current)
    dist_current_neighbor = self.distance(current,neighbor)
    return g_score_current+dist_current_neighbor


# In[23]:


def heuristic_cost_estimate(self, node):

    if self.goal != None:
        heuristic_estimate = self.distance(node,self.goal)
        return heuristic_estimate
    raise(ValueError, "Must create goal node before calculating huristic estimate. Try running PathPlanner.set_goal(goal_node)")


# In[24]:


def calculate_fscore(self, node):

    f_score = self.get_gScore(node) + self.heuristic_cost_estimate(node)
    return f_score
    


# In[25]:


def record_best_path_to(self, current, neighbor):

    self.cameFrom[neighbor] = current
    self.gScore[neighbor] = self.get_tentative_gScore(current,neighbor)
    self.fScore[neighbor] = self.gScore[neighbor] + self.heuristic_cost_estimate(neighbor)


# In[26]:


# Associates implemented functions with PathPlanner class
PathPlanner.create_closedSet = create_closedSet
PathPlanner.create_openSet = create_openSet
PathPlanner.create_cameFrom = create_cameFrom
PathPlanner.create_gScore = create_gScore
PathPlanner.create_fScore = create_fScore
PathPlanner.set_map = set_map
PathPlanner.set_start = set_start
PathPlanner.set_goal = set_goal
PathPlanner.is_open_empty = is_open_empty
PathPlanner.get_current_node = get_current_node
PathPlanner.get_neighbors = get_neighbors
PathPlanner.get_gScore = get_gScore
PathPlanner.distance = distance
PathPlanner.get_tentative_gScore = get_tentative_gScore
PathPlanner.heuristic_cost_estimate = heuristic_cost_estimate
PathPlanner.calculate_fscore = calculate_fscore
PathPlanner.record_best_path_to = record_best_path_to

points = ['A','B','C','D','E','F','H','I','J','K','L','M','N','O','P','Q']
value = None
def menubar(start, str):
    width = 2
    height = 2
    master = Tk()
    tk.Label(master, text=str, bg='#aaa').pack(fill='x')

    variable = StringVar(master)
    variable.set(start[0]) # default value

    w = OptionMenu(master, variable, *start)
    w.config(width=width,height=height)
    w.pack()

    def ok():
        global value
        value = variable.get()
        master.destroy()


    button = Button(master, text="OK", command=ok)
    button.config(width=width)
    button.pack()

    mainloop()

def map_store(names,chr):
    for k,v in names.items():
        if v == chr:
            return k

store_names= {0:'A',1:'B',2:'C',3:'D',4:'F',5:'G',6:'H',7:'I',8:'J',9:'K',
              10:'L',11:'M',12:'N',13:'O',14:'P',15:'Q'}



menubar(points,"Select starting store")
start = map_store(store_names,value)
points.remove(value)
menubar(points,"Select Destination store")
goal = map_store(store_names,value)
planner = PathPlanner(map_10, start, goal)
path = planner.path
dist = planner.fScore
d = []
for p in path:
    d.append(int(dist[p]*10))



print(d)
show_map(map_10,path=path,dist=d)

