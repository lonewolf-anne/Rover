import matplotlib
matplotlib.use('TkAgg') # Forces the windowing system
import matplotlib.pyplot as plt



import heapq
import numpy as np
import matplotlib.pyplot as plt
import random
#random.seed(42)  # Set a seed for reproducibility
# WIDTH=x  |    HEIGHT=y
#with=how far left to right
#height=how far up and down

#Python always goes row before column,meaning [y][x] instead of [x][y]
#but on normal graph it's x,y and x=row and y=column
class roverGrid:
    def __init__(self,width,height):
        self.width=width
        self.height=height
        self.costs=[[random.randint(1,5)for _ in range(width)] for _ in range(height)] 
        #This creates a 2D list (matrix) of random costs between 1 and 5 for each cell in the grid.
    def in_bounds(self,pos):    #this prevents the rover from going out of bounds of the grid
        x,y=pos
        return 0<=x<self.width and 0<=y<self.height
    def get_neighbors(self,pos):
        x,y=pos
        #up,-y
        # down,+y
        # left,-x
        # right,+x
        neighbors_result=[(x,y-1),
                          (x,y+1),
                          (x-1,y),
                          (x+1,y)]
        return [pos for pos in neighbors_result if self.in_bounds(pos)]
    
    def get_cost(self,pos):
        x,y=pos
        #unpacked because python understands the grid from indexing as [y][x] not tuples how we defined it as (x,y) in the pos tuple, so we have to unpack it to access the correct cost from the costs matrix.
        return self.costs[y][x]  #because of the way we defined the grid, we access it as [y][x]
def heuristic(a,b):
    #Unppack point a(current node) and point b(goal node)
    x1,y1=a
    x2,y2=b
    #Calculate the Manhattan distance between the two points, which is the sum of the absolute differences of their coordinates.
    return abs(x1-x2)+abs(y1-y2)

def a_star_search(grid,start,goal):
    frontier=[] # this is the to-do list of nodes to explore, we will use a priority queue (min-heap) to always expand the node with the lowest cost first.
    start_priority=0+heuristic(start,goal) # the priority of the starting node is just the heuristic cost to the goal since we haven't moved yet.
    heapq.heappush(frontier,(start_priority,start)) # we push the starting node onto the frontier with its priority.
    
    came_from={start: None}
    cost_so_far={start: 0}
    
    while frontier:
        current_cost,current_node=heapq.heappop(frontier)
        if current_node==goal:
            break
        for next_node in grid.get_neighbors(current_node):
            new_cost=cost_so_far[current_node]+grid.get_cost(next_node)
            
            
            if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                cost_so_far[next_node]=new_cost
                
                
                
                priority=new_cost+heuristic(next_node,goal) # the priority is the actual cost to reach the node plus the heuristic cost to the goal.
                heapq.heappush(frontier,(priority, next_node))
                
                came_from[next_node]=current_node
    return came_from,cost_so_far



#first algorithm Dijkstra's algorithm
# we are going to put this outside the class because to seperate the concerns of the grid and the pathfinding algorithm, we want to be able to use the same algorithm on different types of grids or environments in the future without having to modify the grid class itself.
def dijkstra_search(grid,start,goal):
    frontier=[] # this is the to-do list of nodes to explore, we will use a priority queue (min-heap) to always expand the node with the lowest cost first.
    heapq.heappush(frontier,(0,start)) # we push the starting node onto the frontier with a cost of 0.
    
    came_from={start: None} # this dictionary will keep track of the path we took to reach each node, we initialize it with the starting node having no parent (None).
    cost_so_far={start: 0}  #this is the ledger that keeps the values of the cost to reach each node, we initialize it with the starting node having a cost of 0.
    
    while frontier:
        current_cost,current_node=heapq.heappop(frontier) # we pop the node with the lowest cost from the frontier, this will be our current node to explore.
        #node=coordinate
        if current_node==goal: # if we have reached the goal, we can stop the search.
            break
        #expansion phase
        for next_node in grid.get_neighbors(current_node):
            #this picks up the 1st turple in get.neighbors and assigns it to next_node, then it picks up the 2nd tuple and assigns it to next_node and so on until it has gone through all the neighbors of the current node.
            new_cost=cost_so_far[current_node]+grid.get_cost(next_node)
            if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                cost_so_far[next_node]=new_cost
                heapq.heappush(frontier,(new_cost, next_node))
                came_from[next_node]=current_node
    return came_from,cost_so_far
       
    #reconstructing the path from the goal back to the start using the came_from dictionary
def reconstruct_path(came_from,start,goal):
    current=goal  #we are currently at the goal 
    path=[]       # the notebook for keeping track of our backward movements
    while current != start:   #Keep goignt untill we reach the strat(0,0)
        path.append(current)    #write where you are currently at in the path notebook
        current=came_from[current]
    path.append(start)
    path.reverse() #reverse the path to get it from start to goal instead of goal to start
    return path

#Now we test but we have to ccreate our parameters for the grid and the start and goal nodes first.
width,height=20,20
start_node=(0,0)
goal_node=(19,19)

#Create the rocer's world
#this runs the __init_ method of the roverGrid class, which creates a grid with random costs for each cell.
my_map=roverGrid(width,height)

#Launch the search algorithm 
#this returns two dictionaries, "parents" and "costs"
parents, cost_ledger =a_star_search(my_map,start_node,goal_node)


#Check if we actually found the goal
if goal_node in parents:
    final_path=reconstruct_path(parents,start_node,goal_node)
    
    print("Mission Accomplished! Path found:")
    print(f'Total Waypoints: {len(final_path)}')
    print(f'Total (Cost) Spent: {cost_ledger[goal_node]}')
    print(f'The path:{final_path}')
else:
    print("Mission Failed! No path found to the goal.") 
    

def visualize_grid(grid,final_path=None):
    #first dispaly the grid(costs)
    plt.imshow(grid.costs,cmap='terrain', origin='upper')
    
    #overlay the path
    if final_path:
        #extract x and y coordinates, from your lists of (x,y)
        x_coords=[p[0] for p in final_path]
        y_coords=[p[1] for p in final_path]
        plt.plot(x_coords,y_coords,color='cyan',linewidth=2,label='Rover Path')
        
    plt.colorbar(label='Terrain Cost(1=smooth, 5=rocky)')
    plt.title('Autonomous Rover Pathfinding')
    plt.xlabel('X coordinate')
    plt.ylabel('Y coordinate')
    plt.legend()
    plt.show()
visualize_grid(my_map,final_path)




