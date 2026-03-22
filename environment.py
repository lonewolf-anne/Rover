import random
import heapq
from time import time
class Environment:
    def __init__(self, width=50, height=50,obstacle_prob=0.15):
        self.width = width
        self.height = height
        self.obstacle_prob = obstacle_prob
        self.grid=self.generate_grid()
        self.start=(0,0)
        self.goal=(width-1,height-1)
        
        self.grid[0][0]=0  # Ensure start is not an obstacle
        self.grid[height-1][width-1]=0  # Ensure goal is not an obstacle
    def generate_grid(self):
        grid = []
        for y in range(self.height):
            row = []
            for x in range(self.width):
                if random.random() < self.obstacle_prob:
                    row.append(-1)  # -1 is an obstacle
                else:
                    terrain=random.choice([0, 1, 2])  # 0: smooth , 1: rocky, 2: rough
                    row.append(terrain)
            grid.append(row)
        return grid
    
    def is_within_bounds(self, x, y):
        return 0 <= x < self.width and 0 <= y < self.height
    
    def is_obstacle(self, x, y):
        return self.grid[y][x] == -1
    def get_cost(self, x, y):
        terrain =self.grid[y][x]
        if terrain==-1:
            return float('inf')  # Impassable
        energy={0:1,1:3,2:6}[terrain]  # Energy cost based on terrain type
        risk={0:1,1:4,2:9}[terrain]  # Risk of tipping cost based on terrain type
        energy_weight=0.5
        risk_weight=0.5
        return energy_weight*energy + risk_weight*risk
    #rules -never tip,never die with data onboard
    
    def get_neighbors(self, x,y):
                                                                         
        neighbors = [
            (x-1, y), #left  
            (x+1, y), #right
            (x, y-1), #up
            (x, y+1) #down
        ]
        valid_neighbors = []
        for nx,ny in neighbors:
            if self.is_within_bounds(nx, ny) and not self.is_obstacle(nx, ny):
                valid_neighbors.append((nx, ny))
        
        return valid_neighbors

 
 
def dijkstra(env):
    start=env.start
    goal=env.goal
    #priority qeue
    queue= []
    heapq.heappush(queue,(0,start))
#tracking best cost to each node
    costs={start:0}
    
    #tracking the path
    came_from={start:None}
    
    while queue:
        current_cost,current=heapq.heappop(queue)
        x,y = current
        if current== goal:
            break
        
        for  nx ,ny in env.get_neighbors(x,y):
            new_cost=current_cost + env.get_cost(nx, ny)
            if (nx,ny) not in costs or new_cost < costs[(nx,ny)]:
                costs[(nx,ny)]=new_cost
                came_from[(nx,ny)]=current
                heapq.heappush(queue,(new_cost,(nx,ny)))
                
    #reconstruct path
    path =[]
    node=goal
    while node is not None:
        path.append(node)
        node=came_from[node]
    path.reverse()  
    return path, costs.get(goal, float('inf'))


env = Environment()
path, total_cost = dijkstra(env)

print("Path length:", len(path))
print("Total cost:", total_cost)
print("Path:", path)

def path_to_directions(path):
    directions = []
    for i in range(1, len(path)):  
        x1, y1=path[i-1]
        x2, y2=path[i]
        dr =y2-y1
        dc=x2-x1
        if dr ==1 and dc==0:
            directions.append("DOWN")
        elif dr==-1 and dc==0:
            directions.append("UP")
        elif dr==0 and dc==1:
            directions.append("RIGHT")
        elif dr==0 and dc==-1:
            directions.append("LEFT")
        else:
            directions.append(f"DIAGONAL({dr},{dc})")  # Should not happen in a grid with 4-connectivit
            
    return directions
directions = path_to_directions(path)
print("Directions:", directions)


#Visualization
def visualize_path(env, path):
    grid_display=[]
    for y in range(env.height):
        row=""
        for x in range(env.width):
            if(x,y)==env.start:
                row+="S"
                
            elif(x,y)==env.goal:
                row+="G"
            elif (x,y) in path:
                row+="*"
            elif env.grid[y][x]==-1:
                row+="O"
            else:
                row+="."
        grid_display.append(row)
    for r in grid_display:
        print(r)
        
def slow_down(cost):
    if cost >5:
        return 0.5  # Slow down for high-cost terrain
    else:
        return 1.0  # Normal speed for low-cost terrain

class Rover:
    def __init__(self,env):  #the rover needs to access the environment
        self.env=env
        self.position = env.start
        self.energy=100
        self.speed=20  #m/s
        self.data=[]
        pass
    def move_to(self,x,y):
        cost=self.env.get_cost(x,y)
        if cost==float('inf'):
            print(f"Cannot move to {x},{y} - obstacle")
            return False
        energy_cost=cost*0.5  # Energy cost is half of the total cost
        if self.energy < energy_cost:
            print(f"Not enough energy to move to {x},{y}")
            return False
        self.energy -= energy_cost
        self.position=(x,y)
        print(f"Rover moved to {x},{y} | remaining energy: {self.energy}")
        return True
rover=Rover(env)
for step in path:
        x,y=step
        rover.move_to(x,y)
        cost=env.get_cost(x,y)
        #speed_factor=slow_down(cost)
        #print(f"Rover moves to {x},{y}| cost={cost}| speed factor={speed_factor}")
                
visualize_path(env, path)
                
    
   