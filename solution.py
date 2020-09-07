# !/usr/bin/env python
import time
start_time = time.time()
# import libraries and add path to dependancies
import numpy as np
import pylab as pl
import sys
sys.path.append('osr_examples/scripts/')
import environment_2d

# generate 2D environment, obstacles, goal and start
pl.ion()                                               # interactive mode on
np.random.seed(4)                                      # seed the generator(same seed number(int64) will always produce same environment)
env = environment_2d.Environment(10, 6, 5)             # create a 10x6 environment with 5 triangular obstacles. If this is changed, arguments to vertex generator in function main() should also be changed
pl.clf()                                               # what does this do? irrelevant.
env.plot()                                             # plot the environment
q = env.random_query()                                 # assigns start and end positions (randomly?)
if q is not None:
    x_start, y_start, x_goal, y_goal = q
    env.plot_query(x_start, y_start, x_goal, y_goal)   # plots start and goal positions


# create a vertex class with coordinates, a function to print coordinates and the set of closest vertices
class vertex():
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.associatedVertices = []

    def set_associatedVertices(self, vertex):
        self.associatedVertices.append(vertex)

    def printer(self):
        print(self.x, self.y)

start_vertex = vertex(x_start, y_start)
goal_vertex = vertex(x_goal, y_goal)
vertices = [start_vertex, goal_vertex] #list of all vertices

# create a path class with the two nodes each path connects, slope of the path, length of the path, and 
# a function to check if the path collides with obstacles
class path():
    def __init__(self, vert1, vert2):
        self.vert1 = vert1
        self.vert2 = vert2
        
    @property
    def slope(self):
        try :
            return ((self.vert2.y-self.vert1.y)/(self.vert2.x-self.vert1.x))
        except ZeroDivisionError:
            print('slope is infinite for this path')
            return None
    
    @property 
    def length(self):
        return np.sqrt(((self.vert2.y-self.vert1.y)**2)+((self.vert2.x-self.vert1.x)**2))
    
    def collisionCheck(self, resolution): #returns true if path collides with obstacle
        for i in range(resolution):
            x_temp = ((((self.vert1.x)*(resolution-i))+((self.vert2.x)*(i)))/(resolution))
            y_temp = ((((self.vert1.y)*(resolution-i))+((self.vert2.y)*(i)))/(resolution))
            if env.check_collision(x_temp, y_temp):
                return True

paths =[]   #list of all paths


#create a function to generate vertices, given the bounds xLow, xHigh, yLow and yHigh of C(free), ensuring
#no vertex is inside an obstacle
def vertex_generator(xLow,xHigh,yLow,yHigh,numberOfVertices):
    for i in range(numberOfVertices):
        while True:
            vert = (vertex((np.random.randint(xLow*10, xHigh*10)/10), (np.random.randint(yLow*10, yHigh*10)/10)))
            if (not env.check_collision(vert.x, vert.y)) and (vert not in vertices):
                vertices.append(vert)
                break
            else:
                continue

#create a function that plots all the vertices
def plot_vertices():
    for i in vertices:
        pl.plot(i.x, i.y, "g+", markersize=15)

#create a function that generates paths between vertices that are within a radius of 3 from each other
#if no other vertex exists within radius 3 of a vertex, then connect it to the two closest vertexes to it         
def path_generator():
    for i in vertices:
        for j in vertices:
            if i!=j:
                if path(i,j).length<3:
                    paths.append(path(i,j))
                    i.set_associatedVertices(j)
                    j.set_associatedVertices(i)          #create paths if vertices are within a 3 unit radius
                    
    #if no vertices are within a 3 unit radius, add the two closest paths
    for i in vertices:
        lowest_index = 0
        current_lowest_length = 0
        lowest2_index = 0
        current_2lowest_length = 0
        if len(i.associatedVertices)<2:
            for j in range(len(vertices)):
                if path(i,vertices[j]).length<current_2lowest_length:
                    if path(i, vertices[j]).length<current_lowest_length:
                        current_2lowest_length = current_lowest_length
                        current_lowest_length = path(i,vertices[j]).length
                        lowest2_index = lowest_index
                        lowest_index = j
                    else:
                        lowest2_index = j
                        current_2lowest_length = path(i, vertices[j]).length
        i.set_associatedVertices(vertices[lowest_index])
        i.set_associatedVertices(vertices[lowest2_index])
        vertices[lowest_index].set_associatedVertices(i)
        vertices[lowest2_index].set_associatedVertices(i)
        paths.append(path(i,vertices[lowest_index]))
        paths.append(path(i,vertices[lowest2_index]))


#create a function that recognizes and deletes paths that cross obstacles
#increase resolution to improve accuracy
def path_obstacle_collision_handler(resolution):
    for i in vertices:
        for j in i.associatedVertices:
            if path(i,j).collisionCheck(resolution):
                for p in paths:
                    if (p.vert1.x == i.x) and (p.vert1.y==i.y) and (p.vert2.x==j.x) and (p.vert2.y==j.y): 
                        paths.remove(p)
                    elif (p.vert2.x == i.x) and (p.vert2.y==i.y) and (p.vert1.x==j.x) and (p.vert1.y==j.y): 
                        paths.remove(p)
                        
                if j in i.associatedVertices:
                    i.associatedVertices.remove(j)
                if i in j.associatedVertices:
                    j.associatedVertices.remove(i)

#create a function to plot all paths including those not in the final path to goal
def plot_paths():
    for i in paths:
        pl.plot([i.vert1.x, i.vert2.x], [i.vert1.y, i.vert2.y])

    
def path_finder():
    current_vertex = start_vertex
    path_to_goal = []
    while current_vertex!= goal_vertex:
        next_vertex = current_vertex.associatedVertices[np.random.randint(len(current_vertex.associatedVertices))]
        path_to_goal.append(path(current_vertex, next_vertex))
        for i in next_vertex.associatedVertices:
            if (i.x==current_vertex.x) and (i.y == current_vertex.y):
                next_vertex.associatedVertices.remove(i)
        current_vertex = next_vertex
        if len(path_to_goal)>300:
            print('Error: Path to goal too long. Please increase path length limiter at line 136. Path length can be reduced with the shortcutting function later.')
            print(path_to_goal[1].length==path_to_goal[3].length)
            break
    return path_to_goal

def path_to_goal_plotter(path_to_goal):
    for i in path_to_goal:
        pl.plot([i.vert1.x, i.vert2.x], [i.vert1.y, i.vert2.y])
        #i.vert2.printer() #prints the vertices on the path
    print("path length is(number of nodes traversed +/- 1): "+str(len(path_to_goal)))
    #goal_vertex.printer()

#shortcutting function using n and n+2 vertexes instead of points on the lines:
def path_shortcutter(path_to_goal, resolution):
    vertices_in_path = [start_vertex]
    new_vertices_in_path = [start_vertex]
    new_path_to_goal = []
    for i in path_to_goal:
        vertices_in_path.append(i.vert2)
    i = 0
    while i < len(vertices_in_path):

        for j in range(10):
            if (i-j+3)<len(vertices_in_path):
                if path(vertices_in_path[i],vertices_in_path[i+3-j]).collisionCheck(resolution):
                    pass
                else:
                    new_vertices_in_path.append(vertices_in_path[i+3-j])
                    i = i+3-j
                    break
        i += 1
    for i in range(1,len(new_vertices_in_path)):
        new_path_to_goal.append(path(new_vertices_in_path[i-1],new_vertices_in_path[i]))
    return new_path_to_goal

def main(obstacle_detection_resolution, option):    
    vertex_generator(0,10,0,6,100)
    plot_vertices()
    path_generator()
    
    len_path_old = len(paths)
    len_path_new = 0
    while len_path_old!=len_path_new:
        len_path_old = len(paths)
        path_obstacle_collision_handler(100)
        len_path_new = len(paths)
    if option == 1:
        plot_paths()
    elif option==2:
        path_to_goal_plotter(path_finder())
    elif option==3:
        path_to_goal_plotter(path_shortcutter(path_finder(), 100))
    else :
        print('enter a valid option number')


print("please type the option number you would like: \n1)plot all paths \n2)plot the initial path to goal(solution to problem 1) \n3)plot the path after shortcutting\n")
option = int(input("only enter numbers (eg.: 1 )\n"))
print('Please do not be alarmed by lack of output. Program usually takes around 60-80 seconds to run.\n')
main(100,option)
print("Time from start till now : %s seconds " % (time.time() - start_time))
print('if there is a narrow part of an obstacle that is being crossed by a path, increase the first parameter in the function main(), whose default value is 100.' )