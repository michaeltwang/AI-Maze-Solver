from queue import Queue, PriorityQueue;
import matplotlib.pyplot as plt
import math, random, time;
import copy
from copy import deepcopy;

def GenerateMap(dim, p):
	#choose a p between 0 and 1
	#for each cell in dim x dim array, decide if occupied using a p chance of it being occupied
    maze = [[True for x in range(dim)] for y in range(dim)]
    for i in range(dim):
        for j in range(dim):
            #keep start and goal true for empty
            if (i==0 and j==0) or (i==dim-1 and j==dim-1):
                continue
            maze[i][j] = not decision(p)
    return maze
	
def decision(probability):
    return random.random() < probability


#Depth First Search
def dfs( maze ):
    start = time.time();
    #None if a space has not been visited, else [xpos,ypos] of parent in shortest path
    visited = [[None for x in range(len(maze))] for y in range(len(maze))];

    goal = len(maze) - 1;

    #stack, contains list [xpos, ypos, distance traveled, "parent" node xpos, "parent" node ypos]
    fringe = [];
    fringe.append([0, 0, 0, None, None]);

    #will store distance to goal whan found
    foundDistance = 0;
    
    #will store max fringe size
    max_fringe_size = 0;
    #will store counter for fringe
    fringe_size_counter = 1;

    while ( fringe ):
        current = fringe.pop();
        fringe_size_counter -= 1;
        xpos = current[0];
        ypos = current[1];
        distance = current[2];

        if ( visited[xpos][ypos] is None ):
            visited[xpos][ypos] = [current[3],current[4]];
        else:
            continue;

        if ( xpos == goal and ypos == goal ):
            foundDistance = distance;
            break;

        #DFS is LIFO, so more likely solutions added to fringe last
        #x-1
        if ( xpos > 0 and not visited[xpos-1][ypos] and maze[xpos-1][ypos] ):
            fringe.append([(xpos-1),ypos,(distance+1),xpos,ypos])
            fringe_size_counter += 1;
            if max_fringe_size < fringe_size_counter:
                max_fringe_size = fringe_size_counter
        
        #y-1
        if ( ypos > 0 and not visited[xpos][ypos-1] and maze[xpos][ypos-1] ):
            fringe.append([xpos,(ypos-1),(distance+1),xpos,ypos]);
            fringe_size_counter += 1;
            if max_fringe_size < fringe_size_counter:
                max_fringe_size = fringe_size_counter
        
        #x+1
        if ( xpos < goal and not visited[xpos+1][ypos] and maze[xpos+1][ypos] ):
            fringe.append([(xpos+1),ypos,(distance+1),xpos,ypos]);
            fringe_size_counter += 1;
            if max_fringe_size < fringe_size_counter:
                max_fringe_size = fringe_size_counter
        
        #y+1
        if ( ypos < goal and not visited[xpos][ypos+1] and maze[xpos][ypos+1] ):
            fringe.append([xpos,(ypos+1),(distance+1),xpos,ypos]);
            fringe_size_counter += 1;
            if max_fringe_size < fringe_size_counter:
                max_fringe_size = fringe_size_counter
        
    runtime = time.time() - start;
    
    if ( foundDistance == 0 ):
        return None, runtime, None;
    else:
        visited[0][0] = None;
        path = [[goal, goal]];
        parent = visited[goal][goal];
        while ( not parent is None ):
            path.insert(0, parent);
            parent = visited[parent[0]][parent[1]];
        return path, runtime, max_fringe_size;

#Breadth First Search
def bfs( maze ):
    start = time.time();
    #None if a space has not been visited, else [xpos,ypos] of parent in shortest path
    visited = [[None for x in range(len(maze))] for y in range(len(maze))];

    goal = len(maze) - 1;

    #queue, contains list [xpos, ypos, distance traveled, "parent" node xpos, "parent" node ypos]
    fringe = Queue();
    fringe.put([0, 0, 0, None, None]);

    #will store shortest distance to goal whan found
    shortest = 0;

    while ( not fringe.empty() ):
        current = fringe.get();
        xpos = current[0];
        ypos = current[1];
        distance = current[2];

        if ( visited[xpos][ypos] is None ):
            visited[xpos][ypos] = [current[3],current[4]];
        else:
            continue;

        if ( xpos == goal and ypos == goal ):
            shortest = distance;
            break;

        #BFS is FIFO, so more likely solutions added to fringe first
        #x+1
        if ( xpos < goal and not visited[xpos+1][ypos] and maze[xpos+1][ypos] ):
            fringe.put([(xpos+1),ypos,(distance+1),xpos,ypos]);
                
        #y+1
        if ( ypos < goal and not visited[xpos][ypos+1] and maze[xpos][ypos+1] ):
            fringe.put([xpos,(ypos+1),(distance+1),xpos,ypos]);

        #x-1
        if ( xpos > 0 and not visited[xpos-1][ypos] and maze[xpos-1][ypos] ):
            fringe.put([(xpos-1),ypos,(distance+1),xpos,ypos]);
                
        #y-1
        if ( ypos > 0 and not visited[xpos][ypos-1] and maze[xpos][ypos-1] ):
            fringe.put([xpos,(ypos-1),(distance+1),xpos,ypos]);
            
    runtime = time.time() - start;
    
    if ( shortest == 0 ):
        return None, runtime;
    else:
        visited[0][0] = None;
        path = [[goal, goal]];
        parent = visited[goal][goal];
        while ( not parent is None ):
            path.insert(0, parent);
            parent = visited[parent[0]][parent[1]];
        return path, runtime;


#A* Euclidean
def astarEuclidean( maze ):
    start = time.time();
    #None if a space has not been visited, else [xpos,ypos] of parent in shortest path
    visited = [[None for x in range(len(maze))] for y in range(len(maze))];

    goal = len(maze) - 1;

    #priority queue, contains list [xpos, ypos, distance traveled, "parent" node xpos, "parent" node ypos]
    fringe = PriorityQueue();
    fringe.put((0, [0, 0, 0, None, None]));

    #will store shortest distance to goal whan found
    shortest = 0;

    while ( not fringe.empty() ):
        current = fringe.get()[1];
        xpos = current[0];
        ypos = current[1];
        distance = current[2];

        if ( visited[xpos][ypos] is None ):
            visited[xpos][ypos] = [current[3],current[4]];
        else:
            continue;

        if ( xpos == goal and ypos == goal ):
            shortest = distance;
            break;

        #x+1
        if ( xpos < goal and not visited[xpos+1][ypos] and maze[xpos+1][ypos] ):
            heuristic = math.sqrt((goal - (xpos+1))**2 + (goal - ypos)**2 );
            estTotal = heuristic + distance + 1;
            fringe.put((estTotal, [(xpos+1),ypos,(distance+1),xpos,ypos]));

        #x-1
        if ( xpos > 0 and not visited[xpos-1][ypos] and maze[xpos-1][ypos] ):
            heuristic = math.sqrt((goal - (xpos-1))**2 + (goal - ypos)**2 );
            estTotal = heuristic + distance + 1;
            fringe.put((estTotal, [(xpos-1),ypos,(distance+1),xpos,ypos]));
                
        #y+1
        if ( ypos < goal and not visited[xpos][ypos+1] and maze[xpos][ypos+1] ):
            heuristic = math.sqrt((goal - xpos)**2 + (goal - (ypos+1))**2 );
            estTotal = heuristic + distance + 1;
            fringe.put((estTotal, [xpos,(ypos+1),(distance+1),xpos,ypos]));
                
        #y-1
        if ( ypos > 0 and not visited[xpos][ypos-1] and maze[xpos][ypos-1] ):
            heuristic = math.sqrt((goal - xpos)**2 + (goal - (ypos-1))**2 );
            estTotal = heuristic + distance + 1;
            fringe.put((estTotal, [xpos,(ypos-1),(distance+1),xpos,ypos]));
            
    runtime = time.time() - start;



    if ( shortest == 0 ):
        return None, runtime;
    else:
        visited[0][0] = None;
        path = [[goal, goal]];
        parent = visited[goal][goal];
        while ( not parent is None ):
            path.insert(0, parent);
            parent = visited[parent[0]][parent[1]];
        return path, runtime;
    
#A* Manhattan
def astarManhattan( maze ):
    start = time.time();
    #None if a space has not been visited, else [xpos,ypos] of parent in shortest path
    visited = [[None for x in range(len(maze))] for y in range(len(maze))];

    goal = len(maze) - 1;

    #priority queue, contains list [xpos, ypos, distance traveled, "parent" node xpos, "parent" node ypos]
    fringe = PriorityQueue();
    fringe.put((0, [0, 0, 0, None, None]));

    #will store shortest distance to goal whan found
    shortest = 0;
    
    #will store max fringe size
    max_fringe_size = 0;
    
    #will store max nodes expanded
    max_nodes_expanded = 0;

    while ( not fringe.empty() ):
        current = fringe.get()[1];
        max_nodes_expanded+=1;
        xpos = current[0];
        ypos = current[1];
        distance = current[2];

        if ( visited[xpos][ypos] is None ):
            visited[xpos][ypos] = [current[3],current[4]];
        else:
            continue;

        if ( xpos == goal and ypos == goal ):
            shortest = distance;
            break;


        #x+1
        if ( xpos < goal and not visited[xpos+1][ypos] and maze[xpos+1][ypos] ):
            heuristic = (goal - (xpos+1)) + (goal - ypos);
            estTotal = heuristic + distance + 1;
            fringe.put((estTotal, [(xpos+1),ypos,(distance+1),xpos,ypos]));
            if max_fringe_size < fringe.qsize():
                max_fringe_size = fringe.qsize()

        #x-1
        if ( xpos > 0 and not visited[xpos-1][ypos] and maze[xpos-1][ypos] ):
            heuristic = (goal - (xpos-1)) + (goal - ypos);
            estTotal = heuristic + distance + 1;
            fringe.put((estTotal, [(xpos-1),ypos,(distance+1),xpos,ypos]));
            if max_fringe_size < fringe.qsize():
                max_fringe_size = fringe.qsize()
                
        #y+1
        if ( ypos < goal and not visited[xpos][ypos+1] and maze[xpos][ypos+1] ):
            heuristic = (goal - xpos) + (goal - (ypos+1));
            estTotal = heuristic + distance + 1;
            fringe.put((estTotal, [xpos,(ypos+1),(distance+1),xpos,ypos]));
            if max_fringe_size < fringe.qsize():
                max_fringe_size = fringe.qsize()
                
        #y-1
        if ( ypos > 0 and not visited[xpos][ypos-1] and maze[xpos][ypos-1] ):
            heuristic = (goal - xpos) + (goal - (ypos-1));
            estTotal = heuristic + distance + 1;
            fringe.put((estTotal, [xpos,(ypos-1),(distance+1),xpos,ypos]));
            if max_fringe_size < fringe.qsize():
                max_fringe_size = fringe.qsize()

    runtime = time.time() - start;

    if ( shortest == 0 ):
        return None, runtime, None, None;
    else:
         #calculate # nodes that were expanded/visited
        total_nodes_expanded = 0
        for each in range(len(maze)):
            for y in range(len(maze)):
                if not visited[each][y] ==None:
                    total_nodes_expanded +=1
        visited[0][0] = None;
        path = [[goal, goal]];
        parent = visited[goal][goal];
        while ( not parent is None ):
            path.insert(0, parent);
            parent = visited[parent[0]][parent[1]];
        return path, runtime, max_fringe_size,total_nodes_expanded;
#################################################################################
    
    
###############################################################################################
    
    
    
def geneticAlgo_DFS_MSP(dim, p):
    set_of_mazes = []
    #generate an initial population of mazes, solve these mazes, keeping track of fitness and order w/ PQ
    for i in range(10):
        solvable = False
        while (not solvable):
            one_maze = GenerateMap(dim,p)
            path, runtime, max_fringe_size = dfs(one_maze)
            if (path is not None):
                max_shortest_path = len(path)
                #print("solvable: ", max_shortest_path)
                solvable = True
                set_of_mazes.append((max_shortest_path,one_maze, path))
            else: 
                #print("unsolvable")
                continue
    #generations for loop
    for generations in range(100):
        #select solutions for recombination random pairings   
        for i in range(len(set_of_mazes)-1):
            #create a valid recombination/child
            solvable = False
            while (not solvable):
                random_index1 = randrange(len(set_of_mazes))
                random_index2 = randrange(len(set_of_mazes))
                temp1 = set_of_mazes[random_index1][1]
                temp2 = set_of_mazes[random_index2][1]
                child_maze = recombine(temp1,temp2,dim)
                #check if solvable, else repeat
                path, runtime, max_fringe_size = dfs(child_maze)
                if (path is not None):
                    #keep track of fitness
                    max_shortest_path = len(path)
                    #print("solvable child: ", max_shortest_path)
                    solvable = True
                    set_of_mazes.append((max_shortest_path,child_maze, path))
                else: 
                   # print("unsolvable child")
                    continue
            #visualizeMaze(child_maze)
        
        #take top n most fit by deleting everything after top n
        set_of_mazes.sort(reverse=True)
        del set_of_mazes[10:]
        #repeat another generation, go to generations for loop
        
    #return most fit maze after n generation
    return set_of_mazes[0][1], set_of_mazes[0][0], set_of_mazes[0][2]


def geneticAlgo_DFS_MFS(dim, p):
    set_of_mazes = []
    #generate an initial population of mazes, solve these mazes, keeping track of fitness and order w/ PQ
    for i in range(10):
        solvable = False
        while (not solvable):
            one_maze = GenerateMap(dim,p)
            path, runtime, max_fringe_size = dfs(one_maze)
            if (path is not None):
                #print("solvable: ", max_fringe_size)
                solvable = True
                set_of_mazes.append((max_fringe_size,one_maze, path))
            else: 
                #print("unsolvable")
                continue
    #generations for loop
    for generations in range(100):
        #select solutions for recombination random pairings   
        for i in range(len(set_of_mazes)-1):
            #create a valid recombination/child
            solvable = False
            while (not solvable):
                random_index1 = randrange(len(set_of_mazes))
                random_index2 = randrange(len(set_of_mazes))
                temp1 = set_of_mazes[random_index1][1]
                temp2 = set_of_mazes[random_index2][1]
                child_maze = recombine(temp1,temp2,dim)
                #check if solvable, else repeat
                path, runtime, max_fringe_size = dfs(child_maze)
                if (path is not None):
                    #keep track of fitness
                    #print("solvable child: ", max_fringe_size)
                    solvable = True
                    set_of_mazes.append((max_fringe_size,child_maze, path))
                else: 
                    #print("unsolvable child")
                    continue
            #visualizeMaze(child_maze)
        
        #take top n most fit by deleting everything after top n
        set_of_mazes.sort(reverse=True)
        del set_of_mazes[10:]
        #repeat another generation, go to generations for loop
        
    #return most fit maze after n generation
    return set_of_mazes[0][1], set_of_mazes[0][0], set_of_mazes[0][2]


def geneticAlgo_astarMan_MFS(dim, p):
    set_of_mazes = []
    #generate an initial population of mazes, solve these mazes, keeping track of fitness and order w/ PQ
    for i in range(10):
        solvable = False
        while (not solvable):
            one_maze = GenerateMap(dim,p)
            path, runtime, max_fringe_size, max_nodes_expanded = astarManhattan(one_maze)
            if (path is not None):
                #print("solvable: ", max_fringe_size)
                solvable = True
                set_of_mazes.append((max_fringe_size,one_maze, path))
            else: 
                #print("unsolvable")
                continue
    #generations for loop
    for generations in range(100):
        #select solutions for recombination random pairings   
        for i in range(len(set_of_mazes)-1):
            #create a valid recombination/child
            solvable = False
            while (not solvable):
                random_index1 = randrange(len(set_of_mazes))
                random_index2 = randrange(len(set_of_mazes))
                temp1 = set_of_mazes[random_index1][1]
                temp2 = set_of_mazes[random_index2][1]
                child_maze = recombine(temp1,temp2,dim)
                #check if solvable, else repeat
                path, runtime, max_fringe_size, max_nodes_expanded = astarManhattan(child_maze)
                if (path is not None):
                    #keep track of fitness
                    #print("solvable child: ", max_fringe_size)
                    solvable = True
                    set_of_mazes.append((max_fringe_size,child_maze, path))
                else: 
                    #print("unsolvable child")
                    continue

            #visualizeMaze(child_maze)
        
        #take top n most fit by deleting everything after top n
        set_of_mazes.sort(reverse=True)
        del set_of_mazes[10:]
        #repeat another generation, go to generations for loop
        
    #return most fit maze after n generation
    return set_of_mazes[0][1], set_of_mazes[0][0], set_of_mazes[0][2]


def geneticAlgo_astarMan_MNE(dim, p):
    set_of_mazes = []
    #generate an initial population of mazes, solve these mazes, keeping track of fitness and order w/ PQ
    for i in range(10):
        solvable = False
        while (not solvable):
            one_maze = GenerateMap(dim,p)
            path, runtime, max_fringe_size, max_nodes_expanded = astarManhattan(one_maze)
            if (path is not None):
                #print("solvable: ", max_nodes_expanded)
                solvable = True
                set_of_mazes.append((max_nodes_expanded,one_maze, path))
            else: 
                #print("unsolvable")
                continue
    #generations for loop
    for generations in range(100):
        #select solutions for recombination random pairings   
        for i in range(len(set_of_mazes)-1):
            #create a valid recombination/child
            solvable = False
            while (not solvable):
                random_index1 = randrange(len(set_of_mazes))
                random_index2 = randrange(len(set_of_mazes))
                temp1 = set_of_mazes[random_index1][1]
                temp2 = set_of_mazes[random_index2][1]
                child_maze = recombine(temp1,temp2,dim)
                #check if solvable, else repeat
                path, runtime, max_fringe_size, max_nodes_expanded = astarManhattan(child_maze)
                if (path is not None):
                    #keep track of fitness
                    #print("solvable child: ", max_nodes_expanded)
                    solvable = True
                    set_of_mazes.append((max_nodes_expanded,child_maze, path))
                else: 
                    #print("unsolvable child")
                    continue
            #visualizeMaze(child_maze)
        
        #take top n most fit by deleting everything after top n
        set_of_mazes.sort(reverse=True)
        del set_of_mazes[10:]
        #repeat another generation, go to generations for loop
        
    #return most fit maze after n generation
    return set_of_mazes[0][1], set_of_mazes[0][0], set_of_mazes[0][2]

    
from random import randrange        
        
def recombine(m1, m2, dim):
    child_maze = [[True for x in range(dim)] for y in range(dim)]
    for i in range(int(dim/2)):
        for j in range(dim):
            child_maze[i][j] = m1[i][j]
    for i in range(int(dim/2), dim):
        for j in range(dim):
            child_maze[i][j] = m2[i][j]
    #mutation 90% chance
    if decision(.9):
        mutated = False
        while (mutated == False):
            random_x_index = randrange(len(child_maze))
            random_y_index = randrange(len(child_maze))
            #find an obstruction to remove 50% chance of removal
            if not child_maze[random_x_index][random_y_index]:
                if decision(.5):
                    child_maze[random_x_index][random_y_index] = True
                mutated2 = False
                while mutated2 == False:
                    random_x_index2 = randrange(len(child_maze))
                    random_y_index2 = randrange(len(child_maze))
                    #find an empty cell other than start and goal to insert an obstruction
                    if child_maze[random_x_index2][random_y_index2] and not ((random_x_index2==0 and random_y_index2==0) or (random_x_index2==dim-1 and random_y_index2==dim-1)):
                        child_maze[random_x_index2][random_y_index2] = False
                        mutated2 = True
                mutated = True                    
                
    return child_maze


###################################################################################
    
def calculateTime():
    for dim in range (2000, 2300, 5):
        start = time.time();
        timeDFS = 0.0
        timeBFS = 0.0 
        timeAE = 0.0
        timeAM = 0.0
        for x in range(1):
            timeDFS += dfs(GenerateMap(dim,0))[1]
            timeBFS += bfs(GenerateMap(dim,0))[1]
            timeAE += astarEuclidean(GenerateMap(dim,0))[1]
            timeAM += astarManhattan(GenerateMap(dim,0))[1]
        print (dim, time.time() - start, timeDFS/25.0, timeBFS/25.0, timeAE/25.0, timeAM/25.0, sep=' ' )

def pzero():
    dim = 30
    min = 0.30
    max = 0.31
    step = 0.001
    steps = int((max - min)/step + 1)
    for x in range(0, steps):
        successes = 0
        for y in range (1000):
            maze = GenerateMap(dim,(min + x*step))
            if (dfs(maze)[0]):
                successes = successes + 1
            # if (bfs(maze)):
            #     print("BFS");
            # if (astarEuclidean(maze)):
            #     print ("Euclid");
            # if (astarManhattan(maze)):
            #     print("Manhattan");
        print (min + x*step, successes, sep=' ')

def avgShortestPath():
    for p in range(302):
        shortest = 0
        count = 0.0
        for x in range(100):
            path = bfs(GenerateMap(30, p/1000))[0]
            if ( not path is None ):
                shortest += len(path)
                count += 1
        print(p/1000, shortest/count)

def astarCompare():
    file = open("astarData.csv","w")
    timeEuclid = 0
    timeManhattan = 0
    for x in range(1000):
        maze = GenerateMap(30, 0.3)
        timeE = astarEuclidean(maze)[1]
        timeM = astarManhattan(maze)[1]
        file.write(str(timeE) + "," + str(timeM) + "\n")
        timeEuclid += timeE
        timeManhattan += timeM
    print("Euclidean Distance: ", timeEuclid/1000.0)
    print("Manhattan Distance: ", timeManhattan/1000.0)

def dbfsCompare():
    file = open("dbfsData.csv","w")
    lengthDFS = 0
    lengthBFS = 0
    timeDFS = 0
    timeBFS = 0
    count = 0
    for x in range(1000):
        maze = GenerateMap(30, 0.3)
        resultDFS = dfs(maze)
        resultBFS = bfs(maze)
        if ( not resultDFS[0] is None ):
            lengthDFS += len(resultDFS[0])
            lengthBFS += len(resultBFS[0])
            count += 1
            file.write(str(len(resultDFS[0])) + "," + str(len(resultBFS[0])) + "," + str(resultDFS[1]) + "," + str(resultBFS[1]) + "\n")
        else:
            file.write("0,0," + str(resultDFS[1]) + "," + str(resultBFS[1]) + "\n")


        timeDFS += resultDFS[1]
        timeBFS += resultBFS[1]
    print("DFS: ", lengthDFS/count, " ", timeDFS/1000.0)
    print("BFS: ", lengthBFS/count, " ", timeBFS/1000.0)

#Make thinned maze
def thin(maze, q):
    thinned_maze = deepcopy(maze)
    obstructions = []
    for i in range(len(thinned_maze)):
        for j in range(len(thinned_maze)):
            #check if the cell is a wall
            if thinned_maze[i][j] == False:
                obstructions.append([i,j])
    #remove 'q' fraction of obstructions randomly
    num_obstr_removed = int(len(obstructions) * q)
    # print(num_obstr_removed)
    # print(len(obstructions))
    for each in range(num_obstr_removed):
        current = random.choice(obstructions)
        ypos = current[0]
        xpos = current[1]
        thinned_maze[ypos][xpos] = True
        obstructions.remove(current)
    return thinned_maze
                
#A* Thinned with Man
def astarThinned( maze, thinned_maze ):
    start = time.time()
    #None if a space has not been visited, else [xpos,ypos] of parent in shortest path
    visited = [[None for x in range(len(maze))] for y in range(len(maze))];
    thinned = [[None for x in range(len(maze))] for y in range(len(maze))];

    goal = len(maze) - 1;

    #priority queue, contains list [xpos, ypos, distance traveled, "parent" node xpos, "parent" node ypos]
    fringe = PriorityQueue();
    fringe.put((0, [0, 0, 0, None, None]));

    #will store shortest distance to goal whan found
    shortest = 0;

    while ( not fringe.empty() ):
        current = fringe.get()[1];
        xpos = current[0];
        ypos = current[1];
        distance = current[2];

        if ( visited[xpos][ypos] is None ):
            visited[xpos][ypos] = [current[3],current[4]];
        else:
            continue;

        if ( xpos == goal and ypos == goal ):
            shortest = distance;
            break;

        #x+1
        if ( xpos < goal and (not visited[xpos+1][ypos]) and maze[xpos+1][ypos] ):
            #print("Searching: ", xpos+1, ", ", ypos, ", ",maze[xpos+1][ypos] )
            if ( thinned[xpos+1][ypos] is None ):
                heuristic = astarThinnedMan(thinned_maze,xpos+1,ypos)
                thinned[xpos+1][ypos] = heuristic
            else:
                heuristic = thinned[xpos+1][ypos]
            estTotal = heuristic + distance + 1;
            fringe.put((estTotal, [(xpos+1),ypos,(distance+1),xpos,ypos]));

        #x-1
        if ( xpos > 0 and (not visited[xpos-1][ypos]) and maze[xpos-1][ypos] ):
            #print("Searching: ", xpos-1, ", ", ypos, ", ",maze[xpos-1][ypos] )
            if ( thinned[xpos-1][ypos] is None ):
                heuristic = astarThinnedMan(thinned_maze,xpos-1,ypos)
                thinned[xpos-1][ypos] = heuristic
            else:
                heuristic = thinned[xpos-1][ypos]            
            estTotal = heuristic + distance + 1;
            fringe.put((estTotal, [(xpos-1),ypos,(distance+1),xpos,ypos]));
                
        #y+1
        if ( ypos < goal and (not visited[xpos][ypos+1]) and maze[xpos][ypos+1] ):
           # print("Searching: ", xpos, ", ", ypos+1, ", ",maze[xpos][ypos+1] )
            if ( thinned[xpos][ypos+1] is None ):
                heuristic = astarThinnedMan(thinned_maze,xpos,ypos+1)
                thinned[xpos][ypos+1] = heuristic
            else:
                heuristic = thinned[xpos][ypos+1]            
            estTotal = heuristic + distance + 1;
            fringe.put((estTotal, [xpos,(ypos+1),(distance+1),xpos,ypos]));
                
        #y-1
        if ( ypos > 0 and (not visited[xpos][ypos-1]) and maze[xpos][ypos-1] ):
            #print("Searching: ", xpos, ", ", ypos-1, ", ",maze[xpos][ypos-1] )
            if ( thinned[xpos][ypos-1] is None ):
                heuristic = astarThinnedMan(thinned_maze,xpos,ypos-1)
                thinned[xpos][ypos-1] = heuristic
            else:
                heuristic = thinned[xpos][ypos-1]            
            estTotal = heuristic + distance + 1;
            fringe.put((estTotal, [xpos,(ypos-1),(distance+1),xpos,ypos]));
    
    runtime = time.time() - start

    if ( shortest == 0 ):
        return None, runtime;
    else:
        visited[0][0] = None;
        path = [[goal, goal]];
        parent = visited[goal][goal];
        while ( not parent is None ):
            path.insert(0, parent);
            parent = visited[parent[0]][parent[1]];
        return path, runtime;

#A* Manhattan thinned
def astarThinnedMan( maze, xx, yy ):
    #None if a space has not been visited, else [xpos,ypos] of parent in shortest path
    visited = [[None for x in range(len(maze))] for y in range(len(maze))];

    goal = len(maze) - 1;

    #priority queue, contains list [xpos, ypos, distance traveled, "parent" node xpos, "parent" node ypos]
    fringe = PriorityQueue();
    fringe.put((0, [xx, yy, 0, None, None]));

    #will store shortest distance to goal whan found
    shortest = 0;

    while ( not fringe.empty() ):
        current = fringe.get()[1];
        xpos = current[0];
        ypos = current[1];
        distance = current[2];

        if ( visited[xpos][ypos] is None ):
            visited[xpos][ypos] = [current[3],current[4]];
        else:
            continue;

        if ( xpos == goal and ypos == goal ):
            shortest = distance;
            break;


        #x+1
        if ( xpos < goal and not visited[xpos+1][ypos] and maze[xpos+1][ypos] ):
            heuristic = (goal - (xpos+1)) + (goal - ypos);
            estTotal = heuristic + distance + 1;
            fringe.put((estTotal, [(xpos+1),ypos,(distance+1),xpos,ypos]));

        #x-1
        if ( xpos > 0 and not visited[xpos-1][ypos] and maze[xpos-1][ypos] ):
            heuristic = (goal - (xpos-1)) + (goal - ypos);
            estTotal = heuristic + distance + 1;
            fringe.put((estTotal, [(xpos-1),ypos,(distance+1),xpos,ypos]));
                
        #y+1
        if ( ypos < goal and not visited[xpos][ypos+1] and maze[xpos][ypos+1] ):
            heuristic = (goal - xpos) + (goal - (ypos+1));
            estTotal = heuristic + distance + 1;
            fringe.put((estTotal, [xpos,(ypos+1),(distance+1),xpos,ypos]));
                
        #y-1
        if ( ypos > 0 and not visited[xpos][ypos-1] and maze[xpos][ypos-1] ):
            heuristic = (goal - xpos) + (goal - (ypos-1));
            estTotal = heuristic + distance + 1;
            fringe.put((estTotal, [xpos,(ypos-1),(distance+1),xpos,ypos]));

    return shortest;

def compareThinned():
    #timeThin = 0
    timeManhattan = 0
    for q in range(1,100):
        timeThinned = 0
        timeManhattan = 0
        for x in range(100):
            maze = GenerateMap(30, 0.3)
            thinned = thin(maze, q/100.0)
            timeThinned += astarThinned(maze, thinned)[1]
            timeManhattan += astarManhattan(maze)[1]
        print(q/100.0, timeThinned/10.0, timeManhattan/10.0, sep=' ')
####################################################################################


def visualizeMaze(maze):
    plt.pcolormesh(maze)
    plt.axes().set_aspect('equal') #set the x and y axes to the same scale
    plt.xticks([]) # remove the tick marks by setting to an empty list
    plt.yticks([]) # remove the tick marks by setting to an empty list
    plt.axes().invert_yaxis() #invert the y-axis so the first row of data is at the top
    plt.show()

def visualizePath(maze,path):
    maze2 = copy.deepcopy(maze)
    for x, row in enumerate(maze):
        for y, item in enumerate(row): 
            if item == True:
                maze2[x][y] = 1000
            else:
                maze2[x][y] = 0
    for coordinate in path:
        maze2[coordinate[0]][coordinate[1]] = 500
    plt.pcolormesh(maze2)
    plt.axes().set_aspect('equal') #set the x and y axes to the same scale
    plt.xticks([]) # remove the tick marks by setting to an empty list
    plt.yticks([]) # remove the tick marks by setting to an empty list
    plt.axes().invert_yaxis() #invert the y-axis so the first row of data is at the top
    plt.show()

#############################################################################################
            
def main():
    dim = int(input("Enter dimension: "))
    p = float(input("Enter probability: "))
    maze = GenerateMap(dim, p)
    visualizeMaze(maze)
    i=0
    while dfs(maze)[0] == None and i<50:
        print("map unsolvable, generating new map...")
        maze = GenerateMap(dim, p)
        visualizeMaze(maze)
        i += 1
    if i>=50:
        print("could not generate a solvable maze in a timely manner. please try again")
        return
    visualizePath(maze,dfs(maze)[0])
    visualizePath(maze,bfs(maze)[0])
    visualizePath(maze,astarEuclidean(maze)[0])
    visualizePath(maze,astarManhattan(maze)[0])

    hardest_maze, max_short_path, path = geneticAlgo_DFS_MSP(dim,p)
    print("DFS hardest maze with max shortest path of: ", max_short_path, "; path: ", path)
    visualizeMaze(hardest_maze)
    visualizePath(hardest_maze,path)
    
    hardest_maze, max_fringe_size, path = geneticAlgo_DFS_MFS(dim,p)
    print("DFS hardest maze with max fringe size of: ", max_fringe_size, "; path: ", path)
    visualizeMaze(hardest_maze)
    visualizePath(hardest_maze,path)
    
    hardest_maze, max_nodes_expanded, path = geneticAlgo_astarMan_MNE(dim,p)
    print("A* Man hardest maze with max nodes expanded of: ", max_nodes_expanded, "; path: ", path)
    visualizeMaze(hardest_maze)
    visualizePath(hardest_maze,path)
    
    hardest_maze, max_fringe_size, path = geneticAlgo_astarMan_MFS(dim,p)
    print("A* Man hardest maze with max fringe size of: ", max_fringe_size, "; path: ", path)
    visualizeMaze(hardest_maze)
    visualizePath(hardest_maze,path)
    
    q = float(input("Enter q (fraction of obstructions to be removed): " ))
    thinned_maze = thin(maze,q)
    visualizeMaze(thinned_maze)
    visualizePath(maze, astarThinned(maze,thinned_maze)[0])


if __name__ == "__main__":
	main()
    # calculateTime()
    # pzero()
    # avgShortestPath()
    # astarCompare()
    # dbfsCompare()
