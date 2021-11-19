from random import shuffle, randrange, random

import pandas as pd
import networkx as nx
 
def make_maze(w = 16, h = 8):
    vis = [[0] * w + [1] for _ in range(h)] + [[1] * (w + 1)]
    ver = [["100"] * w + ['1'] for _ in range(h)] + [[]]
    hor = [["111"] * w + ['1'] for _ in range(h + 1)]
 
    def walk(x, y):
        vis[y][x] = 1
 
        d = [(x - 1, y), (x, y + 1), (x + 1, y), (x, y - 1)]
        shuffle(d)
        for (xx, yy) in d:
            if vis[yy][xx]: continue
            if xx == x: hor[max(y, yy)][x] = "100"
            if yy == y: ver[y][max(x, xx)] = "000"
            walk(xx, yy)
 
    start = (randrange(w), randrange(h))
    walk(*start)
 
    maze_string = ""
    for (a, b) in zip(hor, ver):
        maze_string += ''.join(a + ['\n'] + b + ['\n'])
    
    return maze_string

def make_maze_gridworld(w = 16, h = 8, break_p=0.05, hazard_p=0.05):
    maze_string = make_maze(w, h).strip()
    maze_string_split = maze_string.split("\n")
    maze_array = []

    # make graph to find maximum path (to set start and goal points)
    G = nx.Graph()
    node_id = 0
    node_index_to_id_map = {}
    
    print("creating maze array")
    height = len(maze_string_split)
    width = len(maze_string_split[0])
    num_states = height * width
    for row_number, line in enumerate(maze_string_split):
        maze_line_array = ["0" for _ in range(width)]
        for i, cell in enumerate(line):
            if cell == "1":
                # add random breaks
                if random() <= break_p:
                    maze_line_array[i] = "0"
                    G.add_node(node_id)
                    node_index_to_id_map[(row_number, i)] = node_id
                    node_id += 1
                else:
                    maze_line_array[i] = "1"
                    num_states -= 1
            # add hazards haphazardly
            elif maze_line_array[i] == "0" and random() <= hazard_p:
                maze_line_array[i] = "h"
                num_states -= 1
                G.add_node(node_id)
                node_index_to_id_map[(row_number, i)] = node_id
                node_id += 1
            else:
                G.add_node(node_id)
                node_index_to_id_map[(row_number, i)] = node_id
                node_id += 1
        maze_array.append(maze_line_array)

    # add edges
    print("adding edges")
    for (x,y), node_id in node_index_to_id_map.items():
        neighbors = [(x - 1, y), (x, y + 1), (x + 1, y), (x, y - 1)]
        for neighbor in neighbors:
            neighbor_id = node_index_to_id_map.get(neighbor)
            if neighbor_id:
                G.add_edge(node_id, neighbor_id)
    
    # find two points with the longest shortest path
    max_l = 0
    start = None
    goal = None
    
    print("finding start and goal") 
    for (a,b), source in node_index_to_id_map.items():
        for (c,d), target in node_index_to_id_map.items():
            try:
                l = nx.shortest_path_length(G, source, target)
            except(nx.NetworkXNoPath):
                continue
            if l > max_l:
                max_l = l
                start = (a,b)
                goal = (c,d)
    
    # set start and goal to be the two points in the graph with the longest shortest path
    maze_array[start[0]][start[1]] = "s"
    maze_array[goal[0]][goal[1]] = "g"
        
    return pd.DataFrame(data=maze_array).to_csv(index=False, header=False), num_states

