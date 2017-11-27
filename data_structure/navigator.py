# from constants import Units

from collections import deque

DIRECTIONS = ((1,0),(-1,0), (0,1), (0,-1))


def recurse_path(parents, target, origin):
    if (target == origin):
        return (target,)

    x = parents[target]
    path = [target,]

    prev = target

    # Build path
    while (x in parents):
        last_node = path[-1]
        
        deltas = ((x[0] - prev[0]) == 0, (x[1] - prev[1]) == 0)
        prev_deltas = ((last_node[0] - prev[0]) == 0, (last_node[1] - prev[1]) == 0)
    
        if (deltas != prev_deltas):
            path.append(prev)
        
        prev = x
        x = parents[x]

    return path[::-1]

def get_path(maze, origin, target):
    open_set = deque()
    open_set.append(origin)
    closed_set = set((origin,))
    parents = {}

    while (len(open_set)):
        x = open_set.pop()

        if (x == target):
            return recurse_path(parents, target, origin)

        neighbors = ((x[0]+d[0], x[1]+d[1]) for d in DIRECTIONS)
        for u in neighbors: 
            if(u in closed_set):
                continue
            
            if(0 <= u[0] < 8 and 0 <= u[1] < 4):
                if (not maze[u[1]][u[0]]):
                    open_set.append(u)
                    closed_set.add(u)
                    parents[u] = x

def world_to_block_tuple(pos):
    return (int(pos[0] / Units.METERS_IN_A_FOOT), int(pos[1]  / Units.METERS_IN_A_FOOT))

def block_tuple_to_world(block_pos):
    return np.array([[block_pos[0] + 0.5],[block_pos[1] + 0.5]]) / Units.METERS_IN_A_FOOT
    
class Navigator(object):
    def __init__(self, maze):
        self.maze = maze
        self.target = None
        self.path = None        
    
    def update(self, pos, target):
        self.target = target
        curr_block = world_to_block_tuple(pos)
        target_block = world_to_block_tuple(target)
        self.path = get_path(self.maze, curr_block, target_block)
        return block_tuple_to_world(self.path[0])

    def draw(self, window):
        for i in range(1, len(self.path))
            p1, p2 = block_tuple_to_world(self.path[i-1]), block_tuple_to_world(self.path[i])
            pygame.draw.line(window.screen, Colours.GREEN, (window.m_to_px(p1[0]), window.m_to_px(p1[1])),
                (window.m_to_px(p2[0]), window.m_to_px(p2[1])))

def main():
    maze = ((0, 0, 0, 0, 1, 0, 1, 0),
        (0, 0, 1, 0, 0, 0, 0, 0),
        (0, 1, 0, 1, 1, 0, 1, 0),
        (0, 0, 0, 0, 0, 0, 1, 0),)

    print(get_path(maze, (0,0), (7,3)))

if __name__ == '__main__':
    main()