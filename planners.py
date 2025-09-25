
import json, math, heapq, time
from collections import deque

def load_map(path):
    with open(path) as f:
        grid = [list(map(int,line.split())) for line in f.readlines()]
    with open(path + ".meta.json") as f:
        meta = json.load(f)
    return grid, meta

def in_bounds(grid, r, c):
    return 0 <= r < len(grid) and 0 <= c < len(grid[0])

def neighbors(grid, r, c):
    for dr,dc in [(-1,0),(1,0),(0,-1),(0,1)]:
        nr, nc = r+dr, c+dc
        if in_bounds(grid, nr, nc) and grid[nr][nc] != 0:
            yield (nr,nc)

def reconstruct(parent, goal):
    path = []
    cur = goal
    while cur in parent:
        path.append(cur)
        cur = parent[cur]
    path.append(cur)
    path.reverse()
    return path

def bfs(grid, start, goal):
    frontier = deque([start])
    parent = {}
    visited = set([start])
    nodes_expanded = 0
    while frontier:
        cur = frontier.popleft()
        nodes_expanded += 1
        if cur == goal:
            path = reconstruct(parent, goal)
            cost = sum(grid[r][c] for (r,c) in path[1:])
            return {"path":path,"cost":cost,"nodes_expanded":nodes_expanded,"time":0}
        for n in neighbors(grid, *cur):
            if n not in visited:
                visited.add(n)
                parent[n] = cur
                frontier.append(n)
    return {"path":[],"cost":math.inf,"nodes_expanded":nodes_expanded,"time":0}

def ucs(grid, start, goal):
    frontier = []
    heapq.heappush(frontier, (0, start))
    parent = {}
    best_cost = {start:0}
    nodes_expanded = 0
    while frontier:
        cost_so_far, cur = heapq.heappop(frontier)
        if cost_so_far > best_cost.get(cur, math.inf):
            continue
        nodes_expanded += 1
        if cur == goal:
            path = reconstruct(parent, goal)
            return {"path":path,"cost":cost_so_far,"nodes_expanded":nodes_expanded,"time":0}
        for n in neighbors(grid, *cur):
            step_cost = grid[n[0]][n[1]]
            new_cost = cost_so_far + step_cost
            if new_cost < best_cost.get(n, math.inf):
                best_cost[n] = new_cost
                parent[n] = cur
                heapq.heappush(frontier, (new_cost, n))
    return {"path":[],"cost":math.inf,"nodes_expanded":nodes_expanded,"time":0}

def manhattan(a,b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def compute_min_cost(grid):
    mc = math.inf
    for r in range(len(grid)):
        for c in range(len(grid[0])):
            if grid[r][c] != 0 and grid[r][c] < mc:
                mc = grid[r][c]
    return mc if mc != math.inf else 1

def astar(grid, start, goal, heuristic=None):
    start_time = time.perf_counter()
    frontier = []
    h0 = 0 if heuristic is None else heuristic(start, goal)
    heapq.heappush(frontier, (h0, 0, start))
    parent = {}
    best_cost = {start:0}
    nodes_expanded = 0
    while frontier:
        fscore, cost_so_far, cur = heapq.heappop(frontier)
        if cost_so_far > best_cost.get(cur, math.inf):
            continue
        nodes_expanded += 1
        if cur == goal:
            path = reconstruct(parent, goal)
            return {"path":path,"cost":cost_so_far,"nodes_expanded":nodes_expanded,"time":time.perf_counter()-start_time}
        for n in neighbors(grid, *cur):
            step_cost = grid[n[0]][n[1]]
            new_cost = cost_so_far + step_cost
            h = 0 if heuristic is None else heuristic(n, goal)
            if new_cost < best_cost.get(n, math.inf):
                best_cost[n] = new_cost
                parent[n] = cur
                heapq.heappush(frontier, (new_cost + h, new_cost, n))
    return {"path":[],"cost":math.inf,"nodes_expanded":nodes_expanded,"time":time.perf_counter()-start_time}
