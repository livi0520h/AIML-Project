
# Simple CLI to run planners on a chosen map.
import argparse, json, time
from pathlib import Path
from planners import load_map, astar, ucs, bfs, compute_min_cost, manhattan

parser = argparse.ArgumentParser()
parser.add_argument("--map", required=True, help="Path to map file")
parser.add_argument("--algo", required=True, choices=["bfs","ucs","astar"], help="Planner to run")
args = parser.parse_args()

grid, meta = load_map(args.map)
start = tuple(meta["start"])
goal = tuple(meta["goal"])
min_cost = compute_min_cost(grid)

if args.algo == "bfs":
    r = bfs(grid, start, goal)
elif args.algo == "ucs":
    r = ucs(grid, start, goal)
elif args.algo == "astar":
    r = astar(grid, start, goal, heuristic=lambda a,b: manhattan(a,b)*min_cost)
else:
    r = {"path":[], "cost": float("inf"), "nodes_expanded":None, "time":0}

print(json.dumps(r, indent=2))
