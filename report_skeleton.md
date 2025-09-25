# Short Report (Skeleton)

## Environment model
- Grid maps where each cell contains an integer movement cost >=1 (0 = obstacle).
- Moving obstacles represented via schedules (deterministic) or as unpredictable events (used for local search testing).

## Agent design & planners implemented
- BFS (for uniform-cost baseline)
- Uniform Cost Search (UCS)
- A* with admissible heuristic (Manhattan * min_cell_cost)
- Local-search replanning (waypoint-based simulated annealing / hill-climbing with random restarts)

## Heuristics
- A* uses Manhattan distance multiplied by the minimum traversable cell cost (admissible).

## Experiments
- Maps: small (10x10), medium (30x30), large (60x60), dynamic (20x20)
- Metrics: path cost, nodes expanded, execution time, replanning events (for dynamic map)

## Results
- (Insert tables and plots present in output/plots)

## Analysis & Conclusion
- (Discuss when informed search helps, how UCS compares to A*, etc.)

