# Multi-Agent Pathfinding with Conflict-Based Search

An implementation of Conflict-Based Search (CBS) for multi-agent pathfinding on 4-connected grid maps.

CBS is run on 10 test problems with 5 agents each, on a map from the Dragon Age: Origins benchmark set (from [movingai.com](https://movingai.com/benchmarks/)). Each solution is checked against the known optimal cost.

## The problem

Several agents share one grid map. Each agent has a start cell and a goal cell, and every agent needs a path to its goal so that no two agents are ever in the same cell at the same time.

A cell is either passable (`.` or `G` in the map file) or blocked (everything else). Time moves in steps (t = 0, 1, 2, ...), and all agents move at the same time. At each step, an agent can take one of these actions:

| Action | Cost |
|--------|------|
| Up, down, left, right | 1.0 |
| Wait (stay in the same cell) | 1.0 |

The goal is to minimize the **sum of costs**: the total of every agent's path cost added together.

This version uses **vertex constraints only**. Two agents may not be in the same cell at the same time, but they are allowed to swap cells with each other between two time steps.

## How CBS works

CBS splits the problem into two levels:

- **High level:** a best-first search over a tree of CBS nodes. Each node holds a set of constraints for each agent ("agent *i* cannot be in cell *X* at time *t*"), one path per agent, and a cost (the sum of the path costs).
- **Low level:** A* finds the cheapest path for one agent at a time while obeying that agent's constraints.

The search starts with a root node that has no constraints, so every agent simply takes its own shortest path. The high level then repeatedly takes the cheapest node from the heap:

1. If none of its paths conflict, it is the solution.
2. If two agents are in the same cell at the same time, CBS creates two children. In one child, the first agent is forbidden from that cell at that time. In the other child, the second agent is. Each child replans only under its new constraints, then both are added to the heap.

Because the cheapest node is always expanded first, the first conflict-free node found has the lowest possible sum of costs.

## Implementation

The CBS code is in `search/algorithms.py`. The `State` class, the `Map` class, the low-level `AStar` class, `CBSState.set_constraint` and `main.py` come from provided framework code.

### `CBSState.compute_cost`

Runs A* for every agent, using that agent's constraints. It stores each agent's path in `self._paths[i]` and stores the sum of all path costs in `self._cost`.

### `CBSState.is_solution`

Checks every state on every agent's path. A `State` counts as equal to another only if its x, y **and** time step (its g-value) all match, so a set of states that have already been seen detects any two agents in the same cell at the same time.

- If it finds a conflict, it returns `False, (state, time_step)`.
- Otherwise, it returns `True, None`.

### `CBSState.successors`

Gets the conflict from `is_solution`, finds the agents whose paths contain that state at that time, and creates two children for the first two of them. Each child gets a deep copy of the parent's constraints plus one new constraint for its agent, and then calls `compute_cost` to replan.

### `CBS.search`

Computes the root's cost and runs a best-first search with a binary heap (`heapq`), ordered by each node's sum of costs. It returns the paths and the total cost of the first node with no conflicts.

### The low-level A*

The provided A* is adapted for CBS in two ways:

- **Time is part of the state.** The same cell at different time steps counts as different states. This lets an agent wait in place, and lets A* tell apart a cell that is forbidden at one time but allowed at another.
- **It uses the agent's constraints.** The map's successor function skips any move into a cell that is forbidden at that time step, so every path A* returns obeys the agent's constraints.

## Results

On the easy debugging problem (`dao-map/test_map.map`), agent 0 must go from (1, 1) to (4, 1) and agent 1 from (5, 1) to (2, 1). At the root, both agents take straight paths and collide at (3, 1) at t = 2, for a cost of 6. CBS resolves this by having agent 0 wait one step, giving an optimal cost of 7:

```
Solution paths encountered for the easy test:
0 [[1, 1], [2, 1], [2, 1], [3, 1], [4, 1]]
1 [[5, 1], [4, 1], [3, 1], [2, 1]]
```

On all 10 test problems on `den009d.map`, CBS finds a solution whose sum of costs matches the expected optimal cost:

```
Correctly Solved:  150 150
Correctly Solved:  177 177
Correctly Solved:  223 223
Correctly Solved:  215 215
Correctly Solved:  152 152
Correctly Solved:  272 272
Correctly Solved:  143 143
Correctly Solved:  225 225
Correctly Solved:  205 205
Correctly Solved:  226 226
```

The first number on each line is the expected optimal cost, and the second is the cost CBS found.

## Repository structure

```
.
├── main.py                     # Runs CBS on the easy problem and the test set, checks results
├── requirements.txt            # numpy
├── search/
│   ├── algorithms.py           # State, CBSState, CBS and AStar
│   └── map.py                  # Map loading and the successor function (with constraints)
├── dao-map/                    # Dragon Age: Origins benchmark maps (.map format)
└── test-instances/
    └── test_problems_den009d.txt   # 10 problems: 5 agents each, then the optimal cost
```

Each line in the test file holds four numbers per agent (start x, start y, goal x, goal y), followed by the optimal sum of costs.

`dao-map/` has many maps, but `main.py` only uses `test_map.map` (for the easy problem) and `den009d.map` (for the test set).

## Running the code

You'll need Python 3 and the packages in `requirements.txt`:

```bash
pip install -r requirements.txt
```

Then run the following from the root of the repository:

```bash
python3 main.py
```

The script will:

1. Solve the easy problem and print each agent's path.
2. Load `dao-map/den009d.map` and the 10 problems in `test-instances/test_problems_den009d.txt`.
3. Solve each problem with CBS and compare the cost to the expected optimal cost. It prints `Correctly Solved` when they match, and a mismatch message with the details when they don't.

The script runs from the repository root because the file paths in `main.py` are relative.

## Limitations

- **Vertex conflicts only.** Agents can swap cells between two time steps, which would be a collision for real robots. Handling this would need edge constraints as well.
- **Agents disappear at their goal.** A path ends when the agent reaches its goal, so another agent passing through that goal cell later is not counted as a conflict.
- **No check for unreachable goals.** Because waiting is always possible and each time step counts as a new state, the low-level A* never runs out of states. If a goal can't be reached, the search runs forever instead of reporting failure.
- **CBS can be slow on hard problems.** When many agents conflict in tight spaces, the number of CBS nodes can grow very quickly.

## Acknowledgements

- The maps and the `.map` file format come from Nathan Sturtevant's [Moving AI Lab pathfinding benchmarks](https://movingai.com/benchmarks/).
- The framework code (`main.py`, `search/map.py`, the `State` and `AStar` classes and `CBSState.set_constraint`) was provided and is not my own work.
