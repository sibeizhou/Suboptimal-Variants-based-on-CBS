# Suboptimal CBS Variants for Multi-Agent Path Finding

This project implements and evaluates Conflict-Based Search (CBS) variants for
the Multi-Agent Path Finding (MAPF) problem on grid maps. It compares optimal
CBS with bounded-suboptimal variants that use focal search and conflict
heuristics to reduce search effort while keeping solution quality close to the
optimal baseline.

The repository includes the original project report (`report.pdf`), generated
MAPF benchmark instances, runnable solver code, and smoke tests that validate
collision-free paths on a representative instance.

## Why This Project Matters

MAPF is a core planning problem behind warehouse robots, autonomous fleets, and
multi-drone coordination. The challenge is that each agent may have a short
individual path, but the joint search space grows quickly when agents conflict.

This project explores that trade-off:

- **CBS** finds optimal collision-free paths by resolving conflicts in a
  high-level constraint tree and replanning individual agents with A*.
- **BCBS** adds focal search with separate high-level and low-level weights to
  speed up planning with bounded suboptimality.
- **ECBS** uses a shared weight across both levels for a simpler bounded
  suboptimal search strategy.
- Two conflict heuristics are supported for focal-list ordering:
  number of conflicts and number of conflicting agents.

## Report Summary

The accompanying report studies whether CBS can be made faster and less
memory-intensive by relaxing either the high-level constraint-tree search, the
low-level A* search, or both. The project evaluates two CBS-based
bounded-suboptimal solvers:

- **BCBS (Bounded Suboptimal CBS)** uses focal search at the high level and/or
  low level. Nodes whose costs fall within a weighted bound of the current best
  candidate are placed in a focal list and ordered by a conflict heuristic.
- **ECBS (Enhanced CBS)** uses a shared focal-search weight across both levels,
  making the bounded-suboptimal configuration simpler to tune.

The experiments compare solver variants across CPU time, sum of costs, generated
nodes, and expanded nodes. The report uses CBS as the optimal baseline and
evaluates weights `1`, `1.2`, and `1.5`, where weight `1` is equivalent to
standard CBS.

## Key Findings

- Bounded-suboptimal CBS variants generally reduce runtime and search effort
  compared with optimal CBS, with only a small increase in solution cost.
- The `h2` heuristic, which counts the number of agents involved in conflicts,
  is more robust across instances than simply counting total conflicts (`h1`).
- Low-level focal search often reduces generated/expanded nodes more strongly,
  while high-level focal search can be faster in some weight ranges.
- The best weights are instance-dependent. In the report's experiments,
  `BCBS(1.2, 1.5)` achieved the strongest runtime result on one benchmark group,
  while `ECBS(1.2)` was a strong simple setting for ECBS.
- ECBS tends to generate fewer nodes in some cases, while BCBS can return
  solutions faster depending on the distribution of high- and low-level weights.

## Repository Layout

```text
.
|-- Final Project/code/
|   |-- cbs.py                    # High-level CBS, BCBS, and ECBS search
|   |-- single_agent_planner.py   # Space-time A* low-level planner
|   |-- run_experiments.py        # CLI entry point for demos and sweeps
|   |-- independent.py            # Independent planning baseline
|   |-- prioritized.py            # Prioritized planning baseline
|   `-- instances/                # MAPF benchmark instances
|-- tests/test_smoke.py           # Collision-free smoke tests
|-- report.pdf                    # Full project report
|-- proposal.pdf                  # Original proposal
`-- INTERVIEW_GUIDE.md            # Chinese interview talking points
```

## Quick Start

Use Python 3.9+.

```bash
pip install -r requirements.txt
cd "Final Project/code"
python run_experiments.py --instance "instances/test_40.txt" --solver CBS --method CBS --batch
```

Run a bounded-suboptimal variant:

```bash
python run_experiments.py --instance "instances/test_40.txt" --solver CBS --method BCBS --hc 1 --high-weight 1.5 --low-weight 1.5 --batch
```

Reproduce the full CBS/BCBS/ECBS parameter sweep used by the report:

```bash
python run_experiments.py --instance "instances/test_*.txt" --solver CBS --sweep --batch --output results.csv
```

Run the smoke tests from the repository root:

```bash
python -m unittest discover -s tests
```

## Implementation Highlights

- Models MAPF conflicts as vertex and edge collisions.
- Uses a high-level constraint tree to branch on conflicts.
- Uses space-time A* with wait actions for individual agent replanning.
- Supports focal-list ordering for bounded-suboptimal search.
- Tracks CPU time, sum of costs, generated nodes, and expanded nodes.
- Includes command-line controls for single demo runs and experiment sweeps.
