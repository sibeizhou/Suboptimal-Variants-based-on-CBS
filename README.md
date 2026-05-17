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

## Repository Layout

```text
.
├── Final Project/code/code/
│   ├── cbs.py                    # High-level CBS, BCBS, and ECBS search
│   ├── single_agent_planner.py   # Space-time A* low-level planner
│   ├── run_experiments.py        # CLI entry point for demos and sweeps
│   ├── independent.py            # Independent planning baseline
│   ├── prioritized.py            # Prioritized planning baseline
│   └── instances/                # MAPF benchmark instances
├── tests/test_smoke.py           # Collision-free smoke tests
├── report.pdf                    # Full project report
├── proposal.pdf                  # Original proposal
└── INTERVIEW_GUIDE.md            # Chinese interview talking points
```

## Quick Start

Use Python 3.9+.

```bash
pip install -r requirements.txt
cd "Final Project/code/code"
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

## Example Output

```text
Found a solution!
CPU time (s):    0.00
Sum of costs:    24
Expanded nodes:  2
Generated nodes: 3
```

## Implementation Highlights

- Models MAPF conflicts as vertex and edge collisions.
- Uses a high-level constraint tree to branch on conflicts.
- Uses space-time A* with wait actions for individual agent replanning.
- Supports focal-list ordering for bounded-suboptimal search.
- Tracks CPU time, sum of costs, generated nodes, and expanded nodes.
- Includes command-line controls for single demo runs and experiment sweeps.

## Interview Framing

The most interview-relevant part of this project is the engineering trade-off:
optimal CBS gives the best path cost but can expand many nodes; bounded variants
accept a controlled loss in solution cost to reduce runtime and memory pressure.

A concise resume bullet:

> Implemented and evaluated CBS, BCBS, and ECBS solvers for Multi-Agent Path
> Finding in Python, using space-time A* and focal-search heuristics to reduce
> high-level constraint-tree expansion while preserving collision-free paths.

