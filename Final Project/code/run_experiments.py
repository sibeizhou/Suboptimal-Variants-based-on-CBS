import argparse
import glob
from pathlib import Path
from cbs import CBSSolver
from independent import IndependentSolver
from prioritized import PrioritizedPlanningSolver
from visualize import Animation
from single_agent_planner import get_sum_of_cost

DEFAULT_SOLVER = "CBS"
DEFAULT_INSTANCE = "instances/test_40.txt"


def build_cbs_configs(args):
    """Return one CBS-family configuration, or the paper-style sweep."""
    if not args.sweep:
        high_weight = args.high_weight
        low_weight = args.low_weight
        if args.method == "CBS":
            high_weight = low_weight = 1
        elif args.method == "ECBS":
            low_weight = high_weight
        return [(args.method, args.hc, high_weight, low_weight)]

    methods = ["CBS", "BCBS", "ECBS"]
    hc_values = [1, 2]
    weights = [1, 1.2, 1.5]
    configs = []
    for method in methods:
        for hc in hc_values:
            if method == "CBS":
                configs.append((method, hc, 1, 1))
            elif method == "BCBS":
                for high_weight in weights:
                    for low_weight in weights:
                        if (high_weight, low_weight) != (1, 1):
                            configs.append((method, hc, high_weight, low_weight))
            else:
                for weight in weights:
                    if weight != 1:
                        configs.append((method, hc, weight, weight))
    return configs

def print_mapf_instance(my_map, starts, goals):
    print('Start locations')
    print_locations(my_map, starts)
    print('Goal locations')
    print_locations(my_map, goals)


def print_locations(my_map, locations):
    starts_map = [[-1 for _ in range(len(my_map[0]))] for _ in range(len(my_map))]
    for i in range(len(locations)):
        starts_map[locations[i][0]][locations[i][1]] = i
    to_print = ''
    for x in range(len(my_map)):
        for y in range(len(my_map[0])):
            if starts_map[x][y] >= 0:
                to_print += str(starts_map[x][y]) + ' '
            elif my_map[x][y]:
                to_print += '@ '
            else:
                to_print += '. '
        to_print += '\n'
    print(to_print)


def import_mapf_instance(filename):
    f = Path(filename)
    if not f.is_file():
        raise BaseException(filename + " does not exist.")
    f = open(filename, 'r')
    # first line: #rows #columns
    line = f.readline()
    rows, columns = [int(x) for x in line.split(' ')]
    rows = int(rows)
    columns = int(columns)
    # #rows lines with the map
    my_map = []
    for r in range(rows):
        line = f.readline()
        my_map.append([])
        for cell in line:
            if cell == '@':
                my_map[-1].append(True)
            elif cell == '.':
                my_map[-1].append(False)
    # #agents
    line = f.readline()
    num_agents = int(line)
    # #agents lines with the start/goal positions
    starts = []
    goals = []
    for a in range(num_agents):
        line = f.readline()
        sx, sy, gx, gy = [int(x) for x in line.split(' ')]
        starts.append((sx, sy))
        goals.append((gx, gy))
    f.close()
    return my_map, starts, goals


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Runs various MAPF algorithms')
    parser.add_argument('--instance', type=str, default=DEFAULT_INSTANCE,
                        help='The instance file or glob pattern to run')
    parser.add_argument('--batch', action='store_true', default=False,
                        help='Use batch output instead of animation')
    parser.add_argument('--disjoint', action='store_true', default=False,
                        help='Use the disjoint splitting')
    parser.add_argument('--solver', type=str, default=DEFAULT_SOLVER,
                        choices=['CBS', 'Independent', 'Prioritized'],
                        help='The solver to use')
    parser.add_argument('--method', type=str, default='CBS',
                        choices=['CBS', 'BCBS', 'ECBS'],
                        help='CBS-family variant to run when --solver CBS is selected')
    parser.add_argument('--hc', type=int, default=1, choices=[1, 2],
                        help='Focal-list conflict heuristic: 1=number of conflicts, 2=number of conflicting agents')
    parser.add_argument('--high-weight', type=float, default=1.5,
                        help='High-level focal-search weight for BCBS/ECBS')
    parser.add_argument('--low-weight', type=float, default=1.5,
                        help='Low-level focal-search weight for BCBS')
    parser.add_argument('--sweep', action='store_true', default=False,
                        help='Run the full CBS/BCBS/ECBS parameter sweep used in the report')
    parser.add_argument('--output', type=str, default='results.csv',
                        help='Where to write batch results')

    args = parser.parse_args()

    instance_files = sorted(glob.glob(args.instance))
    if not instance_files:
        raise FileNotFoundError("No instances matched: {}".format(args.instance))

    cbs_configs = build_cbs_configs(args)
    with open(args.output, "w", buffering=1) as result_file:
        for config in cbs_configs:
            for file in instance_files:
                print("***Import an instance***")
                my_map, starts, goals = import_mapf_instance(file)
                # print_mapf_instance(my_map, starts, goals)

                if args.solver == "CBS":
                    cbs = CBSSolver(my_map, starts, goals, config)
                    paths, result = cbs.find_solution(args.disjoint)
                elif args.solver == "Independent":
                    print("***Run Independent***")
                    solver = IndependentSolver(my_map, starts, goals)
                    paths = solver.find_solution()
                    result = "Method used:            Independent\nSum of costs:           {}\n\n".format(get_sum_of_cost(paths))
                elif args.solver == "Prioritized":
                    print("***Run Prioritized***")
                    solver = PrioritizedPlanningSolver(my_map, starts, goals)
                    paths = solver.find_solution()
                    result = "Method used:            Prioritized\nSum of costs:           {}\n\n".format(get_sum_of_cost(paths))
                else:
                    raise RuntimeError("Unknown solver!")

                result_file.write("{}\n".format(file))
                result_file.write(result)
                result_file.write("\n")

                if not args.batch:
                    print("***Test paths on a simulation***")
                    animation = Animation(my_map, starts, goals, paths)
                    # animation.save("output.mp4", 1.0)
                    animation.show()
