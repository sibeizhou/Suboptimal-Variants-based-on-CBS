import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost, get_size_of_map


class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []
        constraints = []
        max_time = 0

        # constraint1 = {'agent': 0, 'loc': [(1,5)], 'timestep': 4} # 1.2
        # constraint2 = {'agent': 1, 'loc': [(1,2), (1,3)], 'timestep': 1} # 1.3

        # constraint1 = {'agent': 0, 'loc': [(1,5)], 'timestep': 10} # 1.4

        # constraint1 = {'agent': 1, 'loc': [(1,4)], 'timestep': 2} # 1.5
        # constraint2 = {'agent': 1, 'loc': [(1,3), (1,2)], 'timestep': 2} # 1.5
        # constraint3 = {'agent': 1, 'loc': [(1,2)], 'timestep': 1} # 1.5
        # constraint4 = {'agent': 1, 'loc': [(1,3)], 'timestep': 2} # 1.5

        # constraints.append(constraint1)
        # constraints.append(constraint2)
        # constraints.append(constraint3)
        # constraints.append(constraint4)

        # 2.4 Time limit
        time_limit = 0

        for i in range(self.num_of_agents):  # Find path for each agent
            if i == 0:
                # 2.4 (default time limit = size of environment - 1)
                time_limit = get_size_of_map(self.my_map) - 1

            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints, time_limit, max_time)
            
            for j in range(i+1, self.num_of_agents):
                # 2.1/2.2 (add vertex/edge constraints for all future agents)
                add_constraints(path, j, constraints)
                # 2.3 (add target constraints for all future agents)
                additional_constraints(path, j, constraints)

            if path is None:
                raise BaseException('No solutions')
            result.append(path)

            if (len(path) - 1) > max_time:
                max_time = (len(path) - 1)

            # 2.4 (add the current the length of path for current agent to the time limit)
            time_limit += (len(path) - 1)

            ##############################
            # Task 2: Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches


            ##############################

        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        return result
