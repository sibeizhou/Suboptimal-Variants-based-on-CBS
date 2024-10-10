from json.encoder import INFINITY
import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost


def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.
    collision = dict()

    Len = max(len(path1), len(path2))

    for i in range(Len):
        if get_location(path1, i) == get_location(path2, i):
            collision = {'a1': 0, 'a2': 0, 'loc': [get_location(path1, i)], 'timestep': i}
            return collision
        elif i != 0:
            if (get_location(path1, i) == get_location(path2, i - 1)) & (
                    get_location(path1, i - 1) == get_location(path2, i)):
                collision = {'a1': 0, 'a2': 0,
                             'loc': [get_location(path1, i - 1), get_location(path1, i), get_location(path2, i - 1),
                                     get_location(path2, i)], 'timestep': i}
                return collision
    return collision


def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.

    collisions = []
    for i in range(len(paths)):
        for j in range(len(paths)):
            if i < j:
                collision = detect_collision(paths[i], paths[j])
                if len(collision) != 0:
                    collision['a1'] = i
                    collision['a2'] = j
                    collisions.append(collision)

    return collisions


def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep
    constraints = []
    if len(collision['loc']) == 1:
        # vertext collision
        constraint1 = {'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep'],
                       'positive': False}
        constraint2 = {'agent': collision['a2'], 'loc': collision['loc'], 'timestep': collision['timestep'],
                       'positive': False}
        constraints.append(constraint1)
        constraints.append(constraint2)
    else:
        # edge collision
        constraint1 = {'agent': collision['a1'], 'loc': collision['loc'][0:2], 'timestep': collision['timestep'],
                       'positive': False}
        constraint2 = {'agent': collision['a2'], 'loc': collision['loc'][2:], 'timestep': collision['timestep'],
                       'positive': False}
        constraints.append(constraint1)
        constraints.append(constraint2)

    return constraints


def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly

    agent_modified = 1  # random.randint(0,1) # use random.randint(0,1) to choose one of the two colliding agents randomly

    constraints = []
    if len(collision['loc']) == 1:
        # vertext collision
        if agent_modified == 0:
            constraint1 = {'agent': collision['a2'], 'loc': collision['loc'], 'timestep': collision['timestep'],
                           'positive': True}
            constraint2 = {'agent': collision['a2'], 'loc': collision['loc'], 'timestep': collision['timestep'],
                           'positive': False}
        else:
            constraint1 = {'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep'],
                           'positive': False}
            constraint2 = {'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep'],
                           'positive': True}
        constraints.append(constraint1)
        constraints.append(constraint2)
    else:
        # edge collision
        if agent_modified == 0:
            constraint1 = {'agent': collision['a2'], 'loc': collision['loc'][0:2], 'timestep': collision['timestep'],
                           'positive': True}
            constraint2 = {'agent': collision['a2'], 'loc': collision['loc'][2:], 'timestep': collision['timestep'],
                           'positive': False}
        else:
            constraint1 = {'agent': collision['a1'], 'loc': collision['loc'][0:2], 'timestep': collision['timestep'],
                           'positive': False}
            constraint2 = {'agent': collision['a1'], 'loc': collision['loc'][2:], 'timestep': collision['timestep'],
                           'positive': True}
        constraints.append(constraint1)
        constraints.append(constraint2)

    return constraints


class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals, tuple):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """
        self.method, self.hc, self.high_focal_search_w, self.low_focal_search_w = tuple
        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.num_of_test = 0
        self.CPU_time = 0

        self.open_list = []
        self.Focal_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        # print("Generate node {}".format(self.num_of_generated))
        # print(node)  # print the inf of node
        # print()
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        # print("Expand node {}".format(id))
        # print(node) # print the inf of node
        # print()
        self.num_of_expanded += 1
        return node

    # For ECBS
    def pop_node_ECBS(self, weight):
        if len(self.Focal_list) > 0:
            node = self.pop_node_fromFocal()
        else:
            self.push_nodes_toFocal_ECBS(weight)
            node = self.pop_node_fromFocal()
        self.num_of_expanded += 1
        return node

    def push_nodes_toFocal_ECBS(self, weight):
        count = 0
        LB = 100000
        for i in range(len(self.open_list)):
            _, _, _, node = self.open_list[i]
            LB = min(node['LB'], LB)

        for j in range(len(self.open_list)):
            _, _, _, node = self.open_list[j]
            if node['cost'] <= LB * weight:
                count = count + 1

        for k in range(count):
            _, _, _, node = heapq.heappop(self.open_list)
            if self.hc == 1:
                self.push_node_toFocal(node)   # change hc
            else:
                self.push_node_toFocal2(node)

    # For BCBS
    def pop_node_BCBS(self, weight):
        if len(self.Focal_list) > 0:
            node = self.pop_node_fromFocal()
        else:
            self.push_nodes_toFocal_BCBS(weight)
            node = self.pop_node_fromFocal()
        # print("Expand node {}".format(id))
        # print(node)  # print the inf of node
        # print()
        self.num_of_expanded += 1
        return node

    def push_nodes_toFocal_BCBS(self, weight):
        min_cost = 1
        count = 0
        for i in range(len(self.open_list)):
            _, _, _, node = self.open_list[i]
            if i == 0:
                min_cost = node['cost']

            if node['cost'] <= min_cost * weight:
                count = count + 1

        for i in range(count):
            _, _, _, node = heapq.heappop(self.open_list)
            if self.hc == 1:
                self.push_node_toFocal(node)   # change hc
            else:
                self.push_node_toFocal2(node)

    def push_node_toFocal(self, node):
        # H_c = Number of conflicts: this heuristic counts the number
        #       of conflicts that are encountered in a specific CT node.
        self.num_of_test += 1
        tuple = (len(node['collisions']), node['cost'], self.num_of_test, node)
        heapq.heappush(self.Focal_list, tuple)

    def get_conflicting_agents(self, node):
        agents = []
        collisions = node['collisions']
        for collision in collisions:
            a1 = collision['a1']
            if a1 not in agents:
                agents.append(a1)
            a2 = collision['a2']
            if a2 not in agents:
                agents.append(a2)

        return len(agents)

    def get_conflicting_agent_pair(self, node):
        pairs = []
        collisions = node['collisions']
        for collision in collisions:
            a1 = collision['a1']
            a2 = collision['a2']
            if (a1, a2) not in pairs and (a2, a1) not in pairs:
                pairs.append((a1, a2))

        return len(pairs)

    def push_node_toFocal2(self, node):
        # H_c = Number of conflicting agents: this heuristic counts
        #       the number of agents (out of k) that have at least one conflict.
        self.num_of_test += 1
        conflicting_agents = self.get_conflicting_agents(node)
        tuple = (conflicting_agents, len(node['collisions']), self.num_of_test, node)
        heapq.heappush(self.Focal_list, tuple)

    def pop_node_fromFocal(self):
        if len(self.Focal_list) > 0:
            _, _, _, node = heapq.heappop(self.Focal_list)
            return node
        else:
            print("There are no node in Focal list!")
            return None

    def copy_path(self, path):
        new_path = []
        for i in range(len(path)):
            vertex = path[i]
            new_path.append(vertex)

        return new_path

    # # 3.3 Function that minus the redundant path that agent reach target
    def path_corrected(self, path):
        if len(path) == 1:
            return path
        new_path = []

        # copy path
        for i in range(len(path)):
            vertex = path[i]
            new_path.insert(0, vertex)

        target = path[len(path) - 1]
        real_target_index = 0
        for i in range(len(new_path)):
            if new_path[i] == target:
                real_target_index += 1
            else:
                real_target_index -= 1
                break
        new_path = new_path[real_target_index:len(path)]

        return_path = []
        for j in range(len(new_path)):
            return_path.insert(0, new_path[j])

        return return_path

    def paths_corrected(self, paths):
        new_paths = []
        for i in range(len(paths)):
            new_paths.append(self.path_corrected(paths[i]))

        return new_paths

    def find_solution(self, disjoint=True):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """
        # Change to use different method (CBS, BCBS, ECBS)
        # self.method = 'BCBS'

        # self.high_focal_search_w = 1.5
        # self.low_focal_search_w = 1.5
        #
        # self.hc = 1 # 1 or 2

        if(self.method == 'CBS'):
            # On CBS, there are no any level (low or high) is using focal queue
            self.high_focal_search_w = 1
            self.low_focal_search_w = 1

        if(self.method == 'ECBS'):
            # On ECBS, the weight for high level is equal to low level
            self.high_focal_search_w = self.low_focal_search_w

        if self.high_focal_search_w > 1:
            high_focal_search = True
        else:
            high_focal_search = False

        self.start_time = timer.time()
        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths

        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': [],
                'LB': 0}

        for i in range(self.num_of_agents):  # Find initial path for each agent
            path, lb = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'], root['paths'], self.low_focal_search_w, self.hc)
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)
            root['LB'] = root['LB'] + lb
        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        # Task 3.1: Testing
        # print(root['collisions'])

        # Task 3.2: Testing
        # for collision in root['collisions']:
        #     print(standard_splitting(collision))

        # Task 4.2: Testing
        # for collision in root['collisions']:
        #     print(disjoint_splitting(collision))

        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit

        while (len(self.open_list) > 0):
            if high_focal_search:
                if self.method == 'ECBS':
                    curr = self.pop_node_ECBS(self.high_focal_search_w)
                elif self.method == 'BCBS':
                    curr = self.pop_node_BCBS(self.high_focal_search_w)
            else:
                curr = self.pop_node()

            if len(curr['collisions']) == 0:
                # Find solution!
                # print(curr['paths'])

                curr['paths'] = self.paths_corrected(curr['paths'])
                # print(curr['paths'])
                # curr['cost'] = get_sum_of_cost(curr['paths'])

                self.print_results(curr)
                result_str = self.get_results(curr)
                return curr['paths'], result_str

            # if self.num_of_generated > 10000: # Time limit for test_47
            #     print("No solustion")
            #     return root['paths']

            collision = curr['collisions'][0]

            disjoint = False  # Change the value of disjoint to decide use disjoint splitting or not

            if (disjoint):
                # 4.2 using disjoint splitting
                constraints = disjoint_splitting(collision)
            else:
                # 3.3 Do no using disjoint splitting
                constraints = standard_splitting(collision)

            for constraint in constraints:
                new_node = {'cost': 0,
                            'constraints': [],
                            'paths': [],
                            'collisions': [],
                            'LB': curr['LB']}
                if constraint not in curr['constraints']:
                    new_node['constraints'] = curr['constraints'] + [constraint]

                for path in curr['paths']:
                    copyed_path = self.copy_path(path)
                    new_node['paths'].append(copyed_path)

                # Negative constraint
                if (constraint['positive'] != True):
                    a_i = constraint['agent']
                    path_i, _ = a_star(self.my_map, self.starts[a_i], self.goals[a_i], self.heuristics[a_i], a_i,
                                    new_node['constraints'], new_node['paths'], self.low_focal_search_w, self.hc)
                    if len(path_i) != 0:
                        new_node['paths'][a_i] = path_i
                        new_node['collisions'] = detect_collisions(new_node['paths'])
                        new_node['cost'] = get_sum_of_cost(new_node['paths'])
                        self.push_node(new_node)
                # Positive constraint
                else:
                    for a_j in range(self.num_of_agents):
                        path_j, _ = a_star(self.my_map, self.starts[a_j], self.goals[a_j], self.heuristics[a_j], a_j,
                                        new_node['constraints'], new_node['paths'], self.low_focal_search_w, self.hc)
                        if len(path_j) != 0:
                            new_node['paths'][a_j] = path_j
                    new_node['collisions'] = detect_collisions(new_node['paths'])
                    new_node['cost'] = get_sum_of_cost(new_node['paths'])
                    self.push_node(new_node)

        self.print_results(root)
        return root['paths']

    def print_results(self, node):
        print("\nFound a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))

    def get_results(self, node):
        CPU_time = timer.time() - self.start_time
        result = "Method used:            {}\n".format(self.method) + \
                 "Weight used (high/low): {}/{}\n".format(self.high_focal_search_w, self.low_focal_search_w) + \
                 "Heuristics for h_c:     h{}\n".format(self.hc) + \
                 "CPU time (s):           {:.2f}\n".format(CPU_time) + \
                 "Sum of costs:           {}\n".format(get_sum_of_cost(node['paths'])) + \
                 "Expanded nodes:         {}\n".format(self.num_of_expanded) + \
                 "Generated nodes:        {}\n\n".format(self.num_of_generated)

        return result