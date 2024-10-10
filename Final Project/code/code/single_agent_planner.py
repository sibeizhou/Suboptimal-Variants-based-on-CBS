import heapq

def move(loc, dir):
    # Modified move: add a 5th dir that stay at the current node
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]

def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst

def get_size_of_map(my_map):
    size = 0
    for i in range(len(my_map)):
        for j in range(len(my_map[0])):
            if not my_map[i][j]:
                size += 1
    return size

def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
               or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
               continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values

def get_max_timestep(constraint_table):
    max_timestep = 0
    for timestep in constraint_table:
        if int(timestep) > max_timestep:
            max_timestep = int(timestep)

    return max_timestep

def build_constraint_table(constraints, agent):
    ##############################
    # Task 1.2/1.3: Return a table that constains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the 
    #               is_constrained function.

    # 1.2
    constraint_table = dict()
    for constraint in constraints:
        if (constraint['agent'] == agent) | (constraint['positive']):
            if (constraint['timestep']) not in constraint_table:
                constraint_table[(constraint['timestep'])] = [constraint]
            else:
                constraint_table[(constraint['timestep'])].append(constraint)

    return constraint_table

def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location

def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path

def is_constrained(agent, curr_loc, next_loc, next_time, constraint_table):
    ##############################
    # Task 1.2/1.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.

    # 1.2
    # if next_time in constraint_table:
    #     constraint = constraint_table[next_time]
    #     return ( next_loc in constraint['loc']) & (constraint['timestep'] == next_time)
    # return False

    # 1.3/ 4.1 (Supporting Positive Constraints)
    if next_time in constraint_table:
        constraints = constraint_table[next_time]
        for constraint in constraints:
            if constraint['positive']: 
                # Positive Constraints
                if (len(constraint['loc']) == 2):
                    # Edge constraint
                    if (constraint['loc'][0] != constraint['loc'][1]):
                        constraint_from = constraint['loc'][1]
                        constraint_to = constraint['loc'][0]
                        if (constraint['agent'] == agent) & (constraint['timestep'] == next_time) & ((curr_loc != constraint_from) | ( next_loc != constraint_to)):
                            return True
                        if (constraint['agent'] != agent) & (constraint['timestep'] == next_time) & ((curr_loc == constraint_from) | ( next_loc == constraint_to)):
                            return True
                else:
                    # Vertex constraint
                    if (constraint['agent'] == agent) & (constraint['timestep'] == next_time) & (next_loc not in constraint['loc']):
                        return True
                    if (constraint['agent'] != agent) & (constraint['timestep'] == next_time) & (next_loc in constraint['loc']):
                        return True
            else: 
                # Negative Constraints
                if (len(constraint['loc']) == 2):
                    # Edge constraint
                    if (constraint['loc'][0] != constraint['loc'][1]):
                        constraint_from = constraint['loc'][0]
                        constraint_to = constraint['loc'][1]
                        if (curr_loc == constraint_from) & ( next_loc == constraint_to) & (constraint['timestep'] == next_time):
                            return True
                else:
                    # Vertex constraint
                    if (next_loc in constraint['loc']) & (constraint['timestep'] == next_time):
                        return True

    # # 2.3 Target constraint
    # constraints = []
    # for value in constraint_table:
    #     constraints += constraint_table[value]
    # for constraint in constraints:
    #     if (len(constraint['loc']) == 2):
    #         if (constraint['loc'][0] == constraint['loc'][1]):
    #             if (next_loc in constraint['loc']):
    #                 return True

    # 4.1 (Supporting Positive Constraints)

    return False

def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))

def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr

def pop_node_BCBS(focal_list, open_list, paths, w, hc):
    if len(focal_list) > 0:
        _, _, _, node = heapq.heappop(focal_list)
    else:
        push_nodes_toFocal(focal_list, open_list, paths, w, hc)
        _, _, _, node = heapq.heappop(focal_list)

    return node

def push_nodes_toFocal(focal_list, open_list, paths, weight, hc):
    min_f = 1
    count = 0
    for i in range(len(open_list)):
        _, _, _, node = open_list[i]
        if i == 0:
            min_f = node['g_val'] + node['h_val']

        if node['g_val'] + node['h_val'] <= min_f * weight:
            count = count + 1

    for i in range(count):
        _, _, _, node = heapq.heappop(open_list)
        if hc == 1:
            conflict = count_conflicts(node, paths)   # h1
        elif hc == 2:
            conflict = get_conflicting_agents(node, paths)  # h2
        else:
            print("Hc value error!")
            print(hc)

        heapq.heappush(focal_list, (conflict, node['g_val'] + node['h_val'], node['loc'], node))

def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']

# 2.1/2.2 Function that add constrains by path of last agent for future agent
def add_constraints(path, future_agent, constraints):
    """Create constraints by the path and add them into constraints"""
    for i in range(len(path)):
        if (i != 0) & (path[i-1] != path[i]):
            constraint = {'agent': future_agent, 'loc': [path[i], path[i-1]], 'timestep': i} # 2.2 edge constraints
            constraints.append(constraint)
        constraint = {'agent': future_agent, 'loc': [path[i]], 'timestep': i} # 2.1 vertext constraints
        constraints.append(constraint)

# 2.3 Function that add additional constrains by target of last agent for future agent
def additional_constraints(path, future_agent, constraints):
    """Create constraints by the path and add them into constraints"""
    target = path[len(path)-1]
    constraint = {'agent': future_agent, 'loc': [target, target], 'timestep': len(path)-1} # special constraint for target constraint: 
                                                                                           # look like edge constraint but the start node == end node
    constraints.append(constraint)

# # 3.3 Function that minus the redundant path that agent reach target
# def path_corrected(path):
#     target = path[len(path) - 1]
#     real_target_index = len(path) - 1
#     for i in range(len(path)):
#         if path[len(path)- 1 - i] == target:
#             real_target_index -= 1
#         else:
#             path = path[0:real_target_index + 2]

# def paths_corrected(paths):
#     for i in range(len(paths)):
#         paths[i] = path_corrected(paths[i])

def count_conflicts(curr, paths):
    curr_path = get_path(curr)
    count = 0
    for path in paths:
        length = min(len(curr_path), len(path))
        for i in range(length):
            if curr_path[i] == path[i]:
                count = count + 1
    return count

def get_conflicting_agents(curr, paths):
    curr_path = get_path(curr)
    count = 0
    for path in paths:
        conflict = False
        length = min(len(curr_path), len(path))
        for i in range(length):
            if curr_path[i] == path[i]:
                conflict = True
        if conflict:
            count = count + 1
    return count

def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints, paths, low_focal_search_w, hc_value):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
        paths       - paths of all agents generated before
    """
    hc = hc_value

    paths_copy = []
    for path in paths:
        if path[0] != start_loc or path[-1] != goal_loc:
            paths_copy.append(path)

    if low_focal_search_w > 1:
        low_focal_search = True
    else:
        low_focal_search = False

    ##############################
    # Task 1.1: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.

    # 1.2 (create the constraint table indexed by time step)
    constraint_table = build_constraint_table(constraints, agent)

    open_list = []
    focal_list = []
    closed_list = dict()
    earliest_goal_timestep = 0
    h_value = h_values[start_loc]
    # 1.1 (add a new key/value pair: timestep)
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep': 0}
    push_node(open_list, root)
    # 1.1 (use (cell,timestep) to index the closed_list)
    closed_list[(root['loc'], root['timestep'])] = root

    max_timestep = get_max_timestep(constraint_table)
    while (len(open_list) > 0):
        if low_focal_search:
            curr = pop_node_BCBS(focal_list, open_list, paths_copy, low_focal_search_w, hc)
        else:
            curr = pop_node(open_list)
        #############################
        # Task 1.4: Adjust the goal test condition to handle goal constraints
        if (curr['loc'] == goal_loc) & (curr['timestep'] >= max_timestep):
            path = get_path(curr)
            lb = h_values[start_loc]
            return path, lb

        # 2.4 (return None if the timestep is larger than the time limit)
        # if (curr['timestep'] > time_limit):
        #     return None

        for dir in range(5): # 1.1 (the 5th child node is stay at current cell)
            child_loc = move(curr['loc'], dir)

            if child_loc[0] < 0 or child_loc[0] >= len(my_map) or child_loc[1] < 0 \
                    or child_loc[1] >= len(max(my_map)):
                continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            
            child = {'loc': child_loc,
                    'g_val': curr['g_val'] + 1,
                    'h_val': h_values[child_loc],
                    'parent': curr,
                    'timestep': curr['timestep'] + 1}
          
            # 1.2 (check is the child node create is constrained)
            if is_constrained(agent, curr['loc'], child['loc'], child['timestep'], constraint_table):
                continue

            if (child['loc'], child['timestep']) in closed_list:
                existing_node = closed_list[(child['loc'], child['timestep'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'], child['timestep'])] = child
                    push_node(open_list, child)
            else:
                closed_list[(child['loc'], child['timestep'])] = child
                push_node(open_list, child)

    return [], h_values[start_loc]  # Failed to find solutions
