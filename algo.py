import scipy.io


def load_map_from_mat(file_path):

    mat_data = scipy.io.loadmat(file_path)
    return mat_data['map'].tolist()


def wavefront_planner(map, start_row, start_col):
    len_rows = len(map)
    len_cols = len(map[0])

    # Initialize value_map with -1
    value_map = [[-1 for _ in range(len_cols)] for _ in range(len_rows)]
    for r in range(len_rows):
        for c in range(len_cols):
            if map[r][c] == 1:
                value_map[r][c] = -1  # Obstacles
            elif map[r][c] == 2:
                value_map[r][c] = 0  # Goal

    queue = []

    for r in range(len_rows):
        for c in range(len_cols):
            if map[r][c] == 2: # add goal to queue
                queue.append((r, c))

    while queue:
        current_r, current_c = queue.pop(0)
        current_value = value_map[current_r][current_c]

        # 8 point directions
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1),
                      (-1, -1), (-1, 1), (1, -1), (1, 1)]

        for dr, dc in directions:
            nr, nc = current_r + dr, current_c + dc
            if 0 <= nr < len_rows and 0 <= nc < len_cols and value_map[nr][nc] == -1:
                value_map[nr][nc] = current_value + 1
                queue.append((nr, nc))

    # backtracking
    trajectory = [(start_row, start_col)]
    current = (start_row, start_col)
    while value_map[current[0]][current[1]] != 0:
        neighbors = [(current[0] + dr, current[1] + dc)
                     for dr, dc in directions]
        neighbors = [(r, c) for r, c in neighbors if 0 <=
                     r < len_rows and 0 <= c < len_cols]
        current = min(neighbors, key=lambda x: value_map[x[0]][x[1]])
        trajectory.append(current)

    return value_map, trajectory

# Visualization


def visualize(map, value_map, trajectory):
    print("value_map:", value_map)

    for row in range(len(value_map)):
        line = ""
        for col in range(len(value_map[0])):
            if (row, col) in trajectory:
                line += " * "  # Mark the trajectory
            elif value_map[row][col] == -1:
                line += " X "  # Obstacle
            elif value_map[row][col] == 0:
                line += " G "  # Goal
            else:
                line += f"{value_map[row][col]:2d} "  # Wavefront values
        print(line)


file_path = "maze.mat"  # Replace with your .mat file path
map = load_map_from_mat(file_path)
start_row, start_col = 13, 2
value_map, trajectory = wavefront_planner(map, start_row, start_col)
visualize(map, value_map, trajectory)
# print("trajectory: ", trajectory)
