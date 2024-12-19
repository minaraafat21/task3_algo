import scipy.io
import numpy as np
import matplotlib.pyplot as plt

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
    # Priority directions: [upper, right, lower, left, upper-right, lower-right, lower-left, upper-left]
    directions = [(-1, 0), (0, 1), (1, 0), (0, -1),
                 (-1, 1), (1, 1), (1, -1), (-1, -1)]

    # Add goals to queue
    for r in range(len_rows):
        for c in range(len_cols):
            if map[r][c] == 2:
                queue.append((r, c))

    while queue:
        current_r, current_c = queue.pop(0)
        current_value = value_map[current_r][current_c]

        for dr, dc in directions:
            nr, nc = current_r + dr, current_c + dc
            if 0 <= nr < len_rows and 0 <= nc < len_cols and value_map[nr][nc] == -1:
                if map[nr][nc] != 1:  # Not an obstacle
                    value_map[nr][nc] = current_value + 1
                    queue.append((nr, nc))

    # Backtracking
    trajectory = [(start_row, start_col)]
    current = (start_row, start_col)
    
    while value_map[current[0]][current[1]] != 0:
        neighbors = [(current[0] + dr, current[1] + dc)
                    for dr, dc in directions]
        valid_neighbors = []
        for r, c in neighbors:
            if (0 <= r < len_rows and 0 <= c < len_cols and 
                value_map[r][c] != -1):
                valid_neighbors.append((r, c))
        
        if not valid_neighbors:
            print("No valid path found!")
            return value_map, trajectory
            
        current = min(valid_neighbors, key=lambda x: value_map[x[0]][x[1]])
        trajectory.append(current)

    return value_map, trajectory

def print_value_map_and_trajectory(value_map, trajectory):
    # Print value_map
    print("value_map =")
    for row in value_map:
        print(" ".join(f"{val:2}" if val != -1 else " 1" for val in row))
    
    # Print trajectory
    print("\ntrajectory =")
    for point in trajectory:
        print(f"{point[0]:>4}  {point[1]}")

def visualize_path(map, value_map, trajectory):
    # Print text output
    print_value_map_and_trajectory(value_map, trajectory)

    plt.figure(figsize=(12, 8))
    
    # Create subplot for the map with trajectory
    plt.subplot(1, 2, 1)
    plt.imshow(np.array(map), cmap='gray')
    
    # Plot trajectory
    trajectory_array = np.array(trajectory)
    plt.plot(trajectory_array[:, 1], trajectory_array[:, 0], 'r-', linewidth=2, label='Path')
    plt.plot(trajectory_array[0, 1], trajectory_array[0, 0], 'go', label='Start')
    plt.plot(trajectory_array[-1, 1], trajectory_array[-1, 0], 'ro', label='Goal')
    
    plt.title('Map with Trajectory')
    plt.legend()
    plt.grid(True)
    
    # Create subplot for the wavefront values
    plt.subplot(1, 2, 2)
    value_map_array = np.array(value_map)
    plt.imshow(value_map_array, cmap='viridis')
    plt.colorbar(label='Distance from Goal')
    
    # Plot trajectory on wavefront visualization
    plt.plot(trajectory_array[:, 1], trajectory_array[:, 0], 'r-', linewidth=2)
    plt.plot(trajectory_array[0, 1], trajectory_array[0, 0], 'go')
    plt.plot(trajectory_array[-1, 1], trajectory_array[-1, 0], 'ro')
    
    plt.title('Wavefront Values')
    plt.grid(True)
    
    plt.tight_layout()
    plt.show()

# Example usage
if __name__ == "__main__":
    file_path = "maze.mat"  # Replace with your .mat file path
    map = load_map_from_mat(file_path)
    start_row, start_col = 45, 4
    value_map, trajectory = wavefront_planner(map, start_row, start_col)
    visualize_path(map, value_map, trajectory)