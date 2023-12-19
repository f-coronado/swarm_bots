# import math
# import numpy as np
# import matplotlib
# import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation

# robots = {}

# def robot(num):
#     for i in range(0, num):
#         robot_name = f"robot{i}"
#         robot_position = [0.0, 0.0, 0.0]
#         robot_orientation = [0.0, 0.0, 0.0]
#         robots[robot_name] = {"position": robot_position, "orientation": robot_orientation}
#     return robots

# def start_positions(num):
#     total_robots = num
#     matrix_size = math.ceil(math.sqrt(total_robots))
#     robot_positions = {
#         f'robot{i}': {'position': [float(i // matrix_size), float(i % matrix_size), 0.0],
#                       'orientation': [0.0, 0.0, 0.0]}
#         for i in range(total_robots)
#     }
#     # print(robot_positions)
#     return robot_positions

# def path_points(rad, num):
#     radius = rad
#     num_points = num
#     theta = np.linspace(0, 2 * np.pi, num_points + 1)  # Add +1 to include the last point
#     points_list = []
#     for i in range(num_points + 1):  # Add +1 to include the last point
#         x_coord = radius * np.cos(theta[i])
#         y_coord = radius * np.sin(theta[i])
#         points_list.append((x_coord, y_coord))
#     return points_list

# def trajectory_dict(lst):
#     rotated_lists = {}
#     rotations = len(lst)

#     for i in range(rotations):
#         first_element = lst.pop(0)
#         lst.append(first_element)
#         rotated_lists[f"robot{i}"] = lst.copy()
#     return rotated_lists

# def update(frame, robots):
#     plt.clf()
#     # print("___________")
#     for robot_name, path in robots.items():
#         x, y = rotated_paths[robot_name][frame]
#         path['position'][0] = x
#         path['position'][1] = y
#         # print(x,y)
#         plt.plot(*path['position'][:2], 'o', label=robot_name)
#     # print("___________")
#     plt.title(f'Frame {frame + 1}')
#     plt.legend()
#     plt.pause(1)  

# def main():
#     num_envs = int(input("Enter the number of nodes: "))
#     circle_rad = int(input("Enter the radius of the circle: "))
#     robot(num=num_envs)
#     start_positions(num=num_envs)

#     global rotated_paths 
#     path = path_points(circle_rad, num_envs)
#     rotated_paths = trajectory_dict(path)

#     fig, ax = plt.subplots()
#     plt.xlabel('X-axis')
#     plt.ylabel('Y-axis')

#     num_frames = len(rotated_paths['robot0'])
#     robots_copy = robots.copy()
#     animation = FuncAnimation(fig, update, frames=num_frames, fargs=(robots_copy,), repeat=False)

#     plt.show()

# if __name__ == '__main__':
#     main()



import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

robots = {}

def create_robots(num):
    for i in range(num):
        robot_name = f"robot{i}"
        robot_position = [0.0, 0.0, 0.0]
        robot_orientation = [0.0, 0.0, 0.0]
        robots[robot_name] = {"position": robot_position, "orientation": robot_orientation}
    return robots

def start_positions(num):
    total_robots = num
    matrix_size = math.ceil(math.sqrt(total_robots))
    robot_positions = {
        f'robot{i}': {'position': [float(i // matrix_size), float(i % matrix_size), 0.0],
                      'orientation': [0.0, 0.0, 0.0]}
        for i in range(total_robots)
    }
    return robot_positions

def path_points(rad, num):
    radius = rad
    num_points = num
    theta = np.linspace(0, 2 * np.pi, num_points + 1)
    points_list = [(radius * np.cos(angle), radius * np.sin(angle)) for angle in theta]
    return points_list

def trajectory_dict(lst): # final, 1st is where they start the loop, end of each element is the final
    rotated_lists = {}
    rotations = len(lst)

    for i in range(rotations):
        first_element = lst.pop(0)
        lst.append(first_element)
        rotated_lists[f"robot{i}"] = lst.copy()
    return rotated_lists

def update(frame, robots, rotated_paths):
    plt.clf()
    for robot_name, path in robots.items():
        x, y = rotated_paths[robot_name][frame]
        path['position'][0] = x
        path['position'][1] = y
        plt.plot(*path['position'][:2], 'o', label=robot_name)
    plt.title(f'Frame {frame + 1}')
    plt.legend()

def main():
    num_envs = int(input("Enter the number of nodes: "))
    circle_rad = int(input("Enter the radius of the circle: "))
    
    create_robots(num=num_envs)
    initial_positions = start_positions(num=num_envs)
    
    for robot_name, pos_data in robots.items():
        pos_data.update(initial_positions[robot_name])

    path = path_points(circle_rad, num_envs)
    rotated_paths = trajectory_dict(path)
    print("rotated paths from trajectory_dict: ", rotated_paths)

    fig, ax = plt.subplots()
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')

    num_frames = len(rotated_paths['robot0'])
    robots_copy = robots.copy()
    animation = FuncAnimation(fig, update, frames=num_frames, fargs=(robots_copy, rotated_paths), repeat=False)

    plt.show()

if __name__ == '__main__':
    main()
