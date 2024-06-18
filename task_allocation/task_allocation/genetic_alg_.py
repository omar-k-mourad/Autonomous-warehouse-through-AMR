from nav2_commander import *
import numpy as np
import random

picking_stations = [(0.0,-2.0), (-1.0,-2.0), (1.0,-2.0)]
#ITC: Indvidual Travel Cost

def calc_ITC(robot_pose, task_pool_pose, task_shelves_coordinates, nav):
    D = travel_cost_robot_first_task(robot_pose, task_pool_pose[0], nav)
    C = travel_cost_of_task_pool(task_pool_pose, nav)
    W = cost_task_pool_to_station(task_shelves_coordinates)
    ITC = D + C + W
    return ITC

def travel_cost_robot_first_task(robot_pose, task_pose, nav):
    """
    task: first task in the pool
    """
    d = len((nav.getPath(start = robot_pose, goal = task_pose, planner_id='GridBased',use_start=True)).poses)
    return d

def travel_cost_of_task_pool(task_pool, nav):
    travel_cost_of_task_pool = 0
    for i in range(len(task_pool) - 1):
        travel_cost_of_task_pool += len((nav.getPath(start = task_pool[i], goal = task_pool[i+1], planner_id='GridBased',use_start=True)).poses)
    return travel_cost_of_task_pool

def cost_task_pool_to_station(task_pool):
    task_pool_cost = 0
    for task in task_pool:
        task_pool_cost += cost_of_task(task)
    return task_pool_cost

def cost_of_task(task_coordinate):
    distances = []    
    for station in picking_stations:
        distances.append(2 * euclidean_distance_numpy(np.array(station), np.array(task_coordinate)))
    return min(distances)

def euclidean_distance_numpy(point1, point2):
  """
  Calculates the Euclidean distance between two points using NumPy.

  Args:
      point1: A NumPy array representing the coordinates of the first point.
      point2: A NumPy array representing the coordinates of the second point.

  Returns:
      The Euclidean distance between the two points.
  """
  difference = point1 - point2
  return np.linalg.norm(difference)

def make_robots_dict(file_name):
    dict = {}
    # Open the file in read mode
    with open(file_name, 'r') as file:
        # Iterate over each line in the file
        for line in file:
            robot_name, xy = line.split(":")
            x,y = xy.strip()[1:-1].split(",")
            x = float(x.strip())
            y = float(y.strip())
            dict[robot_name] = (x , y) 
    return dict

def make_pose_stamps(coordinates, nav):
    pose_stamps = []
    for x,y in coordinates:
        pose_stamps.append(create_pose_stamped(nav, x, y, 0.0))
    return pose_stamps

def split_chromosome(chromosome, num_robots, num_tasks):
    """
    Splits a chromosome representation (list of integers) into a list of lists,
    representing task assignments for each robot.

    Args:
        chromosome (list): The chromosome representation as a list of integers.

    Returns:
        list: A list of lists, where each inner list represents the tasks assigned to a robot.

    Raises:
        ValueError: If the chromosome length is invalid or the chromosome contains non-numeric elements.
    """

    # Check for valid chromosome length
    expected_length = (num_tasks + num_robots - 1)
    if len(chromosome) != expected_length:
        raise ValueError(f"Invalid chromosome length: expected {expected_length}, got {len(chromosome)}")

    # Check for numeric elements (already integers in this case)
    if not all(isinstance(element, int) for element in chromosome):
        raise ValueError("Chromosome must contain only integers")

    # Split the chromosome into sections based on virtual tasks
    robot_tasks = []
    current_robot = []
    for task in chromosome:
        if task <= num_tasks:  # Task belongs to the current robot
            current_robot.append(task)
        else:  # Virtual task, signifies end of current robot's tasks
            robot_tasks.append(current_robot)
        current_robot = []
    robot_tasks.append(current_robot)  # Add the last robot's tasks

    return robot_tasks

def fitness(chromosome, num_robots, num_tasks, robots_poses, task_shelves_coordinates, nav):
    ITCs = []
    robots_task_poles = split_chromosome(chromosome, num_robots, num_tasks)
    for i, task_pole in enumerate(robots_task_poles):
       mapped_task_pole = map_chromosome_tasks_to_shelf_poses(task_pole, task_shelves_coordinates)
       ITCs.append(calc_ITC(robots_poses[i], mapped_task_pole, task_shelves_coordinates, nav))
    
    TTC = sum(ITCs)
    TT = max(ITCs)
    BU = min(ITCs) / max(ITCs)

    max_test = max(chromosome)
    total_test = sum(chromosome)
    fitness_score = (max_test/TT) + (total_test /TTC) + BU 
    
    return fitness_score

def map_chromosome_tasks_to_shelf_poses(task_pole, task_shelves_coordinates):
    mapped_task_pole = []
    for task_no in task_pole:
        mapped_task_pole.append(task_shelves_coordinates[task_no - 1])

    return mapped_task_pole

def selection(population):
    pass

def main():
    rclpy.init()
    nav = BasicNavigator()
    robots_dict = make_robots_dict("pose.csv")
    robots_poses = make_pose_stamps(robots_dict.values(), nav)
    num_robots = 3
    num_tasks = 8
    task_shelves_coordinates =   [(1.4, 0.6), (0.0, 0.6), (0.0, -0.6), (1.4, -0.6), (1, 0.6), (1, -0.6), (2, 0.6), (2, -0.6)]
    chromosome = [3, 1, 2, 9, 5, 6, 10, 8, 4, 7]
    task_pole = [3, 1, 2]

    m = map_chromosome_tasks_to_shelf_poses(task_pole, task_shelves_coordinates)
    print(m)
    #ans = fitness(chromosome, num_robots, num_tasks, robots_poses, task_shelves_coordinates, nav)
    #print(ans)

    rclpy.shutdown()

if __name__ == '__main__':
    main()


