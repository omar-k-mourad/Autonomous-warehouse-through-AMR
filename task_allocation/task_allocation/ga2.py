from nav2_commander import *
from task_allocation.task_allocation.genetic_algorithm import generate_population
import time
import math

picking_stations = [(0.0,-2.0), (-1.0,-2.0), (1.0,-2.0)]

class Costs:
    def __init__(self, robots_num, tasks_num):
        self.C = [[]]
        self.D = [[]]
        self.W = []
        self.robots_num = robots_num
        self.tasks_num = tasks_num

    def update_Costs(self, robots_coordinates, robots_poses, task_shelves_coordinates, task_shelves_poses, distance_f, nav=None):
        if nav is not None:
            self.C = self.create_C(task_shelves_poses, distance_f, nav)
            self.D = self.create_D(robots_poses, task_shelves_poses, distance_f, nav)
        else:
            self.C = self.create_C(task_shelves_coordinates, distance_f)
            self.D = self.create_D(robots_coordinates, task_shelves_coordinates, distance_f)
        self.W = self.create_W(task_shelves_coordinates)

    
    #C,D and W functions
    def create_C(self, tasks_poses, distance_f, nav=None):
        # Cij --> C[i-1][j-(i+1)] : travel cost from ti to tj       condition: i < j
        return [
            [
                distance_f(tasks_poses[i], tasks_poses[j], nav) if nav is not None else distance_f(tasks_poses[i], tasks_poses[j])
                for j in range(i + 1, self.tasks_num)
            ]
            for i in range(self.tasks_num - 1)
        ]

    def create_D(self, robots_poses, tasks_poses, distance_f, nav=None):
        #Dij --> D[i][j] : travel cost from robot i to task j
        return [
            [
                distance_f(robot_pose, task_pose, nav) if nav is not None else distance_f(robot_pose, task_pose)
                for task_pose in tasks_poses
            ]
            for robot_pose in robots_poses
        ]     

    def create_W(self, tasks_coordinates):
        #Wi --> W[i]  : cost of task i
        return [self.cost_of_task(task_coordinate, picking_stations) for task_coordinate in tasks_coordinates]

    def cost_of_task(self, task_coordinate, picking_stations):
        
        return min(2 * euclidean_distance(station, task_coordinate) for station in picking_stations)
    
    def get_Di(self, robot_no, task_no):
        """
        task_no: first task in a pool
        """
        Di = self.D[robot_no][task_no - 1]
        return Di

    def get_Cij(self, task_pool, lenght):
        task_pool = sorted(task_pool)
        return sum([self.C[task_pool[i] - 1][task_pool[i+1] - (task_pool[i] + 1)] for i in range(lenght - 1)])

    def get_Wi(self, task_pool):
        return sum([self.W[task - 1] for task in task_pool])
    
    #ITC calculation 
    def calc_ITC(self, robot_num, task_pool):
        """
        ITC(ri, Ti)
        robot_num: ri
        task_pool: list of tasks numbers in a task pool (Ti)
        """
        lenght = len(task_pool)
        Di = self.get_Di(robot_num, task_pool[0]) #travel cost from ri to t1,  t1: first task in the pool
        Ci = self.get_Cij(task_pool, lenght) #Sum of travel costs between tasks in the pool
        Wi = self.get_Wi(task_pool) #Sum of costs for tasks in the pool
        ITC = Di + Ci + Wi
        return ITC
    
    #Fitness
    def fitness(self, chromosome):
        if self.bad_chromsome(chromosome):
            return 0
        else:
            task_poles = self.split_chromosome(chromosome)
            ITCs = [self.calc_ITC(i, task_poles[i]) for i in range(self.robots_num)]
            TTC = sum(ITCs)
            TT = max(ITCs)
            BU = min(ITCs) / max(ITCs)
            max_test = self.tasks_num
            total_test = sum(range(self.tasks_num + 1))
            fitness_score = (total_test / TTC) + (max_test / TT) + BU 
            return fitness_score

    def split_chromosome(self, chromosome):
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
        expected_length = (self.tasks_num + self.robots_num - 1)
        if len(chromosome) != expected_length:
            raise ValueError(f"Invalid chromosome length: expected {expected_length}, got {len(chromosome)}")

        # Check for numeric elements (already integers in this case)
        if not all(isinstance(element, int) for element in chromosome):
            raise ValueError("Chromosome must contain only integers")

        # Split the chromosome into sections based on virtual tasks
        robot_tasks = []
        current_robot = []
        for task in chromosome:
            if task <= self.tasks_num:  # Task belongs to the current robot
                current_robot.append(task)
            else:  # Virtual task, signifies end of current robot's tasks
                robot_tasks.append(current_robot)
                current_robot = []
        robot_tasks.append(current_robot)  # Add the last robot's tasks

        return robot_tasks
    
    def bad_chromsome(self, chromosome):

        current_robot = []
        for task in chromosome:
            if task <= self.tasks_num:  # Task belongs to the current robot
                current_robot.append(task)
            else:  # Virtual task, signifies end of current robot's tasks
                if len(current_robot) == 0:
                    return True
                current_robot = []
        if len(current_robot) == 0:
            return True
        return False


#Distance functions
def nav_distance(pose1, pose2, nav):
    return len((nav.getPath(start = pose1, goal = pose2, planner_id='GridBased',use_start=True)).poses)
def euclidean_distance(coord1, coord2):
    """Calculate the Euclidean distance between two points."""
    return (math.sqrt((coord2[0] - coord1[0])**2 + (coord2[1] - coord1[1])**2)) * 100

#other
def make_pose_stamps(coordinates, nav):
    pose_stamps = []
    for x,y in coordinates:
        pose_stamps.append(create_pose_stamped(nav, x, y, 0.0))
    return pose_stamps

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


def main():
    rclpy.init()
    nav = BasicNavigator()
    #inputs
    robots_num = 6
    robots_dict = make_robots_dict("robots.txt")
    robots_coordinates = robots_dict.values()
    robots_poses = make_pose_stamps(robots_coordinates, nav)
    task_shelves_coordinates =  [(2.0, 0.6), (2.0, 0.0), (2.0, -0.6),
                                (0.2, -0.4), (0.2, -0.6), (-2.0, 0.6),
                                (-2.0, 0.0), (-2.0, -0.6),
                                (0.6, 0.0), (0.6, 0.5), (0.6, -0.5),
                                (-0.6, 0.0), (-0.6, 0.5), (-0.6, 0.5),
                                (0.6, 0.1), (0.6, 0.6), (0.6, -0.6),
                                (-0.6, 0.1), (-0.6, 0.6), (-0.6, 0.6), (2.0, 0.6), (2.0, 0.0), (2.0, -0.6),
                                (0.2, -0.4), (0.2, -0.6), (-2.0, 0.6),
                                (-2.0, 0.0), (-2.0, -0.6),
                                (0.6, 0.0), (0.6, 0.5), (0.6, -0.5),
                                (-0.6, 0.0), (-0.6, 0.5), (-0.6, 0.5),
                                (0.6, 0.1), (0.6, 0.6), (0.6, -0.6),
                                (-0.6, 0.1), (-0.6, 0.6), (-0.6, 0.6),(2.0, 0.6), (2.0, 0.0), (2.0, -0.6),
                                (0.2, -0.4), (0.2, -0.6), (-2.0, 0.6),
                                (-2.0, 0.0), (-2.0, -0.6),
                                (0.6, 0.0), (0.6, 0.5), (0.6, -0.5),
                                (-0.6, 0.0), (-0.6, 0.5), (-0.6, 0.5),
                                (0.6, 0.1), (0.6, 0.6), (0.6, -0.6),
                                (-0.6, 0.1), (-0.6, 0.6), (-0.6, 0.6)]
    tasks_num = len(task_shelves_coordinates)
    task_shelves_poses = make_pose_stamps(task_shelves_coordinates, nav)
    #C,D and W creation nav_distance
    start = time.time()  
    costs = Costs(robots_num, tasks_num)
    costs.update_Costs(robots_coordinates, robots_poses, task_shelves_coordinates, task_shelves_poses, euclidean_distance)
    end = time.time()
    print("C,D and W creation time using nav: ", end - start," s")
    chromosomes = generate_population(robots_num, tasks_num, 100)
    chromosomes_scores = []
    for i, chromosome in enumerate(chromosomes):
        chromosomes_scores.append(costs.fitness(chromosome))
        print(f"{i} {chromosome}: score {chromosomes_scores[i]}")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
