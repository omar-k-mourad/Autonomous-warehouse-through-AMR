import random
from nav2_commander import *
import time
import math

intial_mutation_rate = 0.1
final_mutation_rate = 0.5

class Costs:
    def __init__(self, robots_num, tasks_num):
        self.C = [[]]
        self.D = [[]]
        self.W = []
        self.robots_num = robots_num
        self.tasks_num = tasks_num

    def update_Costs(self, robots_coordinates, robots_poses, task_shelves_coordinates, task_shelves_poses, distance_f,picking_stations_coordinates, picking_stations_poses, nav=None):
        if nav is not None:
            self.C = self.create_C(task_shelves_poses, distance_f, nav)
            self.D = self.create_D(robots_poses, task_shelves_poses, distance_f, nav)
            self.W = self.create_W(task_shelves_poses, picking_stations_poses, distance_f, nav)
        else:
            self.C = self.create_C(task_shelves_coordinates, distance_f)
            self.D = self.create_D(robots_coordinates, task_shelves_coordinates, distance_f)
            self.W = self.create_W(task_shelves_coordinates, picking_stations_coordinates, distance_f)

    
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

    def create_W(self, tasks_poses, picking_stations_poses, distance_f, nav=None):
        #Wi --> W[i][0]  : cost of task i
        if nav is not None:
            return [self.cost_of_task(task_pose, picking_stations_poses, distance_f, nav) for task_pose in tasks_poses]
        else:
            return [self.cost_of_task(task_pose, picking_stations_poses, distance_f) for task_pose in tasks_poses]

    def cost_of_task(self, task_pose, picking_stations_poses, distance_f, nav=None):
        if nav is not None:
            costs = [2 * distance_f(station_pose, task_pose, nav) for station_pose in picking_stations_poses]
        else:
            costs = [2 * distance_f(station_pose, task_pose) for station_pose in picking_stations_poses]
        min_cost = min(costs)
        min_index = costs.index(min_cost)
        return (min_cost, min_index)           
    
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
        return sum([self.W[task - 1][0] for task in task_pool])
    def get_W(self):
        return self.W
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
            # calculate factor for max test and total test to balance BU, TT and TTC
            f = 130 - (7 * max_test) if (130 - (7 * max_test)) > 0 else 1
            fitness_score = (total_test / TTC)*f + (max_test / TT)*f + BU
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
        print("dict", dict)
        return dict

def generate_population(num_robots, num_tasks, num_chromosomes):
    # Calculate the length of the chromosome
    chromosome_length = num_robots + num_tasks - 1
    
    # Initialize an empty list to store the chromosomes
    population = []
    
    # Generate feasible chromosomes
    for _ in range(num_chromosomes):
        # Generate a permutation of integers from 1 to (n + m - 1)
        chromosome = list(range(1, num_tasks + num_robots))
        random.shuffle(chromosome)
        
        # Add the chromosome to the list
        population.append(chromosome)
    
    return population

def selection_roulette(population, weights, n):
    selected_individuals = random.choices(population, weights=weights, k=n)
    return selected_individuals

def selection_elitist(population, fitness_function, precentage):
    sorted_population =  sorted(population, key=fitness_function, reverse=True)
    elitist_population_count = int((precentage / 100) * len(population))
    return sorted_population[:elitist_population_count + 1]

def order_crossover(parents, crossover_rate):
    '''
    (9 3 4 8 10 2 1 6 5 7)
    (2 5 7 10 9 1 8 4 6 3)

    (9 3 4 | 8 10 2 1 | 6 5 7) 
    (2 5 7 | 10 9 1 8 | 4 6 3)
    
    (8 10 2 1 5 7 9 4 6 3)
    (10 9 1 8 3 4 2 6 5 7)
    '''
    if random.random() < crossover_rate:
        offsprings = []
        cross = round(len(parents[0]) / 3)
        offsprings.append(parents[1][cross:-cross])
        offsprings.append(parents[0][cross:-cross])

        for i in range(2):
            for element in parents[i]:
                if element not in offsprings[i]:
                    offsprings[i].append(element)
        return offsprings[0], offsprings[1]
    else:
        return parents[0], parents[1]
    
def swap_mutation(chromosome, mutation_rate):
  # Randomly select two distinct indices for swapping
  if len(chromosome) < 2:
    return chromosome  # Handle case with less than 2 genes
  

  if random.random() < mutation_rate:  
    index1 = random.randint(0, len(chromosome) - 1)
    index2 = random.randint(0, len(chromosome) - 1)

    # Perform the swap
    chromosome[index1], chromosome[index2] = chromosome[index2], chromosome[index1]

  return chromosome

# Initialize parameters: population size Popsize, maximal generations MaxEpoc, crossover rate Pc
def genetic_alg(pop_size, MaxEpoc, crossover_rate, num_robots, num_tasks, elitist_precentage,
                robots_coordinates, robots_poses, task_shelves_coordinates, task_shelves_poses, distance_strag, picking_stations_coordinates, picking_stations_poses, nav):

    #calculate the distances between different task orders
    print("Calculating C,D and W....")
    start = time.time()  
    costs = Costs(num_robots, num_tasks)
    costs.update_Costs(robots_coordinates, robots_poses, task_shelves_coordinates, task_shelves_poses, distance_strag,picking_stations_coordinates, picking_stations_poses, nav)
    end = time.time()
    print("C,D and W creation time : ", end - start," s")
    print("C:", costs.C)
    print("D:", costs.D)
    print("W:", costs.W)


    # Generate an population of feasible solutions randomly
    population = generate_population(num_robots, num_tasks, pop_size)

    fitness_function = costs.fitness

    # for 𝑖 = 1 to MaxEpoc do
    for i in range(MaxEpoc):

        next_generation = []
        weights=[fitness_function(chromosome) for chromosome in population]
        
        # Set the alterable mutation rate Pm according to the actual evolution generations
        if i < 1000:
            mutation_rate = intial_mutation_rate
        else:
            mutation_rate = final_mutation_rate

        # Evaluate individuals in the initial population / Evaluate the produced offspring;
        # Select individuals according to elitist strategy and roulette wheel method
        elitist_population = selection_elitist(population, fitness_function, elitist_precentage)
        next_generation = elitist_population[:]

        for _ in range(int( (pop_size - len(elitist_population)) / 2) ):
            parents = selection_roulette(population, weights, 2)
            # if random (0, 1) < Pc then Crossover individuals in pairs by a variation of the order crossover operator
            offspring_a, offspring_b = order_crossover(parents, crossover_rate)
            # if random (0, 1) < Pm then Mutate an individual by swap mutation
            offspring_a = swap_mutation(offspring_a, mutation_rate)
            offspring_b = swap_mutation(offspring_b, mutation_rate)

            next_generation += [offspring_a, offspring_b]

        population = next_generation
        if i % 100 == 0:
            print(f"chromosome {i} :\n{population[0]}")
            print(f"fitness of {i} chromosome: {costs.fitness(population[0])}")

    
    print(f"final chromosome: {population[0]}")
    print(f"fitness of last chromosome: {costs.fitness(population[0])}")
    shelf_list = costs.split_chromosome(population[0])
    print(f"shelf_list:\n{shelf_list}")
    W = costs.get_W()
    picking_list = [item[1] for item in W]
    print(f"picking_list:\n{picking_list}")

    robots_tasks= []
    for i in range(num_robots):
        robot_shelfs = []
        robot_tasks = []
        robot_shelfs = shelf_list[i]
        for shelf in robot_shelfs:
            robot_tasks.append((shelf , picking_list[shelf - 1]))
        robots_tasks.append(robot_tasks)

    print(f"robots_tasks (shelf, pick_station):\n{robots_tasks}")

    return robots_tasks