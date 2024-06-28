import random
from nav2_commander import *
import time
import math
from costs import Costs

#Distance functions
def nav_distance(pose1, pose2, nav):
    """Calculate the distance between two points using nav2 planner."""
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
    """
    This function uses extract robot locations from file

    Args:
        file_name: name of file.

    Returns:
        robot_coordinte_dict : dictionary of robot name and it's coordinate

    sample output : {'robot1': (1.8692384106109388, -0.17030886256819153), 'robot2': (0.3829515348145183, 2.0063998491107764)}
    """
    robot_coordinte_dict = {}
    # Open the file in read mode
    with open(file_name, 'r') as file:
        # Iterate over each line in the file
        for line in file:
            robot_name, xy = line.split(":")
            x,y = xy.strip()[1:-1].split(",")
            x = float(x.strip())
            y = float(y.strip())
            robot_coordinte_dict[robot_name] = (x , y) 
        print("dict", dict)
        return robot_coordinte_dict

def generate_population(num_robots, num_tasks, num_chromosomes):
    """
    This function randomly generates the initital population for the genetic alogrithm where each chromosome
    depends on the number of robots and num_tasks

    Args:
        num_robots: the number of robots that are available.
        num_tasks : number of tasks / shelfs to be retrived.
        num_chromosomes : the size of the population

    Returns:
        population : is a list of chromosomes of size num_chromosmes
    """
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
    """
    This function selects n chromosmes from the population based on their weight
    where chromosomes with higher weight have more chance to be selected.

    Args:
        population: list of chromosmes. 
        weights : the fitness of each chromosome.
        n : the number of chromosome to select.

    Returns:
        selected_individuals : list of n selected chromosomes.
    """
    selected_individuals = random.choices(population, weights=weights, k=n)
    return selected_individuals

def selection_elitist(population, fitness_function, precentage):
    """
    This function returns etilist precentage of population.

    Args:
        population: list of chromosmes. 
        fitness_function : refrenced function to calculate the fitness.
        precentage : the precentage of etilist to be returned.

    Returns:
        selected_individuals : list of etilist precentage of population
    """
    sorted_population =  sorted(population, key=fitness_function, reverse=True)
    elitist_population_count = int((precentage / 100) * len(population))
    return sorted_population[:elitist_population_count + 1]

def order_crossover(parents, crossover_rate):
    """
    This function preforms order crossover, where two offsprings are produced from two parents,
    based on a certain rate of crossover.

    Args:
        parents: list containing two parents. 
        crossover_rate : rate at which crossover should occur.

    Returns:
        two offsprings
    
    crossover example :
        parent 1 : (9 3 4 8 10 2 1 6 5 7)
        parent 2 : (2 5 7 10 9 1 8 4 6 3)

        cross parent 1 : (9 3 4 | ->8 10 2 1<- | 6 5 7) 
        cross parent 2 : (2 5 7 | ->10 9 1 8<- | 4 6 3)
    
        offspring 1 : (8 10 2 1 5 7 9 4 6 3)
        offspring 2 : (10 9 1 8 3 4 2 6 5 7)
    """
    # preform crossover by the chosen rate
    if random.random() < crossover_rate:
        offsprings = []
        #select cross point
        cross = round(len(parents[0]) / 3)
        # preform crossover
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
    """
    This function Randomly select two distinct indices and swap them, according to mutation rate.

    Args:
        mutation_rate : rate at which mutation_rate should occur.

    Returns:
        chromosome : chromosome after swap
    
    mutation example :
        inptut chromosome : (9 3 4 8 10 2 1 6 5 7)
        output chromosome : (7 3 4 8 10 2 1 6 5 9)
    """
    # Handle case with less than 2 genes
    if len(chromosome) < 2:
        return chromosome  
    # preform mutation by the chosen rate
    if random.random() < mutation_rate:  
        index1 = random.randint(0, len(chromosome) - 1)
        index2 = random.randint(0, len(chromosome) - 1)

        # Perform the swap
        chromosome[index1], chromosome[index2] = chromosome[index2], chromosome[index1]

    return chromosome

# Initialize parameters: population size Popsize, maximal generations MaxEpoc, crossover rate Pc
def genetic_alg(pop_size, MaxEpoc, crossover_rate, num_robots, num_tasks, elitist_precentage,
                robots_coordinates, robots_poses, task_shelves_coordinates, task_shelves_poses, distance_strag, picking_stations_coordinates, picking_stations_poses, nav):
    """
    Genetic algorithm for efficient task allocation

    Args:
        pop_size: size of the population.
        MaxEpoc : the number of iteration the algorithm will run for to find a solution

    Returns:
        robots_tasks : list of list where each list represnt a set of tasks (shelf -> picking station)
        and the index of the list represent the robot number which will execute this tasks
    """
    #initialization
    intial_mutation_rate = 0.1
    final_mutation_rate = 0.5
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

    # match task number with shelf location and picking station location for each robot's tasks
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