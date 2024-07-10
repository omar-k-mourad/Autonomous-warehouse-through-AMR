import random
from nav2_commander import *
import time
import math
from costs import Costs
import copy
# Distance functions
def nav_distance(pose1, pose2, nav):
    """Calculate the distance between two points using nav2 planner."""
    return len((nav.getPath(start=pose1, goal=pose2, planner_id='GridBased', use_start=True)).poses)

def euclidean_distance(coord1, coord2):
    """Calculate the Euclidean distance between two points."""
    return (math.sqrt((coord2[0] - coord1[0])**2 + (coord1[1] - coord1[1])**2)) * 100

# Other functions
def make_pose_stamps(coordinates, nav):
    """Create pose stamps for the given coordinates."""
    return [create_pose_stamped(nav, x, y, 0.0) for x, y in coordinates]

def make_robots_dict(file_name):
    """
    Extract robot locations from a file.

    Args:
        file_name (str): Name of the file containing robot locations.

    Returns:
        dict: Dictionary of robot names and their coordinates.
    """
    robot_coordinate_dict = {}
    with open(file_name, 'r') as file:
        for line in file:
            robot_name, xy = line.split(":")
            x, y = map(float, xy.strip()[1:-1].split(","))
            robot_coordinate_dict[robot_name] = (x, y)
    print("dict", robot_coordinate_dict)
    return robot_coordinate_dict

def generate_population(num_robots, num_tasks, num_chromosomes):
    """
    Generate an initial population for the genetic algorithm.

    Args:
        num_robots (int): Number of robots.
        num_tasks (int): Number of tasks.
        num_chromosomes (int): Size of the population.

    Returns:
        list: A list of chromosomes representing the initial population.
    """
    return [random.sample(range(1, num_tasks + num_robots), num_tasks + num_robots - 1) for _ in range(num_chromosomes)]

def selection_roulette(population, weights, n):
    """
    Select n chromosomes from the population based on their weights using roulette wheel selection.

    Args:
        population (list): List of chromosomes.
        weights (list): Fitness values of the chromosomes.
        n (int): Number of chromosomes to select.

    Returns:
        list: Selected chromosomes.
    """
    return random.choices(population, weights=weights, k=n)

def selection_elitist(population, weights, percentage):
    """
    Select the top percentage of chromosomes based on their fitness.

    Args:
        population (list): List of chromosomes.
        weights (list): Fitness values of the chromosomes.
        percentage (float): Percentage of top chromosomes to select.

    Returns:
        list: Selected elitist chromosomes.
    """
    combined = sorted(zip(population, weights), key=lambda x: x[1], reverse=True)
    elitist_count = int((percentage / 100) * len(population))
    return [individual for individual, _ in combined[:elitist_count]]

def order_crossover(parents, crossover_rate):
    """
    Perform order crossover on two parent chromosomes to produce offspring.

    Args:
        parents (list): List containing two parent chromosomes.
        crossover_rate (float): Crossover rate.

    Returns:
        tuple: Two offspring chromosomes.
    
    crossover example :
        parent 1 : (9 3 4 8 10 2 1 6 5 7)
        parent 2 : (2 5 7 10 9 1 8 4 6 3)

        cross parent 1 : (9 3 4 | ->8 10 2 1<- | 6 5 7) 
        cross parent 2 : (2 5 7 | ->10 9 1 8<- | 4 6 3)

        offspring 1 : (8 10 2 1 5 7 9 4 6 3)
        offspring 2 : (10 9 1 8 3 4 2 6 5 7)
    """
    if random.random() < crossover_rate:
        cross = len(parents[0]) // 3
        offspring_a = parents[1][cross:-cross] + [gene for gene in parents[0] if gene not in parents[1][cross:-cross]]
        offspring_b = parents[0][cross:-cross] + [gene for gene in parents[1] if gene not in parents[0][cross:-cross]]
        return offspring_a, offspring_b
    return copy.deepcopy(parents[0]), copy.deepcopy(parents[1])

def swap_mutation(chromosome, mutation_rate):
    """
    Perform swap mutation on a chromosome.

    Args:
        chromosome (list): Chromosome to mutate.
        mutation_rate (float): Mutation rate.

    Returns:
        list: Mutated chromosome.
    
    mutation example :
        inptut chromosome : (9 3 4 8 10 2 1 6 5 7)
        output chromosome : (7 3 4 8 10 2 1 6 5 9)
    """
    if len(chromosome) < 2:
        return chromosome
    if random.random() < mutation_rate:
        index1, index2 = random.sample(range(len(chromosome)), 2)
        chromosome[index1], chromosome[index2] = chromosome[index2], chromosome[index1]
    return chromosome

def genetic_alg(pop_size, max_epochs, crossover_rate, num_robots, num_tasks, elitist_percentage, robots_coordinates,
                robots_poses, task_shelves_coordinates, task_shelves_poses, distance_strategy, picking_stations_coordinates,
                picking_stations_poses, nav):
    """
    Genetic algorithm for efficient task allocation.

    Args:
        pop_size (int): Size of the population.
        max_epochs (int): Number of iterations.
        crossover_rate (float): Crossover rate.
        num_robots (int): Number of robots.
        num_tasks (int): Number of tasks.
        elitist_percentage (float): Percentage of elitist population.
        robots_coordinates (list): Coordinates of robots.
        robots_poses (list): Poses of robots.
        task_shelves_coordinates (list): Coordinates of task shelves.
        task_shelves_poses (list): Poses of task shelves.
        distance_strategy (str): Strategy for distance calculation.
        picking_stations_coordinates (list): Coordinates of picking stations.
        picking_stations_poses (list): Poses of picking stations.
        nav: Navigation object.

    Returns:
        list: List of tasks for each robot.
    """
    # Set the intital and final mutation rate
    initial_mutation_rate = 0.1
    final_mutation_rate = 0.5

    #Calculate the distances between different task orders;
    print("Calculating C, D and W....")
    start = time.time()
    costs = Costs(num_robots, num_tasks)
    costs.update_Costs(robots_coordinates, robots_poses, task_shelves_coordinates, task_shelves_poses, distance_strategy,
                       picking_stations_coordinates, picking_stations_poses, nav)
    end = time.time()
    print("C, D and W creation time:", end - start, "s")

    # Generate an population of feasible solutions randomly
    population = generate_population(num_robots, num_tasks, pop_size)

    start = time.time()
    for epoch in range(max_epochs):
        weights = [costs.fitness(chromosome) for chromosome in population]
        
        # Set the alterable mutation rate Pm according to the actual evolution generations
        mutation_rate = initial_mutation_rate if epoch < 1000 else final_mutation_rate

        # Select individuals according to elitist strategy and roulette wheel method
        elitist_population = selection_elitist(population, weights, elitist_percentage)
        next_generation = elitist_population[:]
        while len(next_generation) < pop_size:
            parents = selection_roulette(population, weights, 2)
            # Crossover individuals in pairs by a variation of the order crossover operator depending on crossover rate
            offspring_a, offspring_b = order_crossover(parents, crossover_rate)
            # Mutate an individual by swap mutation according to mutation rate
            next_generation.append(swap_mutation(offspring_a, mutation_rate))
            next_generation.append(swap_mutation(offspring_b, mutation_rate))

        population = next_generation
        if epoch % 100 == 0:
            print(f"Chromosome {epoch}:\n{population[0]}")
            print(f"Fitness of {epoch} chromosome: {costs.fitness(population[0])}")

    end = time.time()
    print(f"Final chromosome: {population[0]}")
    print(f"Fitness of last chromosome: {costs.fitness(population[0])}")
    shelf_list = costs.split_chromosome(population[0])
    print(f"Shelf list:\n{shelf_list}")
    W = costs.get_W()
    picking_list = [item[1] for item in W]
    print(f"Picking list:\n{picking_list}")
    print("Algorithm time:", end - start, "s")

    robots_tasks = [
        [(shelf, picking_list[shelf - 1]) for shelf in shelf_list[i]]
        for i in range(num_robots)
    ]
    print(f"Robots tasks (shelf, pick_station):\n{robots_tasks}")

    return robots_tasks