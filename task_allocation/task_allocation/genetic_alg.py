import random

def generate_chromosomes(num_robots, num_tasks, num_chromosomes):
    # Calculate the length of the chromosome
    chromosome_length = num_robots + num_tasks - 1
    
    # Initialize an empty list to store the chromosomes
    chromosomes = []
    
    # Generate feasible chromosomes
    for _ in range(num_chromosomes):
        # Generate a permutation of integers from 1 to (n + m - 1)
        chromosome = list(range(1, num_tasks + num_robots))
        random.shuffle(chromosome)
        
        # Add the chromosome to the list
        chromosomes.append(chromosome)
    
    return chromosomes

# Example usage
num_robots = 3
num_tasks = 8
num_chromosomes = 100  # Number of chromosomes to generate
count = 0
feasible_chromosomes = generate_chromosomes(num_robots, num_tasks, num_chromosomes)
for i, chromosome in enumerate(feasible_chromosomes, start=1):
    if chromosome[0] == 9 or chromosome[0] == 10 or chromosome[9] == 9 or chromosome[9] == 10:
        count += 1
    print(f"Chromosome {i}: {chromosome}")

print(count)