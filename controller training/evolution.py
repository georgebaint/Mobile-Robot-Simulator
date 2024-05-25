import numpy as np
import random

class Evolution:
    def __init__(self, map, robot, population_size, ANN):
        self.population_size = population_size
        self.genotype_length = 14 * 7 * 2 # Input * hidden layer * output
        self.genotypes_list = self.initialize_genotypes()
        self.map = map
        self.robot = robot
        self.fitness_list = [] # dusting_simulation.fitness
        self.ANN = ANN

    def initialize_genotypes(self):
        # Initialize with random values for each genotype of defined length
        return [np.random.normal(loc=0.0, scale=0.01, size=self.genotype_length) for _ in range(self.population_size)]
    
    def selection(self):
        total_fitness = sum(self.fitness_list)  # Ensure it works for lists
        probabilities = [fitness / total_fitness for fitness in self.fitness_list]

        def create_roulette_wheel(genotypes, probabilities):
            cumulative_probabilities = np.cumsum(probabilities)  # Use numpy for cumulative sum
            r = random.random()
            idx = np.searchsorted(cumulative_probabilities, r)
            return genotypes[idx]

        # Corrected to repeatedly call the roulette function correctly
        selected_genotypes = [create_roulette_wheel(self.genotypes_list, probabilities) for _ in range(self.population_size)]
        return selected_genotypes

    def reproduction(self):
        new_generation = self.selection()
        if len(self.genotypes_list) == len(new_generation):
            self.genotypes_list = new_generation
        else:
            raise Exception('Mismatch in population sizes after selection')

    def crossover(self, pair_prob):
        # Simplified the crossover implementation
        for parent in range(1, self.population_size):
            if random.random() < pair_prob:
                # Ensure that crossover occurs by averaging parent genotypes
                self.genotypes_list[parent] = (self.genotypes_list[parent] + self.genotypes_list[parent-1]) / 2
                
    def mutation(self, mutation_rate=0.01, mutation_strength=0.1):
        for genotype in self.genotypes_list:
            for gene_idx in range(len(genotype)):
                if random.random() < mutation_rate:
                    genotype[gene_idx] += np.random.normal(scale=mutation_strength)

if __name__ == "__main__":
    # Set a random seed for reproducibility
    np.random.seed(42)
    
    # Define parameters for the Evolution class
    map = []
    robot = []
    ann = []
    # ann = ANN(input_size=10)  # Assuming each genotype has 10 parameters
    population_size = 50  # Number of individuals in the population

    # Create an instance of the Evolution class
    evolution = Evolution(map, robot, population_size, ann)

    # Simulate an evolutionary process
    for _ in range(10):  # Run for 10 generations
        evolution.reproduction()  # Reproduce the current population
        evolution.crossover(pair_prob=0.5)  # Apply crossover with a 50% chance per pair
        evolution.mutation()  # Apply mutations
        
        # Dummy fitness assignment (normally you would evaluate each individual)
        evolution.fitness_list = np.random.rand(population_size).tolist()

        print(f"Generation {_+1}: Best fitness {max(evolution.fitness_list)}")