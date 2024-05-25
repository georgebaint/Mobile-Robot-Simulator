import numpy as np
import random
from ann import Ann

class Evolution:
    def __init__(self, population_size):
        self.population_size = population_size
        self.genotype_length = 14 * 7 + 7 * 2 # Input * hidden layer + hidden layer * output
        self.genotypes_list = self.initialize_genotypes()
        # self.map = map
        # self.robot = robot
        self.fitness_list = []

    def initialize_genotypes(self):
        # Initialize with random values for each genotype of defined length
        return [np.random.normal(loc=0.0, scale=0.01, size=self.genotype_length) for _ in range(self.population_size)]
    
    def selection(self):
        total_fitness = sum(self.fitness_list)
        probabilities = [fitness / total_fitness for fitness in self.fitness_list]

        def create_roulette_wheel(genotypes, probabilities):
            # Calculate cumulative probabilities
            cumulative_probabilities = []
            current_sum = 0
            for p in probabilities:
                current_sum += p
                cumulative_probabilities.append(current_sum)

            r = random.random()
            # Find the first index where the cumulative probability is greater than the random number
            for i, cp in enumerate(cumulative_probabilities):
                if r < cp:
                    return genotypes[i]
            return genotypes[-1]
        
            # cumulative_probabilities = np.cumsum(probabilities)
            # r = random.random()
            # idx = np.searchsorted(cumulative_probabilities, r)
            # return genotypes[idx]

        selected_genotypes = [create_roulette_wheel(self.genotypes_list, probabilities) for _ in range(self.population_size)]
        return selected_genotypes

    def reproduction(self):
        new_generation = self.selection()
        if len(self.genotypes_list) == len(new_generation):
            self.genotypes_list = new_generation
        else:
            raise Exception('Mismatch in population sizes after selection')

    def crossover(self, pair_prob):
        for parent in range(1, self.population_size):
            if random.random() < pair_prob:
                # Crossover occurs by averaging parent genotypes
                self.genotypes_list[parent] = (self.genotypes_list[parent] + self.genotypes_list[parent-1]) / 2
                
    def mutation(self, mutation_rate, mutation_strength):
        for genotype in self.genotypes_list:
            for gene_idx in range(self.genotype_length):
                if random.random() < mutation_rate:
                    genotype[gene_idx] += np.random.normal(loc=0.0,scale=mutation_strength)

if __name__ == "__main__":
    
    # map = []
    # robot = []
    population_size = 50  # Number of individuals in the population

    evolution = Evolution(population_size)

    num_generations = 10 # number of epochs

    for _ in range(num_generations): 
        fitness_list = []
        for i in range(population_size):
            ann = Ann(evolution.genotypes_list[i])
            prev_speed = [0,0] # maybe np.array([0,0])

            for j in range(number_of_timesteps): # Basically our simulation 
                sensor_info = ...
                input = np.concatenate((sensor_info, prev_speed))
                output = ann.calculate_output(input)
                prev_speed = output

            fitness_list.append(calculate_fitness_function)
        
        evolution.fitness_list = fitness_list

        # Here we assess the results based on the elements in the evolution fitness_list

        # After we initialize the evolution of our population (this population will be simulated on the next epoch)

        evolution.reproduction()  # Reproduce the current population (basically replacement by selection)
        evolution.crossover(pair_prob=0.2)  # Apply crossover
        evolution.mutation(mutation_rate=0.01, mutation_strength=0.1)  # Apply mutations
        

