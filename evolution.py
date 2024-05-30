import numpy as np
import random
from ann import Ann

class Evolution:
    def __init__(self, population_size, pair_prob, mutation_rate, mutation_strength):
        self.population_size = population_size
        self.genotype_length = 14 * 7 + 7 * 2 # Input * hidden layer + hidden layer * output
        self.genotypes_list = self.initialize_genotypes()
        self.fitness_list = []
        self.pair_prob = pair_prob
        self.mutation_rate = mutation_rate
        self.mutation_strength = mutation_strength

    def initialize_genotypes(self):
        # Initialize with random values for each genotype of defined length

        return [np.random.normal(loc=0.0, scale=0.5, size=self.genotype_length) for _ in range(self.population_size)]
    
    def selection(self):
        total_fitness = sum(self.fitness_list)
        probabilities = [fitness / total_fitness for fitness in self.fitness_list]

        def create_roulette_wheel(genotypes, probabilities):
            # # Calculate cumulative probabilities
            cumulative_probabilities = np.cumsum(probabilities)
            r = random.random()
            idx = np.searchsorted(cumulative_probabilities, r)
            return genotypes[idx]

        selected_genotypes = [create_roulette_wheel(self.genotypes_list, probabilities) for _ in range(self.population_size)]
        return selected_genotypes

    def reproduction(self):
        new_generation = self.selection()
        if len(self.genotypes_list) == len(new_generation):
            self.genotypes_list = new_generation
        else:
            raise Exception('Mismatch in population sizes after selection')

    def crossover(self):
        new_population = self.genotypes_list.copy()  # Create a copy to avoid modifying the list in-place
        for parent in range(self.population_size):
            if random.random() < self.pair_prob:
                # Selecting other_parent different from parent
                other_parent = random.randint(0, self.population_size - 1)
                while other_parent == parent:
                    other_parent = random.randint(0, self.population_size - 1)
                # Crossover occurs by averaging parent genotypes
                new_population[parent] = (self.genotypes_list[parent] + self.genotypes_list[other_parent]) / 2

        self.genotypes_list = new_population
                
    def mutation(self,):
        for genotype in self.genotypes_list:
            for gene_idx in range(self.genotype_length):
                if random.random() < self.mutation_rate:
                    genotype[gene_idx] += np.random.normal(loc=0.0,scale=self.mutation_strength)

    def evolve(self,):
        # Selection is called in reproduction
        self.reproduction()  # Reproduce the current population (basically replacement by selection)
        self.crossover()  # Apply crossover
        self.mutation()  # Apply mutations
