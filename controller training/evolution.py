def initialize_generation(population_size):
    #TODO
    pass

class Genotype:
    def __init__(self):
        #TODO
        pass

class Evolution:
    def __init__(self, generation):
        self.generation = generation

    def evolve(self):
        parent_matches = self.select()
        offspring = [self.crossover(p1, p2) for p1, p2 in parent_matches]
        offspring = [self.mutate(c) for c in offspring]
        return offspring

    def select(self):
        #TODO
        pass

    def crossover(self):
        #TODO
        pass

    def mutate(self, individual):
        #TODO
        pass