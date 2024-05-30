from evolution import Evolution
import pickle
from tqdm import tqdm
from ann import Ann
from dusting_simulation import DustingSimulation
from settings import MAZE_NUM, ITER_COUNT

generations = 20
population_size = 60
evolution = Evolution(population_size, pair_prob=0.5, mutation_rate=0.05, mutation_strength=0.1)

best_genotypes = []
fitness_history = []
genotype_history = []  # List to store the history of genotypes

for gen in range(generations):
    print('Generation %d' % (gen+1))

    if gen!=0:
        evolution.fitness_list = current_generation_fitness_list
        evolution.evolve()
    
    current_generation_fitness_list = []
    current_generation_genotypes = []
    best_fitness = 0
    best_subject = 0

    for subject in range(population_size):
        print('Individual %d / %d' % (subject+1, population_size))

        ann = Ann(evolution.genotypes_list[subject])
        sim = DustingSimulation(maze_id=MAZE_NUM)
        score = sim.run_ann(ITER_COUNT, ann)

        current_generation_fitness_list.append(score)
        current_generation_genotypes.append(evolution.genotypes_list[subject])

        if score > best_fitness:
            best_fitness = score
            best_subject = subject
    
    best_genotypes.append(evolution.genotypes_list[best_subject])
    fitness_history.append(current_generation_fitness_list)
    genotype_history.append(current_generation_genotypes)


with open('training_generations/Map_%d/best_genotypes.pkl' % (MAZE_NUM), 'wb') as f:
    pickle.dump(best_genotypes, f)

with open('training_generations/Map_%d/fitness_history.pkl' % (MAZE_NUM), 'wb') as f:
    pickle.dump(fitness_history, f)

with open('training_generations/Map_%d/genotype_history.pkl' % (MAZE_NUM), 'wb') as f:
    pickle.dump(genotype_history, f)

# def dump_generation(g, n):
#     with open('training_generations/generation_%d.pkl' % (n), 'wb') as f:
#         pickle.dump(g, f)

# def load_generation(n):
#     with open('training_generations/generation_%d.pkl' % (n), 'rb') as f:
#         g = pickle.load(f)
#     return g

# generation_0 = initialize_generation()
# dump_generation(generation_0, 0)

# current_generation = generation_0
    
    # evolution.fitness_list = fitness_list
    # evolution.evolve(pair_prob=0.2, mutation_rate=0.01, mutation_strength=0.1)
    # evolution.reproduction()  # Reproduce the current population (basically replacement by selection)
    # evolution.crossover(pair_prob=0.2)  # Apply crossover
    # evolution.mutation(mutation_rate=0.01, mutation_strength=0.1)  # Apply mutations
    
    # new_generation = ev.evolve()
    # dump_generation(new_generation, i + 1)

#     fitness_lst = []
#     for i in range(population_size):
#         ann = Ann(evolution.genotypes_list[i])
        

#             for j in range(number_of_timesteps): # Basically our simulation 
#                 sensor_info = ...
#                 input = np.concatenate((sensor_info, prev_speed))
#                 output = ann.calculate_output(input)
#                 prev_speed = output

#             fitness_list.append(calculate_fitness_function)
        
#         evolution.fitness_list = fitness_list

# print('done')



#     evolution = Evolution(population_size)


#         # Here we assess the results based on the elements in the evolution fitness_list

#         # After we initialize the evolution of our population (this population will be simulated on the next epoch)

#         evolution.reproduction()  # Reproduce the current population (basically replacement by selection)
#         evolution.crossover(pair_prob=0.2)  # Apply crossover
#         evolution.mutation(mutation_rate=0.01, mutation_strength=0.1)  # Apply mutations