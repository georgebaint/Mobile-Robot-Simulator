from evolution import Evolution
import pickle
from tqdm import tqdm
from ann import Ann
from dusting_simulation import DustingSimulation

def dump_generation(g, n):
    with open('training_generations/generation_%d.pkl' % (n), 'wb') as f:
        pickle.dump(g, f)

def load_generation(n):
    with open('training_generations/generation_%d.pkl' % (n), 'rb') as f:
        g = pickle.load(f)
    return g

epochs = 2
population_size = 3
evolution = Evolution(population_size)

# generation_0 = initialize_generation()
# dump_generation(generation_0, 0)

# current_generation = generation_0
for i in range(epochs):
    print('epoch %d' % (i+1))
    
    fitness_list = []
    
    for j in range(population_size):
        ann = Ann(evolution.genotypes_list[j])
        sim = DustingSimulation(maze_id=2)
        score = sim.run_ann(1600, ann)
        fitness_list.append(score)

    
    evolution.fitness_list = fitness_list
    evolution.reproduction()  # Reproduce the current population (basically replacement by selection)
    evolution.crossover(pair_prob=0.2)  # Apply crossover
    evolution.mutation(mutation_rate=0.01, mutation_strength=0.1)  # Apply mutations
    
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