from evolution import Evolution, initialize_generation
import pickle
from tqdm import tqdm

def dump_generation(g, n):
    with open('training_generations/generation_%d.pkl' % (n), 'wb') as f:
        pickle.dump(g, f)

def load_generation(n):
    with open('training_generations/generation_%d.pkl' % (n), 'rb') as f:
        g = pickle.load(f)
    return g

epochs = 20
population_size = 50

generation_0 = initialize_generation()
dump_generation(generation_0, 0)

current_generation = generation_0
for i in range(epochs):
    print('epoch %d' % (i+1))
    ev = Evolution(generation_0)
    new_generation = ev.evolve()
    dump_generation(new_generation, i + 1)

print('done')