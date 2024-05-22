from evolution import Evolution, initialize_generation
import pickle
from tqdm import tqdm

def dump_generation(g, n):
    with open('generation_%d.pkl' % (n), 'wb') as f:
        pickle.dump(g, f)

def load_generation(n):
    with open('generation_%d.pkl' % (n), 'rb') as f:
        g = pickle.load(f)
    return g

epochs = 20
population_size = 50

generation_0 = initialize_generation()
dump_generation(generation_0)

current_generation = generation_0
for i in range(epochs):
    ev = Evolution(generation_0)
    ev.evolve()