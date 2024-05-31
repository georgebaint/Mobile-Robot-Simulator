from evolution import Evolution
import pickle
from tqdm import tqdm
from ann import Ann
from dusting_simulation import DustingSimulation
from settings import MAZE_NUM, ITER_COUNT

maze_id = 1
with open('training_generations/Map_%d/best_genotypes.pkl' % (maze_id), "rb") as input_file:
    best_subjects = pickle.load(input_file)

i=0
for subject in best_subjects:
    i += 1
    if i in [1, 10, 20]:
        print('Generation %d' % (i))

        ann = Ann(subject)
        sim = DustingSimulation(maze_id=maze_id)
        # sim.draw_text(sim.screen, f"Generation : {i}", (1000,550))
        score = sim.run_ann(ITER_COUNT, ann)