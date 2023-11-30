
from lib.Connect6 import Connect6
from lib.engines.MCTSA0 import MCTSA0
from lib.models.ResNet import ResNet
import cProfile
import torch
import time

connect6 = Connect6()

# device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
device = torch.device("cpu")
print(f"Selected PyTorch device: {device}")

model = ResNet(connect6, 4, 64, device)
# model.load_state_dict(torch.load('lib/models/weights/model_2.pt'))

# optimizer = torch.optim.Adam(model.parameters(), lr=0.001, weight_decay=0.0001)

args = {
    'C': 2,
    'num_searches': 27,              # 60
    'num_iterations': 1,            # 3
    'num_selfPlay_iterations': 1,   # 500
    'num_epochs': 1,                # 4
    'batch_size': 64,               # 64
    'temperature': 1.25,            # 1.25
    'dirichlet_epsilon': 0.25,      # 0.25
    'dirichlet_alpha': 0.3          # 0.3
}


# create an instance of your MCTSA0 class
mcts = MCTSA0(connect6, args, model)

# define a state to search from
state = connect6.getInitialState()

# run the search method with cProfile
#cProfile.run('mcts.search(state)', filename='mcts.prof')
startTime = time.time()
mcts.search(state)
endTime = time.time()
print(f"Time: {endTime - startTime}")