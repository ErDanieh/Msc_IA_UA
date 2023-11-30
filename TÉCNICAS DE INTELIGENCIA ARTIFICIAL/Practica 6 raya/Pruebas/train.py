
import matplotlib.pyplot as plt
from lib.Connect6 import Connect6
from lib.AlphaZero import AlphaZero
from lib.models.ResNet import ResNet
import torch

connect6 = Connect6()



device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"Selected PyTorch device: {device}")




model = ResNet(connect6, 64, 128, device)
# model.load_state_dict(torch.load('lib/models/weights/model_0.pt', map_location=device))

optimizer = torch.optim.Adam(model.parameters(), lr=0.001, weight_decay=0.0001)

args = {
    'C': 4, # Parametro de montecarlo para el calculo de la UCB para explorar mas o expandir mas
    'num_searches': 1,              #  9, 18, 27, 63,90
    'num_iterations': 1,            # 3
    'num_selfPlay_iterations': 10,   # 500
    'num_epochs': 4,                # 4
    'batch_size': 64,               # 64
    'temperature': 0.75,            # 1.25 Temperatura para la distribucion de probabilidad de las jugadas
    'dirichlet_epsilon': 0.25,      # 0.25 Ruido que le añadimos a la primera jugada para que explore diferentes movimientos
    'dirichlet_alpha': 0.3          # 0.3 Ruido que le añadimos a la primera jugada para que explore diferentes movimientos
}

alphaZero = AlphaZero(model, optimizer, connect6, args)
alphaZero.learn()
# alphaZero.selfPlay()