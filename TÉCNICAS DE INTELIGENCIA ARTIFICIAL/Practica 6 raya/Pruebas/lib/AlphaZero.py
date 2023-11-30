
from lib.engines.MCTSA0 import MCTSA0
import numpy as np
import torch
import torch.nn.functional as F
import random
from tqdm import trange
import time
import csv


class AlphaZero:
    
    def __init__(self, model, optimizer, game, args):
        self.model = model
        self.optimizer = optimizer
        self.game = game
        self.args = args
        self.mcts = MCTSA0(game, args, model)
    
    
    def selfPlay(self, debug=False):
        memory = []
        player = 1
        state = self.game.getInitialState()
        action = None
        initial = True
        done = False
        
        # Turno Inicial
        #state = self.game.getNextState(state, player, action, initial=True)
        #player = self.game.getOpponent(player)
        
        # Generación del dataset de entrenamiento
        while not done:
            if debug:
                self.game.print(state)
            
            state, value, done, action, memory, _ = self.step(state, player, initial, memory)
            player = self.game.getOpponent(player)
            initial = False
            
            if done:
                return memory
                # returnMemory = []
                # for histNeutralState, histActionProbs, histPlayer in memory:
                #     histOutcome = value if histPlayer == player else self.game.getOpponentValue(value)
                #     returnMemory.append((
                #         self.game.getEncodedState(histNeutralState),
                #         histActionProbs,
                #         histOutcome
                #     ))
                # return returnMemory
            
    
    def step(self, board, player, initial, memory):
        neutralState = self.game.changePerspective(board, player)
        
        start = time.time()
        actionProbs = self.mcts.search(neutralState)
        end = time.time() - start
        print(f"Tiempo de búsqueda: {end}")
        
        memory.append((neutralState, actionProbs, player))
        
        temperatureActionProbs = np.power(actionProbs, 1 / self.args['temperature'])
        temperatureActionProbs /= np.sum(temperatureActionProbs)
        action = np.random.choice(range(self.game.actionSize), p=temperatureActionProbs, size=2)
        
        while action[0] == action[1]:
            action[1] = np.random.choice(range(self.game.actionSize), p=temperatureActionProbs, size=1)
        
        move = np.array([
            [action[0] // self.game.cols, action[0] % self.game.cols],
            [action[1] // self.game.cols, action[1] % self.game.cols]
        ])
        
        state = self.game.getNextState(board, player, move, initial)
        
        value, done = self.game.getValueAndTerminated(state, move)
    
        if done:
            if value == 0:
                for i in range(len(memory)):
                    memory[i] = (memory[i][0], memory[i][1], 0)
            else:
                for i in range(len(memory)):
                    memory[i] = (memory[i][0], memory[i][1], 1 if memory[i][2] == player else -1)
    
        return state, value, done, move, memory, {}
    
    
    def train(self, memory, it, epoch, filename):
        random.shuffle(memory)
        
        for batchIdx in range(0, len(memory), self.args['batch_size']):
            
            # Generación del batch de entrenamiento
            sample = memory[batchIdx:min(len(memory) - 1, batchIdx + self.args['batch_size'])]
            state, policyTargets, valueTargets = zip(*sample)
            state, policyTargets, valueTargets = np.array(state), np.array(policyTargets), np.array(valueTargets)
    
            # Conversión a tensores del batch de entrenamiento
            state = torch.tensor(
                state, dtype=torch.float32, device=self.model.device
            )
            policyTargets = torch.tensor(
                policyTargets, dtype=torch.float32, device=self.model.device
            )
            valueTargets = torch.tensor(
                valueTargets, dtype=torch.float32, device=self.model.device
            )
                
            # Predicción del batch de entrenamiento
            outPolicy, outValue = self.model(state)
            
            # Cálculo de la función de pérdida
            policyLoss = F.cross_entropy(outPolicy, policyTargets)
            valueTargets = valueTargets.unsqueeze(1)
            valueLoss = F.mse_loss(outValue, valueTargets)
            loss = policyLoss + valueLoss
            
            #Almacenar estadisticas en fichero
            with open(filename, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([it, epoch, batchIdx, policyLoss.item(), valueLoss.item(), loss.item()])
                
            # Actualización de los parámetros del modelo
            self.optimizer.zero_grad()
            loss.backward()
            self.optimizer.step()
            
    
    def learn(self):
        # Preparamos el archivo CSV donde vamos a guardar las métricas
        filename = "lib/models/metrics.csv"
        with open(filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            # Definimos los nombres de las columnas
            writer.writerow(['iteration', 'epoch', 'batchIdx', 'policy_loss', 'value_loss', 'total_loss'])
        
        for iteration in range(self.args['num_iterations']):
            memory = []
            
            # Generación del dataset de entrenamiento
            self.model.eval()
            
            for selfPlayIteration in trange(self.args['num_selfPlay_iterations']):
                memory += self.selfPlay()
            
            # Entrenamiento del modelo
            self.model.train()

            
            for epoch in trange(self.args['num_epochs']):
                self.train(memory, iteration, epoch, filename)
            
            torch.save(self.model.state_dict(), f"lib/models/weights/model_{iteration}.pt")
            torch.save(self.optimizer.state_dict(), f"lib/models/weights/optimizer_{iteration}.pt")