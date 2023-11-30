
import numpy as np
import torch
import math
from numba import jit, njit, prange

import matplotlib.pyplot as plt


def plot2DboardDistribution(distribution: np.ndarray):
    
    distribution = distribution.reshape((19, 19))
    
    # Plot the histogram as a 2D heatmap
    plt.imshow(distribution.T, origin='lower', cmap='cool')
    
    # Add grid on integer values
    plt.xticks(np.arange(0.5, 19, 1), np.arange(0, 19, 1))
    plt.yticks(np.arange(0.5, 19, 1), np.arange(0, 19, 1))
    plt.grid(color='black', linestyle='-', linewidth=0.2)
    
    plt.colorbar()
    plt.savefig('plot.png')
    plt.close()

#import time


class MCTSA0:
    
    def __init__(self, game, args, model):
        self.game = game
        self.args = args
        self.model = model


    @torch.no_grad()
    def search(self, state):
        
        # Creación y expansión del nodo raíz
        root = Node(self.game, self.args, state, visits=1)
        policy = self.rootExpandPolicy(root)
        root.expand(policy)
        
        for search in range(self.args['num_searches']):
            node = root

            while node.isFullyExpanded():
                node = node.select()
            
            value, isTerminal = self.game.getValueAndTerminated(node.state, node.actionTaken)
            value = self.game.getOpponentValue(value)
            
            if not isTerminal:
                policy, value = self.model(
                    torch.tensor(
                        self.game.getEncodedState(node.state),
                        device=self.model.device
                    ).unsqueeze(0)
                )
                policy = torch.softmax(policy, axis=1).squeeze(0).cpu().numpy()
                validMoves = self.game.getValidMovesMask(node.state)
                policy *= validMoves
                policy /= np.sum(policy)
                
                value = value.item()
                
                node.expand(policy)
                
            node.backpropagate(value)
            
        actionProbs = np.zeros(self.game.actionSize, dtype=np.float32)
        
        for child in root.children:
            
            if child.visits > 0:
                for i in range(2):
                    x, y = child.actionTaken[i]
                    actionProbs[x * self.game.cols + y] += child.visits

                
        
        sumActionProbs = np.sum(actionProbs)
        
        if sumActionProbs != 0:
            actionProbs /= sumActionProbs
        plot2DboardDistribution(actionProbs)
        return actionProbs
    
    
    def rootExpandPolicy(self, root):
        policy, _ = self.model(
            torch.tensor(
                self.game.getEncodedState(root.state),
                device=self.model.device
            ).unsqueeze(0)
        )
        policy = torch.softmax(policy, axis=1).squeeze(0).cpu().numpy()
        policy = (1 - self.args['dirichlet_epsilon']) * policy \
         + self.args['dirichlet_epsilon'] * np.random.dirichlet([self.args['dirichlet_alpha']] * self.game.actionSize)
        validMoves = self.game.getValidMovesMask(root.state)
        policy *= validMoves
        policy /= np.sum(policy)
        return policy
       
@njit
def ucb_formula(visits, child_visits, child_valueSum, prior, C):
    q_value = 0 if child_visits == 0 else 1 - ((child_valueSum / child_visits) + 1) * 0.5
    return q_value + C * math.sqrt(visits) * prior / (child_visits + 1)
    
class Node:
    def __init__(self, game, args, state, parent=None, action=None, prior=0, visits=0):
        self.game = game
        self.args = args
        self.state = state
        self.parent = parent
        self.actionTaken = action
        self.prior = prior
        
        self.children = []
        
        self.visits = visits
        self.valueSum = 0
        

    def isFullyExpanded(self):
        return len(self.children) > 0
    
    
    def select(self):
        bestChild = max(self.children, key=self.getUCB, default=None)
        return bestChild

    
    def getUCB(self, child):
        if child.visits == 0:
            q_value = 0
        else:
            q_value = 1 - ((child.valueSum / child.visits) + 1) / 2
        return q_value + self.args['C'] * (math.sqrt(self.visits) / (child.visits + 1)) * child.prior
            
    def expand(self, policy):
        """Expande el nodo actual creando los hijos correspondientes a las acciones posibles.

        Args:
            policy (np.ndarray): Distribución de probabilidad de las casillas.
        """
        nonZeroIndices = np.nonzero(policy)[0]
        for k in nonZeroIndices:
            kAction = np.array([k // self.game.cols, k % self.game.cols])
            
            for i in nonZeroIndices[k+1:]:
                iAction = np.array([i // self.game.cols, i % self.game.cols])
                
                action = np.array([kAction,iAction])
                
                childState = self.state.copy()
                childState = self.game.getNextState(childState, player=1, action=action)
                childState = self.game.changePerspective(childState, player=-1)

                child = Node(self.game, self.args, childState, self, action=action, prior=(policy[k] + policy[i])/ 2)
                self.children.append(child)                
    
    
    def backpropagate(self, value):
        self.valueSum += value
        self.visits += 1
        
        value = self.game.getOpponentValue(value)
        if self.parent is not None:
            self.parent.backpropagate(value)  