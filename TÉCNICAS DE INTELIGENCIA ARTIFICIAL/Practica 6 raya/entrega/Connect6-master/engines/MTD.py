
import numpy as np
from engines.NegamaxUltimate import Negamax


class MTD():
    def __init__(self, scoreFunction, heuristic, firstGuess=0):
        self.scoreFunction = scoreFunction
        self.heuristic = heuristic
        self.negamax = Negamax(scoreFunction, heuristic)
        self.firstGuess = firstGuess
        self.visitedNodes = 0
        self.terminalNodes = 0
        self.prunnedBranches = 0
        self.tableHits = 0


    def reset(self):
        self.firstGuess = 0
        self.visitedNodes = 0
        self.terminalNodes = 0
        self.prunnedBranches = 0
        self.tableHits = 0
        self.negamax.reset()

    
    def updateStadistics(self):
        self.visitedNodes = self.negamax.visitedNodes
        self.terminalNodes = self.negamax.terminalNodes
        self.prunnedBranches = self.negamax.prunnedBranches
        self.tableHits = self.negamax.tableHits


    def search(self, player, board, prevMove=None, depth=4, upper=np.inf, lower=np.NINF):
        g = self.firstGuess
        
        upperBound = np.inf
        lowerBound = -np.inf
        
        while lowerBound < upperBound:
            beta = max(g, lowerBound+1)
            g, action = self.negamax.search(
                player=player,
                board=board,
                prevMove=prevMove,
                depth=depth,
                alpha=beta-1,
                beta=beta
            )
            if g < beta:
                upperBound = g
            else:
                lowerBound = g
                
        self.updateStadistics()
        return g, action