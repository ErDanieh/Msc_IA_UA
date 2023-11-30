
import numpy as np
import random
from numba import njit

from games.Connect6 import getValidMoves, getNextState, checkWin, getPreviousState


class Negamax():
    def __init__(self, scoreFunction, heuristic):
        self.transpositionTable = {}
        self.scoreFunction = scoreFunction
        self.heuristic = heuristic
        self.visitedNodes = 0
        self.terminalNodes = 0
        self.prunnedBranches = 0
        self.tableHits = 0
        
    
    def reset(self):
        self.transpositionTable = {}
        self.visitedNodes = 0
        self.terminalNodes = 0
        self.prunnedBranches = 0
        self.tableHits = 0
    
    
    def search(self, player, board, prevMove=None, depth=2, alpha=np.NINF, beta=np.inf):
        
        self.visitedNodes += 1
        
        # Check if the game is over
        isTerminal = checkWin(board, prevMove)
        
        if isTerminal:
            self.terminalNodes += 1
            return np.inf*(-1), None
        elif depth == 0:
            return self.scoreFunction(board, player), None
        
        # Prepare the variables before the searching the next level
        maxValue = np.NINF
        moveIndex = 0
        moves = getValidMoves(board)
        random.shuffle(moves)
        
        # Check if there are no moves
        if len(moves) == 0:
            return 0, None
        
        # Search the next level
        for i in range(len(moves)):
            move = moves[i]
            nextBoard = getNextState(board.copy(), player, move)
            value, _ = self.search(
                player=-player,
                board=nextBoard,
                prevMove=move,
                depth=depth-1,
                alpha=-beta,
                beta=-alpha
            )
            value = -value
            
            if value > maxValue:
                maxValue = value
                moveIndex = i
            
        return maxValue, moves[moveIndex]

@njit
def getKey(board):
    return hash(str(board))