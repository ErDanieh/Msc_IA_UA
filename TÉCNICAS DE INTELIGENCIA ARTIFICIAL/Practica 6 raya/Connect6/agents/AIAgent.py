from agents.Agent import Agent
import numpy as np
import random
from engines.utils.basicUtils import *
import games.Connect6 as Connect6

class AIAgent(Agent):
    def __init__(self, name="Unknown", AI = None, chess=1, log_moves=False, blocking_strategy=True, depth=4):
        super().__init__(name, chess, log_moves)
        self.AI = AI
        self.blocking_strategy = blocking_strategy
        self.depth = depth
        
    
    def get_move(self, board, preMove=None):
        valid_moves = np.array(Connect6.getValidMoves(board))
        if self.blocking_strategy and maxConsecutive(board, -self.chess) > 3:
            # Busca los movimientos de bloqueos
            moves = get_blocking_moves_4_in_a_row(board, self.chess)
            
            
            if np.shape(moves) != () and np.shape(moves)[0] == 1:
                moves.append(valid_moves[0])
            
            if (moves is None) or is_1d_in_2d(valid_moves, moves[0]) or is_1d_in_2d(valid_moves, moves[1]):
                self.AI.reset()
                print("REALIZANDO BUSQUEDA")
                score, action = self.AI.search(player=self.chess, board=board, prevMove=preMove, depth=self.depth)
                self.log_move(action, score)
                return action  
            
            return np.array(moves)  
        
        
        if self.first_move_done:
            self.AI.reset()
            print("REALIZANDO BUSQUEDA")
            score, action = self.AI.search(player=self.chess, board=board, prevMove=preMove, depth=self.depth)
            self.log_move(action, score)
            return action 
        
        
        if self.chess == -1:
            # First move
            numero = random.randint(6, 12)
            action = np.array([[numero,numero],[9,9]]) # Modified this line
            self.first_move_done = True
            score = 0
        else :
            # First move
            action = np.array([[preMove[0][0]-1,preMove[0][1]-1],[preMove[0][0]+1,preMove[0][1]+1]])
            self.first_move_done = True
            if is_1d_in_2d(valid_moves, action) == False:
                print("REALIZANDO BUSQUEDA")
                score, action = self.AI.search(player=self.chess, board=board, prevMove=preMove, depth=self.depth)
                self.log_move(action, score)
                return action
            
            score = 0
            
        self.log_move(action, score)
        return action
