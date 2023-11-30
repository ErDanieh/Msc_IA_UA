import games.Connect6 as Connect6
from agents.Agent import Agent

class HumanAgent(Agent):
    def __init__(self, name="Human", chess=1, log_moves=False):
        super().__init__(name, chess, log_moves)

    def get_move(self, board):
        move_input = input("Enter your move (e.g., 'AABB' for two stones or 'AA' for a single stone on the first move): ")
        action = Connect6.msg2move(move_input)
        
        self.log_move(action, 0)
        return action