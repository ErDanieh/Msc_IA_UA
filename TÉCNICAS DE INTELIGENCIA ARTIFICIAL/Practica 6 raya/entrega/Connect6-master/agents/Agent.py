import os
import datetime

class Agent:
    def __init__(self, name="Agent", chess=1, log_moves=False):
        self.name = name
        self.chess = chess
        self.log_moves = log_moves
        self.first_move_done = False

        # Abrir el archivo de log si log_moves es True
        if self.log_moves:
            # Asegúrate de que el directorio 'logs' exista
            os.makedirs('logs', exist_ok=True)
            time_str = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            self.log_file = open(f'logs/{self.name}_moves_{time_str}.log', 'a') 
        
    
    def get_move(self, board):
        raise NotImplementedError("This method should be implemented by subclasses")
    
    def log_move(self, move, score):
        if self.log_moves:
            print(f"{self.name} moves: {move}, score {score}", file=self.log_file)
            self.log_file.flush()  # Asegurar que se escriba en el archivo

    def __del__(self):
        if self.log_moves:
            self.log_file.close()


    def __str__(self):
        return f"{self.name} (Player {self.chess})"
