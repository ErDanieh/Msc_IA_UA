import gym
from gym import spaces
import numpy as np
import lib.utils.boardUtils as boardUtils
import lib.engines.MCTSA0 as MCTSA0
import torch
from lib.models import ResNet as ResNetModule


class Connect6Env(gym.Env):
    

    def __init__(self):
        super(Connect6Env, self).__init__()
        self.board_size = 21
        self.rows = self.board_size  # Añade este atributo
        self.cols = self.board_size  # Añade este atributo
        self.actionSize = self.board_size * self.board_size  # Añade este atributo
        self.current_player = 1

        num_resBlocks = 5
        num_hidden = 128
        self.board = self.init_board()
        self.model = ResNetModule.ResNet(self, num_resBlocks, num_hidden)  # Pasa 'self' en lugar de 'self.board'
        #self.model.load_state_dict(torch.load('lib/models/weights/model_0.pt'), strict=False)
        self.model.eval()

        
        self.search_engine = MCTSA0.MCTSA0(game=self, args={'num_searches': 2}, model=self.model)
        self.previous_move = np.array([[0,0], [0,0]])
        
        self.board = self.init_board()
        
        self.action_space = spaces.Discrete(self.board_size * self.board_size)
        self.observation_space = spaces.Box(low=0, high=2, shape=(self.board_size, self.board_size), dtype=int)

    def init_board(self):
        board = [ [0]*self.board_size for i in range(self.board_size)]
        board = np.array(board)
        
        for i in range(21):
            board[i][0] = board[0][i] = board[i][self.board_size - 1] = board[self.board_size - 1][i] = 3
        for i in range(1, self.board_size - 1):
            for j in range(1, self.board_size - 1):
                board[i][j] = 0
                
        return board

    def step(self):
        memory = []
        player = 1
        state = self.init_board()
        done = False
        reward = None

        while not done:
            neutral_state = self.change_perspective(state, player)
            action_probs = self.search_engine.search(self)
            
            # Asumo que quieres elegir una acción basada en las probabilidades de la MCTS
            action = np.random.choice(range(self.board_size * self.board_size), p=action_probs)
            
            memory.append((neutral_state, action_probs, player))
            
            state = boardUtils.make_move(state, action, player)
            
            # Usando la función is_win_by_premove para determinar si el juego ha terminado
            done = self.is_win_by_premove(state, action)
            
            if done:
                # Asumo que quieres que la recompensa sea 1 para el jugador ganador y -1 para el otro jugador.
                reward = 1 if player == 1 else -1
            else:
                player = self.changeTurn(player)

        return state, memory, reward, done, {}

    def firstMove(self):
        center = self.board_size // 2
        offset = np.random.randint(-1, 2)  # Elige entre -1, 0, o 1
        x = center + offset
        y = center + np.random.randint(-1, 2)  # Elige entre -1, 0, o 1
        self.board[x][y] = self.current_player
        self.previous_move = np.array([[x, y]])
        self.changeTurn()

    def change_perspective(self, state, player):
        # Esta función cambiará la perspectiva del tablero. Si el jugador es -1, invertiremos el tablero.
        return state * player

    def changeTurn(self):
        self.current_player = -1 if self.current_player == 1 else 1


    def makeMove(self):
        self.board = boardUtils.make_move(self.board, self.previous_move, self.current_player)

    def render(self):
        print("  " + " ".join(f"{i:2}" for i in range(self.board_size)))
        for i, row in enumerate(self.board):
            print(f"{i:2} " + " ".join(self.piece_str(value) for value in row))
            
    def piece_str(self, value):
        return "O" if value == 1 else "X" if value == 2 else "."

    def is_win_by_premove(self) -> bool:
        directions = [(1, 0), (0, 1), (1, 1), (1, -1)]
        color = self.current_player
        
        for move in self.previous_move:
            x, y = tuple(move)
            for dx, dy in directions:
                count = 1
                
                # Verificar en una dirección
                i, j = x + dx, y + dy
                while 0 <= i < self.board.shape[0] and 0 <= j < self.board.shape[1] and self.board[i][j] == color:
                    count += 1
                    i += dx
                    j += dy

                # Verificar en la dirección opuesta
                i, j = x - dx, y - dy
                while 0 <= i < self.board.shape[0] and 0 <= j < self.board.shape[1] and self.board[i][j] == color:
                    count += 1
                    i -= dx
                    j -= dy

                if count >= 6:
                    return True

        return False
        
    def reset(self):
        self.board = self.init_board()
        self.current_player = 1
        self.previous_move = np.array([[0, 0], [0, 0]])
        return self.board

        



# Usage
env = Connect6Env()
obs = env.reset()

env.render()
done = False



while not done:
    #action = env.action_space.sample()  # replace this with your agent's action
    obs, actions, reward, done, info = env.step()
    env.render()
    if done:
        print(f"Player {3-env.current_player} wins!" if reward == 1 else "Invalid move!")
        
env.close()