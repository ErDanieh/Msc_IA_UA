
import numpy as np
import os

class Connect6:
    
    def __init__(self, rows=19, cols=19):
        self.rows = rows
        self.cols = cols
        self.actionSize = rows * cols
    
    
    def getInitialState(self):
        """
        Returns:
            np.ndarray: Devuelve el estado inicial del juego.
        """
        board = np.zeros((self.rows, self.cols), dtype=np.int8)
        return board
    
    def firstMove(self):
        center = 9
        offset = np.random.randint(-5, 5)  # Elige entre -1, 0, o 1
        x = center + offset
        y = center + np.random.randint(-5, 5)  # Elige entre -1, 0, o 1
        return x, y
        
    
    
    def getNextState(self, board, player, action, initial=False):
        """Devuelve el siguiente estado del juego tras realizar una acción.
        
        Args:
            board (np.ndarray): Estado actual del juego.
            player (int): Jugador que realiza la acción.
            action (np.ndarray): Acción a realizar.
        
        Returns:
            np.ndarray: Devuelve el siguiente estado del juego.
        """
        if initial:
            x, y = self.firstMove()
            board[x][y] = player
            return board
        
        cell1 = tuple(action[0])
        cell2 = tuple(action[1])

        row1, col1 = cell1
        row2, col2 = cell2

        board[row1][col1] = player
        board[row2][col2] = player
        return board
    
    
    def getValidMoves(self, board):
        """Devuelve las acciones válidas del juego.
        
        Args:
            board (np.ndarray): Estado actual del juego.
        
        Returns:
            list: Devuelve las acciones válidas del juego.
        """
        single_moves = np.transpose(np.nonzero(board == 0))
        size = single_moves.shape[0]

        indices = np.triu_indices(size, k=1)

        double_moves = np.zeros((2, 2, indices[0].size), dtype=np.int8)
        double_moves[0] = np.transpose(single_moves[indices[0]])
        double_moves[1] = np.transpose(single_moves[indices[1]])

        double_moves_list = []
        
        for i in range(double_moves.shape[2]):
            double_moves_list.append(double_moves[:,:,i])

        return double_moves_list
    
    
    def getValidMovesMask(self, board):
        validMoves = self.getValidMoves(board)
        mask = np.zeros((self.rows, self.cols), dtype=np.int8)
        
        for move in validMoves:
            x1, y1 = tuple(move[0])
            x2, y2 = tuple(move[1])
            mask[x1][y1] = 1
            mask[x2][y2] = 1
            
        return mask.flatten()
    
    
    def checkWin(self, board, action):
        """Comprueba si el jugador ha ganado.

        Args:
            board (np.ndarray): Estado actual del juego.
            action (np.ndarray): Acción a realizar.

        Returns:
            bool: Devuelve True si el jugador ha ganado, False en caso contrario.
        """

        if action is None:
            return False

        directions = [(1, 0), (0, 1), (1, 1), (1, -1)]
        for i in range(2):
            player = 1 if i == 0 else -1
            for move in action:
                x, y = tuple(move)
                for dx, dy in directions:
                    count = 1
                    # Verificar en una dirección
                    i, j = np.arange(1, 6), np.arange(1, 6)
                    i, j = x + i * dx, y + j * dy
                    mask = (i >= 0) & (i < board.shape[0]) & (j >= 0) & (j < board.shape[1])
                    count += np.sum(board[i[mask], j[mask]] == player)

                    # Verificar en la dirección opuesta
                    i, j = np.arange(1, 6), np.arange(1, 6)
                    i, j = x - i * dx, y - j * dy
                    mask = (i >= 0) & (i < board.shape[0]) & (j >= 0) & (j < board.shape[1])
                    count += np.sum(board[i[mask], j[mask]] == player)

                    if count >= 6:
                        return True

        return False


    def getValueAndTerminated(self, board, action):
        """Comprueba si el juego ha finalizado y quien ha ganado en dicho caso.

        Args:
            board (np.ndarray): Estado actual del juego.
            player (int): Jugador que realiza la acción.

        Returns:
            int, bool: Valor de la partida y si ha finalizado.
        """
        if self.checkWin(board, action):
            return 1, True
        if np.sum(self.getValidMoves(board)) == 0:
            return 0, True
        return 0, False

        
    def getEncodedState(self, board):
        """Codifica el estado del juego para su uso en la red neuronal.
        
        Args:
            board (np.ndarray): Estado actual del juego.
        
        Returns:
            np.ndarray: Devuelve el estado codificado.
        """
        encodedState = np.stack(
            (board == -1, board == 0, board == 1)
        ).astype(np.float32)
        
        return encodedState
    
    
    def getOpponent(self, player):
        """Devuelve el jugador contrario.
        
        Args:
            player (int): Jugador actual.
        
        Returns:
            int: Devuelve el jugador contrario.
        """
        return -player
    
    
    def getOpponentValue(self, value):
        """Devuelve el valor contrario.
        
        Args:
            value (int): Valor actual.
        
        Returns:
            int: Devuelve el valor contrario.
        """
        return -value
    
    
    def changePerspective(self, board, player):
        """Cambia la perspectiva del tablero.
        
        Args:
            board (np.ndarray): Estado actual del juego.
            player (int): Jugador actual.
        
        Returns:
            np.ndarray: Devuelve el tablero con la perspectiva cambiada.
        """
        return board * player
    
    
    def print(self, board):
        """Imprime el tablero por pantalla."""
        
        os.system('cls' if os.name == 'nt' else 'clear')
        print("  ", end="")
        for i in range(len(board)):
            print(chr(ord('A') + i), end=" ")
        print()
        for i in range(len(board)):
            print(chr(ord('A') + i), end=" ")
            for j in range(len(board)):
                if board[i][j] == 0:
                    print(".", end=" ")
                elif board[i][j] == 1:
                    print("X", end=" ")
                else:
                    print("O", end=" ")
            print()