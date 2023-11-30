
import numpy as np
from numba import njit
import os
    

def getInitialState(rows=19, cols=19):
    """
    Returns:
        np.ndarray: Devuelve el estado inicial del juego.
    """
    board = np.zeros((rows, cols), dtype=np.int8)
    return board
  
    
def getCustomInitialState(player1_positions, player2_positions, rows=19, cols=19):
    """
    Crea un tablero inicial con fichas ya colocadas.

    Args:
        player1_positions (list of tuples): Lista de tuplas con las posiciones de las fichas del jugador 1.
        player2_positions (list of tuples): Lista de tuplas con las posiciones de las fichas del jugador -1.
        rows (int): Número de filas del tablero.
        cols (int): Número de columnas del tablero.

    Returns:
        np.ndarray: El estado inicial del juego con las fichas colocadas.
    """
    board = np.zeros((rows, cols), dtype=np.int8)
    
    for pos in player1_positions:
        board[pos] = 1
    
    for pos in player2_positions:
        board[pos] = -1

    return board


def getNextState(board, player, action, firstMove=False):
    """Devuelve el siguiente estado del juego tras realizar una acción.
    
    Args:
        board (np.ndarray): Estado actual del juego.
        player (int): Jugador que realiza la acción.
        action (np.ndarray): Acción a realizar.
    
    Returns:
        np.ndarray: Devuelve el siguiente estado del juego.
    """
    if firstMove:
        board[action[0][0]][action[0][1]] = player
        return board
    
    cell1 = tuple(action[0])
    cell2 = tuple(action[1])

    row1, col1 = cell1
    row2, col2 = cell2

    board[row1][col1] = player
    board[row2][col2] = player
    return board


def getPreviousState(board, player, action, firstMove=False):
    """Devuelve el estado anterior del juego tras realizar una acción.
    
    Args:
        board (np.ndarray): Estado actual del juego.
        player (int): Jugador que realiza la acción.
        action (np.ndarray): Acción a realizar.
    
    Returns:
        np.ndarray: Devuelve el estado anterior del juego.
    """
    if firstMove:
        board[action[0][0]][action[0][1]] = 0
        return board
    
    cell1 = tuple(action[0])
    cell2 = tuple(action[1])

    row1, col1 = cell1
    row2, col2 = cell2

    board[row1][col1] = 0
    board[row2][col2] = 0
    return board


def getValidMoves(board):
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

@njit
def checkWin(board, action):
        
        if action is None:
            return False
        
        directions = [(1, 0), (0, 1), (1, 1), (1, -1)]

        for direction in directions:
            for i in range(2):
                count = 0
                n = x = action[i,0]
                m = y = action[i,1]
                movStone = board[n,m]
                
                if (movStone == 0):
                    return False;
                    
                while x >= 0 and x < 19 and y >= 0 and y < 19 and board[x,y] == movStone:
                    x += direction[0]
                    y += direction[1]
                    count += 1
                x = n - direction[0]
                y = m - direction[1]
                while x >= 0 and x < 19 and y >= 0 and y < 19 and board[x,y] == movStone:
                    x -= direction[0]
                    y -= direction[1]
                    count += 1
                if count >= 6:
                    return True
        return False



def getValueAndTerminated(board, action):
    """Comprueba si el juego ha finalizado y quien ha ganado en dicho caso.

    Args:
        board (np.ndarray): Estado actual del juego.
        player (int): Jugador que realiza la acción.

    Returns:
        int, bool: Valor de la partida y si ha finalizado.
    """
    if checkWin(board, action):
        return 1, True
    if np.sum(getValidMoves(board)) == 0:
        return 0, True
    return 0, False
    
    
def getOpponent(player):
    """Devuelve el jugador contrario.
    
    Args:
        player (int): Jugador actual.
    
    Returns:
        int: Devuelve el jugador contrario.
    """
    return -player


def printBoard(board):
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
        
def changeTurn(board):
    """Cambia el turno del jugador."""
    return -board

def msg2move(msg):
    size = len(msg)
    numbers = [None] * size
    for i in range(size):
        numbers[i] = ord(msg[i]) - ord('A')
    if size == 2:
        return np.array([[numbers[0], numbers[1]], [numbers[0], numbers[1]]])    
    return np.array([[numbers[0], numbers[1]], [numbers[2], numbers[3]]])

def move2msg(move):
    size = move.shape[1] * 2
    msg = [''] * size
    for i in range(move.shape[0]):
        for j in range(move.shape[1]):
            msg[i*2+j] = chr(move[i][j] + ord('A'))
    return ''.join(msg)