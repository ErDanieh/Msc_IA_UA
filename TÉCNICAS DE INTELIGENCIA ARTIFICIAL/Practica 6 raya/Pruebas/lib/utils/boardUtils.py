# TODO: aqui se colocaran todas las funciones que se usaran para el tablero

import numpy as np


def make_move(board, move, chess_type):
    """Actualiza el tablero (board) con el movimiento realizado doble (move) del tipo de ficha indicado (chess_type)

    Args:
        board (_type_): array 2D que representa el tablero
        move (_type_): movimiento doble [[x1,y1],[x2,y2]]
        chess_type (_type_): tipo de ficha que se esta moviendo

    Returns:
        _type_: board actualizado
    """
    cell1 = tuple(move[0])
    cell2 = tuple(move[1])

    row1, col1 = cell1
    row2, col2 = cell2

    board[row1][col1] = chess_type
    board[row2][col2] = chess_type
    return board


def unmake_move(board, move):
    """Actualiza el tablero (board) deshaciendo un movimiento doble (move)

    Args:
        board (_type_): array 2D que representa el tablero
        move (_type_): movimiento doble [[x1,y1],[x2,y2]]

    Returns:
        _type_: board actualizado
    """
    cell1 = tuple(move[0])
    cell2 = tuple(move[1])

    row1, col1 = cell1
    row2, col2 = cell2

    board[row1][col1] = 0
    board[row2][col2] = 0
    return board


def find_possible_moves(board: np.ndarray):
    """Devuelve todos los pares de movimientos posibles con la forma (2,2,k), siendo k el número de pares

    Args:
        board (_type_): array 2D que representa el tablero

    Returns:
        _type_: todos los pares de movimientos posibles en el tablero
    """
    single_moves = np.transpose(np.nonzero(board == 0))
    size = single_moves.shape[0]

    # Generate indices of the upper triangle of the matrix
    indices = np.triu_indices(size, k=1)

    double_moves = np.zeros((2, 2, indices[0].size), dtype=int)
    double_moves[0] = np.transpose(single_moves[indices[0]])
    double_moves[1] = np.transpose(single_moves[indices[1]])

    return double_moves


def check_first_move(board):
    """Comprueba si todavía no se ha realizado un movimiento en el tablero (board)

    Args:
        board (_type_): array 2D que representa el tablero array 2D que representa el tablero

    Returns:
        _type_: Devuelve verdadero si no se ha realizado ningún movimiento, falso en caso contrario
    """
    return board.size == board[board == 0].size


def get_fist_move(board): # TODO comprobar e implementar versión más sofisticada
    """Decide el primer movimiento a realizar en una partida

    Args:
        board (_type_): array 2D que representa el tablero array 2D que representa el tablero

    Returns:
        _type_: Devuelve un array de la forma [x,y] que representa la posición donde se debe colocar la ficha
    """
    return find_possible_moves(board)[0,:,0]


def is_win_by_premove(board: np.ndarray, pre_move: np.ndarray) -> bool:
    """Comprueba si el anterior movimiento ha sido un movimiento ganador

    Args:
        board (_type_): array 2D que representa el tablero array 2D que representa el tablero
        premove (_type_): movimiento doble [[x1,y1],[x2,y2]]
        
    Returns:
        _type_: Devuelve verdadero si el movimiento (premove) finaliza la partida y falso en caso contrario
    """
    
    if pre_move is None or pre_move[0] is None:
        return False
    
    directions = [(1, 0), (0, 1), (1, 1), (1, -1)]
    color = board[tuple(pre_move[0].T)]
    
    
    for direction in directions:
        indices = np.flatnonzero(board == color)
        for index in indices:
            position = np.unravel_index(index, board.shape)
            x, y = position
            count = 1
            while True:
                x += direction[0]
                y += direction[1]
                if not (0 <= x < board.shape[0] and 0 <= y < board.shape[1]):
                    break
                if board[x, y] != color:
                    break
                count += 1
            x, y = position
            while True:
                x -= direction[0]
                y -= direction[1]
                if not (0 <= x < board.shape[0] and 0 <= y < board.shape[1]):
                    break
                if board[x, y] != color:
                    break
                count += 1
            if count >= 6:
                return True
    return False


def is_game_over(board):
    # Check rows
    for i in range(19):
        for j in range(14):
            if all(board[i, j+k] == board[i, j] for k in range(6) if board[i, j+k] != 0):
                return True

    # Check columns
    for i in range(14):
        for j in range(19):
            if all(board[i+k, j] == board[i, j] for k in range(6) if board[i+k, j] != 0):
                return True

    # Check diagonals
    for i in range(14):
        for j in range(14):
            if all(board[i+k, j+k] == board[i, j] for k in range(6) if board[i+k, j+k] != 0):
                return True
            if all(board[i+k, j+5-k] == board[i, j+5] for k in range(6) if board[i+k, j+5-k] != 0):
                return True

    return False