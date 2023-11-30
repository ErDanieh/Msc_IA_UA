from numba import njit, jit
import numpy as np

#@njit
from numba import njit

#@njit
def getValidMoves(bitboard):
    valid_moves = bitboard.get_moves()
    n = len(valid_moves)
    move_pairs = [(valid_moves[i], valid_moves[j]) for i in range(n) for j in range(i+1, n)]
    return move_pairs

#@njit
def getNextState(board, player, action, firstMove=False):
    if firstMove:
        board.make_move(action[0], player)
        return board
    board.make_move(action[0], player)
    board.make_move(action[1], player)
    return board

#@njit
def checkWin(board, action=None):
    if action is None:
        return False
    pattern = np.array([1,1,1,1,1,1])
    return check_pattern(board, pattern)

#@njit
def bitboard_to_2d(bitboard: int):
    return np.array([[(bitboard >> i) & 1 for i in range(j*19, j*19 + 19)] for j in range(19)], dtype=np.uint8)

@jit
def check_pattern(matrix, pattern):
    rows, cols = matrix.shape
    pattern_length = len(pattern)

    # Create a sliding window view of the array for each direction
    row_view = np.lib.stride_tricks.sliding_window_view(matrix, (1, pattern_length))
    col_view = np.lib.stride_tricks.sliding_window_view(matrix, (pattern_length, 1))
    diag_view1 = np.lib.stride_tricks.sliding_window_view(matrix, (pattern_length, pattern_length))
    diag_view2 = np.lib.stride_tricks.sliding_window_view(np.fliplr(matrix), (pattern_length, pattern_length))

    # Check if pattern exists in the sliding window view for each direction
    if (row_view == pattern).all(axis=(2,3)).any() or \
       (col_view == pattern).all(axis=(2,3)).any() or \
       (diag_view1 == pattern).all(axis=(2,3)).any() or \
       (diag_view2 == pattern).all(axis=(2,3)).any():
        return True

    return False

# @njit
# def check_pattern(board_2d, pattern):
#     pattern_str = ''.join(map(str, pattern))
#     # Check rows
#     for i in range(board_2d.shape[0]):
#         row_str = ''
#         for j in range(board_2d.shape[1]):
#             row_str += str(board_2d[i][j])
#         if pattern_str in row_str:
#             return True
#     # Check columns
#     for j in range(board_2d.shape[1]):
#         col_str = ''
#         for i in range(board_2d.shape[0]):
#             col_str += str(board_2d[i][j])
#         if pattern_str in col_str:
#             return True
#     # Check diagonals
#     for i in range(board_2d.shape[0] - len(pattern) + 1):
#         for j in range(board_2d.shape[1] - len(pattern) + 1):
#             diag_str = ''
#             for k in range(len(pattern)):
#                 diag_str += str(board_2d[i+k][j+k])
#             if pattern_str in diag_str:
#                 return True
#     for i in range(board_2d.shape[0] - len(pattern) + 1):
#         for j in range(len(pattern) - 1, board_2d.shape[1]):
#             diag_str = ''
#             for k in range(len(pattern)):
#                 diag_str += str(board_2d[i+k][j-k])
#             if pattern_str in diag_str:
#                 return True
#     return False

@njit 
def strBoard(player, opponent):
    board_str = ''
    for i in range(19*19):
        if player & (1 << i):
            board_str += '1'
        elif opponent & (1 << i):
            board_str += '2'
        else:
            board_str += '0'
        if (i + 1) % 19 == 0:
            board_str += '\n'
    return board_str


class BitBoard:
    def __init__(self):
        self.player = 0  # Bitboard for the current player
        self.opponent = 0  # Bitboard for the opponent

    def make_move(self, move, player):
        if player is True:
            self.player |= 1 << move
        else:
            self.opponent |= 1 << move

    def unmake_move(self, move, player):
        if player is True:
            self.player &= ~(1 << move)
        else:
            self.opponent &= ~(1 << move)

    def is_valid_move(self, move):
        return move >= 0 and move < 19*19 and not ((self.player | self.opponent) & (1 << move))

    def get_moves(self):
        valid_moves = ~(self.player | self.opponent)
        return [i for i in range(19*19) if valid_moves & (1 << i)]
    
    def __copy__(self):
        # Create a new instance of the class
        cls = self.__class__
        new_instance = cls.__new__(cls)
        # Copy the instance's attributes
        new_instance.__dict__.update(self.__dict__)
        return new_instance