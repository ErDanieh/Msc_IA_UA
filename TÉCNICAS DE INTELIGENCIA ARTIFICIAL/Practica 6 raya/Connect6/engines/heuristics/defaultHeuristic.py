from numba import njit
from engines.utils.basicUtils import maxConsecutive


@njit
def heuristic(board, player, move=None):
    ones = maxConsecutive(board, player)
    score = 0
    
    if ones > 2:
        score += 10**(ones-1)
    
    return score