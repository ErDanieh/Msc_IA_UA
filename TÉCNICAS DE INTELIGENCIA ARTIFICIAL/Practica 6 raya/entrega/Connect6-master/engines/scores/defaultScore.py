from numba import njit
from engines.utils.basicUtils import maxConsecutive


@njit
def getScore(board, player):
    ones = maxConsecutive(board, player)
    enemy_ones = maxConsecutive(board, -player)
    score = 0
    
    if ones > 2:
        score += 10**(ones-1)
    # if enemy_ones > 2:
    #     score -= 20**(enemy_ones-1)
    
    return score