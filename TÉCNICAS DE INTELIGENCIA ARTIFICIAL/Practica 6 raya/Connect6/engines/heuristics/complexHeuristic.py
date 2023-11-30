from numba import njit
from engines.utils.basicUtils import *

@njit
def heuristic(board, player, move=None):
    opponent = -player
    score = 0
    
    # Check the max consecutive pieces for both player and opponent
    player_max_consecutive = maxConsecutive(board, player)
    opponent_max_consecutive = maxConsecutive(board, opponent)
    
    if opponent_max_consecutive > 5:
        return np.inf
    
    # Score based on the max consecutive pieces for player
    if player_max_consecutive > 2:
        score += 4 ** (player_max_consecutive)
    
    # Penalize based on the max consecutive pieces for opponent
    if opponent_max_consecutive > 3:
        score -= 5 ** (opponent_max_consecutive)
    elif opponent_max_consecutive > 4:
        score -= 20 ** (opponent_max_consecutive)
    
    score_fork, score_block = evaluate_board(board, player, opponent)
    # Additional logic to check for potential blocks and forks
    score += score_block
    score += score_fork

    # Additional heuristics can go here, like board centrality, potential to create multiple lines, etc.
    
    return score