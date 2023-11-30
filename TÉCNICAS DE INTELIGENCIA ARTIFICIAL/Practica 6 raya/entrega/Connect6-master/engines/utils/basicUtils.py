from numba import njit, jit
import numpy as np


@njit
def maxConsecutive(board, number):
    max_count = 0
    directions = [(1, 0), (0, 1), (1, 1), (1, -1)]

    for n in range(19):
        for m in range(19):
            if board[n, m] == number:
                for direction in directions:
                    count = 0
                    x = n
                    y = m
                    while x >= 0 and x < 19 and y >= 0 and y < 19 and board[x, y] == number:
                        x += direction[0]
                        y += direction[1]
                        count += 1
                    max_count = max(max_count, count)
    return max_count


@njit
def consecutives(board, number):
    max_count = 0
    directions = [(1, 0), (0, 1), (1, 1), (1, -1)]
    consec = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    for n in range(19):
        for m in range(19):
            if board[n, m] == number:
                for direction in directions:
                    count = 0
                    x = n
                    y = m
                    while x >= 0 and x < 19 and y >= 0 and y < 19 and board[x, y] == number:
                        x += direction[0]
                        y += direction[1]
                        count += 1
                    consec[count] += 1
                    max_count = max(max_count, count)
    return consec

@njit
def is_blocking_move(board, move):
    directions = [(0, 1), (1, 0), (1, 1), (1, -1)]
    for i in range(2):
        x = move[i,0]
        y = move[i,1]
        if board[x][y] != 0:
            return False
        board[x][y] = -1
        for direction in directions:
            count = 0
            nx, ny = x + direction[0], y + direction[1]
            while 0 <= nx < 19 and 0 <= ny < 19 and board[nx][ny] == 1:
                nx += direction[0]
                ny += direction[1]
                count += 1
            if count >= 6:
                return True
        board[x][y] = 0
    return False


@njit
def check_for_blocks(board, player, opponent):
    block_score = 0
    # Iterate over all possible lines on the board
    for n in range(19):
        for m in range(19):
            if board[n][m] == 0:  # A potential block position
                # Check all directions for potential blocks
                for dx, dy in [(1, 0), (0, 1), (1, 1), (-1, 1), (-1, 0), (0, -1), (-1, -1), (1, -1)]:
                    opponent_count = 0
                    player_count = 0
                    for i in range(1, 6):  # Check the next five positions
                        x = n + i * dx
                        y = m + i * dy
                        if 0 <= x < 19 and 0 <= y < 19:
                            if board[x][y] == opponent:
                                opponent_count += 1
                            elif board[x][y] == player:
                                player_count += 1
                            else:
                                # It's an empty space
                                pass
                        else:
                            # Outside of board bounds
                            break
                    
                    # If there's a sequence of opponent pieces that can be blocked
                    # and the player has a presence as well
                    if opponent_count > 3 and player_count > 0:
                        # More weight is given if the player is actively contesting the area
                        block_score += (10 ** opponent_count) * player_count

    return block_score

@njit
def check_for_forks(board, player):
    fork_score = 0
    # Iterate over all possible fork positions on the board
    for n in range(19):
        for m in range(19):
            # Check if the current position is empty and can be a fork point
            if board[n][m] == 0:
                fork_count = 0
                # Check all directions for potential forks
                for dx, dy in [(1, 0), (0, 1), (1, 1), (-1, 1), (-1, 0), (0, -1), (-1, -1), (1, -1)]:
                    count = 0
                    for i in range(1, 5):
                        x = n + i * dx
                        y = m + i * dy
                        if 0 <= x < 19 and 0 <= y < 19:
                            if board[x][y] == player:
                                count += 1
                            else:
                                break
                        else:
                            break
                    if count >= 2:  # This threshold can be adjusted
                        fork_count += 1
                if fork_count >= 2:  # If two or more lines of attack can start here
                    fork_score += 100 * fork_count  # Score heavily for fork opportunities

    return fork_score



@njit
def evaluate_board(board, player, opponent):
    block_score = 0
    fork_score = 0

    # Iterate over all possible positions on the board
    for n in range(19):
        for m in range(19):
            if board[n][m] == 0:  # A potential block or fork position
                fork_count = 0
                # Check all directions for potential blocks and forks
                for dx, dy in [(1, 0), (0, 1), (1, 1), (-1, 1), (-1, 0), (0, -1), (-1, -1), (1, -1)]:
                    opponent_count = 0
                    player_count = 0
                    empty_count = 0
                    for i in range(1, 7):  # Check up to five positions in this direction
                        x = n + i * dx
                        y = m + i * dy
                        if 0 <= x < 19 and 0 <= y < 19:
                            if board[x][y] == opponent:
                                opponent_count += 1
                            elif board[x][y] == player:
                                player_count += 1
                            else:
                                empty_count += 1
                        else:
                            # Outside of board bounds
                            break

                    # Calculate blocks
                    if opponent_count > 3 and player_count > 0:
                        block_score += (10 ** opponent_count) * player_count

                    # Calculate forks
                    if player_count >= 2 and empty_count + player_count == 5:
                        fork_count += 1
                
                # Add to fork score if a fork point is detected
                if fork_count >= 2:
                    fork_score += 100 * fork_count

    return block_score, fork_score



@njit
def find_moves_to_block(board, player):
    opponent = 1 if player == -1 else -1
    directions = [(1, 0), (0, 1), (1, 1), (-1, 1)]
    moves = []

    for n in range(19):
        for m in range(19):
            if board[n][m] == 0:
                for dx, dy in directions:
                    opponent_count = 0
                    for i in range(1, 5):
                        x = n + i * dx
                        y = m + i * dy
                        if 0 <= x < 19 and 0 <= y < 19:
                            if board[x][y] == opponent:
                                opponent_count += 1
                            elif board[x][y] == player:
                                opponent_count = -5
                                break
                            else:
                                break
                        else:
                            opponent_count = -5
                            break

                    if opponent_count == 4:
                        potential_moves = []
                        # Check for space before the sequence
                        if 0 <= n - dx < 19 and 0 <= m - dy < 19 and board[n - dx][m - dy] == 0:
                            potential_moves.append((n - dx, m - dy))
                        # Check for space after the sequence
                        if 0 <= n + 4 * dx < 19 and 0 <= m + 4 * dy < 19 and board[n + 4 * dx][m + 4 * dy] == 0:
                            potential_moves.append((n + 4 * dx, m + 4 * dy))
                        # Add up to two moves to block the sequence
                        moves.extend(potential_moves[:2])
                        if len(moves) >= 2:
                            return moves[:2]  # Return only two moves

    # If no moves were found, or only one move was found
    return moves


def get_blocking_moves_4_in_a_row(board, player):
    directions = [(0, 1), (1, 0), (1, 1), (1, -1), (-1, 0), (0, -1), (-1, -1), (-1, 1)]
    blocking_moves = []
    for i in range(19):
        for j in range(19):
            if board[i][j] != 0:
                continue
            board[i][j] = player
            for direction in directions:
                count = 0
                nx, ny = i + direction[0], j + direction[1]
                while 0 <= nx < 19 and 0 <= ny < 19 and board[nx][ny] == -player:
                    nx += direction[0]
                    ny += direction[1]
                    count += 1
                if count == 4:
                    blocking_moves.append(np.array([i, j]))
                    if len(blocking_moves) == 2:
                        board[i][j] = 0
                        return blocking_moves
            board[i][j] = 0
    return blocking_moves if blocking_moves else None

    
    
def is_1d_in_2d(arr_2d, arr_1d):
    return any(np.array_equal(sub_arr, arr_1d) for sub_arr in arr_2d)

# def maxBlockingMove(board):
#     max_count = 0
#     directions = [
#         (1, 0), (0, 1), (1, 1), (1, -1),
#         (-1, 0), (0, -1), (-1, -1), (-1, 1)
#     ]

#     for n in range(19):
#         for m in range(19):
#             if board[n, m] == -1:
#                 for direction in directions:
#                     count = 0
#                     x = n
#                     y = m
#                     while x >= 0 and x < 19 and y >= 0 and y < 19 and board[x, y] == -1:
#                         x += direction[0]
#                         y += direction[1]
#                         count += 1
#                     if x >= 0 and x < 19 and y >= 0 and y < 19:
#                         if board[x, y] == 1:
#                             max_count = max(max_count, count)
#     return max_count