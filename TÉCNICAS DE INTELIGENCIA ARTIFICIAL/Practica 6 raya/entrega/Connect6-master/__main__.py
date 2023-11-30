import games.Connect6 as Connect6
import numpy as np
import sys
import time

# from engines.Negamax import Negamax
# from engines.NegamaxAlphaBetaMemory import Negamax
from engines.NegamaxUltimate import Negamax
from engines.MTD import MTD as MTD
# from engines.heuristics.defaultHeuristic import heuristic as heuristic
from engines.heuristics.complexHeuristic import heuristic as heuristic
# from engines.heuristics.patternHeuristic import heuristic as heuristic

from engines.scores.defaultScore import getScore as getScore


def main():
    ENGINE_NAME = "aaadar"
    BOARD = Connect6.getInitialState()
    DEPTH = 3
    TURN = 1
    
    ENGINE = Negamax(scoreFunction=getScore, heuristic=heuristic)
    #ENGINE = MTD(scoreFunction=getScore, heuristic=heuristic)
    
    msg = ""
    move = None
    onHelp(ENGINE_NAME)
    
    while True:
        msg = input().strip()
        #log_to_file(msg)
        
        if msg == "name":
            print(f"name {ENGINE_NAME}")
        elif msg == "exit" or msg == "quit":
            break
        elif msg == "print":
            printBoard(BOARD)
        elif msg.startswith("black"):
            move = msg2move(msg[6:])
            BOARD = Connect6.getNextState(
                board=BOARD,
                player=1,
                action=move
            )
            TURN = 1
        elif msg.startswith("white"):
            move = msg2move(msg[6:])
            BOARD = Connect6.getNextState(
                board=BOARD,
                player=-1,
                action=move
            )
            TURN = -1
        elif msg == "next":
            TURN = -TURN
            move = searchMove(ENGINE, BOARD, TURN, move, DEPTH)
            msg = f"move {move2msg(move)}"
            print(msg)
            flushOutput()

        elif msg.startswith("new"):
            BOARD = Connect6.getInitialState()
            
            if msg[4:] == "black":
                move = msg2move("JJ")
                BOARD = Connect6.getNextState(
                    board=BOARD,
                    player=1,
                    action=move
                )
                TURN = 1
                msg = "move JJ"
                print(msg)
                flushOutput()
            else:
                TURN = -1
                
        elif msg.startswith("move"):
            move = msg2move(msg[5:])
            BOARD = Connect6.getNextState(
                board=BOARD,
                player=-TURN,
                action=move
            )
            if Connect6.checkWin(BOARD, move):
                print(f"Player {-TURN} Wins")
            else:
                move = searchMove(ENGINE, BOARD, TURN, move, DEPTH)
                msg = f"move {move2msg(move)}"
                BOARD = Connect6.getNextState(
                    board=BOARD,
                    player=TURN,
                    action=move
                )
                print(msg)
                flushOutput()
                if Connect6.checkWin(BOARD, move):
                    print(f"Player {TURN} Wins")
        elif msg.startswith("depth"):
            d = int(msg[6:])
            if d > 0:
                DEPTH = d
            print(f"Set the search depth to {DEPTH}.\n")
            pass
        elif msg.startswith("maxtime"):
            # t = float(msg[8:])
            # if t > 0.1:
            #     self.m_max_time = t
            # print(f"Set the max time to {self.m_max_time}.\n")
            pass
        elif msg.startswith("iterationsLimit"):
            # i = int(msg[16:])
            # if i > 0:
            #     self.m_iteratinos_limit = i
            # print(f"Set the max iterations to {self.m_iteratinos_limit}.\n")
            pass
        elif msg == "help":
            onHelp(ENGINE_NAME)
        elif msg == "vcf":
            pass
        elif msg == "unvcf":
            pass
    return 0


def searchMove(engine, board, player, prevMove, depth=6, maxTime=None, iterations=None):
    
    cantidadFichas = np.sum(abs(board))
    if cantidadFichas == 0:
        return np.array([[9,9],[9,9]])
    elif cantidadFichas == 1:
        print("Primer movimiento")
        valid_moves = np.array(Connect6.getValidMoves(board))
        action = np.array([[prevMove[0][0]-1,prevMove[0][1]-1],[prevMove[0][0]+1,prevMove[0][1]-1]])
        if is_1d_in_2d(valid_moves, action):
            return action
    
    engine.reset()
    
    start = time.perf_counter()
    score, action = engine.search(
        player=player,
        board=board,
        prevMove=prevMove,
        depth=depth
    )
    end = time.perf_counter()
    
    if True:
        print("Branches:\t\n", engine.prunnedBranches)
        print("Table Hits:\t\n", engine.tableHits)
        print("Terminal:\t\n", engine.terminalNodes)
        print("Visited:\t\n", engine.visitedNodes)
    
    print(f"AB Time:\t{end - start:.3f}")
    print(f"Node:\t{0}\n")
    print(f"Score:\t{score:.3f}")
    return action


def printBoard(inputBoard):
    board = np.copy(inputBoard)
    board = np.flip(board, 0)
    
    print("   " + "".join([chr(i + ord('A'))+" " for i in range(0, 19)]))
    for i in range(19):
        print(f"{chr(ord('A') + i)}", end=" ")
        for j in range(19):
            x = 18 - j
            y = i
            stone = board[x][y]
            if stone == 0:
                print(" -", end="")
            elif stone == 1:
                print(" *", end="")
            elif stone == -1:
                print(" O", end="")
        print(" ", end="")        
        print(f"{chr(ord('A') + i)}", end="\n")
    print("   " + "".join([chr(i + ord('A'))+" " for i in range(0, 19)]))


def onHelp(ENGINE_NAME):
        print(
            f"On help for GameEngine {ENGINE_NAME}\n"
            " name        - print the name of the Game Engine.\n"
            " print       - print the board.\n"
            " exit/quit   - quit the game.\n"
            " black XXXX  - place the black stone on the position XXXX on the board.\n"
            " white XXXX  - place the white stone on the position XXXX on the board, X is from A to S.\n"
            " next        - the engine will search the move for the next step.\n"
            " move XXXX   - tell the engine that the opponent made the move XXXX,\n"
            "              and the engine will search the move for the next step.\n"
            " new black   - start a new game and set the engine to black player.\n"
            " new white   - start a new game and set it to white.\n"
            " depth d     - set the alpha beta search depth, default is 6.\n"
            " vcf         - set vcf search.\n"
            " unvcf       - set none vcf search.\n"
            " maxtime t   - set the max time for the search, default is 3.0.\n"
            " iterationsLimit i - set the max iterations for the search, default is 1000000.\n"
            " help        - print this help.\n")


def flushOutput():
    sys.stdout.flush()


def msg2move(msg):
    size = len(msg)
    numbers = [None] * size
    for i in range(size):
        numbers[i] = ord(msg[i]) - ord('A')
    if size == 2:
        return np.array([[numbers[0], numbers[1]], [numbers[0], numbers[1]]])    
    return np.array([[numbers[0], numbers[1]], [numbers[2], numbers[3]]])

def move2msg(move):
    size = 4
    msg = [''] * size
    k = 0
    for i in range(2):
        for j in range(2):
            msg[k] = chr(move[i][j] + ord('A'))
            k += 1
    return ''.join(msg)


def is_1d_in_2d(arr_2d, arr_1d):
    return any(np.array_equal(sub_arr, arr_1d) for sub_arr in arr_2d)

if __name__ == '__main__':
    main()