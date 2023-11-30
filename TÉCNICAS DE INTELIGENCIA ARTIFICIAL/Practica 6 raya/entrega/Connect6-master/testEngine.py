
import games.Connect6 as Connect6
from engines.Negamax import Negamax
import numpy as np
import random
import time

from engines.scores.defaultScore import getScore
from engines.heuristics.defaultHeuristic import heuristic

def main():
    #connect6 = Connect6(rows=19, cols=19)
    negamax = Negamax(scoreFunction=getScore, heuristic=heuristic)

    partidas = 1
    wins = playVSrandom(negamax, partidas, True)
    print(f"Wins: {wins}")
    print(f"Score: {np.sum(wins)}")

def playVSrandom(negamax, partidas, debug=False):
    
    wins = []
    
    for i in range(partidas):
        player = 1
        state = Connect6.getInitialState()
        action = None
        done = False
        
        # Turno Inicial
        _ = Connect6.getNextState(state, player, np.array([[9, 9], [0, 0]]), firstMove=True)
        player = Connect6.getOpponent(player)
        
        # Generación del dataset de entrenamiento
        while not done:
            if player == 1:
                state, done, action, value = negaMaxStep(state, player, negamax, action)
            else:
                state, done, action, value = randomStep(state, player)
            
            if debug:
                Connect6.printBoard(state)
            
            if done:
                if player == -1:
                    value = Connect6.getOpponentValue(value)
                wins.append(value)
            
            player = Connect6.getOpponent(player)
    return wins


def negaMaxStep(board, player, negamax, action):
    #print(get_key(board))
    start = time.time()
    negamax.transpositionTable = {}
    score, action = negamax.search(player, board, action)
    
    if np.sum(board == 0) == 0:
        print("Empate")
        print(f"Score: {score}")
    end = time.time() - start
    print(f"Tiempo de ejecución: {end}")
    # print(f"Valor: {value}")
    # print(action)
    
    if action is None:
        return board, True, None, 0
    
    state = Connect6.getNextState(board, player, action, False)
    value, done = Connect6.getValueAndTerminated(state, action)
    
    return state, done, action, value


def randomStep(board, player):
    
    possibleMoves = Connect6.getValidMoves(board)
    move = random.sample(possibleMoves, 1)[0]
    # print(move)
    state = Connect6.getNextState(board, player, move, False)
    value, done = Connect6.getValueAndTerminated(state, move)
    
    if np.sum(board == 0) == 0:
        print("Empate")
    
    return state, done, move, value


if __name__ == '__main__':
    main()