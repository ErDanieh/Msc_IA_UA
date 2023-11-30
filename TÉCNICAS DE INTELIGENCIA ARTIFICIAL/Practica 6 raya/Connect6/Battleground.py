import games.Connect6 as Connect6
from agents.AIAgent import AIAgent
from agents.HumanAgent import HumanAgent
from engines.Negamax import Negamax
from engines.heuristics.defaultHeuristic import heuristic as dummy_heuristic
from engines.heuristics.complexHeuristic import heuristic as complex_heuristic
from engines.scores.defaultScore import getScore
import random
import numpy as np
import time

def play_game(agent1, agent2, board, firstMove=True):
    
    current_agent = agent1
    preMove = None
    while True:
        inicio = time.time()
        if (current_agent == agent1):
            print(f"Turno de {agent1}")
        else:
            print(f"Turno de {agent2}")
        action = current_agent.get_move(board, preMove)
        preMove = action
        board = Connect6.getNextState(board, current_agent.chess, action, firstMove)
        Connect6.printBoard(board)
        fin = time.time() - inicio
        print(f"Tiempo de ejecución: {fin}")
        firstMove = False
        _, terminated = Connect6.getValueAndTerminated(board, action)
        if terminated:
            print(f"Game Over! {current_agent.name} wins!")
            return current_agent
        current_agent = agent1 if current_agent == agent2 else agent2


def organize_tournament(agents, board_setup_func, game_func):
    # Asegúrate de que haya un número par de agentes para los emparejamientos
    if len(agents) % 2 != 0:
        raise ValueError("El número de agentes debe ser par para un torneo eliminatorio.")
    
    # Baraja los agentes para emparejamientos aleatorios
    random.shuffle(agents)
    
    # Mientras haya más de un agente, sigue jugando rondas
    while len(agents) > 1:
        
        next_round_agents = []
        
        # Empareja los agentes y juega una ronda
        for i in range(0, len(agents), 2):
            agent1 = agents[i]
            agent2 = agents[i + 1]
            print(f"\nEnfrentamiento: {agent1.name} vs {agent2.name}")
            
            board = board_setup_func()  # Inicializar tablero para el juego
            winner = game_func(agent1, agent2, board)  # Juega el juego
            
            # Añade al ganador a la lista de la siguiente ronda
            next_round_agents.append(winner)
        
        # Actualiza la lista de agentes con los ganadores de esta ronda
        agents = next_round_agents
        print(f"\n--- Avanzan a la siguiente ronda: {[agent.name for agent in agents]} ---")
    
    # El último agente restante es el ganador del torneo
    tournament_winner = agents[0]
    print(f"\nCampeón del torneo: {tournament_winner.name}")
    return tournament_winner

# JUEGO DE TORNEO
def play_tournament_game(agent1, agent2, board):
    winner = play_game(agent1, agent2, board)
    print(f"Ganador: {winner.name}")
    return winner




# Preparar agentes para el torneo
# agents = [AIAgent(name=f"AI Bot {i}", AI=Negamax(scoreFunction=getScore, heuristic=heuristic), chess=1 if i % 2 == 0 else -1, log_moves=True) for i in range(8)]

# Iniciar torneo
# organize_tournament(agents, Connect6.getInitialState, play_tournament_game)

# Ejemplo de cómo iniciar un juego

Nega = Negamax(scoreFunction = getScore, heuristic= complex_heuristic)
Nega2 = Negamax(scoreFunction = getScore, heuristic= dummy_heuristic)   
agent1 = AIAgent(name="Dummy", chess=-1 ,AI = Nega2, log_moves=True) 
#agent2 = HumanAgent(name="Human", chess=1, log_moves=False) # Suponiendo que la IA es el jugador -1n
agent2 = AIAgent(name="Complex2", chess=1 ,AI = Nega, log_moves=True,depth=6) 
# play_game(agent1, agent2, Connect6.getInitialState())

player1_positions = [(3, 3), (4, 4), (5, 5)]
player2_positions = [(17, 17), (17, 16), (17, 15),(17, 18), (12, 17), (12, 16), (12, 15),(12, 18)]

custom_board = Connect6.getCustomInitialState(player1_positions, player2_positions)
board = Connect6.getInitialState()

play_game(agent1, agent2, board)