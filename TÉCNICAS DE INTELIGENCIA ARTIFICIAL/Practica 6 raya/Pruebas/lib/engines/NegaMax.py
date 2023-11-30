import time
import numpy as np
import lib.utils.boardUtils as boardUtils
import lib.utils.searchUtils as searchUtils


class NegaMax: # TODO: añadir estadisticas de nodos expandidos, etc.
    def __init__(self):
        self.start_time = time.time()  # Almacena el tiempo actual en segundos desde la época
        self.time_elapsed = 0  # Este es para controlar el tiempo transcurrido si lo necesitas
        self.board = None  # Tablero de juego
        self.first_move = False  # Indica si es el primer movimiento
        self.moves = None  # Movimientos posibles
        self.m_total_nodes = 0


    def before_search(self, board, color, depth):
        """_summary_

        Args:
            board (_type_): _description_
            color (_type_): _description_
            depth (_type_): _description_
        """
        self.board = np.array([row[:] for row in board])
        self.m_chess_type = color
        self.o_chess_type = 1 if self.m_chess_type == 1 else 2
        self.depth = depth
        self.first_move = boardUtils.check_first_move(self.board)
        

    def negaMaxInit(self, bestMove, depth: int, color: bool):
        
        if(self.first_move):
            self.first_move = False
            move = self.get_first_move()
            bestMove.positions[0].x = move[0][0]
            bestMove.positions[0].y = move[0][1]
            bestMove.positions[1].x = move[1][0]
            bestMove.positions[1].y = move[1][1]
            return 0
        
        chess_type = self.m_chess_type if color else self.o_chess_type

        value = np.NINF
        maxValue = np.NINF
        bestMoveIndex = None

        self.moves = boardUtils.find_possible_moves(self.board)
        size = np.shape(self.moves)[2]
        
        for i in range(size):
            move = self.moves[:,:,i]
            
            self.board = boardUtils.make_move(self.board, move, chess_type)
            
            value = np.maximum(value, -self.negaMax(move, depth-1, not color))
            
            self.board = boardUtils.unmake_move(self.board, move)
            
            if value > maxValue:
                maxValue = value
                bestMoveIndex = i

        bestMove.positions[0].x = self.moves[0][0][bestMoveIndex]
        bestMove.positions[0].y = self.moves[0][1][bestMoveIndex]
        bestMove.positions[1].x = self.moves[1][0][bestMoveIndex]
        bestMove.positions[1].y = self.moves[1][1][bestMoveIndex]
        return value
    
    
    def negaMax(self, preMove, depth: int, color: bool):
        
        if  depth == 0 or searchUtils.isTerminal(self.board,preMove):
            return searchUtils.result(color)
        
        chess_type = self.m_chess_type if color else self.o_chess_type
        
        value = np.NINF

        self.moves = boardUtils.find_possible_moves(self.board)
        size = np.shape(self.moves)[2]
        
        for i in range(size):
            move = self.moves[:,:,i]
            
            self.board = boardUtils.make_move(self.board, move, chess_type)
            
            value = np.maximum(value, -self.negaMax(move, depth-1, not color))
            
            self.board = boardUtils.unmake_move(self.board, move)

        return value