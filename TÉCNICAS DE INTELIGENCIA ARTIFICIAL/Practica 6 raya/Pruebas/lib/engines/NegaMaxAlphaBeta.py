import time
import numpy as np
import lib.utils.boardUtils as boardUtils
import lib.utils.searchUtils as searchUtils


class NegaMaxAlphaBeta:
    def __init__(self):
        self.start_time = time.time()  # Almacena el tiempo actual en segundos desde la época
        self.time_elapsed = 0  # Este es para controlar el tiempo transcurrido si lo necesitas
        self.board = None  # Tablero de juego
        self.first_move = False  # Indica si es el primer movimiento
        self.moves = None  # Movimientos posibles
        self.m_total_nodes = 0 # Total de nodos visitados
        self.start_time = time.time() # Tiempo inicial
        self.max_time = 10 # Tiempo máximo de búsqueda


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
        

    def negaMaxInit(self, bestMove, depth: int, color: bool): # TODO: añadir move ordering
        
        if(self.first_move):
            self.first_move = False
            move = self.get_first_move()
            bestMove.positions[0].x = move[0][0]
            bestMove.positions[0].y = move[0][1]
            bestMove.positions[1].x = move[1][0]
            bestMove.positions[1].y = move[1][1]
            return 0
        
        chess_type = self.m_chess_type if color else self.o_chess_type

        alpha = np.NINF
        beta = np.inf

        value = np.NINF
        maxValue = np.NINF
        bestMoveIndex = None

        self.moves = boardUtils.find_possible_moves(self.board)
        size = np.shape(self.moves)[2]
        
        #moves = sorted(moves, key=lambda move: -self.heuristic_evaluation(boardUtils.make_move(board, move, color), chess_type, lastMove))
        
        for i in range(size):
            move = self.moves[:,:,i]
            self.m_total_nodes += 1
            
            self.board = boardUtils.make_move(self.board, move, chess_type)
            
            value = np.maximum(value, -self.negamax(move, depth-1, not color, np.NINF, np.inf, move))
            alpha = max(alpha, value)
            
            self.board = boardUtils.unmake_move(self.board, move)
            
            if value > maxValue:
                maxValue = value
                bestMoveIndex = i

            if alpha >= beta:
                break

        bestMove.positions[0].x = self.moves[0][0][bestMoveIndex]
        bestMove.positions[0].y = self.moves[0][1][bestMoveIndex]
        bestMove.positions[1].x = self.moves[1][0][bestMoveIndex]
        bestMove.positions[1].y = self.moves[1][1][bestMoveIndex]
        return maxValue
    
    
    def negamax(self, board, depth, alpha, beta, color, lastMove): # TODO: revisar el codigo
        
        chess_type = self.m_chess_type if color else self.o_chess_type
        
        if depth == 0 or searchUtils.isTimeLimitExceeded(self.start_time, self.max_time):
            return self.heuristic_evaluation(board, chess_type, lastMove)
        best_value = -np.inf
        moves = boardUtils.find_possible_moves(board)
        if not moves:
            return -self.negamax(board, depth-1, -beta, -alpha, -color, None)
        # Sort moves by their expected value
        moves = sorted(moves, key=lambda move: -self.heuristic_evaluation(boardUtils.make_move(board, move, color), chess_type, lastMove))
        for move in moves:
            self.m_total_nodes += 1
            new_board = boardUtils.make_move(board, move, color)
            value = -self.negamax(new_board, depth-1, -beta, -alpha, -color, move)
            best_value = max(best_value, value)
            alpha = max(alpha, value)
            if alpha >= beta:
                break
        return best_value
    
    
    # def heuristic_evaluation(self, board, color, lastMove): # TODO: buscar una mejor heuristica u optimizar esta
    #     directions = [(1, 0), (0, 1), (1, 1), (1, -1)]
    #     weights = {2: 1, 3: 10, 4: 100, 5: 1000, 6: 10000}

    #     total_value = 0

    #     for row in range(board.shape[0]):
    #         for col in range(board.shape[1]):
    #             if board[row][col] == color:
    #                 for direction in directions:
    #                     consecutive = 1
    #                     r, c = row + direction[0], col + direction[1]
    #                     while 0 <= r < board.shape[0] and 0 <= c < board.shape[1] and board[r][c] == color:
    #                         consecutive += 1
    #                         r += direction[0]
    #                         c += direction[1]

    #                     if consecutive in weights:
    #                         total_value += weights[consecutive]

    #     return total_value
    
    def heuristic_evaluation(self, board, color, lastMove): # TODO: buscar una mejor heuristica u optimizar esta
        directions = [(1, 0), (0, 1), (1, 1), (1, -1)]
        weights = {2: 1, 3: 10, 4: 100, 5: 1000, 6: 10000}

        total_value = 0

        for row in range(board.shape[0]):
            for col in range(board.shape[1]):
                if board[row][col] == color or board[row][col] == -color:
                    for direction in directions:
                        consecutive = 1
                        r, c = row + direction[0], col + direction[1]
                        while 0 <= r < board.shape[0] and 0 <= c < board.shape[1] and board[r][c] == board[row][col]:
                            consecutive += 1
                            r += direction[0]
                            c += direction[1]

                        if consecutive in weights:
                            if board[row][col] == color:
                                total_value += weights[consecutive]
                            else:
                                total_value -= weights[consecutive]

        return total_value