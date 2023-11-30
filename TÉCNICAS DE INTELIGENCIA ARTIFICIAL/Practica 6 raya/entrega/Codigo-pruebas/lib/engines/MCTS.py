
import numpy as np
import random
import time

class MCTS:
    def __init__(self, policy=2):
        self.m_total_nodes = 0  # Total de nodos visitados
        self.policy = policy    # Política de selección de nodos
    
    
    def search(self, board, chessType, previousMove=None, timeLimit=10.0, iterationLimit=1000000):
        """Realiza una búsqueda del mejor movimiento utilizando el algoritmo Monte Carlo Tree Search

        Args:
            board (list[list[int]]): 21x21 celdas que representan el tablero de juego (posee un borde de 1 casilla 
            con valor 3)
            chessType (int): Tipo de ficha del jugador. EL (1) representa las fichas negras y el (2) las blancas.
            previousMove (np.ndarray): El anterior movimiento realizado en la partida. Defaults to None.
            timeLimit (float): Límite de tiempo de la búsqueda. Defaults to 10.0.
            iterationLimit (int): Límite de iteraciones de la búsqueda. Defaults to 1000000.

        Returns:
            Node: Devuelve el nodo con el mejor movimiento
        """
        start_time = time.time()
        board = np.array([row[:] for row in board])
        
        rootNode = Node(board, isRoot=True)
        rootNode.chessType = chessType
        rootNode.previousMove = previousMove

        iterations = 0
        while time.time() - start_time < timeLimit: 
            selectedNode = self.select(rootNode)
            leafNode = self.expand(selectedNode)
            self.m_total_nodes += 1
            value = self.simulation(leafNode, chessType)
            self.backpropagation(leafNode, value)
            
            iterations += 1
            if iterations >= iterationLimit:
                break
            
        return rootNode.bestChild()

    def select(self, node):
        while not node.isTerminal() and node.isFullyExpanded():
            newNode = node.bestUCT() # Selection strategy
            if newNode is None:
                break
            node = newNode
            
        return node

    def expand(self, node):
        unexplored_child = node.getUnexploredChild()
        if unexplored_child is not None:
            return node.addChild(unexplored_child)
        else:
            return node

    def simulation(self, node, chessType):
        while not node.isTerminal():
            newNode = self.rolloutPolicy(node)
            if newNode is None:
                break
            node = newNode
            
        return node.result(chessType)
        
    def backpropagation(self, node, value):
        while node is not None:
            node.visits += 1
            node.wins += value
            node = node.parent
            
    def rolloutPolicy(self, node): # Devuelve un hijo aleatorio (por ahora)
        if self.policy == 0:
            return node.randomPolicy()
        elif self.policy == 1:
            return node.bestUCT()
        elif self.policy == 2:
            return node.heuristicPolicy()
        elif self.policy == 3:
            return node.neuralNetworkPolicy()
        else:
            return node.monteCarloPolicy()

class Node:
    def __init__(self, board, isRoot=False):
        self.visits = 0
        self.wins = 0
        self.parent = None
        self.children = []
        self.board = board
        self.previousMove = None
        self.exploredMoves = []
        self.unexploredMoves = None
        self.totalMoves = 0
        self.chessType = None
        
        if isRoot:
            self.previousMove = None
            self.unexploredMoves = find_possible_moves(board)
            #random.shuffle(self.unexploredMoves)
            self.totalMoves = len(self.unexploredMoves)
        
    
    def bestChild(self): # Devuelve el hijo con más visitas
        best_score = np.NINF
        best_child = None
        for child in self.children:
            score = child.winRate()
            if score > best_score:
                best_score = score
                best_child = child
                      
        return best_child
    
    def winRate(self): # Devuelve el porcentaje de victorias
        if self.visits == 0:
            winRate = 0
        else:
            winRate = self.wins / self.visits
        return winRate
    
    def bestUCT(self): # Devuelve el hijo con mejor UCT
        best_score = np.NINF
        best_child = None
        for child in self.children:
            score = child.UCTscore()
            if score > best_score:
                best_score = score
                best_child = child
                
        return best_child
    
    
    def isFullyExpanded(self): # Devuelve si el nodo tiene todos los hijos expandidos, es decir se han probado todos los movimientos posibles desde ese estado del tablero
        return self.totalMoves == len(self.children)
    
    
    def isTerminal(self): # TODO: empate, gana blanco o gana negro
        return np.all(self.board != 0) or is_win_by_premove(self.board, self.previousMove)
    
    
    def getUnexploredChild(self): # Devuelve un hijo sin explorar
        
        if not self.unexploredMoves:
            return None
        
        move = self.unexploredMoves.pop()
        chessType = 1 if self.chessType == 2 else 2
        board = make_move(self.board, move, chessType)
        
        node = Node(board)

        node.parent = self
        node.previousMove = move
        node.unexploredMoves = self.unexploredMoves
        node.totalMoves = len(self.unexploredMoves)
        node.chessType = chessType
        
        return node
    
    
    def addChild(self, child): # Añade un hijo
        self.children.append(child)
        return child
    
    # Upper Confidence Bound for Search Trees
    def UCTscore(self, C=1):
        return self.wins/self.visits + C * np.sqrt(np.log(self.parent.visits)/self.visits)

    
    def result(self, chessType): # TODO: devuelve quien ha ganado la partida en una simulacion
        winnerType = winner(self.board, self.previousMove)
        if winnerType == 0:
            #logging.debug("\t\t* Empate")
            return 0
        elif winnerType == chessType:
            #logging.debug("\t\t* Gana Max")
            return 1
        else:
            #logging.debug("\t\t* Gana Min")
            return -1
    
    
    def randomPolicy(self): # Devuelve aleatoriamente un hijo sin explorar
        
        if not self.unexploredMoves:
            return None
        
        randomInt = random.randint(0, len(self.unexploredMoves)-1)
        move = self.unexploredMoves[randomInt]
        #self.unexploredMoves.remove(move)
        chessType = 1 if self.chessType == 2 else 2
        board = make_move(self.board, move, chessType)
        
        node = Node(board)

        node.parent = self
        node.previousMove = move
        node.unexploredMoves = self.unexploredMoves
        node.unexploredMoves.pop(randomInt)
        node.totalMoves = len(self.unexploredMoves) - 1 
        node.chessType = chessType
        
        return node
    
    
    def heuristicPolicy(self): # TODO: seleccionar una heuristica
        best_score = np.NINF
        best_child = None
        for child in self.children:
            score = heuristic_evaluation(child.board, child.chessType, child.previousMove)
            if score > best_score:
                best_score = score
                best_child = child
                
        return best_child
    
    
    def monteCarloPolicy(self):
        node = self
        while node.unexploredMoves == [] and node.children != []:
            node = node.bestUCT()
        if node.unexploredMoves != []:
            return node.randomPolicy()
        else:
            return node.monteCarloPolicy()
    
    
    def neuralNetworkPolicy(self): # TODO
        pass
    
def find_possible_moves(board: np.ndarray):
    """Devuelve todos los pares de movimientos posibles con la forma (2,2,k), siendo k el número de pares

    Args:
        board (_type_): array 2D que representa el tablero

    Returns:
        _type_: todos los pares de movimientos posibles en el tablero
    """
    single_moves = np.transpose(np.nonzero(board == 0))
    size = single_moves.shape[0]

    # Generate indices of the upper triangle of the matrix
    indices = np.triu_indices(size, k=1)

    double_moves = np.zeros((2, 2, indices[0].size), dtype=int)
    double_moves[0] = np.transpose(single_moves[indices[0]])
    double_moves[1] = np.transpose(single_moves[indices[1]])

    double_moves_list = []
    
    for i in range(double_moves.shape[2]):
        double_moves_list.append(double_moves[:,:,i])

    return double_moves_list

    """_summary_
    def is_win_by_premove(board: np.ndarray, pre_move: np.ndarray) -> bool:
    "Comprueba si el anterior movimiento ha sido un movimiento ganador

    Args:
        board (_type_): array 2D que representa el tablero array 2D que representa el tablero
        premove (_type_): movimiento doble [[x1,y1],[x2,y2]]
        
    Returns:
        _type_: Devuelve verdadero si el movimiento (premove) finaliza la partida y falso en caso contrario
    "
    directions = [(1, 0), (0, 1), (1, 1), (1, -1)]
    color = board[tuple(pre_move[0].T)]

    for direction in directions:
        indices = np.flatnonzero(board == color)
        for index in indices:
            position = np.unravel_index(index, board.shape)
            x, y = position
            count = 1
            while True:
                x += direction[0]
                y += direction[1]
                if not (0 <= x < board.shape[0] and 0 <= y < board.shape[1]):
                    break
                if board[x, y] != color:
                    break
                count += 1
            x, y = position
            while True:
                x -= direction[0]
                y -= direction[1]
                if not (0 <= x < board.shape[0] and 0 <= y < board.shape[1]):
                    break
                if board[x, y] != color:
                    break
                count += 1
            if count >= 6:
                return True
    return False
    """


def is_win_by_premove(board: np.ndarray, pre_move: np.ndarray) -> bool:
    directions = [(1, 0), (0, 1), (1, 1), (1, -1)]
    color = board[tuple(pre_move[0].T)]
    
    for move in pre_move:
        x, y = tuple(move)
        for dx, dy in directions:
            count = 1
            
            # Verificar en una dirección
            i, j = x + dx, y + dy
            while 0 <= i < board.shape[0] and 0 <= j < board.shape[1] and board[i][j] == color:
                count += 1
                i += dx
                j += dy

            # Verificar en la dirección opuesta
            i, j = x - dx, y - dy
            while 0 <= i < board.shape[0] and 0 <= j < board.shape[1] and board[i][j] == color:
                count += 1
                i -= dx
                j -= dy

            if count >= 6:
                return True

    return False



def make_move(board, move, chess_type):
    """Actualiza el tablero (board) con el movimiento realizado doble (move) del tipo de ficha indicado (chess_type)

    Args:
        board (_type_): array 2D que representa el tablero
        move (_type_): movimiento doble [[x1,y1],[x2,y2]]
        chess_type (_type_): tipo de ficha que se esta moviendo

    Returns:
        _type_: board actualizado
    """
    cell1 = tuple(move[0])
    cell2 = tuple(move[1])

    row1, col1 = cell1
    row2, col2 = cell2

    board[row1][col1] = chess_type
    board[row2][col2] = chess_type
    return board


def winner(board: np.ndarray, pre_move: np.ndarray) -> bool:
    """Comprueba si el anterior movimiento ha sido un movimiento ganador

    Args:
        board (_type_): array 2D que representa el tablero array 2D que representa el tablero
        premove (_type_): movimiento doble [[x1,y1],[x2,y2]]
        
    Returns:
        _type_: Devuelve verdadero si el movimiento (premove) finaliza la partida y falso en caso contrario
    """
    directions = [(1, 0), (0, 1), (1, 1), (1, -1)]
    color = board[tuple(pre_move[0].T)]

    for direction in directions:
        indices = np.flatnonzero(board == color)
        for index in indices:
            position = np.unravel_index(index, board.shape)
            x, y = position
            count = 1
            while True:
                x += direction[0]
                y += direction[1]
                if not (0 <= x < board.shape[0] and 0 <= y < board.shape[1]):
                    break
                if board[x, y] != color:
                    break
                count += 1
            x, y = position
            while True:
                x -= direction[0]
                y -= direction[1]
                if not (0 <= x < board.shape[0] and 0 <= y < board.shape[1]):
                    break
                if board[x, y] != color:
                    break
                count += 1
            if count >= 6:
                return color
    return 0


def heuristic_evaluation(board, color, lastMove):
    directions = [(1, 0), (0, 1), (1, 1), (1, -1)]
    weights = {2: 1, 3: 10, 4: 100, 5: 1000, 6: 10000}
    
    # Define un rango para explorar alrededor del último movimiento
    # Puedes ajustar el rango según la lógica de tu juego
    RANGE = 6

    min_row = max(0, min(lastMove[0][0], lastMove[1][0]) - RANGE)
    max_row = min(board.shape[0], max(lastMove[0][0], lastMove[1][0]) + RANGE)
    
    min_col = max(0, min(lastMove[0][1], lastMove[1][1]) - RANGE)
    max_col = min(board.shape[1], max(lastMove[0][1], lastMove[1][1]) + RANGE)

    total_value = 0

    for row in range(min_row, max_row):
        for col in range(min_col, max_col):
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
