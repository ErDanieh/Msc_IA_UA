import time

# TODO: aqui se colocaran funciones como result, isTerminal, isTimeLimitExceeded, etc.
import numpy as np
from tools import *
# import logging
import numpy as np

import lib.utils.boardUtils as boardUtils


def isTerminal(board, preMove):
    """Determina si una jugada es terminal, es decir, si finaliza el juego."""
    return boardUtils.is_win_by_premove(board, preMove)


def isTimeLimitExceeded(initTime, maxTime):
    """Determina si se ha excedido el límite de tiempo desde initTime.
    
    Args:
    - initTime (float): El tiempo inicial en segundos desde la época.
    - maxTime (float): El tiempo máximo permitido en segundos.

    Returns:
    - bool: Verdadero si se ha excedido el límite de tiempo, falso en caso contrario.
    """
    currentTime = time.time()
    elapsed = currentTime - initTime
    return elapsed > maxTime


def result(color: bool):
    """Devuelve el valor del resultado basado en el color."""
    return 0 if color else Defines.MININT + 1
