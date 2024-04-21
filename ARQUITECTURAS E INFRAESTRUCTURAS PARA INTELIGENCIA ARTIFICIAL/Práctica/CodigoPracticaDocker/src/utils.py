import pandas as pd
import numpy as np
import re

from tqdm import tqdm


def extractFeaturesByVersion(df: pd.DataFrame, version: str) -> pd.DataFrame:
    """
    Extracts features from the dataset based on the given version.
    
    Parameters
    ----------
    df : pd.DataFrame
        Input data from which features need to be extracted.
    version : str
        Version string based on which features are extracted.

    Returns
    -------
    pd.DataFrame
        Data with extracted features.

    Raises
    ------
    ValueError
        If the provided version is not supported.
    """

    # Check if the version matches the pattern vX.1.X, where X is a number or is 'latest'
    if re.match(r'v\d\.1\.\d', version) or version == 'latest':
        return predictionFeatureV1(df)
    
    raise ValueError(f"Version de extraccion de features no valida. Version: {version}")



def predictionFeatureV1(df: pd.DataFrame) -> pd.DataFrame:
    """
    Extracts features from the dataset.

    Parameters
    ----------
    df : pd.DataFrame. Requires ['quantity', 'product_weight', 'product_length', 'product_height', 'product_width', 'packing_name', 'packing_length', 'packing_width', 'packing_height'] columns

    Returns
    -------
    pd.DataFrame. ['volumen_cajas', 'n_productos', 'peso_total', 'peso_media', 'peso_mediana', 'volumen_total', 'volumen_media', 'volumen_mediana', 'densidad_total', 'densidad_mediana']

    Throws
    ------
    ValueError : if the dataframe has not the correct columns

    """

    # Columns required
    required_columns = ['quantity', 'product_weight', 'product_length',
       'product_height', 'product_width']
    
    # Check if the dataframe has the correct columns
    if (not isDataFrameCorrect(df, required_columns)):
        # Exception input df not correct
        raise ValueError("Error: el dataframe de entrada no tiene las columnas correctas. Debe tener las siguientes columnas: ", str(required_columns))


    # Calcular el volumen de cada producto. Se multiplica por 0.1 para pasar de mm a cm
    df['volumen_productos'] = 0.1 * df['product_length'] * 0.1 * df['product_height'] * 0.1 * df['product_width']

    # Calcular cantidad de productos
    n_productos = df['quantity'].sum()

    # Numero de productos distintos
    n_productos_distintos = len(df)

    # Calcular características relacionadas con el peso
    peso_total = (df['quantity'] * df['product_weight']).sum()
    peso_media = (df['product_weight'] * df['quantity']).sum() / n_productos
    peso_mediana = np.median(np.repeat(df['product_weight'], df['quantity']))

    # Calcular características relacionadas con el volumen
    volumen_total = (df['quantity'] * df['volumen_productos']).sum()
    volumen_media = (df['volumen_productos'] * df['quantity']).sum() / n_productos
    volumen_mediana = np.median(np.repeat(df['volumen_productos'], df['quantity']))

    # Calcular características relacionadas con peso y volumen
    densidad_total = peso_total / volumen_total
    densidad_mediana = peso_mediana / volumen_mediana

    # Devolver un DataFrame con todas las estadísticas
    features = pd.DataFrame({
        "n_productos":      [n_productos],
        "n_productos_distintos": [n_productos_distintos],
        "peso_total":       [peso_total],
        "peso_media":       [peso_media],
        "peso_mediana":     [peso_mediana],
        "volumen_total":    [volumen_total],
        "volumen_media":    [volumen_media],
        "volumen_mediana":  [volumen_mediana],
        "densidad_total":   [densidad_total],
        "densidad_mediana": [densidad_mediana]
    })


    return features




def isDataFrameCorrect(df: pd.DataFrame, cols: list) -> bool:
    """
    Checks if the dataframe has the correct columns.

    Parameters
    ----------
    df : pd.DataFrame

    Returns
    -------
    bool
    """

    return set(df.columns) == set(cols)




