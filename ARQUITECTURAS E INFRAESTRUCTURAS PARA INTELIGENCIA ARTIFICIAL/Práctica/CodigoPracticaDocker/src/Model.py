import xgboost as xgb
import pandas as pd


class Model:
    '''
    Clase que representa al modelo de prediccion de volumen
    '''

    model: xgb.Booster = xgb.Booster()

    def __init__(self, modelVersion):
        self.model.load_model('./artifacts/' + modelVersion + '/model.json')


    def predict(self, df):
        """
        Realiza una prediccion de volumen para un dataframe de productos.

        :param df: Dataframe de features de productos.
        """
        if self.model is None:
            raise Exception("Error: El modelo no ha cargado correctamente.")

        # Convertimos el dataframe en un DMatrix
        convertedData = xgb.DMatrix(df)

        return self.model.predict(convertedData)

