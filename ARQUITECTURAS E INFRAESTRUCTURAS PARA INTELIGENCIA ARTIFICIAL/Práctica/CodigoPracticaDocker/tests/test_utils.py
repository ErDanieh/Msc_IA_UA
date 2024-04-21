import unittest
import pandas as pd
from src import utils



class TestUtils(unittest.TestCase):

    # Before each test
    def setUp(self):
        self.df_correct = pd.DataFrame(
            {
                'quantity': [1, 2, 2],
                'product_weight': [0.2, 0.25, 1],
                'product_length': [120, 25, 30],
                'product_height': [50, 12, 30],
                'product_width': [5, 200, 35],
                'package_name': ['a4', 'a4', 'b3'],
                'box_length': [50, 50, 50],
                'box_width': [120, 120, 70],
                'box_height': [40, 40, 40]
            })


        self.df_incorrect = pd.DataFrame(
            {
                'quantity': [1, 2, 2],
                'product_weight': [0.2, 0.25, 1],
                'product_length': [120, 25, 30],
                'product_height': [50, 12, 30],
                'product_width': [5, 200, 35],
                'package_name': ['a4', 'a4', 'b3'],
                'box_length': [50, 50, 50],
                'box_width': [120, 120, 70],
                'box_height': [40, 40, 40],
                'extra_column': [1, 2, 3]
            })

        self.df_prediction_correct = pd.DataFrame(
            {
                'quantity': [1, 2, 2],
                'product_weight': [0.2, 0.25, 1],
                'product_length': [120, 25, 30],
                'product_height': [50, 12, 30],
                'product_width': [5, 200, 35],
            })

        self.df_prediction_incorrect = pd.DataFrame(
            {
                'quantity': [1, 2, 2],
                'product_weight': [0.2, 0.25, 1],
                'product_length': [120, 25, 30],
                'product_height': [50, 12, 30],
                'product_width': [5, 200, 35],
                'extra_column': [1, 2, 3]
            })


    def test_isDataFrameCorrect(self):

        cols = ['quantity', 'product_weight', 'product_length', 
                'product_height', 'product_width', 'package_name', 
                'box_length', 'box_width', 'box_height']


        self.assertTrue(utils.isDataFrameCorrect(self.df_correct, cols))
        self.assertFalse(utils.isDataFrameCorrect(self.df_incorrect, cols))


    def test_extractFeatures_badInput(self):
        with self.assertRaises(ValueError):
            utils.extractFeatures(self.df_incorrect)

    def test_extractFeatures_values(self):
        n_features = 10

        n_productos = 5
        peso_total = 2.7
        peso_media = 0.54
        peso_mediana = 0.25
        volumen_total = 213.0
        volumen_media = 42.6
        volumen_mediana = 31.5
        densidad_total = 0.0126
        densidad_mediana = 0.0079
        volumen_cajas = 380000

        features = utils.extractFeatures(self.df_correct)

        self.assertEqual(features["n_productos"][0], n_productos)
        self.assertEqual(features["peso_total"][0], peso_total)
        self.assertAlmostEqual(features["peso_media"][0], peso_media, places=3)
        self.assertAlmostEqual(features["peso_mediana"][0], peso_mediana, places=3)
        self.assertAlmostEqual(features["volumen_total"][0], volumen_total, places=3)
        self.assertAlmostEqual(features["volumen_media"][0], volumen_media, places=3)
        self.assertAlmostEqual(features["volumen_mediana"][0], volumen_mediana, places=3)
        self.assertAlmostEqual(features["densidad_total"][0], densidad_total, places=3)
        self.assertAlmostEqual(features["densidad_mediana"][0], densidad_mediana, places=3)
        self.assertAlmostEqual(features["volumen_cajas"][0], volumen_cajas, places=3)

        self.assertEqual(features.size, n_features)

    def test_extractPredictionFeatures_values(self):
        n_features = 9

        n_productos = 5
        peso_total = 2.7
        peso_media = 0.54
        peso_mediana = 0.25
        volumen_total = 213.0
        volumen_media = 42.6
        volumen_mediana = 31.5
        densidad_total = 0.0126
        densidad_mediana = 0.0079

        features = utils.extractPredictionFeatures(self.df_prediction_correct)

        self.assertEqual(features["n_productos"][0], n_productos)
        self.assertEqual(features["peso_total"][0], peso_total)
        self.assertAlmostEqual(features["peso_media"][0], peso_media, places=3)
        self.assertAlmostEqual(features["peso_mediana"][0], peso_mediana, places=3)
        self.assertAlmostEqual(features["volumen_total"][0], volumen_total, places=3)
        self.assertAlmostEqual(features["volumen_media"][0], volumen_media, places=3)
        self.assertAlmostEqual(features["volumen_mediana"][0], volumen_mediana, places=3)
        self.assertAlmostEqual(features["densidad_total"][0], densidad_total, places=3)
        self.assertAlmostEqual(features["densidad_mediana"][0], densidad_mediana, places=3)

        self.assertEqual(features.size, n_features)


    def test_extractPredictionFeatures_badInput(self):
        with self.assertRaises(ValueError):
            utils.extractFeatures(self.df_prediction_incorrect)


if __name__ == "__main__":
    unittest.main()
