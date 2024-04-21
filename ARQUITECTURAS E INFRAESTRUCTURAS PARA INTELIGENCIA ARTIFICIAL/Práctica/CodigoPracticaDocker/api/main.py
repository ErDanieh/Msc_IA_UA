###############################################
##################  Imports  ##################
###############################################
from dotenv import load_dotenv
from fastapi import FastAPI, HTTPException, status, Response, Request
from pydantic import BaseModel
from src import Model
from src.utils import extractFeaturesByVersion 
from typing import List, Optional, Any
from xgboost.core import XGBoostError
import os
import pandas as pd

###############################################
################## Constants ##################
###############################################
from .constants import *  # Importa todas las constantes definidas en constants.py

DEFAULT_CONVERSION_RATE = float(os.getenv("DEFAULT_CONSTANT_PREDICTION") or "270")
DEFAULT_SLOPE_CORRECTION = float(os.getenv("DEFAULT_SLOPE_CORRECTION") or "1.13")

###############################################
################## Env Vars ###################
###############################################
load_dotenv()

###############################################
################# Requests ####################
###############################################

class PredictionRequestProduct(BaseModel):
    '''
    Clase que representa un producto en el cuerpo de la peticion POST para el endpoint de prediccion.
    '''
    sku: Optional[str] = None
    product_weight: Optional[float] = None
    product_length: Optional[float] = None
    product_width: Optional[float] = None
    product_height: Optional[float] = None
    quantity: Optional[int] = None



class PredictionRequest(BaseModel):
    '''
    Clase que representa el cuerpo de la peticion POST para el endpoint de prediccion.
    '''
    order: Optional[str] = None
    data: Optional[Any] = None
    conversion_rate: Optional[Any] = DEFAULT_CONVERSION_RATE
    slope_correction: Optional[Any] = DEFAULT_SLOPE_CORRECTION
    



###############################################

app = FastAPI(docs_url='/', redoc_url='/new_redoc')


###############################################

###############################################
################## VALIDATORS #################
###############################################

def validate_products(products):
    product_fields = ["sku", "quantity", "product_weight", "product_length", "product_height", "product_width"]
    for index, product in enumerate(products):
        missing_fields = [field for field in product_fields if field not in product]
        if missing_fields:
            raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=f"Faltan los campos {missing_fields} en el producto en la posición {index}.")

def extract_ip_caller(request: Request):
    client_ip = request.client.host
    x_forwarded_for = request.headers.get("X-Forwarded-For")
    if x_forwarded_for:
        # X-Forwarded-For puede contener múltiples IPs, el cliente original es generalmente el primero
        client_ip = x_forwarded_for.split(",")[0]
    return client_ip

def validate_entry(data):
    if data.data is None:
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=ERROR_REQUIRED_DATA)
    
    if not isinstance(data.data, list):
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=ERROR_REQUIRED_DATA_PRODUCTS_FORMAT)
    
    validate_products(data.data)


def validate_model_version(version):
    folder_path = os.path.abspath("./artifacts")
    folder_list = os.listdir(folder_path)
    if version not in folder_list:
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail=ERROR_MODEL_NOT_FOUND)

###############################################
################## ENDPOINTS ##################
###############################################

@app.post("/{version}/predict", 
          response_description="Prediccion de volumen para un conjunto de datos.",
          status_code=status.HTTP_200_OK)
async def predict(version: str, response: Response, data: PredictionRequest, request: Request):
    '''
    Endpoint que devuelve la prediccion de volumen para un conjunto de datos.

    Parameters
    ----------
    order : str
    data : dict -> { data: [{"sku": "123", "product_length": 34.3, "product_width": 12.3, "product_height": 12.3, "quantity": 2}, {...}] }
    conversion_rate : float
    slope_correction : float

    Returns
    -------
    { "prediction": float, "isOrder": bool}
    '''

    error_msg = "" # Error message to be returned in the response
    is_order = True if data.order else False
    client_ip = extract_ip_caller(request = request)

    # Validaciones
    validate_model_version(version = version)
    validate_entry(data = data)

    # Getters
    conversion_rate, slope_correction = get_conversion_rate_and_slope_correction(data = data, isOrder = is_order)
    pesoTotalProductos = get_total_weight(data = data)
  
    try:
        model = Model.Model(version)
        df = process_input_data(data.data)

        # Check that quantity is greater than 0 for all products
        if not (df["quantity"] > 0).all():
            response.status_code = status.HTTP_400_BAD_REQUEST
            raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=ERROR_QUANTITY_BIGGER_THAN_ZERO)

        # Extract features
        features = extractFeaturesByVersion(df, version)
        
        # Calculate prediction
        prediction_model = (model.predict(features)[0].item() / 1_000_000) * slope_correction

        # Base response
        response = {
            "prediction": prediction_model,
            "pesoVolumetricoEnKg": prediction_model/conversion_rate,
            "pesoTotalProductos": pesoTotalProductos,
            "isOrder": is_order
        }


    except XGBoostError:
        response.status_code = status.HTTP_500_INTERNAL_SERVER_ERROR
        return { "error": ERROR_MODEL_NOT_FOUND}

    except Exception as e:
        if response.status_code == status.HTTP_200_OK:  # If no error code has been set
            response.status_code = status.HTTP_500_INTERNAL_SERVER_ERROR

        return { "error": str(e)}

    if error_msg != "":
        response = {
            "prediction": prediction_model,
            "isOrder": is_order,
            "error": error_msg}

    return response

@app.post("/latest/predict", 
          response_description="Prediccion de volumen para un conjunto de datos.",
          status_code=status.HTTP_200_OK)
async def predict(response: Response, data: PredictionRequest, request: Request):
    '''
    Endpoint que devuelve la prediccion de volumen para un conjunto de datos.

    Parameters
    ----------
    order : str
    data : dict -> { data: [{"sku": "123", "product_length": 34.3, "product_width": 12.3, "product_height": 12.3, "quantity": 2}, {...}] }
    conversion_rate : float
    slope_correction : float

    Returns
    -------
    { "prediction": float, "isOrder": bool}
    '''

    error_msg = "" # Error message to be returned in the response
    is_order = True if data.order else False
    client_ip = extract_ip_caller(request = request)

    # Validaciones
    validate_model_version(version = version)
    validate_entry(data = data)

    # Getters
    conversion_rate, slope_correction = get_conversion_rate_and_slope_correction(data = data, isOrder = is_order)
    pesoTotalProductos = get_total_weight(data = data)
  
    try:
        model = Model.Model(version)
        df = process_input_data(data.data)

        # Check that quantity is greater than 0 for all products
        if not (df["quantity"] > 0).all():
            response.status_code = status.HTTP_400_BAD_REQUEST
            raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=ERROR_QUANTITY_BIGGER_THAN_ZERO)

        # Extract features
        features = extractFeaturesByVersion(df, version)
        
        # Calculate prediction
        prediction_model = (model.predict(features)[0].item() / 1_000_000) * slope_correction

        # Base response
        response = {
            "prediction": prediction_model,
            "pesoVolumetricoEnKg": prediction_model/conversion_rate,
            "pesoTotalProductos": pesoTotalProductos,
            "isOrder": is_order
        }


    except XGBoostError:
        response.status_code = status.HTTP_500_INTERNAL_SERVER_ERROR
        return { "error": ERROR_MODEL_NOT_FOUND}

    except Exception as e:
        if response.status_code == status.HTTP_200_OK:  # If no error code has been set
            response.status_code = status.HTTP_500_INTERNAL_SERVER_ERROR

        return { "error": str(e)}

    if error_msg != "":
        response = {
            "prediction": prediction_model,
            "isOrder": is_order,
            "error": error_msg}

    return response



@app.get("/healthcheck", 
         response_description="Estado de salud de la API.",
         status_code=status.HTTP_200_OK)
async def perform_health_check():
    return { "status": HEALTHCHECK_MESSAGE_OKAY }




###############################################
################## HELPERS ####################
###############################################

def process_input_data(data: List[PredictionRequestProduct]) -> pd.DataFrame:
    '''
    Procesa los datos de entrada para el endpoint de prediccion.

    Parameters
    ----------
    data : List[PredictionRequestProduct]

    Returns
    -------
    pd.DataFrame -> { "sku": str, "product_length": float, "product_width": float, "product_height": float, "quantity": int }
    '''

    sku_dict = {}

    for product in data:
        if product.get("sku") in sku_dict:
            sku_dict[product.get("sku")]['quantity'] += product.get("quantity")
        else:
            sku_dict[product.get("sku")] = {
                'product_length': product.get("product_length"),
                'product_width': product.get("product_width"),
                'product_height': product.get("product_height"),
                'product_weight': product.get("product_weight"),
                'quantity': product.get("quantity"),
            }

    # Creamos una lista con los productos únicos y la cantidad total para cada SKU
    products = list(sku_dict.values())

    # Convertimos la lista de productos en un DataFrame
    df = pd.DataFrame(products)

    return df

def get_total_weight(data):
    total_weight = 0.0

    for product in data.data:
        total_weight += (product.get("product_weight") * product.get("quantity"))
    
    return total_weight

def get_conversion_rate_and_slope_correction(data, isOrder = False):

    conversion_rate = float(DEFAULT_CONVERSION_RATE)
    slope_correction = DEFAULT_CONVERSION_RATE

    if isOrder:
        if data.conversion_rate != None and not isinstance(data.conversion_rate, (float, int)):
            raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=ERROR_CONVERSION_RATE_TYPE)
        if data.conversion_rate != None:
            conversion_rate = data.conversion_rate 

    if data.slope_correction != None and isinstance(data.slope_correction, (int, float)) :
        slope_correction = data.slope_correction

    return conversion_rate, slope_correction

