###############################################
################## Constants ##################
###############################################

# API Messages
API_MESSAGE_OKAY = "API request successful."
API_MESSAGE_ERROR_XGBOOST = "API request failed. XGBoost error."

# API Codes
API_CODE_OKAY = 100
API_CODE_ERROR_XGBOOST = 200

# Error messages
ERROR_REQUIRED_DATA = "El campo 'data' es obligatorio."
ERROR_REQUIRED_DATA_PRODUCTS_FORMAT = "'data' debe ser una lista de productos."
ERROR_CONVERSION_RATE_TYPE ="El campo 'conversion_rate' debe ser un float o un int." 
ERROR_MODEL_NOT_FOUND = "No se ha encontrado la version del modelo."
ERROR_QUANTITY_BIGGER_THAN_ZERO ="La cantidad debe ser mayor a 0 para todos los productos."
ERROR_DB_MODEL_OBSERVABILITY_ENTRY_CREATION =  "Incapaz de añadir una entrada a API Log o Model Observability (Base de datos)."
ERROR_INVALID_CREDENTIALS = "Invalid authentication credentials"

# Orders status
ORDER_STATUS_PENDING = "pending"
ORDER_STATUS_COMPLETED = "completed"

# Healthcheck
HEALTHCHECK_MESSAGE_OKAY = "OK"
HEALTHCHECK_MESSAGE_ERROR = "ERROR"
