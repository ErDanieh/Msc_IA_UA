Feature:

Background:
  * url 'https://model-api.minte.app'
  Given header x-api-key = '1234567890'
Scenario:
  * path 'v1/predict'
  * request { "conversion_rate": 270, "data": [ {"sku": "030820","quantity": 2, "product_width": 140, "product_height": 10, "product_length": 40, "product_weight": 0.154 }, {"sku": "032452","quantity": 10,"product_width": 81.9,"product_height": 120,"product_length": 96.25,"product_weight": 0.37}]}
  * method post
  * status 200
  * def expectedResponse = { prediction: 0.028697310058593748, isOrder: false}
  * match response == expectedResponse

Scenario:
  * path 'v1/predict'
  * request { "conversion_rate": 270, "data": [ {"quantity": 2, "product_width": 140, "product_height": 10, "product_length": 40, "product_weight": 0.154 }, {"sku": "032452","quantity": 10,"product_width": 81.9,"product_height": 120,"product_length": 96.25,"product_weight": 0.37}]}
  * method post
  * status 400
  * def expectedResponse = {"detail":"Faltan los campos ['sku'] en el producto en la posición 0."}
  * match response == expectedResponse

Scenario:

  * path 'v1/predict'
  * request { "conversion_rate": 270, "data": [ { "product_width": 140, "product_height": 10, "product_length": 40, "product_weight": 0.154 }, {"sku": "032452","quantity": 10,"product_width": 81.9,"product_height": 120,"product_length": 96.25,"product_weight": 0.37}]}
  * method post
  * status 400
  * def expectedResponse = {"detail":"Faltan los campos ['sku', 'quantity'] en el producto en la posición 0."}
  * match response == expectedResponse

Scenario:

  * path 'v1/predict'
  * request { "conversion_rate": 270, "data": [ { "product_height": 10, "product_length": 40, "product_weight": 0.154 }, {"sku": "032452","quantity": 10,"product_width": 81.9,"product_height": 120,"product_length": 96.25,"product_weight": 0.37}]}
  * method post
  * status 400
  * def expectedResponse = { "detail":"Faltan los campos ['sku', 'quantity', 'product_width'] en el producto en la posición 0." }
  * match response == expectedResponse


Scenario:

  * path 'v1/predict'
  * request { "conversion_rate": 270, "data": [ { "product_length": 40, "product_weight": 0.154 }, {"sku": "032452","quantity": 10,"product_width": 81.9,"product_height": 120,"product_length": 96.25,"product_weight": 0.37}]}
  * method post
  * status 400
  * def expectedResponse = { "detail":"Faltan los campos ['sku', 'quantity', 'product_height', 'product_width'] en el producto en la posición 0." }
  * match response == expectedResponse

Scenario:

  * path 'v1/predict'
  * request { "conversion_rate": 270, "data": [ {  "product_weight": 0.154 }, {"sku": "032452","quantity": 10,"product_width": 81.9,"product_height": 120,"product_length": 96.25,"product_weight": 0.37}]}
  * method post
  * status 400
  * def expectedResponse = { "detail":"Faltan los campos ['sku', 'quantity', 'product_length', 'product_height', 'product_width'] en el producto en la posición 0." }
  * match response == expectedResponse

Scenario:

  * path 'v1/predict'
  * request { "conversion_rate": 270, "data": [ {  }, {"sku": "032452","quantity": 10,"product_width": 81.9,"product_height": 120,"product_length": 96.25,"product_weight": 0.37}]}
  * method post
  * status 400
  * def expectedResponse = { "detail":"Faltan los campos ['sku', 'quantity', 'product_weight', 'product_length', 'product_height', 'product_width'] en el producto en la posición 0." }
  * match response == expectedResponse


Scenario:

  * path 'v1/predict'
  * request { "conversion_rate": 270, "data": ""}
  * method post
  * status 400
  * def expectedResponse = { "detail":"Error: 'data' debe ser una lista de productos." }
  * match response == expectedResponse


Scenario:

  * path 'v1/predict'
  * request { "conversion_rate": 270, "data": [ {"sku": "032452", "quantity": 2, "product_width": 140, "product_height": 10, "product_length": 40, "product_weight": 0.154 }, {"quantity": 10,"product_width": 81.9,"product_height": 120,"product_length": 96.25,"product_weight": 0.37}]}
  * method post
  * status 400
  * def expectedResponse = {"detail":"Faltan los campos ['sku'] en el producto en la posición 1."}
  * match response == expectedResponse


Scenario:

  * path 'v1/predict'
  * request { "conversion_rate": 270, "data": ""}
  * method post
  * status 400
  * def expectedResponse = {"detail":"Error: 'data' debe ser una lista de productos."}
  * match response == expectedResponse


Scenario:

  * path 'v1.1/predict'
  * request {"order": "Order", "conversion_rate": "" ,"data": [ {"sku": "030820","quantity": 2, "product_width": 140, "product_height": 10, "product_length": 40, "product_weight": 0.154 }, {"sku": "032452","quantity": 10,"product_width": 81.9,"product_height": 120,"product_length": 96.25,"product_weight": 0.37}]}
  * method post
  * status 400
  * def expectedResponse = {"detail":"Error: El campo 'conversion_rate' debe ser un float o un int."}
  * match response == expectedResponse


Scenario:

  * path 'v2/predict'
  * request {"order": "Order", "conversion_rate": 270 ,"data": [ {"sku": "030820","quantity": 2, "product_width": 140, "product_height": 10, "product_length": 40, "product_weight": 0.154 }, {"sku": "032452","quantity": 10,"product_width": 81.9,"product_height": 120,"product_length": 96.25,"product_weight": 0.37}]}
  * method post
  * status 500
  * def expectedResponse = {"detail":"Error: No se ha encontrado la version del modelo."}
  * match response == expectedResponse

Scenario:

  * path 'v1/predict'
  * request { "data": [ {"sku": "030820","quantity": 2, "product_width": 140, "product_height": 10, "product_length": 40, "product_weight": 0.154 }, {"sku": "032452","quantity": 10,"product_width": 81.9,"product_height": 120,"product_length": 96.25,"product_weight": 0.37}]}
  * method post
  * status 200
  * def expectedResponse = { prediction: 0.028697310058593748, isOrder: false}
  * match response == expectedResponse

Scenario:
  * path 'v1/predict'
  * request {"order": "Order" ,"data": [ {"sku": "030820","quantity": 2, "product_width": 140, "product_height": 10, "product_length": 40, "product_weight": 0.154 }, {"sku": "032452","quantity": 10,"product_width": 81.9,"product_height": 120,"product_length": 96.25,"product_weight": 0.37}]}
  * method post
  * status 200
  * def expectedResponse = { prediction: 0.028697310058593748, isOrder: true}
  * match response == expectedResponse


Scenario:
  * path 'v1/predict'
  * request { "order": "123123" ,"conversion_rate": 270, "data": [ {"sku": "030820","quantity": 2, "product_width": 140, "product_height": 10, "product_length": 40, "product_weight": 0.154 }, {"sku": "032452","quantity": 10,"product_width": 81.9,"product_height": 120,"product_length": 96.25,"product_weight": 0.37}]}
  * method post
  * status 200
  * def expectedResponse = { prediction: 0.028697310058593748, isOrder: true}
  * match response == expectedResponse


