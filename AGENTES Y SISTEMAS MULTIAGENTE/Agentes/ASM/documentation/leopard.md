# Documentación de la Clase "leopard"
Información General
- Nombre del Modelo: leopard
- Basado en: Plantilla interna vacía
- Autor: Carlos
- Descripción de la Clase

Esta clase, denominada "leopard", representa uno de los principales depredadores.

## Estructura y Características
- Nombre: leopard
- Habilidades: [moving]
- Control: simple_bdi (sistema de creencias, deseos e intenciones simplificado).

### Atributos
- Todos los heredados de la clase Animal
- wanderSpeedMultiplier:  Multiplicador de velocidad de movimiento
- huntingSpeedMultiplier: Multiplicador de velocidad de caza;
- target_prey: Agente presa

`Inicialización (init)`
Establece la ubicación.

#### Globales
- `totalHuntedPrey`: Contador de presas cazadas.

### Percepciones del entorno
`Percepción de la presa:`
Percibe si el leopard se encuentra sobre la presa y se la come.

### Reglas
`Regla de deseo de cazar:`
Genera un nuevo deseo de cazar a partir de la creencia del deseo de cazar

### Planes, Reflejos y Acciones
`Plan wander:`
Define un comportamiento de movimiento errante.

`Plan hunt:`
Define un comportamiento de movimiento hacia la presa a cazar.

`Reflex check_hunger:`
Si el leopard comienza a tener hambre, busca una presa y, si la encuentra, añade la creencia correspondiente para cazar.

`Action choose_target_prey:`
Devuelve una de las presas que se encuentran a una determinada distancia del leopard.

### Observaciones y Consideraciones
`Comportamiento de Movimiento:` La función wander proporciona un movimiento errático, adecuado para simular el comportamiento animal.

