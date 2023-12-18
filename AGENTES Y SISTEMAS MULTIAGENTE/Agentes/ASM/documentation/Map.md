# Documentación del Modelo "Map"
- Información General
- Nombre del Modelo: Map
- Autor: Daniel
- Descripción del Modelo
Este modelo, denominado "Map",esta diseñado para simular un entorno que incluye elementos de agua en una cuadrícula.

## Estructura del Modelo
### Grid (Grille)
- Descripción: Una cuadrícula de 50x50 celdas.
- Color: Cada celda de la cuadrícula tiene un color verde asignado (#green).

### Species (Especie): Water

- Propósito: Representar elementos de agua en la cuadrícula.
- Inicialización (init):
- Ubicación: Cada instancia de water se coloca inicialmente en una ubicación aleatoria dentro de la cuadrícula.
- Vecinos: Se determinan los vecinos de la celda actual.
- Creación de Agua Adicional:
Se utiliza un bucle para iterar sobre cada vecino.
Se genera un número aleatorio entre 0 y 10.
Si el número es menor que 2, se crea una nueva instancia de water en la ubicación del vecino.

`Visualización`: Cada instancia de water se dibuja como un cuadrado de color azul (#blue) de 4 unidades de tamaño.

`Generación de Números Aleatorios`: El modelo utiliza la función rnd(10) para generar números aleatorios, los cuales influyen en la expansión de la especie water en la cuadrícula.

`Observaciones y Consideraciones`

- Expansión del Agua: La lógica actual permite que la especie water se expanda rápidamente, dependiendo de los números aleatorios generados.
- Visualización: La representación gráfica de water es sencilla, utilizando cuadrados azules para su visualización.

### Experimentos

- `PredatorsAndPreys`: Experimento gráfico que muestra la interacción entre depredadores y presas.
- `PredatorsAndPreysStats`: Experimento que muestra estadísticas de la interacción entre depredadores y presas.
- `PSO`: Experimento que muestra la optimización de parámetros mediante el algoritmo PSO.