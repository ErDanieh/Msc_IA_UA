# Documentación de la Clase "Animal"
Información General
- Nombre del Modelo: Animal
- Basado en: Plantilla interna vacía
- Autor: Daniel
- Descripción de la Clase
Esta clase, denominada "Animal", está diseñada para ser la clase base de diferentes animales en un modelo de simulación. Importa y se basa en el modelo "Map" definido anteriormente.

## Estructura y Características
- Nombre: animal
- Habilidades: [moving]
- Control: simple_bdi (sistema de creencias, deseos e intenciones simplificado).


### Atributos
- age: Edad del animal.
- stepsElapsed: Pasos transcurridos, usado para el seguimiento del tiempo.
- position: Posición inicial en la cuadrícula (grille).
- speedMultiplier: Multiplicador de velocidad.
- juniorAge, matureAge, oldAge: Umbrales de edad para diferentes etapas de vida.
- sex: Género del animal (0 para macho, 1 para hembra).
- wantReproduce: Indica si el animal desea reproducirse.
- gestationTime: Tiempo de gestación.
- gestationTimeElapsed: Tiempo transcurrido de gestación.
- pregnant: Indica si el animal está embarazado.
- hunger, thirst: Niveles de hambre y sed.
- waterLocation: Ubicación del agua más cercana.

`Inicialización (init)`
- Inicializa la ubicación, deseos, sexo, y edad del animal.
- Establece los niveles de hambre y sed.

#### Globales
- `totalNaturalDeathPrey:` Contador de muertes naturales de presas.
- `totalPreyBirths:` Contador de nacimientos de presas.

### Planes y Reflejos
- `Reflejo updateHungerAndThirst:` Actualiza los niveles de hambre y sed y simula la muerte por inanición o deshidratación.
- `Reflejo addStep:` Incrementa stepsElapsed y gestiona el embarazo.
- `Reflejo addYear:` Incrementa la edad del animal anualmente y gestiona su deseo de reproducción.
- `Reflejo giveBirth:` Simula el parto si se cumplen las condiciones de gestación.
- `Reflejo death:` Gestiona la muerte natural y espontánea basada en la edad y eventos aleatorios.
- `Acción choose_target_water:` Elige un objetivo de agua más cercano.
- `Acción reproduction:` Gestiona la reproducción y el nacimiento de nuevos individuos.
- `Percepción de Agua:` Bebe agua cuando está cerca y tiene sed.
- `Percepción de Otros Animales:` Gestiona la interacción reproductiva con otros animales de la misma especie.
- `Plan goDrink:` Gestiona el movimiento hacia el agua.
- `Plan wander:` Comportamiento errático de movimiento.


### Observaciones y Consideraciones
`Comportamiento de Movimiento:` La función wander proporciona un movimiento errático, adecuado para simular el comportamiento animal.

`Mortalidad y Reproducción:` La clase incluye mecanismos para la muerte espontánea y la reproducción, aunque el detalle de la reproducción aún no está implementado.