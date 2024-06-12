Contenidos del fichero: 
    - apln_ta.ipynb: Jupyter Notebook con el código de la práctica.
    - /Corpus: Carpeta con los corpus utilizados en la práctica.
        - bilingue.en y bilingue.es: Corpus bilingüe de 5000 frases hechos a mano a partir de los corpus completos
        - Dev.en y Dev.es: Corpus de desarrollo de 2000 frases hechos a mano a partir de los corpus completos
        - Test.en y Test.es: Corpus de test de 2000 frases hechos a mano a partir de los corpus completos
        - MonolingueEN.txt y MonolingueES.txt: Corpus monolingües de 5000 frases hechos a mano a partir de los corpus completos utilizados para el entrenamiento de los modelos.

Explicación del Jupyter Notebook:
A lo largo de todo el cuadernillo he ido poniendo comentarios explicativos de cada paso que he ido realizando.
Las funciones más relevantes del notebook tienen además una explicación más detallada de los parámetros que reciben y de lo que hacen.

Partes del cuadernillo:

Setup:
1. Instalación de paquetes necesarios
2. Imports necesarios y montaje de Google Drive

Creación de los corpus:
3. En esta parte del código se ve como he realizado las particiones de los datasets a mano a partir de los corpus completos descargados de OPUS. No es necesario que la ejecutes,
    ya que los corpus ya están creados y guardados en la carpeta Corpus y se los pones en la carpeta de apln de tu Google Drive debería funcionar sin problemas.

Creación de los modelos:
4. En esta parte del notebook lo que hago es descargar los tokenizadores y data_collators que necesito, así como definir los hiperparámetros que voy a utilizar para entrenar los modelos.

5. Creación de funciones auxiliares para obtener los dataset

6. Funciones para las métricas de los modelos, he tenido que definir 2 El por que de la realización de las 2 funciones de evaluación en vez de una única es que no podía parar el
 tokenizer por parámetro y al ser diferente para los 2 modelos, pues tuve que repetir código

Funciones auxiliares para Iterative Back Translation:
7. Función para combinar los datasets bilingues con las traducciones de monolingue
8. Dos funciones para generar las traducciones necesarias para el aumento de datos (las funciones hacen lo mismo pero he visto que el tokenizer va más rápido que con la pipeline)
9. Función de entrenamiento de los modelos y que guarda los modelos en para poder cargarlos en el futuro, además de decargarlos de Hugging Face

10. Creación y carga de los datasets en memoria

Iterative Back Translation:
11. El enfoque he seguido es desde mi punto de vista el más funcional de todos, ya que me he encargado de realizar funciones a medida para cada paso del proceso, de esta manera
    he podido controlar mejor el proceso y realizar los cambios necesarios para que funcione correctamente. Y así no he tenido que repetir mucho código.
    He seguido el algoritmo planteado en las diapositivas de clase y he ido realizando los pasos uno a uno, guardando los modelos en cada iteración y cargándolos en la siguiente.
    También he tenido cuidado en el tema de sustituir los datasets de entrenamiento por los aumentados en cada iteración.
    Tampoco hay mucho que decir porque creo que me ha quedado un código bastante descriptivo y claro.
    
    Quiero recalcar que no he podido entrenar los modelos con todos los datos que se pedian en la práctica, ya que el tiempo de entrenamiento era muy elevado y no he podido realizarlo al
    quedarme sin tiempo de GPU en Google Colab. Aún así, he realizado el proceso completo con los datos que he podido y he comprobado que el proceso funciona correctamente. Y si tuviera 
    más tiempo de GPU podría realizar el proceso completo sin problemas.

12. Evaluación de los modelos y comparación de los resultados obtenidos en cada iteración.


