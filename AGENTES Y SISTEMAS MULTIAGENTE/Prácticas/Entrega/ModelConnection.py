
import asyncio
from asyncio import Future
from pathlib import Path
from typing import Dict
import re

from gama_client.base_client import GamaBaseClient
from gama_client.command_types import CommandTypes
from gama_client.message_types import MessageTypes

experiment_future: Future
play_future: Future
pause_future: Future
expression_future: Future
step_future: Future
step_back_future: Future
stop_future: Future

modelLocation = "/Users/daniel/Desktop/Master IA/AGENTES Y SISTEMAS MULTIAGENTE/Agentes/CMG/models/robots.gaml"
experimentName = "experimento1"

    
async def message_handler(message: Dict):
    print("received", message)
    if "command" in message:
        if message["command"]["type"] == CommandTypes.Load.value:
            experiment_future.set_result(message)
        elif message["command"]["type"] == CommandTypes.Play.value:
            play_future.set_result(message)
        elif message["command"]["type"] == CommandTypes.Pause.value:
            pause_future.set_result(message)
        elif message["command"]["type"] == CommandTypes.Expression.value:
            expression_future.set_result(message)
        elif message["command"]["type"] == CommandTypes.Step.value:
            step_future.set_result(message)
        elif message["command"]["type"] == CommandTypes.StepBack.value:
            step_back_future.set_result(message)
        elif message["command"]["type"] == CommandTypes.Stop.value:
            stop_future.set_result(message)


def extraer_coordenadas(content):
    # Encuentra todas las instancias de coordenadas dentro de los corchetes
    coordenadas = re.findall(r'\{([^}]*)\}', content)
    # Convierte las cadenas encontradas en tuplas de dos números flotantes (ignorando el tercer valor)
    coordenadas = [tuple(map(float, coord.split(',')[:2])) for coord in coordenadas]
    return coordenadas


async def main():
    global experiment_future
    global play_future
    global pause_future
    global expression_future
    global step_future
    global step_back_future
    global stop_future
    
    server_url = "localhost"
    server_port = 6868
    
    client = GamaBaseClient(server_url, server_port, message_handler)
    
    print("connecting to Gama server")
    await client.connect()
    
    print("initialize a gaml model")
    experiment_future = asyncio.get_running_loop().create_future()
    await client.load(modelLocation, experimentName, True, True, True, True, None)
    gama_response = await experiment_future
    
    
    try:
        experiment_id = gama_response["content"]
    except Exception as e:
        print("error while initializing", gama_response, e)
        return
    
    # Inicializar el experimento
    print("initialization successful, running the model")
    play_future = asyncio.get_running_loop().create_future()
    await client.play(experiment_id)
    gama_response = await play_future
    if gama_response["type"] != MessageTypes.CommandExecutedSuccessfully.value:
        print("error while trying to run the experiment", gama_response)
        return
    
    # Sacamos los nombres de todos los agentes
    expression_future = asyncio.get_running_loop().create_future()
    await client.expression(experiment_id, r"do get_agents_pos;")
    gama_response = await expression_future 
    # Limpiamos la respuesta
    agentsPositions = extraer_coordenadas(gama_response["content"])
    print("Posiciones de los agentes: ", agentsPositions[0])
    


    if gama_response["type"] != MessageTypes.CommandExecutedSuccessfully.value:
        print("error while trying to run the experiment", gama_response)
        return

    
    expression_future = asyncio.get_running_loop().create_future()
    await client.expression(experiment_id, r"agentsAlive")
    gama_response = await expression_future
    print("asking simulation the value of: agentsAlive=",  gama_response["content"])
    
    
    
if __name__ == "__main__":
    asyncio.run(main())