#!/bin/bash

xhost +local:

sudo docker compose up -d

# Check if the container is running
if sudo docker container inspect -f '{{.State.Running}}' robotica_container >/dev/null 2>&1; then
    # Container is running, connect to it
    sudo docker exec -it robotica_container bash
else
    echo "The robotica_container is not running."
fi