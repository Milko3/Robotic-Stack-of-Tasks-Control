#!/bin/bash
# Startar containern. Bygger imagen automatiskt om den saknas.
MSYS_NO_PATHCONV=1 docker compose run --name sot_ros2 --rm sot bash
