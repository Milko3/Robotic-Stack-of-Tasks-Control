#!/bin/bash
# Bygger imagen (behövs bara första gången, eller om Dockerfile ändras)
set -e
cd "$(dirname "$0")"
docker build -t sot-ros2:humble .
echo "Docker image built successfully."
