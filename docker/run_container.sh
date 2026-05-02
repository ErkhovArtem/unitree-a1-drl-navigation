#!/bin/bash

# Allow local Docker clients to use host X11
xhost +local:docker

echo "Starting ros_melodic_container (detached)..."
docker compose up -d

echo "Container is up. Next steps:"
echo "  1. Attach with VS Code Docker extension, or"
echo "  2. Shell: ./attach_container.sh"