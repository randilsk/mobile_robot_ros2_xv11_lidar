#!/bin/bash
# Script to deploy Docker image to Raspberry Pi

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== ROS2 Robot Docker Deployment Script ===${NC}"
echo ""

# Check if running on Raspberry Pi
if [ -f /proc/device-tree/model ]; then
    MODEL=$(cat /proc/device-tree/model)
    echo -e "${YELLOW}Detected: $MODEL${NC}"
fi

echo ""
echo -e "${GREEN}Step 1: Building Docker image...${NC}"
docker-compose build

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Docker image built successfully${NC}"
else
    echo -e "${RED}✗ Docker build failed${NC}"
    exit 1
fi

echo ""
echo -e "${GREEN}Step 2: Checking serial devices...${NC}"
echo "Available serial ports:"
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "No serial devices found"

echo ""
echo -e "${YELLOW}Note: Update the device paths in docker-compose.yml if needed${NC}"
echo ""

echo -e "${GREEN}Step 3: Starting container...${NC}"
docker-compose up -d

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Container started successfully${NC}"
    echo ""
    echo "To access the container:"
    echo "  docker-compose exec ros2 bash"
    echo ""
    echo "To view logs:"
    echo "  docker-compose logs -f"
    echo ""
    echo "To stop the container:"
    echo "  docker-compose down"
else
    echo -e "${RED}✗ Failed to start container${NC}"
    exit 1
fi
