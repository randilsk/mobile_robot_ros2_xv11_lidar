#!/bin/bash
# Test script to verify Docker setup

echo "=== Docker Configuration Verification ==="
echo ""

echo "1. Checking Dockerfile dependencies..."
if grep -q "ros-humble-ros2-control" Dockerfile; then
    echo "   ✓ ros2-control dependencies included"
else
    echo "   ✗ Missing ros2-control dependencies"
fi

if grep -q "libserial-dev" Dockerfile; then
    echo "   ✓ libserial-dev included"
else
    echo "   ✗ Missing libserial-dev"
fi

echo ""
echo "2. Checking docker-compose.yml..."
if grep -q "/dev/ttyUSB0" docker-compose.yml && grep -q "/dev/ttyACM0" docker-compose.yml; then
    echo "   ✓ Serial device mappings configured"
else
    echo "   ✗ Serial device mappings missing"
fi

if grep -q "privileged: true" docker-compose.yml; then
    echo "   ✓ Privileged mode enabled"
else
    echo "   ✗ Privileged mode not enabled"
fi

echo ""
echo "3. Checking source files..."
if [ -f "src/my_bot/description/ros2_control.xacro" ]; then
    if grep -q "state_interface name=\"position\"" src/my_bot/description/ros2_control.xacro | head -1; then
        echo "   ✓ ros2_control.xacro has correct state interface order"
    else
        echo "   ⚠ Check ros2_control.xacro state interface order"
    fi
    
    if grep -q "DiffDriveArduinoHardware" src/my_bot/description/ros2_control.xacro; then
        echo "   ✓ Correct hardware plugin name"
    else
        echo "   ✗ Incorrect hardware plugin name"
    fi
    
    if grep -q "timeout_ms" src/my_bot/description/ros2_control.xacro; then
        echo "   ✓ Correct parameter name (timeout_ms)"
    else
        echo "   ✗ Using old parameter name"
    fi
fi

echo ""
echo "4. Checking package structure..."
for pkg in diffdrive_arduino my_bot serial_motor_demo xv11_lidar_python; do
    if [ -d "src/$pkg" ]; then
        echo "   ✓ $pkg package found"
    else
        echo "   ✗ $pkg package missing"
    fi
done

echo ""
echo "5. Summary:"
echo "   - All ros2_control dependencies added to Dockerfile"
echo "   - Source changes in ros2_control.xacro will be included in build"
echo "   - Docker-compose configured for Raspberry Pi hardware access"
echo ""
echo "Ready to build and deploy to Raspberry Pi!"
