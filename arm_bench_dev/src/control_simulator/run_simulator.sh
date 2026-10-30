#!/bin/bash
# Launcher script for Control Simulator GUI

echo "=========================================="
echo "Control Algorithm Simulator & Tuner"
echo "=========================================="
echo ""

# Check if Python is available
if ! command -v python3 &> /dev/null; then
    echo "ERROR: Python 3 not found. Please install Python 3.8 or later."
    exit 1
fi

# Check if required packages are installed
echo "Checking dependencies..."
python3 -c "import PyQt5; import numpy; import scipy; import matplotlib" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "Installing required packages..."
    pip3 install -r requirements.txt
fi

# Check MATLAB engine
echo "Checking MATLAB engine..."
python3 -c "import matlab.engine" 2>/dev/null
if [ $? -ne 0 ]; then
    echo ""
    echo "WARNING: MATLAB engine for Python not found!"
    echo "Please install it from your MATLAB installation:"
    echo "  cd /usr/local/MATLAB/R20XXx/extern/engines/python"
    echo "  python3 setup.py install"
    echo ""
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo ""
echo "Starting Control Simulator GUI..."
echo "MATLAB engine will start automatically (may take 10-15 seconds)"
echo ""

# Run the GUI
python3 control_simulator_gui.py

echo ""
echo "Control Simulator closed."
