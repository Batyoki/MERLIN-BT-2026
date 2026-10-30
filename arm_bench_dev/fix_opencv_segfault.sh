#!/bin/bash
# Fix segmentation fault by replacing opencv-python with opencv-python-headless

echo "🔧 Fixing OpenCV/tkinter segmentation fault..."
echo ""
echo "The issue: opencv-python includes Qt backend which conflicts with tkinter"
echo "The fix: Replace with opencv-python-headless (no GUI backend)"
echo ""

# Check if in virtual environment
if [ -z "$VIRTUAL_ENV" ]; then
    echo "⚠️  No virtual environment detected!"
    echo "   Please activate your virtual environment first:"
    echo "   source /home/yash_sai/Yash/Arm/arm-test/bin/activate"
    exit 1
fi

echo "Virtual environment: $VIRTUAL_ENV"
echo ""

# Uninstall opencv-python
echo "1. Uninstalling opencv-python..."
pip uninstall -y opencv-python opencv-contrib-python

# Install opencv-python-headless
echo ""
echo "2. Installing opencv-python-headless..."
pip install opencv-python-headless

echo ""
echo "✅ Fix complete!"
echo ""
echo "Now try: arm-bench start"
