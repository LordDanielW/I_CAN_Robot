#!/bin/bash
# Test Ollama & Qwen without ROS2
# Quick test script to verify Ollama is running and model is available

echo "=========================================="
echo "Ollama & Qwen Test (No ROS2)"
echo "=========================================="
echo ""

# Check if Ollama is installed
if ! command -v ollama &> /dev/null; then
    echo "❌ Ollama not found!"
    echo "Install: curl -fsSL https://ollama.com/install.sh | sh"
    exit 1
fi

echo "✓ Ollama installed"
echo ""

# Check if Ollama service is running
if ! pgrep -x "ollama" > /dev/null; then
    echo "⚠ Ollama service not running"
    echo "Starting Ollama in background..."
    ollama serve > /dev/null 2>&1 &
    sleep 3
fi

echo "✓ Ollama service running"
echo ""

# Check if qwen2.5:7b model is available
echo "Checking for qwen2.5:7b model..."
if ollama list | grep -q "qwen2.5:7b"; then
    echo "✓ Model qwen2.5:7b available"
else
    echo "❌ Model qwen2.5:7b not found!"
    echo "Pulling model (this may take a while)..."
    ollama pull qwen2.5:7b
fi

echo ""
echo "=========================================="
echo "Testing Ollama with simple query..."
echo "=========================================="
echo ""

# Test query
PROMPT="Say 'Hello from Ollama!' in one short sentence."

echo "Prompt: $PROMPT"
echo ""
echo "Response:"
echo "---"
ollama run qwen2.5:7b "$PROMPT" 2>/dev/null
echo "---"
echo ""

echo "=========================================="
echo "✓ Ollama test complete!"
echo "=========================================="
echo ""
echo "To use Ollama in ROS2:"
echo "  ros2 run ican_brain ollama_tool_node"
echo "  ros2 run ican_brain prompt_node"
echo ""
echo "To test manually:"
echo "  ollama run qwen2.5:7b"
echo "=========================================="
