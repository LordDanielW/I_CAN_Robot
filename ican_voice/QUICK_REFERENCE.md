# Voice Node - Quick Reference Card

## One-Line Commands

### Run the Node
```bash
ros2 launch ican_voice voice_node.launch.py
```

### Listen to Speech Output
```bash
ros2 topic echo /human/speech
```

### Simulate Speech (No Mic Needed)
```bash
ros2 topic pub /human/speech std_msgs/msg/String "data: 'hello robot'" --once
```

### Test with Different Model
```bash
ros2 launch ican_voice voice_node.launch.py model_size:=tiny.en
```

### Force CPU Usage
```bash
ros2 launch ican_voice voice_node.launch.py device:=cpu
```

### Check Installation
```bash
cd ~/ros2_ws/src/I_CAN_Robot/ican_voice && ./test_voice_node.sh
```

## Integration Examples

### With Brain Node (Future)
```bash
# Terminal 1: Voice
ros2 launch ican_voice voice_node.launch.py

# Terminal 2: Brain (to be created)
ros2 run ican_orchestrator brain_node

# Terminal 3: Monitor
ros2 topic echo /human/speech
```

### Testing Pipeline Without Audio
```bash
# Simulate user saying: "Robot, walk forward"
ros2 topic pub /human/speech std_msgs/msg/String "data: 'robot walk forward'" --once

# The brain node will receive this and act on it
```

## Common Issues

| Problem | Solution |
|---------|----------|
| No audio devices | Expected in WSL2, see WSL2_AUDIO_SETUP.md |
| Model download slow | First run only, models cached to ~/.cache/huggingface |
| High CPU usage | Use smaller model: `model_size:=tiny.en` |
| False triggers | Increase energy_threshold: `energy_threshold:=800.0` |
| Missing words | Decrease silence_duration: `silence_duration:=1.0` |

## Environment Setup (Always Run First)

```bash
source ~/ros2_mcp_env/bin/activate
cd ~/ros2_ws
source install/setup.bash
```

## Key Topic

- **Published**: `/human/speech` (std_msgs/String) - Your spoken words as text

---
**Ready**: Yes ✅ | **Tested**: Yes ✅ | **Documented**: Yes ✅
