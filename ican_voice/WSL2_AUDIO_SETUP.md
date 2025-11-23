# WSL2 Audio Setup Guide

**Status**: WSL2 does not natively support audio input devices. You have several options to work around this.

## Problem

As you can see from the output, PyAudio cannot find any audio devices in WSL2:
```
Found 0 audio devices
```

## Solutions (Ordered by Difficulty)

### Option 1: Use PulseAudio Server on Windows (Recommended)

Set up a PulseAudio server on Windows and connect WSL2 to it.

#### Step 1: Install PulseAudio on Windows
1. Download PulseAudio for Windows: https://www.freedesktop.org/wiki/Software/PulseAudio/Ports/Windows/Support/
2. Or use chocolatey: `choco install pulseaudio`

#### Step 2: Configure PulseAudio on Windows
Edit `C:\Users\<YourUser>\AppData\Roaming\pulse\default.pa` and add:
```
load-module module-native-protocol-tcp auth-ip-acl=127.0.0.1;172.16.0.0/12
load-module module-esound-protocol-tcp auth-ip-acl=127.0.0.1;172.16.0.0/12
load-module module-waveout sink_name=output source_name=input
```

#### Step 3: Configure WSL2 to use Windows PulseAudio
In WSL2, add to `~/.bashrc`:
```bash
export PULSE_SERVER=tcp:$(grep nameserver /etc/resolv.conf | awk '{print $2}')
```

Then reload: `source ~/.bashrc`

#### Step 4: Test
```bash
python3 -c "import pyaudio; pa = pyaudio.PyAudio(); print(f'Found {pa.get_device_count()} audio devices')"
```

### Option 2: Use WSLg with PulseAudio

If you have Windows 11 with WSLg:

```bash
# Install PulseAudio in WSL
sudo apt-get install pulseaudio

# Start PulseAudio
pulseaudio --start

# Test
pactl info
```

### Option 3: USB Audio Device Passthrough

Attach a USB microphone directly to WSL2:

1. Install usbipd-win on Windows: https://github.com/dorssel/usbipd-win
2. List USB devices in Windows PowerShell (Admin):
   ```powershell
   usbipd list
   ```
3. Attach your USB microphone:
   ```powershell
   usbipd bind --busid <BUSID>
   usbipd attach --wsl --busid <BUSID>
   ```
4. In WSL, check:
   ```bash
   lsusb
   arecord -l
   ```

### Option 4: Develop on Native Linux or Dual Boot

For robotics development with hardware access, consider:
- Dual boot with Ubuntu
- Native Linux installation
- Raspberry Pi or other embedded Linux board

### Option 5: Run Voice Node on Windows, Rest on WSL2 (Hybrid)

Since ROS 2 supports Windows, you could:
1. Run the voice_node on Windows natively
2. Run the brain and behavior nodes on WSL2
3. Connect via ROS 2 network (ROS_DOMAIN_ID)

## Testing Without Audio

For development and testing the pipeline without microphone:

### Create a Test Publisher

```bash
# In one terminal (after sourcing workspace):
ros2 run ican_voice voice_node

# In another terminal, simulate speech:
ros2 topic pub /human/speech std_msgs/msg/String "data: 'Hello robot, walk forward'" --once
```

This lets you test the brain and behavior nodes without needing audio input!

## Current Status

✅ **Software Installed**: All Python packages are installed correctly  
✅ **Package Built**: ican_voice built successfully  
❌ **Audio Device**: No audio devices available in WSL2  

## Next Steps

1. **For Testing**: Use the test publisher method above to continue development
2. **For Production**: Choose one of the audio setup options above
3. **Continue Development**: Move to Node B (The Brain) while audio is being sorted out

The architecture supports testing each node independently, so you can proceed with:
- Node B: The Brain (ican_orchestrator/brain_node.py)
- Node C: The Senses (ican_mcp_server/senses_server.py)  
- Node D: The Behaviors (ican_mcp_server/behavior_server.py)

And connect them together without real audio input for now!
