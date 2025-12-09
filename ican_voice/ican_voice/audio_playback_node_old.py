#!/usr/bin/env python3
"""
Audio Playback Node - Multi-rate audio player
Supports both Kokoro (24kHz) and Piper (22050Hz) TTS outputs
Uses PulseAudio for reliable playback
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, String
import threading
from queue import Queue
import os
import numpy as np
import subprocess
import tempfile
import wave



class AudioPlayer(Node):
    def __init__(self):
        super().__init__('audio_player')
        
        # Parameters
        self.declare_parameter('sample_rate', 44100)  # A2DP Bluetooth standard rate
        self.declare_parameter('channels', 2)  # Stereo for A2DP
        self.declare_parameter('use_pulse', True)  # Use PulseAudio
        self.declare_parameter('device_name', 'bluez_sink.41_42_F4_7A_02_A6.a2dp_sink')
        
        self.sample_rate = self.get_parameter('sample_rate').value
        self.channels = self.get_parameter('channels').value
        self.use_pulse = self.get_parameter('use_pulse').value
        self.device_name = self.get_parameter('device_name').value
        
        # TTS sample rate (may differ from output device rate)
        self.tts_sample_rate = 24000  # Default TTS rate
        
        # Ensure A2DP profile is active
        try:
            subprocess.run(['pactl', 'set-card-profile', 'bluez_card.41_42_F4_7A_02_A6', 'a2dp_sink'],
                         check=False, capture_output=True)
        except:
            pass
        
        # Audio queue for smooth playback
        self.audio_queue = Queue(maxsize=100)
        
        # PyAudio setup (suppress ALSA warnings during initialization)
        with suppress_alsa_errors():
            self.p = pyaudio.PyAudio()
        self.stream = None
        self.init_audio_stream()
        
        # Subscribers
        self.sub_audio = self.create_subscription(
            UInt8MultiArray,
            'audio/tts_stream',
            self.audio_callback,
            10
        )
        
        # Optional: Listen to TTS status for auto rate detection
        if self.auto_rate_detect:
            self.sub_status = self.create_subscription(
                String,
                'tts/status',
                self.status_callback,
                10
            )
        
        # Playback thread
        self.running = True
        self.playback_thread = threading.Thread(target=self.playback_worker, daemon=True)
        self.playback_thread.start()
        
        self.get_logger().info(f'Audio Player Ready @ {self.sample_rate}Hz. Listening on /audio/tts_stream...')
    
    def init_audio_stream(self):
        """Initialize or reinitialize audio stream"""
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
        
        stream_kwargs = {
            'format': pyaudio.paInt16,
            'channels': self.channels,
            'rate': self.sample_rate,
            'output': True,
            'frames_per_buffer': self.buffer_size
        }
        
        # Set specific device if requested
        if self.device_index >= 0:
            stream_kwargs['output_device_index'] = self.device_index
            device_info = self.p.get_device_info_by_index(self.device_index)
            self.get_logger().info(f'Using audio device {self.device_index}: {device_info["name"]}')
        
        self.stream = self.p.open(**stream_kwargs)
        self.get_logger().info(f'Audio stream opened @ {self.sample_rate}Hz')
    
    def status_callback(self, msg):
        """Handle TTS status for auto sample rate detection"""
        status = msg.data
        
        new_rate = None
        if 'kokoro' in status.lower():
            new_rate = 24000
        elif 'piper' in status.lower():
            new_rate = 22050
        
        if new_rate and new_rate != self.tts_sample_rate:
            self.tts_sample_rate = new_rate
            self.get_logger().info(f'TTS sample rate updated to: {new_rate}Hz')
    
    def resample_audio(self, audio_data, from_rate, to_rate):
        """Resample audio data from one rate to another"""
        if from_rate == to_rate:
            return audio_data
        
        # Convert bytes to int16 numpy array
        audio_array = np.frombuffer(audio_data, dtype=np.int16)
        
        # Calculate resampling ratio
        ratio = to_rate / from_rate
        new_length = int(len(audio_array) * ratio)
        
        # Simple linear interpolation resampling
        indices = np.linspace(0, len(audio_array) - 1, new_length)
        resampled = np.interp(indices, np.arange(len(audio_array)), audio_array)
        
        # Convert back to int16 bytes
        return resampled.astype(np.int16).tobytes()
    
    def audio_callback(self, msg):
        """Queue incoming audio data"""
        try:
            audio_data = bytes(msg.data)
            self.audio_queue.put(audio_data, timeout=0.1)
        except:
            self.get_logger().warn('Audio queue full, dropping packet')
    
    def playback_worker(self):
        """Background thread for continuous playback"""
        while self.running:
            try:
                audio_data = self.audio_queue.get(timeout=1.0)
                if audio_data and self.stream:
                    # Resample if TTS rate differs from output rate
                    if self.tts_sample_rate != self.sample_rate:
                        audio_data = self.resample_audio(audio_data, self.tts_sample_rate, self.sample_rate)
                    
                    # Write audio in smaller chunks to avoid buffer issues
                    chunk_size = self.buffer_size * 2  # 2 bytes per int16 sample
                    for i in range(0, len(audio_data), chunk_size):
                        chunk = audio_data[i:i+chunk_size]
                        self.stream.write(chunk, len(chunk) // 2)  # num_frames = bytes / 2
            except Exception as e:
                if 'audio_data' in locals() and audio_data:  # Only log if we had data
                    self.get_logger().warn(f'Playback error: {e}')
                continue
    
    def destroy_node(self):
        """Cleanup resources"""
        self.running = False
        
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
        
        self.p.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AudioPlayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()