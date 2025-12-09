#!/usr/bin/env python3
"""
Audio Playback Node - FFmpeg-based streaming player
Uses ffplay to stream raw PCM audio data to Bluetooth speaker
This approach works reliably when paplay/PyAudio fail with Bluetooth

Strategy: Buffer incoming audio chunks and pipe them to ffplay as raw PCM
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, String
import threading
from queue import Queue, Empty
import subprocess
import numpy as np
import os
import signal


class AudioPlayerFFmpeg(Node):
    def __init__(self):
        super().__init__('audio_player_ffmpeg')
        
        # Parameters
        self.declare_parameter('sample_rate', 24000)  # Kokoro outputs 24kHz
        self.declare_parameter('channels', 1)  # Mono input from TTS
        self.declare_parameter('auto_rate_detect', True)
        self.declare_parameter('buffer_time', 0.5)  # Buffer before playing (seconds)
        
        self.sample_rate = self.get_parameter('sample_rate').value
        self.channels = self.get_parameter('channels').value
        self.auto_rate_detect = self.get_parameter('auto_rate_detect').value
        self.buffer_time = self.get_parameter('buffer_time').value
        
        # Audio buffer
        self.audio_buffer = bytearray()
        self.buffer_lock = threading.Lock()
        
        # Playback state
        self.ffplay_process = None
        self.running = True
        self.playing = False
        
        # Timing for buffer management
        self.last_audio_time = 0
        self.playback_timeout = 1.0  # Start playing after this silence
        
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
        
        # Playback management thread
        self.playback_thread = threading.Thread(target=self.playback_manager, daemon=True)
        self.playback_thread.start()
        
        self.get_logger().info('🔊 FFmpeg Audio Player Ready')
        self.get_logger().info(f'   Sample rate: {self.sample_rate}Hz, Channels: {self.channels}')
        self.get_logger().info(f'   Listening on /audio/tts_stream...')
    
    def status_callback(self, msg):
        """Handle TTS status for auto sample rate detection"""
        status = msg.data.lower()
        
        new_rate = None
        if 'kokoro' in status:
            new_rate = 24000
        elif 'piper' in status:
            new_rate = 22050
        
        if new_rate and new_rate != self.sample_rate:
            self.sample_rate = new_rate
            self.get_logger().info(f'Sample rate updated to: {new_rate}Hz')
    
    def audio_callback(self, msg):
        """Receive and buffer incoming audio data"""
        audio_data = bytes(msg.data)
        
        if len(audio_data) > 0:
            with self.buffer_lock:
                self.audio_buffer.extend(audio_data)
            
            self.last_audio_time = self.get_clock().now().nanoseconds / 1e9
            
            # Log buffer size periodically
            buffer_samples = len(self.audio_buffer) // 2  # 2 bytes per int16
            buffer_duration = buffer_samples / self.sample_rate
            
            if buffer_duration > 0.1 and int(buffer_duration * 10) % 5 == 0:
                self.get_logger().debug(f'Buffer: {buffer_duration:.2f}s')
    
    def playback_manager(self):
        """Manage playback - wait for enough data, then play"""
        self.get_logger().info('Playback manager started')
        
        while self.running:
            try:
                # Check if we have buffered audio to play
                with self.buffer_lock:
                    buffer_size = len(self.audio_buffer)
                
                # Calculate buffer duration
                buffer_samples = buffer_size // 2
                buffer_duration = buffer_samples / self.sample_rate
                
                current_time = self.get_clock().now().nanoseconds / 1e9
                time_since_last = current_time - self.last_audio_time if self.last_audio_time > 0 else 999
                
                # Start playback if:
                # 1. We have enough buffered data, OR
                # 2. We have some data and haven't received more for a while (stream ended)
                should_play = False
                
                if buffer_duration >= self.buffer_time:
                    should_play = True
                    self.get_logger().info(f'Buffer full ({buffer_duration:.2f}s), starting playback...')
                elif buffer_size > 0 and time_since_last > self.playback_timeout:
                    should_play = True
                    self.get_logger().info(f'Stream ended, playing remaining {buffer_duration:.2f}s...')
                
                if should_play and not self.playing:
                    self.play_buffer()
                
                # Small sleep to prevent busy waiting
                threading.Event().wait(0.05)
                
            except Exception as e:
                self.get_logger().error(f'Playback manager error: {e}')
                threading.Event().wait(0.1)
        
        self.get_logger().info('Playback manager stopped')
    
    def play_buffer(self):
        """Play the buffered audio using ffplay"""
        with self.buffer_lock:
            if len(self.audio_buffer) == 0:
                return
            
            # Get all buffered audio
            audio_data = bytes(self.audio_buffer)
            self.audio_buffer.clear()
        
        self.playing = True
        
        try:
            buffer_samples = len(audio_data) // 2
            duration = buffer_samples / self.sample_rate
            self.get_logger().info(f'▶️  Playing {duration:.2f}s of audio via ffplay...')
            
            # Use ffplay to play raw PCM audio
            # -f s16le: signed 16-bit little-endian
            # -ar: sample rate
            # -ac: channels  
            # -nodisp: no video display
            # -autoexit: exit when done
            # -loglevel quiet: suppress output
            cmd = [
                'ffplay',
                '-f', 's16le',
                '-ar', str(self.sample_rate),
                '-ac', str(self.channels),
                '-nodisp',
                '-autoexit',
                '-loglevel', 'quiet',
                '-i', 'pipe:0'  # Read from stdin
            ]
            
            self.ffplay_process = subprocess.Popen(
                cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            
            # Write audio data to ffplay's stdin
            self.ffplay_process.stdin.write(audio_data)
            self.ffplay_process.stdin.close()
            
            # Wait for playback to complete
            self.ffplay_process.wait()
            
            self.get_logger().info('✓ Playback complete')
            
        except Exception as e:
            self.get_logger().error(f'FFplay error: {e}')
        finally:
            self.playing = False
            self.ffplay_process = None
    
    def destroy_node(self):
        """Cleanup resources"""
        self.running = False
        
        # Kill any running ffplay process
        if self.ffplay_process:
            try:
                self.ffplay_process.terminate()
                self.ffplay_process.wait(timeout=1)
            except:
                self.ffplay_process.kill()
        
        self.playback_thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AudioPlayerFFmpeg()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
