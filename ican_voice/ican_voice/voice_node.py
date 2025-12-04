#!/usr/bin/env python3
"""
Voice Node - The Ears of the I_CAN Robot

This node captures audio from the microphone, transcribes it using faster-whisper,
and publishes the transcribed text to the /human/speech topic.

Author: I_CAN_Robot Team
License: Apache-2.0
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyaudio
import numpy as np
import threading
import queue
from faster_whisper import WhisperModel
import time


class VoiceNode(Node):
    """
    ROS 2 Node that listens to the microphone and transcribes speech.
    
    Uses PyAudio for audio capture and faster-whisper for transcription.
    Audio processing runs in a separate thread to avoid blocking the ROS 2 spin loop.
    """
    
    def __init__(self):
        super().__init__('voice_node')
        
        # Declare parameters
        self.declare_parameter('model_size', 'base.en')
        self.declare_parameter('device', 'auto')  # 'auto', 'cuda', or 'cpu'
        self.declare_parameter('compute_type', 'int8')  # 'int8', 'float16', 'float32'
        self.declare_parameter('mic_device_index', -1)  # -1 for default
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('chunk_size', 1024)
        self.declare_parameter('channels', 1)
        self.declare_parameter('energy_threshold', 500.0)  # RMS energy threshold for speech detection
        self.declare_parameter('silence_duration', 1.5)  # Seconds of silence to end phrase
        self.declare_parameter('min_phrase_duration', 0.5)  # Minimum phrase length in seconds
        
        # Get parameters
        self.model_size = self.get_parameter('model_size').value
        self.device = self.get_parameter('device').value
        self.compute_type = self.get_parameter('compute_type').value
        self.mic_device_index = self.get_parameter('mic_device_index').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.chunk_size = self.get_parameter('chunk_size').value
        self.channels = self.get_parameter('channels').value
        self.energy_threshold = self.get_parameter('energy_threshold').value
        self.silence_duration = self.get_parameter('silence_duration').value
        self.min_phrase_duration = self.get_parameter('min_phrase_duration').value
        
        # Create publisher
        self.speech_pub = self.create_publisher(String, '/human/speech', 10)
        
        # Initialize Whisper model
        self.get_logger().info(f'Loading Whisper model: {self.model_size} on {self.device}...')
        try:
            self.model = WhisperModel(
                self.model_size,
                device=self.device,
                compute_type=self.compute_type
            )
            self.get_logger().info('Whisper model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load Whisper model: {e}')
            raise
        
        # Audio queue for thread communication
        self.audio_queue = queue.Queue()
        
        # Control flags
        self.running = True
        self.is_listening = False
        
        # Start audio thread
        self.audio_thread = threading.Thread(target=self._audio_loop, daemon=True)
        self.audio_thread.start()
        
        self.get_logger().info('Voice node initialized and listening...')
    
    def _audio_loop(self):
        """
        Main audio capture and transcription loop.
        Runs in a separate thread to avoid blocking ROS 2.
        """
        # Initialize PyAudio
        audio = pyaudio.PyAudio()
        
        try:
            # Get mic device index
            if self.mic_device_index == -1:
                device_index = None  # Use default
                self.get_logger().info('Using default microphone')
            else:
                device_index = self.mic_device_index
                self.get_logger().info(f'Using microphone device index: {device_index}')
            
            # Open audio stream
            stream = audio.open(
                format=pyaudio.paInt16,
                channels=self.channels,
                rate=self.sample_rate,
                input=True,
                input_device_index=device_index,
                frames_per_buffer=self.chunk_size
            )
            
            self.get_logger().info('Audio stream opened')
            
            # Audio buffer for accumulating speech
            audio_buffer = []
            silence_start = None
            speech_started = False
            
            while self.running:
                try:
                    # Read audio chunk
                    data = stream.read(self.chunk_size, exception_on_overflow=False)
                    audio_chunk = np.frombuffer(data, dtype=np.int16)
                    
                    # Calculate RMS energy
                    rms = np.sqrt(np.mean(audio_chunk**2))
                    
                    # Check if speech is detected
                    is_speech = rms > self.energy_threshold
                    
                    if is_speech:
                        # Speech detected
                        if not speech_started:
                            self.get_logger().debug('Speech started')
                            speech_started = True
                            audio_buffer = []
                        
                        audio_buffer.append(audio_chunk)
                        silence_start = None  # Reset silence timer
                        
                    elif speech_started:
                        # Silence detected after speech
                        audio_buffer.append(audio_chunk)  # Keep recording for a bit
                        
                        if silence_start is None:
                            silence_start = time.time()
                        
                        # Check if silence duration exceeded
                        if time.time() - silence_start >= self.silence_duration:
                            # End of phrase detected
                            phrase_duration = len(audio_buffer) * self.chunk_size / self.sample_rate
                            
                            if phrase_duration >= self.min_phrase_duration:
                                self.get_logger().debug(f'Phrase complete ({phrase_duration:.2f}s), transcribing...')
                                
                                # Convert buffer to audio array
                                audio_data = np.concatenate(audio_buffer)
                                
                                # Transcribe in this thread
                                self._transcribe_and_publish(audio_data)
                            else:
                                self.get_logger().debug(f'Phrase too short ({phrase_duration:.2f}s), ignoring')
                            
                            # Reset state
                            audio_buffer = []
                            speech_started = False
                            silence_start = None
                
                except Exception as e:
                    self.get_logger().error(f'Error in audio loop: {e}')
                    time.sleep(0.1)  # Prevent tight loop on error
            
            # Clean up
            stream.stop_stream()
            stream.close()
            
        except Exception as e:
            self.get_logger().error(f'Fatal error in audio thread: {e}')
        finally:
            audio.terminate()
            self.get_logger().info('Audio thread terminated')
    
    def _transcribe_and_publish(self, audio_data):
        """
        Transcribe audio data and publish to ROS topic.
        
        Args:
            audio_data: numpy array of int16 audio samples
        """
        try:
            # Convert to float32 and normalize
            audio_float = audio_data.astype(np.float32) / 32768.0
            
            # Transcribe using Whisper
            segments, info = self.model.transcribe(
                audio_float,
                language='en',
                beam_size=1,  # Faster, less accurate
                vad_filter=True,  # Use built-in VAD
                vad_parameters=dict(
                    min_silence_duration_ms=500
                )
            )
            
            # Collect all segments
            transcription = ' '.join([segment.text.strip() for segment in segments])
            
            if transcription:
                self.get_logger().info(f'Transcribed: "{transcription}"')
                
                # Publish to ROS topic
                msg = String()
                msg.data = transcription
                self.speech_pub.publish(msg)
            else:
                self.get_logger().debug('Empty transcription, not publishing')
                
        except Exception as e:
            self.get_logger().error(f'Transcription error: {e}')
    
    def destroy_node(self):
        """Clean shutdown of the node."""
        self.get_logger().info('Shutting down voice node...')
        self.running = False
        
        # Wait for audio thread to finish
        if self.audio_thread.is_alive():
            self.audio_thread.join(timeout=2.0)
        
        super().destroy_node()


def main(args=None):
    """Main entry point for the voice node."""
    rclpy.init(args=args)
    
    try:
        node = VoiceNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in voice node: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
