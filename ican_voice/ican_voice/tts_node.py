#!/usr/bin/env python3
"""
Kokoro TTS Node - GPU-Accelerated Text-to-Speech
Uses Kokoro-82M for high-quality, low-latency speech synthesis
Optimized for RTX 5070 Ti
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8MultiArray
import torch
import numpy as np
import threading
from queue import Queue
import struct

# Kokoro imports
try:
    from kokoro import KPipeline
    KOKORO_AVAILABLE = True
except ImportError:
    KOKORO_AVAILABLE = False

# Fallback to Piper if Kokoro not available
try:
    import subprocess
    PIPER_AVAILABLE = True
except:
    PIPER_AVAILABLE = False


class KokoroTTSNode(Node):
    def __init__(self):
        super().__init__('tts_streamer')
        
        # Parameters
        self.declare_parameter('voice', 'af_heart')  # Default: American Female "Heart"
        self.declare_parameter('lang_code', 'a')  # 'a'=US English, 'b'=British
        self.declare_parameter('speed', 1.0)
        self.declare_parameter('device', 'cuda')  # 'cuda' or 'cpu'
        self.declare_parameter('sample_rate', 24000)  # Kokoro outputs 24kHz
        self.declare_parameter('chunk_size', 4096)
        self.declare_parameter('use_kokoro', True)  # Fallback to Piper if False
        self.declare_parameter('piper_model', '/home/fire/piper_voices/en_US-ryan-high.onnx')
        
        # Get parameters
        self.voice = self.get_parameter('voice').value
        self.lang_code = self.get_parameter('lang_code').value
        self.speed = self.get_parameter('speed').value
        self.device = self.get_parameter('device').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.chunk_size = self.get_parameter('chunk_size').value
        self.use_kokoro = self.get_parameter('use_kokoro').value
        self.piper_model = self.get_parameter('piper_model').value
        
        # Subscribers
        self.sub_text = self.create_subscription(String, 'tts/speak', self.speak_callback, 10)
        
        # Publishers
        self.pub_audio = self.create_publisher(UInt8MultiArray, 'audio/tts_stream', 10)
        self.pub_status = self.create_publisher(String, 'tts/status', 10)
        
        # Speech queue for non-blocking synthesis
        self.speech_queue = Queue()
        
        # Initialize TTS engine
        self.pipeline = None
        self.init_tts_engine()
        
        # Start worker thread
        self.worker_thread = threading.Thread(target=self.process_queue, daemon=True)
        self.worker_thread.start()
        
        self.get_logger().info('TTS Node Ready. Listening on /tts/speak...')
    
    def init_tts_engine(self):
        """Initialize Kokoro or fallback to Piper"""
        if self.use_kokoro and KOKORO_AVAILABLE:
            try:
                self.get_logger().info(f'Initializing Kokoro TTS on {self.device}...')
                
                # Check CUDA availability
                if self.device == 'cuda' and not torch.cuda.is_available():
                    self.get_logger().warn('CUDA not available, falling back to CPU')
                    self.device = 'cpu'
                
                # Initialize Kokoro pipeline
                self.pipeline = KPipeline(lang_code=self.lang_code)
                
                # Get available voices
                self.get_logger().info(f'Kokoro initialized. Voice: {self.voice}, Speed: {self.speed}x')
                self.publish_status('kokoro_ready')
                return
                
            except Exception as e:
                self.get_logger().error(f'Kokoro init failed: {e}')
        
        # Fallback to Piper
        if PIPER_AVAILABLE:
            self.get_logger().info('Using Piper TTS fallback')
            self.use_kokoro = False
            self.sample_rate = 22050  # Piper medium models use 22050Hz
            self.publish_status('piper_ready')
        else:
            self.get_logger().error('No TTS engine available!')
            self.publish_status('error_no_tts')
    
    def publish_status(self, status: str):
        """Publish TTS status"""
        msg = String()
        msg.data = status
        self.pub_status.publish(msg)
    
    def speak_callback(self, msg):
        """Queue text for synthesis"""
        text = msg.data.strip()
        if text:
            self.speech_queue.put(text)
            self.get_logger().info(f'Queued: "{text[:50]}..."' if len(text) > 50 else f'Queued: "{text}"')
    
    def process_queue(self):
        """Worker thread: process speech queue"""
        while rclpy.ok():
            try:
                text = self.speech_queue.get(timeout=1.0)
                self.synthesize_and_stream(text)
            except:
                continue
    
    def synthesize_and_stream(self, text: str):
        """Generate and stream audio"""
        self.publish_status('synthesizing')
        
        try:
            if self.use_kokoro and self.pipeline:
                self.synthesize_kokoro(text)
            else:
                self.synthesize_piper(text)
            
            self.publish_status('done')
            
        except Exception as e:
            self.get_logger().error(f'Synthesis error: {e}')
            self.publish_status('error')
    
    def synthesize_kokoro(self, text: str):
        """Generate speech using Kokoro"""
        import time
        start = time.time()
        
        # Generate audio with Kokoro
        # Kokoro yields (graphemes, phonemes, audio_chunk) tuples
        audio_chunks = []
        
        for gs, ps, audio in self.pipeline(text, voice=self.voice, speed=self.speed):
            if audio is not None:
                audio_chunks.append(audio)
        
        if not audio_chunks:
            self.get_logger().warn('No audio generated')
            return
        
        # Concatenate all chunks
        audio_data = np.concatenate(audio_chunks)
        
        # Convert to int16 PCM
        audio_int16 = (audio_data * 32767).astype(np.int16)
        audio_bytes = audio_int16.tobytes()
        
        elapsed = time.time() - start
        duration = len(audio_int16) / self.sample_rate
        rtf = elapsed / duration if duration > 0 else 0
        
        self.get_logger().info(f'Generated {len(audio_bytes)} bytes in {elapsed:.2f}s (RTF: {rtf:.2f})')
        
        # Stream in chunks
        self.stream_audio(audio_bytes)
    
    def synthesize_piper(self, text: str):
        """Fallback: Generate speech using Piper"""
        import time
        start = time.time()
        
        cmd = [
            'piper',
            '--model', self.piper_model,
            '--output-raw'
        ]
        
        try:
            process = subprocess.Popen(
                cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            
            audio_bytes, stderr = process.communicate(input=text.encode('utf-8'))
            
            if process.returncode != 0:
                self.get_logger().error(f'Piper failed: {stderr.decode()}')
                return
            
            elapsed = time.time() - start
            self.get_logger().info(f'Piper generated {len(audio_bytes)} bytes in {elapsed:.2f}s')
            
            self.stream_audio(audio_bytes)
            
        except FileNotFoundError:
            self.get_logger().error('Piper not found. Install with: pip install piper-tts')
    
    def stream_audio(self, audio_bytes: bytes):
        """Stream audio data in chunks over ROS topic"""
        for i in range(0, len(audio_bytes), self.chunk_size):
            chunk = audio_bytes[i:i + self.chunk_size]
            
            msg = UInt8MultiArray()
            msg.data = list(chunk)
            self.pub_audio.publish(msg)
    
    def destroy_node(self):
        """Cleanup"""
        self.speech_queue.put(None)  # Signal worker to stop
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KokoroTTSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()