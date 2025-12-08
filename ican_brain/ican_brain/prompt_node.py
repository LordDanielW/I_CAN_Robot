#!/usr/bin/env python3
"""
Prompt Node - Intelligent prompt construction and routing

This node acts as the brain's input processor:
1. Receives text from speech recognition (whisper/vosk)
2. Buffers text to form complete sentences
3. Queries tool_manager for available tools
4. Constructs rich prompts with robot context and tool descriptions
5. Sends prompts to LLM (ollama_tool_node)

Author: I_CAN Robot Project
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
import re
import time
from difflib import SequenceMatcher


class PromptNode(Node):
    """Intelligent prompt construction with tool awareness"""
    
    def __init__(self):
        super().__init__('prompt_node')
        self.get_logger().info('Prompt Node initializing...')
        
        # Configuration
        self.declare_parameter('buffer_timeout', 2.0)  # Seconds to wait before processing
        self.declare_parameter('wake_word', 'hey spot')
        self.declare_parameter('wake_word_threshold', 0.75)  # Similarity threshold (0-1)
        
        self.buffer_timeout = self.get_parameter('buffer_timeout').value
        self.wake_word = self.get_parameter('wake_word').value.lower()
        self.wake_word_threshold = self.get_parameter('wake_word_threshold').value
        
        # Common misrecognitions to match
        self.wake_word_variants = [
            'hey spot',
            'hi spot',
            'hey spock',
            'hi spock',
            'hey scott',
            'hi scott',
            'hey spots',
            'a spot',
            'hay spot',
            'stop',
            'spot',
            'please spot',
        ]
        
        # Text buffer for accumulating speech
        self.text_buffer = []
        self.last_text_time = None
        self.listening_active = False
        
        # Available tools cache
        self.available_tools = []
        self.tools_description = ""
        
        # Publishers/Subscribers
        self.speech_sub = self.create_subscription(
            String,
            '/speech_text',
            self.speech_callback,
            10
        )
        
        self.prompt_pub = self.create_publisher(
            String,
            '/llm_prompt',
            10
        )
        
        self.status_pub = self.create_publisher(
            String,
            '/prompt_status',
            10
        )
        
        # Timer to check buffer timeout
        self.timer = self.create_timer(0.5, self.check_buffer_timeout)
        
        # Query available tools from tool manager
        self.query_tools_timer = self.create_timer(5.0, self.query_available_tools)
        
        # Service client for tool manager
        self.tool_query_client = self.create_client(
            Trigger,
            '/tools/list'
        )
        
        self.get_logger().info('Prompt Node ready')
        self.get_logger().info(f'  Wake word: "{self.wake_word}"')
        self.get_logger().info(f'  Wake word threshold: {self.wake_word_threshold}')
        self.get_logger().info(f'  Buffer timeout: {self.buffer_timeout}s')
        self.get_logger().info('  Subscribed to: /speech_text')
        self.get_logger().info('  Publishing to: /llm_prompt, /prompt_status')
        
        # Initial tool query
        self.query_available_tools()
    
    def similarity(self, a, b):
        """Calculate similarity between two strings (0-1)"""
        return SequenceMatcher(None, a.lower(), b.lower()).ratio()
    
    def detect_wake_word(self, text):
        """Detect wake word with fuzzy matching"""
        text_lower = text.lower()
        
        # First check exact variants
        for variant in self.wake_word_variants:
            if variant in text_lower:
                return variant
        
        # Then check fuzzy matching on first few words
        words = text_lower.split()
        if len(words) >= 2:
            two_word_phrase = ' '.join(words[:2])
            
            # Check similarity to wake word
            if self.similarity(two_word_phrase, self.wake_word) >= self.wake_word_threshold:
                return two_word_phrase
            
            # Check similarity to each variant
            for variant in self.wake_word_variants:
                if self.similarity(two_word_phrase, variant) >= self.wake_word_threshold:
                    return two_word_phrase
        
        return None
    
    def query_available_tools(self):
        """Query tool manager for available tools"""
        if not self.tool_query_client.wait_for_service(timeout_sec=1.0):
            if len(self.available_tools) == 0:  # Only log if we haven't gotten tools yet
                self.get_logger().warn('Tool manager not available yet')
            return
        
        try:
            request = Trigger.Request()
            future = self.tool_query_client.call_async(request)
            future.add_done_callback(self.tool_query_callback)
        except Exception as e:
            self.get_logger().error(f'Failed to query tools: {e}')
    
    def tool_query_callback(self, future):
        """Handle tool query response"""
        try:
            response = future.result()
            if response.success:
                self.tools_description = response.message
                # Parse tool list (format: "tool1:description;tool2:description;...")
                self.available_tools = [t.split(':')[0].strip() for t in response.message.split(';') if t]
                self.get_logger().info(f'Updated tool list: {self.available_tools}')
            else:
                self.get_logger().warn('Tool query returned no tools')
        except Exception as e:
            self.get_logger().error(f'Error processing tool list: {e}')
    
    def speech_callback(self, msg):
        """Process incoming speech text"""
        text = msg.data.strip()
        if not text:
            return
        
        self.get_logger().info(f'Speech: "{text}"')
        
        # Check for wake word (exact or fuzzy match)
        detected_wake_word = self.detect_wake_word(text)
        
        if detected_wake_word:
            # Wake word detected - start listening
            self.listening_active = True
            self.text_buffer = []
            
            text_lower = text.lower()
            
            # Find where the wake word appears
            wake_index = text_lower.find(detected_wake_word)
            if wake_index == -1:
                # Fuzzy match - use first 2 words as wake word position
                words = text.split()
                if len(words) >= 2:
                    wake_index = len(words[0]) + 1 + len(words[1])
                else:
                    wake_index = len(text)
            
            # Extract text after wake word
            command_text = text[wake_index + len(detected_wake_word):].strip()
            
            if command_text:
                self.text_buffer.append(command_text)
                self.last_text_time = time.time()
            
            status_msg = String()
            status_msg.data = "LISTENING"
            self.status_pub.publish(status_msg)
            
            self.get_logger().info(f'🎤 Wake word detected ("{detected_wake_word}")! Listening...')
            
        elif self.listening_active:
            # Accumulate text while listening
            self.text_buffer.append(text)
            self.last_text_time = time.time()
    
    def check_buffer_timeout(self):
        """Check if buffer should be processed"""
        if not self.listening_active or not self.text_buffer:
            return
        
        if self.last_text_time is None:
            return
        
        # Check if enough time has passed since last text
        time_since_last = time.time() - self.last_text_time
        
        if time_since_last >= self.buffer_timeout:
            # Process the buffer
            self.process_buffer()
    
    def process_buffer(self):
        """Process accumulated text and send to LLM"""
        if not self.text_buffer:
            return
        
        # Combine all buffered text
        user_input = ' '.join(self.text_buffer).strip()
        
        self.get_logger().info(f'📝 Processing command: "{user_input}"')
        
        # Construct rich prompt with robot context
        prompt = self.construct_prompt(user_input)
        
        # Send to LLM
        prompt_msg = String()
        prompt_msg.data = prompt
        self.prompt_pub.publish(prompt_msg)
        
        # Publish status
        status_msg = String()
        status_msg.data = "PROCESSING"
        self.status_pub.publish(status_msg)
        
        # Reset state
        self.text_buffer = []
        self.last_text_time = None
        self.listening_active = False
        
        self.get_logger().info('✓ Prompt sent to LLM')
    
    def construct_prompt(self, user_input: str) -> str:
        """Construct a rich prompt with robot context and tool descriptions"""
        
        # Robot identity and purpose
        system_context = """You are the AI brain of Spot, a robotic seeing-eye guide dog.
Your purpose is to assist and protect your human companion through intelligent decision-making and tool usage.

IMPORTANT BEHAVIORS:
- Be helpful, concise, and action-oriented
- When the user asks you to do something, USE THE AVAILABLE TOOLS
- Don't just describe what you would do - actually call the tools
- Respond naturally but include tool calls when appropriate

"""
        
        # Add tool descriptions if available
        if self.available_tools:
            system_context += f"""AVAILABLE TOOLS:
You have access to the following tools. To use a tool, include its exact command in your response.

{self.tools_description}

TOOL USAGE EXAMPLES:
- User: "Roll a d20" → Response: "Rolling for you! TOOL:roll(d20)"
- User: "Flip a coin" → Response: "Flipping a coin... TOOL:coin_flip(1)"
- User: "Show stats" → Response: "Generating character stats: TOOL:roll_stats()"

Remember: Include "TOOL:function_name(params)" in your response when you want to use a tool.

"""
        else:
            system_context += "⚠️ No tools currently available.\n\n"
        
        # Combine with user input
        full_prompt = f"""{system_context}

USER COMMAND: {user_input}

YOUR RESPONSE (include TOOL:commands if needed):"""
        
        return full_prompt


def main(args=None):
    rclpy.init(args=args)
    
    prompt_node = PromptNode()
    
    try:
        rclpy.spin(prompt_node)
    except KeyboardInterrupt:
        prompt_node.get_logger().info('Shutting down prompt node...')
    finally:
        prompt_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
