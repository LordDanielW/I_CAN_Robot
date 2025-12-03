#!/usr/bin/env python3
"""
Simple TTS Test Script (No ROS)
Tests pyttsx3 text-to-speech engine directly
"""

import pyttsx3

def test_tts():
    print("Initializing pyttsx3 TTS engine...")
    
    try:
        # Initialize the TTS engine
        engine = pyttsx3.init()
        
        # Get available voices
        voices = engine.getProperty('voices')
        print(f"\nFound {len(voices)} voices:")
        for i, voice in enumerate(voices):
            print(f"  {i}: {voice.name} ({voice.id})")
        
        # Configure voice properties
        engine.setProperty('rate', 150)     # Speed of speech
        engine.setProperty('volume', 0.9)   # Volume (0.0 to 1.0)
        
        # Test message
        test_message = "Hello! This is a test of the text to speech system. I am working correctly."
        
        print(f"\nSpeaking: '{test_message}'")
        engine.say(test_message)
        engine.runAndWait()
        
        print("\n✓ TTS test successful!")
        print("If you heard the message, pyttsx3 is working correctly.")
        
    except Exception as e:
        print(f"\n✗ TTS test failed: {e}")
        print("\nTroubleshooting:")
        print("  - Install espeak: sudo apt install espeak espeak-ng")
        print("  - Install pyttsx3: pip install pyttsx3")
        return False
    
    return True

if __name__ == '__main__':
    print("=" * 60)
    print("pyttsx3 TTS Test Script")
    print("=" * 60)
    test_tts()
