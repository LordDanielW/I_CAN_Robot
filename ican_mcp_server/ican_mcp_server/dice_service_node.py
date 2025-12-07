#!/usr/bin/env python3
"""
Dice Service Node - ROS2 service-based dice rolling

This node provides dice rolling functionality via ROS2 services,
making it accessible to other ROS2 nodes like ollama_node.

Author: I_CAN Robot Project
License: Apache-2.0
"""

import random
import re
from typing import Optional

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String


class DiceServiceNode(Node):
    """ROS 2 Node for dice rolling utilities via services"""
    
    def __init__(self):
        super().__init__('dice_service_node')
        self.get_logger().info('Dice Service Node initialized')
        
        # Statistics tracking
        self.total_rolls = 0
        self.roll_history = []
        self.max_history = 100
        
        # Create service for dice rolling
        self.roll_service = self.create_service(
            Trigger,
            '/dice/roll',
            self.handle_roll
        )
        
        # Create subscriber for dice commands
        self.command_sub = self.create_subscription(
            String,
            '/dice/command',
            self.command_callback,
            10
        )
        
        # Create publisher for results
        self.result_pub = self.create_publisher(
            String,
            '/dice/result',
            10
        )
        
        self.get_logger().info('Dice services ready:')
        self.get_logger().info('  - Service: /dice/roll')
        self.get_logger().info('  - Subscriber: /dice/command')
        self.get_logger().info('  - Publisher: /dice/result')
    
    def command_callback(self, msg):
        """Handle dice command from topic"""
        command = msg.data.strip()
        self.get_logger().info(f'Received dice command: {command}')
        
        result = self.process_dice_command(command)
        
        # Publish result
        result_msg = String()
        result_msg.data = result
        self.result_pub.publish(result_msg)
        
        self.get_logger().info(f'Published result: {result[:100]}...')
    
    def handle_roll(self, request, response):
        """Handle service request"""
        # Simple d20 roll for service interface
        result = self.roll_dice_notation("d20")
        response.success = True
        response.message = result
        return response
    
    def process_dice_command(self, command: str) -> str:
        """Process various dice commands"""
        command_lower = command.lower()
        
        # Parse different command types
        if 'stats' in command_lower or 'ability' in command_lower:
            return self.roll_stats()
        elif 'coin' in command_lower or 'flip' in command_lower:
            count = self._extract_number(command, default=1)
            return self.coin_flip(count)
        elif 'percentile' in command_lower or 'd100' in command_lower:
            return self.roll_dice_notation("d100")
        elif 'history' in command_lower:
            limit = self._extract_number(command, default=10)
            return self.get_roll_history(limit)
        else:
            # Try to parse as dice notation
            dice_match = re.search(r'\d*d\d+[\+\-]?\d*', command_lower)
            if dice_match:
                notation = dice_match.group(0)
                return self.roll_dice_notation(notation)
            else:
                return "❌ Unknown command. Use dice notation (e.g., 3d6, d20+5) or commands like 'stats', 'coin flip', 'history'"
    
    def _extract_number(self, text: str, default: int = 1) -> int:
        """Extract a number from text"""
        numbers = re.findall(r'\d+', text)
        return int(numbers[0]) if numbers else default
    
    def roll_dice(self, sides: int, count: int = 1) -> list[int]:
        """Roll dice and track statistics"""
        if sides < 2:
            raise ValueError("Dice must have at least 2 sides")
        if count < 1:
            raise ValueError("Must roll at least 1 die")
        if count > 1000:
            raise ValueError("Cannot roll more than 1000 dice at once")
        
        rolls = [random.randint(1, sides) for _ in range(count)]
        
        # Update statistics
        self.total_rolls += count
        roll_entry = {
            'sides': sides,
            'count': count,
            'results': rolls,
            'sum': sum(rolls)
        }
        self.roll_history.append(roll_entry)
        
        if len(self.roll_history) > self.max_history:
            self.roll_history = self.roll_history[-self.max_history:]
        
        self.get_logger().info(f'Rolled {count}d{sides}: {rolls} (sum: {sum(rolls)})')
        
        return rolls
    
    def roll_dice_notation(self, dice_notation: str) -> str:
        """Roll dice using RPG notation"""
        pattern = r'^(\d*)d(\d+)([\+\-]\d+)?$'
        match = re.match(pattern, dice_notation.lower().strip())
        
        if not match:
            return f"❌ Invalid dice notation: '{dice_notation}'"
        
        count_str, sides_str, modifier_str = match.groups()
        count = int(count_str) if count_str else 1
        sides = int(sides_str)
        modifier = int(modifier_str) if modifier_str else 0
        
        try:
            rolls = self.roll_dice(sides, count)
            roll_sum = sum(rolls)
            total = roll_sum + modifier
            
            rolls_display = ', '.join(str(r) for r in rolls)
            
            result = f"🎲 Rolling {dice_notation}:\n"
            result += f"   Rolls: [{rolls_display}]\n"
            result += f"   Sum: {roll_sum}\n"
            
            if modifier != 0:
                modifier_display = f"+{modifier}" if modifier > 0 else str(modifier)
                result += f"   Modifier: {modifier_display}\n"
                result += f"   Total: {total}\n"
            else:
                result += f"   Total: {total}\n"
            
            if count == 1 and sides == 20:
                if rolls[0] == 20:
                    result += "   🌟 NATURAL 20! Critical success!\n"
                elif rolls[0] == 1:
                    result += "   💀 NATURAL 1! Critical failure!\n"
            
            return result
            
        except ValueError as e:
            return f"❌ Error: {str(e)}"
    
    def roll_stats(self) -> str:
        """Roll ability scores (4d6 drop lowest, six times)"""
        stats = []
        details = []
        
        for i in range(6):
            rolls = self.roll_dice(6, 4)
            rolls_sorted = sorted(rolls)
            dropped = rolls_sorted[0]
            kept = rolls_sorted[1:]
            stat_value = sum(kept)
            
            stats.append(stat_value)
            details.append({
                'rolls': rolls,
                'dropped': dropped,
                'kept': kept,
                'value': stat_value
            })
        
        result = "📊 Rolling Ability Scores (4d6 drop lowest):\n\n"
        ability_names = ["Strength", "Dexterity", "Constitution", 
                        "Intelligence", "Wisdom", "Charisma"]
        
        for i, (name, detail) in enumerate(zip(ability_names, details), 1):
            rolls_display = ', '.join(str(r) for r in detail['rolls'])
            kept_display = ', '.join(str(r) for r in detail['kept'])
            
            result += f"{i}. {name}: {detail['value']}\n"
            result += f"   Rolled: [{rolls_display}] (dropped {detail['dropped']})\n\n"
        
        result += f"Sum: {sum(stats)}, Average: {sum(stats)/6:.1f}\n"
        return result
    
    def coin_flip(self, count: int = 1) -> str:
        """Flip coins"""
        if count < 1 or count > 100:
            return "❌ Error: Count must be between 1 and 100"
        
        flips = [random.choice(["Heads", "Tails"]) for _ in range(count)]
        
        if count == 1:
            return f"🪙 Coin flip: {flips[0]}"
        else:
            heads = flips.count("Heads")
            tails = flips.count("Tails")
            
            result = f"🪙 Flipped {count} coins:\n"
            result += f"   Results: {', '.join(flips)}\n"
            result += f"   Heads: {heads} ({heads/count*100:.1f}%)\n"
            result += f"   Tails: {tails} ({tails/count*100:.1f}%)\n"
            return result
    
    def get_roll_history(self, limit: int = 10) -> str:
        """Get recent roll history"""
        if limit < 1:
            return "❌ Error: Limit must be at least 1"
        
        limit = min(limit, 100)
        
        if not self.roll_history:
            return "📋 No dice rolls yet"
        
        recent = self.roll_history[-limit:]
        result = f"📋 Last {len(recent)} dice rolls:\n\n"
        
        for i, roll in enumerate(reversed(recent), 1):
            count = roll['count']
            sides = roll['sides']
            results = roll['results']
            total = roll['sum']
            
            rolls_display = ', '.join(str(r) for r in results)
            result += f"{i}. {count}d{sides}: [{rolls_display}] = {total}\n"
        
        result += f"\nTotal rolls made: {self.total_rolls}"
        return result


def main(args=None):
    rclpy.init(args=args)
    
    dice_service_node = DiceServiceNode()
    
    try:
        rclpy.spin(dice_service_node)
    except KeyboardInterrupt:
        dice_service_node.get_logger().info('Shutting down dice service...')
    finally:
        dice_service_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
