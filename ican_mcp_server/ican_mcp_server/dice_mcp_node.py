#!/usr/bin/env python3
"""
Dice MCP Server - MCP Server for Rolling Dice

This node acts as an MCP Server that provides dice rolling functionality.
It can roll various types of dice (d4, d6, d8, d10, d12, d20, d100) and
calculate sums, including support for standard RPG notation (e.g., 3d6+2).

Author: I_CAN Robot Project
License: Apache-2.0
"""

import random
import re
from typing import Optional

import rclpy
from rclpy.node import Node

from mcp.server.fastmcp import FastMCP


class DiceRollerNode(Node):
    """ROS 2 Node for dice rolling utilities"""
    
    def __init__(self):
        super().__init__('dice_roller_node')
        self.get_logger().info('DiceRoller node initialized')
        
        # Statistics tracking
        self.total_rolls = 0
        self.roll_history = []
        self.max_history = 100  # Keep last 100 rolls
    
    def roll_dice(self, sides: int, count: int = 1) -> list[int]:
        """
        Roll dice and track statistics.
        
        Args:
            sides: Number of sides on each die
            count: Number of dice to roll
            
        Returns:
            List of roll results
        """
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
        
        # Trim history if needed
        if len(self.roll_history) > self.max_history:
            self.roll_history = self.roll_history[-self.max_history:]
        
        self.get_logger().info(f'Rolled {count}d{sides}: {rolls} (sum: {sum(rolls)})')
        
        return rolls


# Global node instance
dice_node: Optional[DiceRollerNode] = None

# Initialize FastMCP server
mcp = FastMCP("DiceRoller")


@mcp.tool()
def roll(
    dice_notation: str = "1d6"
) -> str:
    """
    Roll dice using standard RPG notation (e.g., 3d6, 2d20+5, d100).
    
    Notation format: [count]d[sides][+/-modifier]
    - count: number of dice (default: 1)
    - d: separator
    - sides: number of sides (2, 4, 6, 8, 10, 12, 20, 100)
    - modifier: optional bonus/penalty to add to sum
    
    Args:
        dice_notation: Dice to roll in RPG notation (e.g., "3d6", "d20+5", "2d10-3")
    
    Returns:
        Detailed results of the dice roll including individual values and total
    
    Examples:
        roll("d20") -> Roll one 20-sided die
        roll("3d6") -> Roll three 6-sided dice
        roll("2d10+5") -> Roll two 10-sided dice and add 5
        roll("4d6-2") -> Roll four 6-sided dice and subtract 2
    """
    if dice_node is None:
        return "❌ Error: Dice node not initialized"
    
    # Parse dice notation
    pattern = r'^(\d*)d(\d+)([\+\-]\d+)?$'
    match = re.match(pattern, dice_notation.lower().strip())
    
    if not match:
        return (
            f"❌ Invalid dice notation: '{dice_notation}'\n"
            f"Format: [count]d[sides][+/-modifier]\n"
            f"Examples: d6, 3d20, 2d10+5, 4d6-2"
        )
    
    count_str, sides_str, modifier_str = match.groups()
    
    # Parse values
    count = int(count_str) if count_str else 1
    sides = int(sides_str)
    modifier = int(modifier_str) if modifier_str else 0
    
    # Validate common dice types
    valid_dice = [2, 4, 6, 8, 10, 12, 20, 100]
    if sides not in valid_dice:
        return (
            f"⚠️ Warning: d{sides} is not a standard die type.\n"
            f"Common types: d4, d6, d8, d10, d12, d20, d100\n"
            f"Rolling anyway..."
        )
    
    try:
        # Roll the dice
        rolls = dice_node.roll_dice(sides, count)
        
        # Calculate results
        roll_sum = sum(rolls)
        total = roll_sum + modifier
        
        # Format output
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
        
        # Add context for special rolls
        if count == 1 and sides == 20:
            if rolls[0] == 20:
                result += "   🌟 NATURAL 20! Critical success!\n"
            elif rolls[0] == 1:
                result += "   💀 NATURAL 1! Critical failure!\n"
        
        return result
        
    except ValueError as e:
        return f"❌ Error: {str(e)}"
    except Exception as e:
        dice_node.get_logger().error(f'Error rolling dice: {e}')
        return f"❌ Error rolling dice: {str(e)}"


@mcp.tool()
def roll_multiple(
    dice_type: str = "d6",
    count: int = 1
) -> str:
    """
    Roll multiple dice of the same type (simple interface).
    
    Args:
        dice_type: Type of die to roll (d4, d6, d8, d10, d12, d20, d100)
        count: Number of dice to roll (default: 1)
    
    Returns:
        Detailed results of the dice rolls
    
    Examples:
        roll_multiple("d20", 1) -> Roll one d20
        roll_multiple("d6", 4) -> Roll four d6
    """
    # Convert to dice notation and use main roll function
    dice_notation = f"{count}{dice_type}"
    return roll(dice_notation)


@mcp.tool()
def roll_stats() -> str:
    """
    Roll ability scores for a character (4d6 drop lowest, six times).
    
    This is a common method for generating RPG character stats.
    Rolls 4d6, drops the lowest die, repeats 6 times.
    
    Returns:
        Six ability scores with detailed breakdown
    """
    if dice_node is None:
        return "❌ Error: Dice node not initialized"
    
    try:
        stats = []
        details = []
        
        for i in range(6):
            # Roll 4d6
            rolls = dice_node.roll_dice(6, 4)
            rolls_sorted = sorted(rolls)
            
            # Drop lowest
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
        
        # Format output
        result = "📊 Rolling Ability Scores (4d6 drop lowest):\n\n"
        
        ability_names = ["Strength", "Dexterity", "Constitution", 
                        "Intelligence", "Wisdom", "Charisma"]
        
        for i, (name, detail) in enumerate(zip(ability_names, details), 1):
            rolls_display = ', '.join(str(r) for r in detail['rolls'])
            kept_display = ', '.join(str(r) for r in detail['kept'])
            
            result += f"{i}. {name}: {detail['value']}\n"
            result += f"   Rolled: [{rolls_display}]\n"
            result += f"   Kept: [{kept_display}] (dropped {detail['dropped']})\n\n"
        
        result += f"Sum of stats: {sum(stats)}\n"
        result += f"Average: {sum(stats)/6:.1f}\n"
        
        return result
        
    except Exception as e:
        dice_node.get_logger().error(f'Error rolling stats: {e}')
        return f"❌ Error rolling stats: {str(e)}"


@mcp.tool()
def coin_flip(count: int = 1) -> str:
    """
    Flip a coin (or multiple coins).
    
    Args:
        count: Number of coins to flip (default: 1)
    
    Returns:
        Results of coin flips
    """
    if count < 1 or count > 100:
        return "❌ Error: Count must be between 1 and 100"
    
    flips = [random.choice(["Heads", "Tails"]) for _ in range(count)]
    
    if count == 1:
        result = f"🪙 Coin flip: {flips[0]}"
    else:
        heads = flips.count("Heads")
        tails = flips.count("Tails")
        
        result = f"🪙 Flipped {count} coins:\n"
        result += f"   Results: {', '.join(flips)}\n"
        result += f"   Heads: {heads} ({heads/count*100:.1f}%)\n"
        result += f"   Tails: {tails} ({tails/count*100:.1f}%)\n"
    
    return result


@mcp.tool()
def get_roll_history(limit: int = 10) -> str:
    """
    Get the history of recent dice rolls.
    
    Args:
        limit: Number of recent rolls to show (default: 10, max: 100)
    
    Returns:
        List of recent dice rolls with results
    """
    if dice_node is None:
        return "❌ Error: Dice node not initialized"
    
    if limit < 1:
        return "❌ Error: Limit must be at least 1"
    
    limit = min(limit, 100)
    
    if not dice_node.roll_history:
        return "📋 No dice rolls yet"
    
    recent = dice_node.roll_history[-limit:]
    
    result = f"📋 Last {len(recent)} dice rolls:\n\n"
    
    for i, roll in enumerate(reversed(recent), 1):
        count = roll['count']
        sides = roll['sides']
        results = roll['results']
        total = roll['sum']
        
        rolls_display = ', '.join(str(r) for r in results)
        result += f"{i}. {count}d{sides}: [{rolls_display}] = {total}\n"
    
    result += f"\nTotal rolls made: {dice_node.total_rolls}"
    
    return result


@mcp.tool()
def roll_percentile() -> str:
    """
    Roll percentile dice (d100) - useful for percentage checks.
    
    Returns a value between 1-100, commonly used for skill checks
    and random tables in RPGs.
    
    Returns:
        Percentile roll result
    """
    return roll("d100")


def main():
    """Main entry point for the dice MCP server"""
    global dice_node
    
    # Initialize ROS 2
    rclpy.init()
    
    # Create the ROS 2 node
    dice_node = DiceRollerNode()
    
    dice_node.get_logger().info('Starting MCP server for DiceRoller...')
    
    try:
        # Run the MCP server (this blocks)
        mcp.run()
    except KeyboardInterrupt:
        dice_node.get_logger().info('Shutting down dice server...')
    finally:
        # Cleanup
        dice_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
