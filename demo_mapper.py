#!/usr/bin/env python3
"""
Library Mapping Demo
Demonstrates autonomous library/room mapping
"""
import requests
import time
import logging
import sys

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class LibraryMappingClient:
    """Client for library mapping control"""
    
    def __init__(self, host='localhost', port=5000):
        self.base_url = f'http://{host}:{port}/api'
        
    def enable_motors(self):
        """Enable motors"""
        response = requests.post(f'{self.base_url}/enable')
        return response.json()['success']
    
    def disable_motors(self):
        """Disable motors"""
        response = requests.post(f'{self.base_url}/disable')
        return response.json()['success']
    
    def start_mapping(self):
        """Start autonomous mapping"""
        response = requests.post(
            f'{self.base_url}/mapping',
            json={'enable': True}
        )
        return response.json()['success']
    
    def stop_mapping(self):
        """Stop mapping"""
        response = requests.post(
            f'{self.base_url}/mapping',
            json={'enable': False}
        )
        return response.json()['success']
    
    def get_mapping_status(self):
        """Get mapping status"""
        response = requests.get(f'{self.base_url}/mapping/status')
        return response.json()
    
    def save_map(self, filename=None):
        """Save current map"""
        data = {'filename': filename} if filename else {}
        response = requests.post(f'{self.base_url}/mapping/save', json=data)
        return response.json()['success']
    
    def get_status(self):
        """Get full robot status"""
        response = requests.get(f'{self.base_url}/status')
        return response.json()


def print_status(status):
    """Print mapping status in nice format"""
    print("\n" + "=" * 60)
    print("MAPPING STATUS")
    print("=" * 60)
    print(f"State: {status.get('state', 'UNKNOWN')}")
    print(f"Progress: {status.get('exploration_progress', 0)*100:.1f}%")
    print(f"Explored cells: {status.get('explored_cells', 0):,} / {status.get('total_cells', 0):,}")
    print(f"Time elapsed: {status.get('elapsed_time', 0):.0f}s")
    print(f"Distance traveled: {status.get('distance_traveled', 0):.2f}m")
    print(f"Path length: {status.get('path_length', 0)} waypoints")
    
    if status.get('current_goal'):
        goal = status['current_goal']
        print(f"Current goal: ({goal[0]:.2f}, {goal[1]:.2f})")
    
    if status.get('map_complete', False):
        print("\n🎉 MAPPING COMPLETE!")
    print("=" * 60)


def demo_mapping_session(client, duration=300):
    """
    Run a full mapping session
    
    Args:
        client: LibraryMappingClient instance
        duration: Max duration in seconds (default 5 min)
    """
    print("\n" + "=" * 60)
    print("STARTING LIBRARY MAPPING SESSION")
    print("=" * 60)
    print(f"Duration: {duration}s ({duration//60} minutes)")
    print(f"Web interface: http://localhost:5000")
    print(f"Map view: http://localhost:5000/video/map")
    print("\nPress Ctrl+C to stop at any time")
    print("=" * 60)
    
    # Enable motors
    print("\nEnabling motors...")
    if not client.enable_motors():
        print("❌ Failed to enable motors")
        return
    print("✓ Motors enabled")
    
    time.sleep(1)
    
    # Start mapping
    print("\nStarting autonomous mapping...")
    if not client.start_mapping():
        print("❌ Failed to start mapping")
        client.disable_motors()
        return
    print("✓ Mapping started")
    
    start_time = time.time()
    last_print_time = 0
    update_interval = 5  # Print status every 5 seconds
    
    try:
        while time.time() - start_time < duration:
            # Get status
            status = client.get_mapping_status()
            
            # Print status periodically
            current_time = time.time()
            if current_time - last_print_time >= update_interval:
                print_status(status)
                last_print_time = current_time
            
            # Check if mapping complete
            if status.get('map_complete', False):
                print("\n✓ Exploration complete!")
                break
            
            # Check state
            if status.get('state') == 'ERROR':
                print("\n⚠ Mapping error detected")
                break
            
            time.sleep(1)
    
    except KeyboardInterrupt:
        print("\n\nInterrupted by user!")
    
    # Stop mapping
    print("\nStopping mapping...")
    client.stop_mapping()
    
    # Get final status
    final_status = client.get_mapping_status()
    print_status(final_status)
    
    # Save map
    print("\nSaving map...")
    if client.save_map():
        print("✓ Map saved successfully")
        print("  Check current directory for library_map_* files")
    else:
        print("❌ Failed to save map")
    
    # Disable motors
    print("\nDisabling motors...")
    client.disable_motors()
    
    print("\n" + "=" * 60)
    print("MAPPING SESSION COMPLETE")
    print("=" * 60)
    
    # Summary
    elapsed = final_status.get('elapsed_time', 0)
    progress = final_status.get('exploration_progress', 0) * 100
    distance = final_status.get('distance_traveled', 0)
    
    print(f"\nSummary:")
    print(f"  Time: {elapsed:.0f}s ({elapsed/60:.1f} minutes)")
    print(f"  Coverage: {progress:.1f}%")
    print(f"  Distance: {distance:.2f}m")
    print(f"  Average speed: {distance/elapsed if elapsed > 0 else 0:.2f}m/s")


def demo_manual_exploration(client):
    """Monitor mapping progress while manually controlling"""
    print("\n" + "=" * 60)
    print("MANUAL EXPLORATION WITH MAPPING")
    print("=" * 60)
    print("This demo enables mapping while you control the robot manually")
    print("The robot will build a map as you drive it around")
    print("=" * 60)
    
    # Enable motors
    print("\nEnabling motors...")
    client.enable_motors()
    
    # Start mapping
    print("Starting mapping (background)...")
    client.start_mapping()
    
    print("\n✓ Mapping active!")
    print("Now control the robot via web interface at http://localhost:5000")
    print("Watch the map build in real-time at /video/map")
    print("\nPress Ctrl+C when done exploring")
    
    try:
        while True:
            status = client.get_mapping_status()
            
            progress = status.get('exploration_progress', 0) * 100
            cells = status.get('explored_cells', 0)
            
            print(f"\rProgress: {progress:5.1f}% | Cells: {cells:,}  ", end='', flush=True)
            
            time.sleep(2)
    
    except KeyboardInterrupt:
        print("\n\nStopping...")
    
    # Stop and save
    client.stop_mapping()
    
    final_status = client.get_mapping_status()
    print_status(final_status)
    
    print("\nSave map? (y/n): ", end='')
    if input().lower() == 'y':
        client.save_map()
        print("✓ Map saved")
    
    client.disable_motors()
    print("\n✓ Done!")


def main():
    print("=" * 60)
    print("LIBRARY MAPPING DEMOS")
    print("=" * 60)
    
    # Connect to robot
    client = LibraryMappingClient()
    
    # Check connection
    try:
        status = client.get_status()
        print(f"\n✓ Connected to robot")
        
        mapping_status = client.get_mapping_status()
        if not mapping_status.get('mapper_available', False):
            print("\n❌ Library mapper not available!")
            print("Make sure main.py is running with mapping support")
            return
        
        print("✓ Library mapper available")
    
    except Exception as e:
        print(f"\n❌ Cannot connect to robot: {e}")
        print("Make sure the robot system is running (python3 main.py)")
        return
    
    # Menu
    while True:
        print("\n" + "=" * 60)
        print("Available Demos:")
        print("-" * 60)
        print("1. Full Autonomous Mapping (5 minutes)")
        print("2. Extended Mapping (15 minutes)")
        print("3. Manual Exploration with Mapping")
        print("4. Check Current Status")
        print("5. Save Current Map")
        print("0. Exit")
        print()
        
        choice = input("Select demo (0-5): ").strip()
        
        if choice == '0':
            print("Goodbye!")
            break
        
        elif choice == '1':
            demo_mapping_session(client, duration=300)  # 5 minutes
        
        elif choice == '2':
            demo_mapping_session(client, duration=900)  # 15 minutes
        
        elif choice == '3':
            demo_manual_exploration(client)
        
        elif choice == '4':
            status = client.get_mapping_status()
            print_status(status)
        
        elif choice == '5':
            print("\nEnter filename (or press Enter for auto): ", end='')
            filename = input().strip()
            filename = filename if filename else None
            
            if client.save_map(filename):
                print("✓ Map saved successfully")
            else:
                print("❌ Failed to save map")
        
        else:
            print("Invalid choice!")


if __name__ == '__main__':
    main()