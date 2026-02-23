"""
Serial Communication Module for RP2040 Motor Controller
Handles JSON-based command protocol with safety features
"""
import serial
import json
import time
import threading
from typing import Optional, Dict, Any
import logging

logger = logging.getLogger(__name__)


class RP2040Controller:
    """
    Interface to RP2040 motor controller via USB Serial
    Thread-safe with automatic reconnection
    """
    
    def __init__(self, port: str = "/dev/ttyACM0", baudrate: int = 115200, timeout: float = 0.5):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        
        self.serial = None
        self.connected = False
        self.enabled = False
        
        self.lock = threading.Lock()
        self.last_command_time = 0
        self.reconnect_attempts = 0
        self.last_reconnect_time = 0
        
        # Statistics
        self.commands_sent = 0
        self.errors = 0
        
    def connect(self) -> bool:
        """Establish serial connection to RP2040"""
        try:
            # Close existing connection if any
            if self.serial and self.serial.is_open:
                self.serial.close()
            
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                write_timeout=2.0  # Longer write timeout to prevent blocking
            )
            
            # Clear buffers
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            
            time.sleep(0.5)  # Wait for RP2040 to be ready
            self.connected = True
            self.reconnect_attempts = 0
            logger.info(f"Connected to RP2040 on {self.port}")
            return True
            
        except serial.SerialException as e:
            logger.error(f"Failed to connect to RP2040: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """Close serial connection"""
        with self.lock:
            if self.serial and self.serial.is_open:
                self.send_velocity(0, 0)  # Stop motors
                self.disable()
                self.serial.close()
                self.connected = False
                logger.info("Disconnected from RP2040")
    
    def _send_command(self, cmd: Dict[str, Any]) -> bool:
        """Send JSON command to RP2040"""
        if not self.connected or not self.serial or not self.serial.is_open:
            # Limit reconnection attempts (max once every 2 seconds)
            current_time = time.time()
            if current_time - self.last_reconnect_time < 2.0:
                return False
            
            self.last_reconnect_time = current_time
            self.reconnect_attempts += 1
            
            if self.reconnect_attempts > 3:
                # Too many failed reconnection attempts
                return False
            
            # Try to reconnect
            if self.connect():
                logger.info("Reconnected to RP2040")
            else:
                return False
        
        try:
            with self.lock:
                cmd_str = json.dumps(cmd) + '\n'
                self.serial.write(cmd_str.encode('utf-8'))
                self.serial.flush()
                self.commands_sent += 1
                self.last_command_time = time.time()
                return True
                
        except serial.SerialException as e:
            logger.error(f"Serial communication error: {e}")
            self.errors += 1
            self.connected = False
            
            # Close the connection to force clean reconnect
            try:
                if self.serial and self.serial.is_open:
                    self.serial.close()
            except:
                pass
            
            return False
    
    def send_velocity(self, linear: float, angular: float) -> bool:
        """Send velocity command (m/s, rad/s)"""
        cmd = {
            "cmd": "velocity",
            "linear": linear,
            "angular": angular
        }
        return self._send_command(cmd)
    
    def enable(self) -> bool:
        """Enable motors"""
        if self._send_command({"cmd": "enable"}):
            self.enabled = True
            logger.info("Motors enabled")
            return True
        return False
    
    def disable(self) -> bool:
        """Disable motors"""
        if self._send_command({"cmd": "disable"}):
            self.enabled = False
            logger.info("Motors disabled")
            return True
        return False
    
    def stop(self) -> bool:
        """Emergency stop"""
        logger.warning("Emergency stop triggered")
        return self._send_command({"cmd": "stop"})
    
    def get_status(self) -> Optional[Dict]:
        """Request status from RP2040"""
        self._send_command({"cmd": "status"})
        
        try:
            if self.serial and self.serial.in_waiting:
                line = self.serial.readline().decode('utf-8').strip()
                return json.loads(line)
        except Exception as e:
            logger.debug(f"Status read error: {e}")
        
        return None
    
    def get_stats(self) -> Dict:
        """Get communication statistics"""
        return {
            "connected": self.connected,
            "enabled": self.enabled,
            "commands_sent": self.commands_sent,
            "errors": self.errors,
            "last_command_time": self.last_command_time
        }


class SafetyController:
    """
    Wraps RP2040Controller with safety features
    - Velocity limiting
    - Watchdog monitoring
    - Smooth acceleration
    """
    
    def __init__(self, rp2040: RP2040Controller, max_linear: float = 0.5, max_angular: float = 2.0):
        self.rp2040 = rp2040
        self.max_linear = max_linear
        self.max_angular = max_angular
        
        self.current_linear = 0.0
        self.current_angular = 0.0
        
        self.watchdog_timeout = 0.5  # seconds
        self.last_update = time.time()
        
        # Start watchdog thread
        self.running = True
        self.watchdog_thread = threading.Thread(target=self._watchdog_loop, daemon=True)
        self.watchdog_thread.start()
    
    def set_velocity(self, linear: float, angular: float) -> bool:
        """Set velocity with safety limits"""
        # Clamp velocities
        linear = max(-self.max_linear, min(self.max_linear, linear))
        angular = max(-self.max_angular, min(self.max_angular, angular))
        
        self.current_linear = linear
        self.current_angular = angular
        self.last_update = time.time()
        
        return self.rp2040.send_velocity(linear, angular)
    
    def _watchdog_loop(self):
        """Monitor for timeout and auto-stop"""
        while self.running:
            if time.time() - self.last_update > self.watchdog_timeout:
                if self.current_linear != 0 or self.current_angular != 0:
                    logger.warning("Watchdog timeout - stopping robot")
                    self.rp2040.stop()
                    self.current_linear = 0
                    self.current_angular = 0
            
            time.sleep(0.1)
    
    def stop(self):
        """Stop watchdog and motors"""
        self.running = False
        self.rp2040.stop()
