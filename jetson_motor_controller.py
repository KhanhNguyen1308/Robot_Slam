"""
Jetson Nano GPIO Motor Controller for A4988 Stepper Drivers
Direct control of stepper motors without external microcontroller
"""
import Jetson.GPIO as GPIO
import time
import threading
import logging
from typing import Optional, Dict, Any
import math

logger = logging.getLogger(__name__)


class A4988StepperMotor:
    """
    Control a single stepper motor via A4988 driver
    """
    
    def __init__(self, step_pin: int, dir_pin: int, enable_pin: int,
                 ms1_pin: int = None, ms2_pin: int = None, ms3_pin: int = None,
                 steps_per_rev: int = 200, microsteps: int = 16):
        self.step_pin = step_pin
        self.dir_pin = dir_pin
        self.enable_pin = enable_pin
        self.ms1_pin = ms1_pin
        self.ms2_pin = ms2_pin
        self.ms3_pin = ms3_pin
        self.microsteps = microsteps
        self.steps_per_rev = steps_per_rev * microsteps  # Full steps * microstepping
        
        # State
        self.enabled = False
        self.current_speed = 0  # steps per second
        self.direction = 1  # 1 = forward, -1 = backward
        
        # Threading
        self.running = False
        self.thread = None
        self.lock = threading.Lock()
        
        # Initialize GPIO
        GPIO.setup(self.step_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.dir_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.enable_pin, GPIO.OUT, initial=GPIO.HIGH)  # HIGH = disabled
        
        # Setup MS pins if provided
        if self.ms1_pin is not None:
            GPIO.setup(self.ms1_pin, GPIO.OUT, initial=GPIO.LOW)
        if self.ms2_pin is not None:
            GPIO.setup(self.ms2_pin, GPIO.OUT, initial=GPIO.LOW)
        if self.ms3_pin is not None:
            GPIO.setup(self.ms3_pin, GPIO.OUT, initial=GPIO.LOW)
        
        # Configure microstepping
        self.set_microstepping(microsteps)
        
    def set_microstepping(self, microsteps: int):
        """Set microstepping resolution (1, 2, 4, 8, 16)"""
        if not all([self.ms1_pin, self.ms2_pin, self.ms3_pin]):
            return  # MS pins not configured
        
        # Microstepping lookup table for A4988
        ms_config = {
            1:  (0, 0, 0),  # Full step
            2:  (1, 0, 0),  # Half step
            4:  (0, 1, 0),  # Quarter step
            8:  (1, 1, 0),  # Eighth step
            16: (1, 1, 1),  # Sixteenth step
        }
        
        if microsteps not in ms_config:
            logger.warning(f"Invalid microstep value {microsteps}, using 16")
            microsteps = 16
        
        ms1, ms2, ms3 = ms_config[microsteps]
        GPIO.output(self.ms1_pin, GPIO.HIGH if ms1 else GPIO.LOW)
        GPIO.output(self.ms2_pin, GPIO.HIGH if ms2 else GPIO.LOW)
        GPIO.output(self.ms3_pin, GPIO.HIGH if ms3 else GPIO.LOW)
        
        self.microsteps = microsteps
        logger.info(f"Microstepping set to 1/{microsteps}")
    
    def enable(self):
        """Enable the motor driver"""
        GPIO.output(self.enable_pin, GPIO.LOW)  # LOW = enabled
        self.enabled = True
        
    def disable(self):
        """Disable the motor driver"""
        GPIO.output(self.enable_pin, GPIO.HIGH)  # HIGH = disabled
        self.enabled = False
        self.stop()
        
    def set_direction(self, forward: bool):
        """Set rotation direction"""
        self.direction = 1 if forward else -1
        GPIO.output(self.dir_pin, GPIO.HIGH if forward else GPIO.LOW)
        
    def set_speed(self, steps_per_sec: float):
        """Set motor speed in steps per second"""
        with self.lock:
            self.current_speed = abs(steps_per_sec)
            self.set_direction(steps_per_sec >= 0)
            
    def stop(self):
        """Stop the motor"""
        with self.lock:
            self.current_speed = 0
            
    def _step_loop(self):
        """Background thread for generating step pulses"""
        while self.running:
            with self.lock:
                speed = self.current_speed
            
            if speed > 0 and self.enabled:
                # Calculate delay between steps
                delay = 1.0 / (2.0 * speed)  # Divide by 2 for pulse width
                
                # Generate step pulse
                GPIO.output(self.step_pin, GPIO.HIGH)
                time.sleep(min(delay, 0.000005))  # Min 5us pulse width
                GPIO.output(self.step_pin, GPIO.LOW)
                time.sleep(delay)
            else:
                time.sleep(0.001)  # Sleep 1ms when idle
                
    def start_thread(self):
        """Start background stepping thread"""
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._step_loop, daemon=True)
            self.thread.start()
            
    def stop_thread(self):
        """Stop background stepping thread"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)


class JetsonMotorController:
    """
    Differential drive robot controller using Jetson Nano GPIO and A4988 drivers
    Controls two stepper motors for differential drive
    """
    
    def __init__(self, 
                 left_step_pin: int = 33,   # Pin 33 (GPIO 13)
                 left_dir_pin: int = 35,    # Pin 35 (GPIO 19)
                 left_enable_pin: int = 37, # Pin 37 (GPIO 26)
                 left_ms1_pin: int = None,  # Optional MS1
                 left_ms2_pin: int = None,  # Optional MS2
                 left_ms3_pin: int = None,  # Optional MS3
                 right_step_pin: int = 32,  # Pin 32 (GPIO 12)
                 right_dir_pin: int = 36,   # Pin 36 (GPIO 16)
                 right_enable_pin: int = 38, # Pin 38 (GPIO 20)
                 right_ms1_pin: int = None, # Optional MS1
                 right_ms2_pin: int = None, # Optional MS2
                 right_ms3_pin: int = None, # Optional MS3
                 wheel_diameter: float = 0.066,  # 66mm wheels
                 wheel_base: float = 0.165,      # 165mm between wheels
                 steps_per_rev: int = 200,
                 microsteps: int = 16):
        
        # Robot parameters
        self.wheel_diameter = wheel_diameter
        self.wheel_base = wheel_base
        self.steps_per_rev = steps_per_rev * microsteps
        
        # Calculate conversions
        self.wheel_circumference = math.pi * self.wheel_diameter  # meters
        self.steps_per_meter = self.steps_per_rev / self.wheel_circumference
        
        # Initialize GPIO
        GPIO.setmode(GPIO.BOARD)  # Use physical pin numbering
        GPIO.setwarnings(False)
        
        # Create motor objects
        self.left_motor = A4988StepperMotor(
            left_step_pin, left_dir_pin, left_enable_pin,
            left_ms1_pin, left_ms2_pin, left_ms3_pin,
            steps_per_rev, microsteps
        )
        
        self.right_motor = A4988StepperMotor(
            right_step_pin, right_dir_pin, right_enable_pin,
            right_ms1_pin, right_ms2_pin, right_ms3_pin,
            steps_per_rev, microsteps
        )
        
        # State
        self.connected = True  # Always connected (GPIO)
        self.enabled = False
        self.current_linear = 0.0
        self.current_angular = 0.0
        
        # Statistics
        self.commands_sent = 0
        self.errors = 0
        self.last_command_time = 0
        
        logger.info(f"Jetson GPIO motor controller initialized")
        logger.info(f"Left motor: STEP={left_step_pin}, DIR={left_dir_pin}, EN={left_enable_pin}")
        if left_ms1_pin:
            logger.info(f"Left motor MS: MS1={left_ms1_pin}, MS2={left_ms2_pin}, MS3={left_ms3_pin}")
        logger.info(f"Right motor: STEP={right_step_pin}, DIR={right_dir_pin}, EN={right_enable_pin}")
        if right_ms1_pin:
            logger.info(f"Right motor MS: MS1={right_ms1_pin}, MS2={right_ms2_pin}, MS3={right_ms3_pin}")
        logger.info(f"Microstepping: 1/{microsteps}")
        logger.info(f"Steps per meter: {self.steps_per_meter:.1f}")
        
    def connect(self) -> bool:
        """Initialize GPIO (compatibility with serial controller interface)"""
        try:
            self.left_motor.start_thread()
            self.right_motor.start_thread()
            logger.info("Motor control threads started")
            return True
        except Exception as e:
            logger.error(f"Failed to start motor threads: {e}")
            return False
            
    def disconnect(self):
        """Clean up GPIO"""
        self.disable()
        self.send_velocity(0, 0)
        time.sleep(0.1)
        
        self.left_motor.stop_thread()
        self.right_motor.stop_thread()
        
        GPIO.cleanup()
        logger.info("GPIO cleaned up")
        
    def enable(self) -> bool:
        """Enable both motors"""
        self.left_motor.enable()
        self.right_motor.enable()
        self.enabled = True
        logger.info("Motors enabled")
        return True
        
    def disable(self) -> bool:
        """Disable both motors"""
        self.left_motor.disable()
        self.right_motor.disable()
        self.enabled = False
        logger.info("Motors disabled")
        return True
        
    def send_velocity(self, linear: float, angular: float) -> bool:
        """
        Send velocity command to differential drive robot
        
        Args:
            linear: Linear velocity in m/s (forward/backward)
            angular: Angular velocity in rad/s (rotation)
        """
        try:
            # Differential drive kinematics
            # v_left = linear - (angular * wheel_base / 2)
            # v_right = linear + (angular * wheel_base / 2)
            
            v_left = linear - (angular * self.wheel_base / 2.0)
            v_right = linear + (angular * self.wheel_base / 2.0)
            
            # Convert m/s to steps/s
            steps_left = v_left * self.steps_per_meter
            steps_right = v_right * self.steps_per_meter
            
            # Set motor speeds
            self.left_motor.set_speed(steps_left)
            self.right_motor.set_speed(steps_right)
            
            self.current_linear = linear
            self.current_angular = angular
            self.commands_sent += 1
            self.last_command_time = time.time()
            
            return True
            
        except Exception as e:
            logger.error(f"Velocity command error: {e}")
            self.errors += 1
            return False
    
    def set_velocity(self, linear: float, angular: float) -> bool:
        """Alias for send_velocity (compatibility with serial controller interface)"""
        return self.send_velocity(linear, angular)
            
    def stop(self) -> bool:
        """Emergency stop"""
        logger.warning("Emergency stop triggered")
        self.send_velocity(0, 0)
        return True
    
    def set_microstepping(self, microsteps: int) -> bool:
        """
        Change microstepping resolution for both motors
        
        Args:
            microsteps: Microstepping value (1, 2, 4, 8, 16)
        
        Returns:
            True if successful
        """
        try:
            self.left_motor.set_microstepping(microsteps)
            self.right_motor.set_microstepping(microsteps)
            
            # Recalculate steps per meter
            self.steps_per_rev = 200 * microsteps
            self.steps_per_meter = self.steps_per_rev / self.wheel_circumference
            
            logger.info(f"Microstepping changed to 1/{microsteps}, steps/meter: {self.steps_per_meter:.1f}")
            return True
        except Exception as e:
            logger.error(f"Failed to set microstepping: {e}")
            return False
        
    def get_stats(self) -> Dict:
        """Get controller statistics"""
        return {
            "connected": self.connected,
            "enabled": self.enabled,
            "commands_sent": self.commands_sent,
            "errors": self.errors,
            "last_command_time": self.last_command_time,
            "linear_velocity": self.current_linear,
            "angular_velocity": self.current_angular
        }
        
    def get_status(self) -> Optional[Dict]:
        """Get status (compatibility with serial controller)"""
        return self.get_stats()


class SafetyController:
    """
    Safety wrapper for motor controller with velocity limits
    """
    
    def __init__(self, motor_controller: JetsonMotorController,
                 max_linear: float = 0.5,
                 max_angular: float = 2.0):
        self.controller = motor_controller
        self.max_linear = max_linear
        self.max_angular = max_angular
        
    def send_velocity(self, linear: float, angular: float) -> bool:
        """Send velocity with safety limits"""
        # Clamp to limits
        linear = max(-self.max_linear, min(self.max_linear, linear))
        angular = max(-self.max_angular, min(self.max_angular, angular))
        
        return self.controller.send_velocity(linear, angular)
    
    def set_velocity(self, linear: float, angular: float) -> bool:
        """Alias for send_velocity (compatibility)"""
        return self.send_velocity(linear, angular)
        
    def enable(self) -> bool:
        return self.controller.enable()
        
    def disable(self) -> bool:
        return self.controller.disable()
        
    def stop(self) -> bool:
        return self.controller.stop()
        
    def __getattr__(self, name):
        """Forward other methods to controller"""
        return getattr(self.controller, name)
