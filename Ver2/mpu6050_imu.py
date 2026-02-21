"""
MPU6050 IMU Driver for Jetson Nano
Connects to I2C bus 1 for accurate orientation tracking
"""
import smbus2
import time
import numpy as np
import logging
import threading
from typing import Optional, Tuple
from collections import deque

logger = logging.getLogger(__name__)


class MPU6050:
    """
    Driver for MPU6050 6-axis IMU (accelerometer + gyroscope)
    Connected to Jetson Nano I2C bus 1
    """
    
    # MPU6050 Registers
    PWR_MGMT_1 = 0x6B
    SMPLRT_DIV = 0x19
    CONFIG = 0x1A
    GYRO_CONFIG = 0x1B
    ACCEL_CONFIG = 0x1C
    INT_ENABLE = 0x38
    
    ACCEL_XOUT_H = 0x3B
    ACCEL_YOUT_H = 0x3D
    ACCEL_ZOUT_H = 0x3F
    TEMP_OUT_H = 0x41
    GYRO_XOUT_H = 0x43
    GYRO_YOUT_H = 0x45
    GYRO_ZOUT_H = 0x47
    
    # Default I2C address
    DEFAULT_ADDRESS = 0x68
    
    # Sensitivity scale factors
    ACCEL_SCALE_2G = 16384.0
    ACCEL_SCALE_4G = 8192.0
    ACCEL_SCALE_8G = 4096.0
    ACCEL_SCALE_16G = 2048.0
    
    GYRO_SCALE_250 = 131.0
    GYRO_SCALE_500 = 65.5
    GYRO_SCALE_1000 = 32.8
    GYRO_SCALE_2000 = 16.4
    
    def __init__(self, 
                 bus_number: int = 1,
                 address: int = DEFAULT_ADDRESS,
                 gyro_range: int = 250,  # degrees/sec
                 accel_range: int = 2):  # g
        """
        Initialize MPU6050
        
        Args:
            bus_number: I2C bus number (1 for Jetson Nano default)
            address: I2C address (default 0x68)
            gyro_range: Gyroscope range (250, 500, 1000, 2000 deg/s)
            accel_range: Accelerometer range (2, 4, 8, 16 g)
        """
        self.bus_number = bus_number
        self.address = address
        self.bus = None
        
        # Set scale factors based on range
        self.gyro_range = gyro_range
        self.accel_range = accel_range
        
        if gyro_range == 250:
            self.gyro_scale = self.GYRO_SCALE_250
        elif gyro_range == 500:
            self.gyro_scale = self.GYRO_SCALE_500
        elif gyro_range == 1000:
            self.gyro_scale = self.GYRO_SCALE_1000
        else:
            self.gyro_scale = self.GYRO_SCALE_2000
            
        if accel_range == 2:
            self.accel_scale = self.ACCEL_SCALE_2G
        elif accel_range == 4:
            self.accel_scale = self.ACCEL_SCALE_4G
        elif accel_range == 8:
            self.accel_scale = self.ACCEL_SCALE_8G
        else:
            self.accel_scale = self.ACCEL_SCALE_16G
        
        # Calibration offsets
        self.gyro_offset = np.array([0.0, 0.0, 0.0])
        self.accel_offset = np.array([0.0, 0.0, 0.0])
        
        # Continuous reading
        self.reading_thread = None
        self.running = False
        self.lock = threading.Lock()
        
        # Latest readings
        self.latest_accel = np.array([0.0, 0.0, 0.0])
        self.latest_gyro = np.array([0.0, 0.0, 0.0])
        self.latest_temp = 0.0
        
        # Integration for orientation
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.last_update_time = None
        
        logger.info(f"MPU6050 initialized on bus {bus_number}, addr 0x{address:02X}")
    
    def connect(self) -> bool:
        """Connect to MPU6050 via I2C"""
        try:
            self.bus = smbus2.SMBus(self.bus_number)
            
            # Wake up MPU6050 (it starts in sleep mode)
            self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)
            time.sleep(0.1)
            
            # Set sample rate divider (1kHz / (1 + SMPLRT_DIV))
            # SMPLRT_DIV = 9 -> 100Hz sample rate
            self.bus.write_byte_data(self.address, self.SMPLRT_DIV, 9)
            
            # Configure DLPF (Digital Low Pass Filter)
            # CONFIG = 3 -> ~44Hz bandwidth
            self.bus.write_byte_data(self.address, self.CONFIG, 3)
            
            # Configure gyroscope range
            gyro_config = self._get_gyro_config_value()
            self.bus.write_byte_data(self.address, self.GYRO_CONFIG, gyro_config)
            
            # Configure accelerometer range
            accel_config = self._get_accel_config_value()
            self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, accel_config)
            
            # Verify connection by reading WHO_AM_I register (should be 0x68)
            who_am_i = self.bus.read_byte_data(self.address, 0x75)
            if who_am_i != 0x68:
                logger.error(f"MPU6050 WHO_AM_I check failed: 0x{who_am_i:02X} != 0x68")
                return False
            
            logger.info(f"MPU6050 connected successfully (WHO_AM_I: 0x{who_am_i:02X})")
            logger.info(f"Gyro range: ±{self.gyro_range}°/s, Accel range: ±{self.accel_range}g")
            
            return True
            
        except Exception as e:
            logger.error(f"Failed to connect to MPU6050: {e}")
            return False
    
    def _get_gyro_config_value(self) -> int:
        """Get gyroscope configuration register value"""
        if self.gyro_range == 250:
            return 0x00
        elif self.gyro_range == 500:
            return 0x08
        elif self.gyro_range == 1000:
            return 0x10
        else:
            return 0x18
    
    def _get_accel_config_value(self) -> int:
        """Get accelerometer configuration register value"""
        if self.accel_range == 2:
            return 0x00
        elif self.accel_range == 4:
            return 0x08
        elif self.accel_range == 8:
            return 0x10
        else:
            return 0x18
    
    def read_raw_data(self, addr: int) -> int:
        """Read 16-bit signed data from two registers"""
        high = self.bus.read_byte_data(self.address, addr)
        low = self.bus.read_byte_data(self.address, addr + 1)
        
        # Combine high and low bytes
        value = (high << 8) | low
        
        # Convert to signed
        if value > 32768:
            value -= 65536
        
        return value
    
    def read_accel(self) -> np.ndarray:
        """
        Read accelerometer data
        
        Returns:
            numpy array [ax, ay, az] in g
        """
        ax = self.read_raw_data(self.ACCEL_XOUT_H) / self.accel_scale
        ay = self.read_raw_data(self.ACCEL_YOUT_H) / self.accel_scale
        az = self.read_raw_data(self.ACCEL_ZOUT_H) / self.accel_scale
        
        return np.array([ax, ay, az]) - self.accel_offset
    
    def read_gyro(self) -> np.ndarray:
        """
        Read gyroscope data
        
        Returns:
            numpy array [gx, gy, gz] in degrees/sec
        """
        gx = self.read_raw_data(self.GYRO_XOUT_H) / self.gyro_scale
        gy = self.read_raw_data(self.GYRO_YOUT_H) / self.gyro_scale
        gz = self.read_raw_data(self.GYRO_ZOUT_H) / self.gyro_scale
        
        return np.array([gx, gy, gz]) - self.gyro_offset
    
    def read_temperature(self) -> float:
        """
        Read temperature
        
        Returns:
            Temperature in Celsius
        """
        raw_temp = self.read_raw_data(self.TEMP_OUT_H)
        temp = (raw_temp / 340.0) + 36.53
        return temp
    
    def calibrate(self, samples: int = 1000):
        """
        Calibrate IMU by averaging readings while stationary
        
        Args:
            samples: Number of samples to average
        """
        logger.info(f"Calibrating MPU6050 with {samples} samples (keep IMU stationary)...")
        
        gyro_sum = np.zeros(3)
        accel_sum = np.zeros(3)
        
        for i in range(samples):
            gyro_sum += self.read_gyro()
            accel_sum += self.read_accel()
            time.sleep(0.005)  # 5ms between samples
            
            if (i + 1) % 100 == 0:
                logger.info(f"  Calibration progress: {i + 1}/{samples}")
        
        self.gyro_offset = gyro_sum / samples
        
        # For accelerometer, only calibrate X and Y (Z should read ~1g due to gravity)
        accel_avg = accel_sum / samples
        self.accel_offset[0] = accel_avg[0]
        self.accel_offset[1] = accel_avg[1]
        self.accel_offset[2] = accel_avg[2] - 1.0  # Subtract 1g for Z-axis
        
        logger.info(f"Calibration complete!")
        logger.info(f"  Gyro offset: [{self.gyro_offset[0]:.3f}, {self.gyro_offset[1]:.3f}, {self.gyro_offset[2]:.3f}] °/s")
        logger.info(f"  Accel offset: [{self.accel_offset[0]:.3f}, {self.accel_offset[1]:.3f}, {self.accel_offset[2]:.3f}] g")
    
    def start_reading(self):
        """Start continuous reading thread"""
        if not self.running:
            self.running = True
            self.last_update_time = time.time()
            self.reading_thread = threading.Thread(target=self._reading_loop, daemon=True)
            self.reading_thread.start()
            logger.info("MPU6050 continuous reading started")
    
    def stop_reading(self):
        """Stop continuous reading thread"""
        self.running = False
        if self.reading_thread:
            self.reading_thread.join(timeout=2.0)
        logger.info("MPU6050 continuous reading stopped")
    
    def _reading_loop(self):
        """Continuous reading loop with orientation integration"""
        while self.running:
            try:
                current_time = time.time()
                
                # Read sensor data
                accel = self.read_accel()
                gyro = self.read_gyro()
                temp = self.read_temperature()
                
                # Calculate dt
                if self.last_update_time is not None:
                    dt = current_time - self.last_update_time
                    
                    # Integrate gyroscope for orientation
                    # Convert degrees/sec to radians/sec
                    gyro_rad = np.radians(gyro)
                    
                    # Simple integration (can be improved with complementary filter)
                    self.yaw += gyro_rad[2] * dt
                    self.pitch += gyro_rad[0] * dt
                    self.roll += gyro_rad[1] * dt
                    
                    # Normalize angles to [-pi, pi]
                    self.yaw = np.arctan2(np.sin(self.yaw), np.cos(self.yaw))
                    self.pitch = np.arctan2(np.sin(self.pitch), np.cos(self.pitch))
                    self.roll = np.arctan2(np.sin(self.roll), np.cos(self.roll))
                
                # Update latest readings (thread-safe)
                with self.lock:
                    self.latest_accel = accel
                    self.latest_gyro = gyro
                    self.latest_temp = temp
                
                self.last_update_time = current_time
                
                # Sleep to maintain ~100Hz update rate
                time.sleep(0.01)
                
            except Exception as e:
                logger.error(f"MPU6050 reading error: {e}")
                time.sleep(0.1)
    
    def get_accel(self) -> np.ndarray:
        """Get latest accelerometer reading (thread-safe)"""
        with self.lock:
            return self.latest_accel.copy()
    
    def get_gyro(self) -> np.ndarray:
        """Get latest gyroscope reading (thread-safe)"""
        with self.lock:
            return self.latest_gyro.copy()
    
    def get_orientation(self) -> Tuple[float, float, float]:
        """
        Get current orientation from gyro integration
        
        Returns:
            (yaw, pitch, roll) in radians
        """
        return self.yaw, self.pitch, self.roll
    
    def get_yaw(self) -> float:
        """Get current yaw angle (rotation around Z-axis) in radians"""
        return self.yaw
    
    def reset_orientation(self):
        """Reset integrated orientation to zero"""
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        logger.info("MPU6050 orientation reset")
    
    def set_yaw(self, yaw: float):
        """Set current yaw angle manually"""
        self.yaw = yaw
    
    def disconnect(self):
        """Disconnect from IMU"""
        self.stop_reading()
        if self.bus:
            self.bus.close()
        logger.info("MPU6050 disconnected")


class ComplementaryFilter:
    """
    Complementary filter for fusing accelerometer and gyroscope data
    Provides more accurate orientation estimation
    """
    
    def __init__(self, alpha: float = 0.98):
        """
        Initialize complementary filter
        
        Args:
            alpha: Filter coefficient (0-1, higher = trust gyro more)
                   Typical value: 0.98 for 100Hz update rate
        """
        self.alpha = alpha
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
    
    def update(self, accel: np.ndarray, gyro: np.ndarray, dt: float):
        """
        Update orientation estimate
        
        Args:
            accel: Accelerometer reading [ax, ay, az] in g
            gyro: Gyroscope reading [gx, gy, gz] in rad/s
            dt: Time step in seconds
        """
        # Calculate orientation from accelerometer
        accel_pitch = np.arctan2(accel[0], np.sqrt(accel[1]**2 + accel[2]**2))
        accel_roll = np.arctan2(accel[1], np.sqrt(accel[0]**2 + accel[2]**2))
        
        # Integrate gyroscope
        gyro_pitch = self.pitch + gyro[0] * dt
        gyro_roll = self.roll + gyro[1] * dt
        gyro_yaw = self.yaw + gyro[2] * dt
        
        # Complementary filter for pitch and roll
        self.pitch = self.alpha * gyro_pitch + (1 - self.alpha) * accel_pitch
        self.roll = self.alpha * gyro_roll + (1 - self.alpha) * accel_roll
        
        # Yaw only from gyroscope (accelerometer can't measure yaw)
        self.yaw = gyro_yaw
        
        # Normalize angles
        self.yaw = np.arctan2(np.sin(self.yaw), np.cos(self.yaw))
        self.pitch = np.arctan2(np.sin(self.pitch), np.cos(self.pitch))
        self.roll = np.arctan2(np.sin(self.roll), np.cos(self.roll))
    
    def get_orientation(self) -> Tuple[float, float, float]:
        """Get current orientation (yaw, pitch, roll) in radians"""
        return self.yaw, self.pitch, self.roll
    
    def reset(self):
        """Reset orientation to zero"""
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
