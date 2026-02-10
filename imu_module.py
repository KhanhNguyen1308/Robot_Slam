"""
IMU Module - ADXL345 Accelerometer
I2C Bus 1 interface for motion sensing and orientation
"""
import smbus
import time
import threading
import numpy as np
import logging
from typing import Tuple, Optional
from collections import deque

logger = logging.getLogger(__name__)


class ADXL345:
    """
    ADXL345 3-axis accelerometer driver
    I2C interface on bus 1
    """
    
    # I2C addresses
    ADXL345_ADDRESS = 0x53
    
    # Registers
    POWER_CTL = 0x2D
    DATA_FORMAT = 0x31
    DATAX0 = 0x32
    DATAX1 = 0x33
    DATAY0 = 0x34
    DATAY1 = 0x35
    DATAZ0 = 0x36
    DATAZ1 = 0x37
    
    # Settings
    MEASURE = 0x08
    RANGE_2G = 0x00
    RANGE_4G = 0x01
    RANGE_8G = 0x02
    RANGE_16G = 0x03
    
    def __init__(self, bus=1, address=0x53, g_range=2):
        """
        Initialize ADXL345
        
        Args:
            bus: I2C bus number (default 1)
            address: I2C address (default 0x53)
            g_range: Measurement range in g (2, 4, 8, 16)
        """
        self.bus_num = bus
        self.address = address
        self.bus = None
        
        # Scale factor (LSB/g)
        self.scale_factors = {
            2: 256.0,
            4: 128.0,
            8: 64.0,
            16: 32.0
        }
        self.scale = self.scale_factors.get(g_range, 256.0)
        self.g_range = g_range
        
        # Calibration offsets
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.offset_z = 0.0
        
        # Current readings
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 0.0
        
        # Derived measurements
        self.pitch = 0.0
        self.roll = 0.0
        
        # Connected flag
        self.connected = False
        
    def connect(self) -> bool:
        """Initialize I2C connection and configure sensor"""
        try:
            self.bus = smbus.SMBus(self.bus_num)
            
            # Set measurement mode
            self.bus.write_byte_data(self.address, self.POWER_CTL, self.MEASURE)
            
            # Set range
            range_code = {2: self.RANGE_2G, 4: self.RANGE_4G, 
                         8: self.RANGE_8G, 16: self.RANGE_16G}
            self.bus.write_byte_data(self.address, self.DATA_FORMAT, 
                                    range_code.get(self.g_range, self.RANGE_2G))
            
            # Test read
            self.read_raw()
            
            self.connected = True
            logger.info(f"ADXL345 connected on I2C bus {self.bus_num}, range ±{self.g_range}g")
            return True
            
        except Exception as e:
            logger.error(f"Failed to connect to ADXL345: {e}")
            self.connected = False
            return False
    
    def read_raw(self) -> Tuple[int, int, int]:
        """Read raw accelerometer values"""
        if not self.connected:
            return (0, 0, 0)
        
        try:
            # Read 6 bytes starting from DATAX0
            data = self.bus.read_i2c_block_data(self.address, self.DATAX0, 6)
            
            # Convert to signed 16-bit values
            x = np.int16((data[1] << 8) | data[0])
            y = np.int16((data[3] << 8) | data[2])
            z = np.int16((data[5] << 8) | data[4])
            
            return (x, y, z)
            
        except Exception as e:
            logger.error(f"Failed to read ADXL345: {e}")
            return (0, 0, 0)
    
    def read_accel(self) -> Tuple[float, float, float]:
        """
        Read acceleration in g
        
        Returns:
            (ax, ay, az) in g units
        """
        x_raw, y_raw, z_raw = self.read_raw()
        
        # Convert to g and apply calibration
        self.accel_x = (x_raw / self.scale) - self.offset_x
        self.accel_y = (y_raw / self.scale) - self.offset_y
        self.accel_z = (z_raw / self.scale) - self.offset_z
        
        return (self.accel_x, self.accel_y, self.accel_z)
    
    def calibrate(self, samples=100):
        """
        Calibrate accelerometer (must be level)
        
        Args:
            samples: Number of samples to average
        """
        logger.info("Calibrating ADXL345... Keep sensor level and still")
        
        sum_x, sum_y, sum_z = 0, 0, 0
        
        for i in range(samples):
            x, y, z = self.read_accel()
            sum_x += x
            sum_y += y
            sum_z += z
            time.sleep(0.01)
        
        self.offset_x = sum_x / samples
        self.offset_y = sum_y / samples
        self.offset_z = (sum_z / samples) - 1.0  # Should read 1g when level
        
        logger.info(f"Calibration complete: offset=({self.offset_x:.3f}, {self.offset_y:.3f}, {self.offset_z:.3f})")
    
    def get_orientation(self) -> Tuple[float, float]:
        """
        Calculate pitch and roll from accelerometer
        
        Returns:
            (pitch, roll) in degrees
        """
        ax, ay, az = self.read_accel()
        
        # Calculate pitch and roll
        self.pitch = np.degrees(np.arctan2(ay, np.sqrt(ax**2 + az**2)))
        self.roll = np.degrees(np.arctan2(-ax, az))
        
        return (self.pitch, self.roll)
    
    def get_magnitude(self) -> float:
        """Get total acceleration magnitude"""
        ax, ay, az = self.read_accel()
        return np.sqrt(ax**2 + ay**2 + az**2)
    
    def close(self):
        """Close I2C connection"""
        if self.bus:
            self.bus.close()
        self.connected = False


class IMUFilter:
    """
    IMU data filtering and sensor fusion
    Combines accelerometer with complementary filter
    """
    
    def __init__(self, imu: ADXL345, alpha=0.98, update_rate=50):
        """
        Args:
            imu: ADXL345 instance
            alpha: Complementary filter coefficient (0.9-0.99)
            update_rate: Update frequency in Hz
        """
        self.imu = imu
        self.alpha = alpha
        self.update_rate = update_rate
        self.dt = 1.0 / update_rate
        
        # Filtered orientation
        self.pitch = 0.0
        self.roll = 0.0
        
        # Velocity estimation (dead reckoning)
        self.vel_x = 0.0
        self.vel_y = 0.0
        
        # Position estimation
        self.pos_x = 0.0
        self.pos_y = 0.0
        
        # Acceleration history for velocity estimation
        self.accel_history = deque(maxlen=10)
        
        # Running thread
        self.running = False
        self.thread = None
        
    def start(self):
        """Start IMU filtering thread"""
        if self.running:
            return
        
        self.running = True
        self.thread = threading.Thread(target=self._update_loop, daemon=True)
        self.thread.start()
        logger.info("IMU filter started")
    
    def stop(self):
        """Stop IMU filtering thread"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        logger.info("IMU filter stopped")
    
    def _update_loop(self):
        """Main update loop"""
        last_time = time.time()
        
        while self.running:
            current_time = time.time()
            dt = current_time - last_time
            
            # Get raw orientation from accelerometer
            pitch_accel, roll_accel = self.imu.get_orientation()
            
            # Complementary filter (without gyro, just low-pass filter)
            self.pitch = self.alpha * self.pitch + (1 - self.alpha) * pitch_accel
            self.roll = self.alpha * self.roll + (1 - self.alpha) * roll_accel
            
            # Get acceleration
            ax, ay, az = self.imu.read_accel()
            self.accel_history.append((ax, ay, az, current_time))
            
            # Estimate velocity (very rough - needs proper integration)
            if len(self.accel_history) >= 2:
                # Simple trapezoidal integration
                prev_accel = self.accel_history[-2]
                dt_accel = current_time - prev_accel[3]
                
                # Only integrate if acceleration is significant (noise filtering)
                if abs(ax) > 0.1:
                    self.vel_x += (ax + prev_accel[0]) / 2 * dt_accel * 9.81
                else:
                    self.vel_x *= 0.95  # Decay
                
                if abs(ay) > 0.1:
                    self.vel_y += (ay + prev_accel[1]) / 2 * dt_accel * 9.81
                else:
                    self.vel_y *= 0.95  # Decay
            
            last_time = current_time
            
            # Sleep to maintain update rate
            time.sleep(max(0, self.dt - (time.time() - current_time)))
    
    def get_orientation(self) -> Tuple[float, float]:
        """Get filtered pitch and roll"""
        return (self.pitch, self.roll)
    
    def get_velocity_estimate(self) -> Tuple[float, float]:
        """Get estimated velocity (very rough)"""
        return (self.vel_x, self.vel_y)
    
    def detect_collision(self, threshold=2.0) -> bool:
        """
        Detect sudden acceleration (collision)
        
        Args:
            threshold: Acceleration threshold in g
        
        Returns:
            True if collision detected
        """
        magnitude = self.imu.get_magnitude()
        return magnitude > threshold
    
    def is_tilted(self, max_angle=15.0) -> bool:
        """
        Check if robot is tilted beyond threshold
        
        Args:
            max_angle: Maximum tilt angle in degrees
        
        Returns:
            True if tilted beyond threshold
        """
        return abs(self.pitch) > max_angle or abs(self.roll) > max_angle


# Example usage
if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    
    # Initialize IMU
    imu = ADXL345(bus=1, g_range=2)
    
    if not imu.connect():
        print("Failed to connect to IMU")
        exit(1)
    
    # Calibrate
    print("Keep sensor level and still for calibration...")
    time.sleep(2)
    imu.calibrate()
    
    # Start filtering
    filter = IMUFilter(imu)
    filter.start()
    
    try:
        print("\nReading IMU data (Ctrl+C to stop):")
        while True:
            # Raw acceleration
            ax, ay, az = imu.read_accel()
            
            # Filtered orientation
            pitch, roll = filter.get_orientation()
            
            # Magnitude
            mag = imu.get_magnitude()
            
            print(f"Accel: ({ax:6.2f}, {ay:6.2f}, {az:6.2f})g  "
                  f"Orient: P={pitch:6.1f}° R={roll:6.1f}°  "
                  f"Mag: {mag:.2f}g", end='\r')
            
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\n\nStopping...")
        filter.stop()
        imu.close()