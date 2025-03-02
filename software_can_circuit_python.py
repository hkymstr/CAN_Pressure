"""
Data Acquisition System for Raspberry Pi Pico using CircuitPython
- 8MHz clock generation for MCP2515
- ADC128S022 sampling via SPI0
- CD74HC4067 16-channel MUX control 
- SD Card logging via SPI0
- CAN communication via SPI1 using MCP2515
"""
import board
import busio
import digitalio
import pwmio
import time
import os
import struct
import adafruit_sdcard
import storage
import microcontroller

class DataAcquisitionSystem:
    def __init__(self):
        # Define pins
        # SPI0 for ADC and SD Card
        self.spi0 = busio.SPI(board.GP2, board.GP3, board.GP4)  # SCK, MOSI, MISO
        
        # SPI1 for CAN Controller
        self.spi1 = busio.SPI(board.GP10, board.GP11, board.GP8)  # SCK, MOSI, MISO
        
        # Chip Select pins
        self.sd_cs = digitalio.DigitalInOut(board.GP1)
        self.sd_cs.direction = digitalio.Direction.OUTPUT
        self.sd_cs.value = True  # Deselected
        
        self.adc_cs = digitalio.DigitalInOut(board.GP5)
        self.adc_cs.direction = digitalio.Direction.OUTPUT
        self.adc_cs.value = True  # Deselected
        
        self.can_cs = digitalio.DigitalInOut(board.GP9)
        self.can_cs.direction = digitalio.Direction.OUTPUT
        self.can_cs.value = True  # Deselected
        
        self.mux_cs = digitalio.DigitalInOut(board.GP14)
        self.mux_cs.direction = digitalio.Direction.OUTPUT
        self.mux_cs.value = True  # Deselected
        
        # MUX control pins
        self.mux_s0 = digitalio.DigitalInOut(board.GP16)
        self.mux_s0.direction = digitalio.Direction.OUTPUT
        self.mux_s1 = digitalio.DigitalInOut(board.GP17)
        self.mux_s1.direction = digitalio.Direction.OUTPUT
        self.mux_s2 = digitalio.DigitalInOut(board.GP18)
        self.mux_s2.direction = digitalio.Direction.OUTPUT
        self.mux_s3 = digitalio.DigitalInOut(board.GP19)
        self.mux_s3.direction = digitalio.Direction.OUTPUT
        
        # 8MHz clock for MCP2515
        self.can_clk = pwmio.PWMOut(board.GP7, frequency=8000000, duty_cycle=32768)  # 50% duty cycle
        
        # Mount SD card
        try:
            self.sdcard = adafruit_sdcard.SDCard(self.spi0, self.sd_cs)
            self.vfs = storage.VfsFat(self.sdcard)
            storage.mount(self.vfs, "/sd")
            print("SD card mounted successfully")
        except Exception as e:
            print(f"Error mounting SD card: {e}")
        
        # Initialize CAN controller
        self.init_can_controller()
        
        # Data storage
        self.adc_data = [0] * 23  # Total 23 channels: 7 direct + 16 through MUX
        self.last_log_time = time.monotonic()
        self.last_can_time = time.monotonic()

    def set_mux_channel(self, channel):
        """Set the MUX channel (0-15)"""
        self.mux_cs.value = False  # Enable MUX
        self.mux_s0.value = bool(channel & 0x01)
        self.mux_s1.value = bool((channel >> 1) & 0x01)
        self.mux_s2.value = bool((channel >> 2) & 0x01)
        self.mux_s3.value = bool((channel >> 3) & 0x01)
        time.sleep(0.000005)  # 5 microseconds for MUX to settle
    
    def read_adc_channel(self, channel):
        """Read a specific ADC channel (0-7)"""
        # Prepare command: start bit (1) + single-ended (1) + channel (3 bits) + zeros
        command = 0x80 | ((channel & 0x07) << 4)
        
        self.adc_cs.value = False  # Select ADC
        time.sleep(0.000001)  # 1 microsecond delay
        
        # Send command and read result
        while not self.spi0.try_lock():
            pass
        
        try:
            self.spi0.configure(baudrate=2000000, phase=0, polarity=0)
            result = bytearray(2)
            self.spi0.write_readinto(bytes([command, 0x00]), result)
        finally:
            self.spi0.unlock()
        
        self.adc_cs.value = True  # Deselect ADC
        
        # Convert result (12-bit value)
        value = ((result[0] & 0x0F) << 8) | result[1]
        return value
    
    def sample_all_channels(self):
        """Sample all 23 ADC channels (7 direct + 16 through MUX)"""
        # First sample channels 1-7 directly
        for i in range(1, 8):
            self.adc_data[i] = self.read_adc_channel(i)
        
        # Then sample channels through MUX (connected to ADC channel 0)
        for i in range(16):
            self.set_mux_channel(i)
            time.sleep(0.00005)  # 50 microseconds to allow MUX to settle
            self.adc_data[i + 7] = self.read_adc_channel(0)
    
    def log_data_to_sd(self):
        """Log all ADC data to SD card"""
        try:
            # Check if directory exists
            try:
                os.listdir("/sd")
            except OSError:
                print("SD card directory issue - remounting")
                try:
                    storage.mount(self.vfs, "/sd")
                except Exception as e:
                    print(f"Remount failed: {e}")
                    return
            
            # Open file in append mode
            log_path = "/sd/adc_log.csv"
            
            # Check if file exists, if not create with header
            try:
                os.stat(log_path)
                file_exists = True
            except OSError:
                file_exists = False
            
            if not file_exists:
                with open(log_path, 'w') as f:
                    f.write("timestamp")
                    for i in range(23):
                        f.write(f",adc{i}")
                    f.write("\n")
            
            # Append data
            with open(log_path, 'a') as f:
                # Write timestamp
                timestamp = time.monotonic()
                f.write(f"{timestamp}")
                
                # Write all ADC values
                for value in self.adc_data:
                    f.write(f",{value}")
                
                f.write("\n")
            
        except Exception as e:
            print(f"Error logging to SD card: {e}")
    
    def init_can_controller(self):
        """Initialize the MCP2515 CAN controller"""
        # Reset MCP2515
        self.can_cs.value = False
        
        while not self.spi1.try_lock():
            pass
        
        try:
            self.spi1.configure(baudrate=10000000, phase=0, polarity=0)
            self.spi1.write(bytes([0xC0]))  # RESET command
        finally:
            self.spi1.unlock()
            
        self.can_cs.value = True
        time.sleep(0.01)  # 10ms wait for reset
        
        # Configure MCP2515 for 500kbps with 8MHz clock
        while not self.spi1.try_lock():
            pass
            
        try:
            self.spi1.configure(baudrate=10000000, phase=0, polarity=0)
            
            # Set Configuration mode
            self.can_cs.value = False
            self.spi1.write(bytes([0x02, 0x0F, 0x80]))  # Write to CANCTRL, config mode
            self.can_cs.value = True
            time.sleep(0.001)
            
            # Set bit timing registers
            self.can_cs.value = False
            self.spi1.write(bytes([0x02, 0x2A, 0x04]))  # Write to CNF3, CNF2
            self.can_cs.value = True
            
            self.can_cs.value = False
            self.spi1.write(bytes([0x02, 0x28, 0x00]))  # Write to CNF1
            self.can_cs.value = True
            
            # Set normal mode
            self.can_cs.value = False
            self.spi1.write(bytes([0x02, 0x0F, 0x00]))  # Write to CANCTRL, normal mode
            self.can_cs.value = True
        finally:
            self.spi1.unlock()

    def send_can_message(self, id, data):
        """Send data via CAN bus"""
        while not self.spi1.try_lock():
            pass
            
        try:
            # Prepare CAN message
            self.can_cs.value = False
            self.spi1.write(bytes([0x40, 0x08]))  # Load TX buffer 0 command, standard ID
            self.can_cs.value = True
            
            # ID and data length
            self.can_cs.value = False
            buffer = bytearray([id >> 3, (id & 0x07) << 5, 0, 0, len(data)])
            buffer.extend(data)
            self.spi1.write(buffer)
            self.can_cs.value = True
            
            # Request to send
            self.can_cs.value = False
            self.spi1.write(bytes([0x81]))  # RTS TX buffer 0
            self.can_cs.value = True
        finally:
            self.spi1.unlock()
    
    def send_adc_data_via_can(self):
        """Send all ADC data via CAN bus (might need multiple messages)"""
        # We need to split the 23 channels into multiple CAN messages
        # Each CAN message can carry up to 8 bytes
        
        # Convert 12-bit ADC values to bytes (2 bytes per value)
        all_bytes = bytearray()
        for value in self.adc_data:
            all_bytes.extend(struct.pack(">H", value))
        
        # Send data in chunks
        chunk_size = 8  # 8 bytes per CAN message
        for i in range(0, len(all_bytes), chunk_size):
            chunk = all_bytes[i:i+min(chunk_size, len(all_bytes)-i)]
            # Use different IDs for different chunks
            self.send_can_message(0x100 + (i // chunk_size), chunk)
            time.sleep(0.005)  # 5ms delay between messages
    
    def run(self):
        """Main execution loop"""
        print("Data Acquisition System running...")
        
        try:
            while True:
                current_time = time.monotonic()
                
                # Sample all ADC channels
                self.sample_all_channels()
                
                # Log data to SD card once per second
                if current_time - self.last_log_time >= 1:
                    self.log_data_to_sd()
                    self.last_log_time = current_time
                
                # Send data via CAN twice per second
                if current_time - self.last_can_time >= 0.5:
                    self.send_adc_data_via_can()
                    self.last_can_time = current_time
                
                # Small delay to prevent busy waiting
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("Program stopped by user")
        finally:
            # Unmount the SD card
            try:
                storage.umount('/sd')
            except:
                pass

# Main program
if __name__ == "__main__":
    system = DataAcquisitionSystem()
    system.run()
