"""
Data Acquisition System for Raspberry Pi Pico
- 8MHz clock generation for MCP2515
- ADC128S022 sampling via SPI0
- CD74HC4067 16-channel MUX control 
- SD Card logging via SPI0
- CAN communication via SPI1 using MCP2515
"""
import machine
import utime
import os
import ustruct
import sdcard
import uos

class DataAcquisitionSystem:
    def __init__(self):
        # Define pins
        # SPI0 for ADC and SD Card
        self.spi0 = machine.SPI(0, 
                               baudrate=2000000,
                               polarity=0,
                               phase=0,
                               bits=8,
                               sck=machine.Pin(2),
                               mosi=machine.Pin(3),
                               miso=machine.Pin(4))
        
        # SPI1 for CAN Controller
        self.spi1 = machine.SPI(1,
                               baudrate=10000000,
                               polarity=0,
                               phase=0,
                               bits=8,
                               sck=machine.Pin(10),
                               mosi=machine.Pin(11),
                               miso=machine.Pin(8))
        
        # Chip Select pins
        self.sd_cs = machine.Pin(1, machine.Pin.OUT, value=1)  # SD Card CS
        self.adc_cs = machine.Pin(5, machine.Pin.OUT, value=1)  # ADC CS
        self.can_cs = machine.Pin(9, machine.Pin.OUT, value=1)  # CAN CS
        self.mux_cs = machine.Pin(14, machine.Pin.OUT, value=1)  # MUX CS
        
        # MUX control pins
        self.mux_s0 = machine.Pin(16, machine.Pin.OUT)
        self.mux_s1 = machine.Pin(17, machine.Pin.OUT)
        self.mux_s2 = machine.Pin(18, machine.Pin.OUT)
        self.mux_s3 = machine.Pin(19, machine.Pin.OUT)
        
        # 8MHz clock for MCP2515
        self.can_clk = machine.PWM(machine.Pin(7))
        self.can_clk.freq(8000000)
        self.can_clk.duty_u16(32768)  # 50% duty cycle
        
        # Mount SD card
        self.sd = sdcard.SDCard(self.spi0, self.sd_cs)
        uos.mount(self.sd, '/sd')
        self.log_file = None
        
        # Initialize CAN controller
        self.init_can_controller()
        
        # Data storage
        self.adc_data = [0] * 23  # Total 23 channels: 7 direct + 16 through MUX
        self.last_log_time = 0
        self.last_can_time = 0

    def set_mux_channel(self, channel):
        """Set the MUX channel (0-15)"""
        self.mux_cs.value(0)  # Enable MUX
        self.mux_s0.value(channel & 0x01)
        self.mux_s1.value((channel >> 1) & 0x01)
        self.mux_s2.value((channel >> 2) & 0x01)
        self.mux_s3.value((channel >> 3) & 0x01)
        utime.sleep_us(5)  # Small delay for MUX to settle
    
    def read_adc_channel(self, channel):
        """Read a specific ADC channel (0-7)"""
        # Prepare command: start bit (1) + single-ended (1) + channel (3 bits) + zeros
        command = 0x80 | ((channel & 0x07) << 4)
        
        self.adc_cs.value(0)  # Select ADC
        utime.sleep_us(1)
        
        # Send command and read result
        self.spi0.write(bytes([command, 0x00]))
        result = bytearray(2)
        self.spi0.readinto(result)
        
        self.adc_cs.value(1)  # Deselect ADC
        
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
            utime.sleep_us(50)  # Allow MUX to settle
            self.adc_data[i + 7] = self.read_adc_channel(0)
    
    def log_data_to_sd(self):
        """Log all ADC data to SD card"""
        try:
            # Open file in append mode
            with open('/sd/adc_log.csv', 'a') as f:
                # Write timestamp
                timestamp = utime.time()
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
        self.can_cs.value(0)
        self.spi1.write(bytes([0xC0]))  # RESET command
        self.can_cs.value(1)
        utime.sleep_ms(10)  # Wait for reset
        
        # Configure MCP2515 for 500kbps with 8MHz clock
        self.can_cs.value(0)
        self.spi1.write(bytes([0x02, 0x2A, 0x04]))  # Write to CNF3, CNF2
        self.can_cs.value(1)
        
        self.can_cs.value(0)
        self.spi1.write(bytes([0x02, 0x28, 0x00]))  # Write to CNF1
        self.can_cs.value(1)
        
        # Set normal mode
        self.can_cs.value(0)
        self.spi1.write(bytes([0x02, 0x0F, 0x00]))  # Write to CANCTRL, normal mode
        self.can_cs.value(1)

    def send_can_message(self, id, data):
        """Send data via CAN bus"""
        # Prepare CAN message
        self.can_cs.value(0)
        self.spi1.write(bytes([0x40, 0x08]))  # Load TX buffer 0 command, standard ID
        self.can_cs.value(1)
        
        # ID and data length
        self.can_cs.value(0)
        buffer = bytearray([id >> 3, (id & 0x07) << 5, 0, 0, len(data)])
        buffer.extend(data)
        self.spi1.write(buffer)
        self.can_cs.value(1)
        
        # Request to send
        self.can_cs.value(0)
        self.spi1.write(bytes([0x81]))  # RTS TX buffer 0
        self.can_cs.value(1)
    
    def send_adc_data_via_can(self):
        """Send all ADC data via CAN bus (might need multiple messages)"""
        # We need to split the 23 channels into multiple CAN messages
        # Each CAN message can carry up to 8 bytes
        
        # Convert 12-bit ADC values to bytes (2 bytes per value)
        all_bytes = bytearray()
        for value in self.adc_data:
            all_bytes.extend(ustruct.pack(">H", value))
        
        # Send data in chunks
        chunk_size = 8  # 8 bytes per CAN message
        for i in range(0, len(all_bytes), chunk_size):
            chunk = all_bytes[i:i+chunk_size]
            # Use different IDs for different chunks
            self.send_can_message(0x100 + (i // chunk_size), chunk)
            utime.sleep_ms(5)  # Small delay between messages
    
    def run(self):
        """Main execution loop"""
        print("Data Acquisition System running...")
        
        try:
            # Create log file header if it doesn't exist
            if not 'adc_log.csv' in os.listdir('/sd'):
                with open('/sd/adc_log.csv', 'w') as f:
                    f.write("timestamp")
                    for i in range(23):
                        f.write(f",adc{i}")
                    f.write("\n")
            
            while True:
                current_time = utime.time()
                
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
                utime.sleep_ms(100)
                
        except KeyboardInterrupt:
            print("Program stopped by user")
        finally:
            # Unmount the SD card
            try:
                uos.umount('/sd')
            except:
                pass

# Main program
if __name__ == "__main__":
    system = DataAcquisitionSystem()
    system.run()
