import os
import json
import serial
import queue
import threading
import time
import sys
import signal
import sounddevice as sd
from vosk import Model, KaldiRecognizer
from datetime import datetime
import logging
from typing import Optional, List, Dict, Any, Tuple
import struct
from dataclasses import dataclass
from enum import IntEnum

# --- PACKET PROTOCOL CONSTANTS ---
FRAME_START = 0xAA

class PacketType(IntEnum):
    COMMAND   = 0x01
    ACK       = 0x02
    ERROR     = 0x03
    SENSOR    = 0x10
    HEARTBEAT = 0x20
    EMERGENCY = 0x30

# --- CONFIGURATION ---
SERIAL_PORT = '/dev/ttyO1'  # BBB UART1
BAUD_RATE = 115200  # Higher baud for packet communication
MODEL_PATH = "model"        # Folder containing Vosk offline model
AUDIO_SAMPLE_RATE = 16000
AUDIO_BLOCK_SIZE = 8000
LISTEN_TIMEOUT = 30         # Seconds before timeout
MAX_RETRIES = 3
ACK_TIMEOUT = 0.2          # 200ms for ACK confirmation
HEARTBEAT_INTERVAL = 0.5   # 500ms heartbeat
OBSTACLE_THRESHOLD = 10    # 10cm obstacle distance threshold
EMERGENCY_STOP_TIMEOUT = 2.0  # 2 seconds without heartbeat
MAX_PACKET_SIZE = 64
MAX_COMMAND_RETRIES = 3

# Logging setup
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('robot_brain.log'),
        logging.StreamHandler(sys.stdout)
    ]
)
logger = logging.getLogger(__name__)

@dataclass
class Packet:
    """Packet structure for reliable communication."""
    ptype: int
    payload: bytes = b''
    timestamp: float = 0.0
    
    def encode(self) -> bytes:
        """Encode packet to bytes with CRC."""
        header = bytes([FRAME_START, self.ptype, len(self.payload)])
        frame = header + self.payload
        crc = crc8(frame)
        return frame + bytes([crc])

def crc8(data: bytes) -> int:
    """Calculate CRC-8 for data verification."""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0x07
            else:
                crc = crc << 1
            crc &= 0xFF
    return crc

class GracefulKiller:
    """Handle shutdown gracefully."""
    kill_now = False
    
    def __init__(self):
        signal.signal(signal.SIGINT, self._handle_signal)
        signal.signal(signal.SIGTERM, self._handle_signal)
    
    def _handle_signal(self, signum, frame):
        logger.info(f"Received signal {signum}. Shutting down gracefully...")
        self.killer.kill_now = True

class RobotBrain:
    def __init__(self):
        self.killer = GracefulKiller()
        self.arduino: Optional[serial.Serial] = None
        self.model: Optional[Model] = None
        self.rec: Optional[KaldiRecognizer] = None
        self.audio_queue = queue.Queue(maxsize=100)
        self.is_listening = False
        self.command_queue = queue.Queue(maxsize=50)
        self.packet_queue = queue.Queue(maxsize=100)
        self.heartbeat_running = False
        self.emergency_state = False
        self.last_command_time = 0
        self.command_acks = {}  # Track pending command ACKs
        
        self.health_status = {
            'arduino': False,
            'audio': False,
            'stt': False,
            'arduino_ack': False,
            'last_heartbeat': 0,
            'last_sensor': 0,
            'obstacle_detected': False,
            'distance_cm': 0,
            'packets_sent': 0,
            'packets_received': 0,
            'crc_errors': 0,
            'uptime': 0.0
        }
        self.command_history = []
        
        self._initialize_hardware()
        self._initialize_stt()
        self._start_background_threads()
        
        logger.info("RobotBrain initialized with packet protocol")

    def _initialize_hardware(self):
        """Initialize Arduino connection with retries and handshake."""
        for attempt in range(MAX_RETRIES):
            try:
                self.arduino = serial.Serial(
                    SERIAL_PORT, 
                    BAUD_RATE, 
                    timeout=0.1,  # Short timeout for packet reading
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE
                )
                time.sleep(2)  # Allow Arduino reset
                
                # Test handshake with packet protocol
                if self._handshake_test():
                    self.health_status['arduino'] = True
                    logger.info(f"Arduino connected and responding on {SERIAL_PORT}")
                    return
                else:
                    logger.warning(f"Arduino connected but not responding to handshake (attempt {attempt + 1})")
                    self.arduino.close()
                    self.arduino = None
                    
            except serial.SerialException as e:
                logger.warning(f"Arduino connection attempt {attempt + 1} failed: {e}")
                if attempt == MAX_RETRIES - 1:
                    logger.error("Arduino connection failed permanently. Running in simulation mode.")
                    self.arduino = None
                else:
                    time.sleep(1)

    def _handshake_test(self) -> bool:
        """Test communication with Arduino using packet protocol."""
        if not self.arduino:
            return False
            
        # Send test packet and wait for ACK
        test_packet = Packet(ptype=PacketType.COMMAND, payload=b'T')
        try:
            self._send_packet_raw(test_packet)
            
            # Wait for ACK
            start_time = time.time()
            while time.time() - start_time < ACK_TIMEOUT:
                self._read_packets()  # Process incoming packets
                if self.health_status['arduino_ack']:
                    logger.debug("Packet handshake successful with Arduino")
                    return True
                time.sleep(0.001)
                
            logger.warning("No ACK received from Arduino during handshake")
            return False
        except Exception as e:
            logger.error(f"Handshake test failed: {e}")
            return False

    def _send_packet_raw(self, packet: Packet) -> bool:
        """Send raw packet to Arduino without ACK waiting."""
        if not self.arduino or not self.arduino.is_open:
            return False
            
        try:
            data = packet.encode()
            self.arduino.write(data)
            self.arduino.flush()
            self.health_status['packets_sent'] += 1
            logger.debug(f"Sent packet type {packet.ptype:02X}, size {len(packet.payload)}")
            return True
        except Exception as e:
            logger.error(f"Packet send failed: {e}")
            self.health_status['arduino'] = False
            return False

    def send_packet_with_ack(self, packet: Packet, max_retries: int = MAX_COMMAND_RETRIES) -> Tuple[bool, str]:
        """Send packet and wait for ACK with retries."""
        if not self.arduino or not self.arduino.is_open:
            return False, "Arduino not connected"
            
        # Generate command ID for tracking
        cmd_id = struct.pack('H', self.health_status['packets_sent'] & 0xFFFF)
        packet_with_id = Packet(ptype=packet.ptype, payload=cmd_id + packet.payload)
        
        for attempt in range(max_retries):
            # Send packet
            if not self._send_packet_raw(packet_with_id):
                return False, "Send failed"
                
            # Wait for ACK
            start_time = time.time()
            while time.time() - start_time < ACK_TIMEOUT:
                self._read_packets()
                
                # Check if this specific command was acknowledged
                expected_ack_id = self.health_status['packets_sent']
                if expected_ack_id in self.command_acks:
                    del self.command_acks[expected_ack_id]
                    logger.debug(f"Command ACK received for packet {expected_ack_id}")
                    return True, ""
                    
                time.sleep(0.001)
                
            logger.warning(f"ACK timeout for packet {self.health_status['packets_sent']}, attempt {attempt + 1}")
            
        return False, "ACK timeout after retries"

    def _read_packets(self) -> int:
        """Read and process packets from Arduino. Returns number of packets processed."""
        if not self.arduino or not self.arduino.is_open:
            return 0
            
        packets_processed = 0
        try:
            while self.arduino.in_waiting >= 4:  # Minimum packet size
                # Look for frame start
                if self.arduino.read(1) != bytes([FRAME_START]):
                    continue
                
                # Read header
                header = self.arduino.read(2)
                if len(header) < 2:
                    continue
                    
                ptype = header[0]
                length = header[1]
                
                if length > MAX_PACKET_SIZE:
                    logger.error(f"Packet too large: {length} bytes")
                    continue
                    
                # Read payload and CRC
                payload = self.arduino.read(length)
                if len(payload) < length:
                    continue
                    
                crc_rx = self.arduino.read(1)
                if len(crc_rx) < 1:
                    continue
                
                # Verify CRC
                frame = bytes([FRAME_START, ptype, length]) + payload
                if crc8(frame) != crc_rx[0]:
                    self.health_status['crc_errors'] += 1
                    logger.warning(f"CRC error in packet type {ptype:02X}")
                    continue
                
                # Process valid packet
                self.health_status['packets_received'] += 1
                packet = Packet(ptype=ptype, payload=payload, timestamp=time.time())
                
                try:
                    self._process_packet(packet)
                except Exception as e:
                    logger.error(f"Error processing packet: {e}")
                    
                packets_processed += 1
                
        except Exception as e:
            logger.error(f"Error reading packets: {e}")
            
        return packets_processed

    def _process_packet(self, packet: Packet):
        """Process incoming packet based on type."""
        if packet.ptype == PacketType.ACK:
            self.health_status['arduino_ack'] = True
            self.health_status['last_heartbeat'] = packet.timestamp
            
            # Extract command ID if present
            if len(packet.payload) >= 2:
                cmd_id = struct.unpack('H', packet.payload[:2])[0]
                self.command_acks[cmd_id] = True
                logger.debug(f"Received ACK for command {cmd_id}")
            
        elif packet.ptype == PacketType.SENSOR:
            if len(packet.payload) >= 2:
                distance = struct.unpack('>H', packet.payload[:2])[0]
                self.health_status['distance_cm'] = distance
                self.health_status['last_sensor'] = packet.timestamp
                
                # Check for obstacle
                if distance < OBSTACLE_THRESHOLD and distance > 0:
                    self.health_status['obstacle_detected'] = True
                    # Trigger emergency stop if moving
                    if not self.emergency_state:
                        self.emergency_stop("Obstacle detected")
                else:
                    self.health_status['obstacle_detected'] = False
                    
                logger.debug(f"Sensor reading: {distance}cm")
                
        elif packet.ptype == PacketType.ERROR:
            error_code = packet.payload[0] if packet.payload else 0
            logger.error(f"Arduino reported error: {error_code:02X}")
            
        elif packet.ptype == PacketType.EMERGENCY:
            logger.critical("Emergency packet received from Arduino")
            self.emergency_state = True
            self.speak("Emergency signal received!", priority="high")
            
        else:
            logger.warning(f"Unknown packet type: {packet.ptype:02X}")

    def _initialize_stt(self):
        """Initialize Vosk speech recognition."""
        if not os.path.exists(MODEL_PATH):
            logger.error(f"Vosk model not found at {MODEL_PATH}")
            sys.exit(1)
            
        try:
            self.model = Model(MODEL_PATH)
            self.rec = KaldiRecognizer(self.model, AUDIO_SAMPLE_RATE)
            self.rec.SetWords(True)
            self.health_status['stt'] = True
            logger.info("Vosk STT initialized successfully")
        except Exception as e:
            logger.error(f"STT initialization failed: {e}")
            sys.exit(1)

    def _start_background_threads(self):
        """Start all background threads."""
        # Health monitor thread
        self.health_thread = threading.Thread(target=self._health_monitor, daemon=True)
        self.health_thread.start()
        
        # Command processor thread
        self.command_thread = threading.Thread(target=self._command_processor, daemon=True)
        self.command_thread.start()
        
        # Heartbeat thread
        self.heartbeat_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self.heartbeat_thread.start()
        
        # Packet reader thread
        self.packet_thread = threading.Thread(target=self._packet_reader_loop, daemon=True)
        self.packet_thread.start()
        
        # Emergency monitor thread
        self.emergency_thread = threading.Thread(target=self._emergency_monitor, daemon=True)
        self.emergency_thread.start()

    def _packet_reader_loop(self):
        """Background thread to continuously read packets."""
        logger.info("Packet reader thread started")
        
        while not self.killer.kill_now:
            try:
                packets_read = self._read_packets()
                if packets_read == 0:
                    time.sleep(0.001)  # Small sleep to prevent CPU spin
            except Exception as e:
                logger.error(f"Packet reader error: {e}")
                time.sleep(0.1)

    def _heartbeat_loop(self):
        """Background thread sending heartbeat packets."""
        self.heartbeat_running = True
        logger.info("Heartbeat thread started")
        
        while not self.killer.kill_now and self.heartbeat_running:
            try:
                heartbeat_packet = Packet(ptype=PacketType.HEARTBEAT)
                self._send_packet_raw(heartbeat_packet)
                self.health_status['last_heartbeat'] = time.time()
                time.sleep(HEARTBEAT_INTERVAL)
            except Exception as e:
                logger.error(f"Heartbeat error: {e}")
                time.sleep(1)

    def _emergency_monitor(self):
        """Monitor for emergency conditions."""
        logger.info("Emergency monitor thread started")
        
        while not self.killer.kill_now:
            try:
                # Check heartbeat timeout
                if time.time() - self.health_status['last_heartbeat'] > EMERGENCY_STOP_TIMEOUT:
                    if not self.emergency_state:
                        self.emergency_stop("Heartbeat lost")
                
                # Check for stale ACK
                if not self.health_status['arduino_ack']:
                    # Try to recover by sending test packet
                    test_packet = Packet(ptype=PacketType.COMMAND, payload=b'T')
                    self._send_packet_raw(test_packet)
                
                time.sleep(0.1)
            except Exception as e:
                logger.error(f"Emergency monitor error: {e}")
                time.sleep(1)

    def speak(self, text: str, priority: str = "normal") -> bool:
        """Enhanced speech synthesis with error handling."""
        try:
            timestamp = datetime.now().strftime("%H:%M:%S")
            prefix = f"[{timestamp}] "
            full_text = prefix + text
            
            print(f"🤖 ROBOT: {text}")
            logger.info(f"Speech: {text}")
            
            # eSpeak with enhanced parameters
            cmd = (
                f'espeak -ven+f3 -s150 -p50 -a120 '
                f'--stdout "{full_text}" | aplay 2>/dev/null || '
                f'espeak -ven+f3 -s150 -p50 -a120 "{full_text}"'
            )
            os.system(cmd)
            return True
        except Exception as e:
            logger.error(f"Speech failed: {e}")
            return False

    def move(self, cmd: str) -> Tuple[bool, str]:
        """Send movement command with packet protocol."""
        valid_cmds = {'F', 'B', 'L', 'R', 'S', 'U', 'D'}
        if cmd not in valid_cmds:
            return False, "Invalid command"
            
        # Check for obstacle before movement commands
        if cmd != 'S' and self.health_status['obstacle_detected']:
            return False, "Obstacle detected"
            
        packet = Packet(ptype=PacketType.COMMAND, payload=cmd.encode())
        success, error = self.send_packet_with_ack(packet)
        
        if success:
            self.last_command_time = time.time()
            self.command_history.append({
                'cmd': cmd, 
                'ack': True, 
                'time': self.last_command_time,
                'packet_id': self.health_status['packets_sent']
            })
            logger.info(f"Movement command '{cmd}' sent and acknowledged")
        else:
            logger.error(f"Movement command '{cmd}' failed: {error}")
            
        return success, error

    def emergency_stop(self, reason: str = "Emergency stop"):
        """Hard emergency stop that overrides everything."""
        logger.critical(f"EMERGENCY STOP: {reason}")
        
        # Send emergency stop command
        packet = Packet(ptype=PacketType.EMERGENCY, payload=b'S')
        for _ in range(3):  # Send multiple times for reliability
            self._send_packet_raw(packet)
            time.sleep(0.01)
        
        # Clear command queue
        while not self.command_queue.empty():
            try:
                self.command_queue.get_nowait()
                self.command_queue.task_done()
            except queue.Empty:
                break
        
        # Set emergency state
        self.emergency_state = True
        
        # Speak warning
        self.speak(f"Emergency! {reason}", priority="high")
        
        # Log emergency
        self.command_history.append({
            'cmd': 'EMERGENCY_STOP',
            'reason': reason,
            'time': time.time()
        })

    def _command_processor(self):
        """Process commands from queue asynchronously."""
        logger.info("Command processor thread started")
        
        while not self.killer.kill_now:
            try:
                # Get command from queue with timeout
                cmd, cmd_type = self.command_queue.get(timeout=0.1)
                
                # Skip if in emergency state
                if self.emergency_state and cmd_type == 'movement' and cmd != 'S':
                    logger.warning(f"Ignoring movement command '{cmd}' during emergency")
                    self.command_queue.task_done()
                    continue
                
                # Process the command
                if cmd_type == 'movement':
                    success, error = self.move(cmd)
                    if not success:
                        self.speak(f"Command failed: {error}")
                        if "Obstacle" in error:
                            self.emergency_stop("Obstacle detected")
                elif cmd_type == 'voice':
                    self.process_logic(cmd)
                    
                self.command_queue.task_done()
                
            except queue.Empty:
                continue
            except Exception as e:
                logger.error(f"Command processor error: {e}")

    def queue_command(self, cmd: str, cmd_type: str = 'voice'):
        """Queue a command for asynchronous processing."""
        try:
            self.command_queue.put((cmd, cmd_type), block=False)
            return True
        except queue.Full:
            logger.warning(f"Command queue full, dropping command: {cmd}")
            return False

    def get_status_report(self) -> Dict[str, Any]:
        """Comprehensive system status."""
        return {
            'timestamp': datetime.now().isoformat(),
            'health': self.health_status.copy(),
            'command_history': self.command_history[-10:],
            'listening': self.is_listening,
            'audio_queue_size': self.audio_queue.qsize(),
            'command_queue_size': self.command_queue.qsize(),
            'emergency_state': self.emergency_state,
            'obstacle_detected': self.health_status['obstacle_detected'],
            'distance_cm': self.health_status['distance_cm'],
            'packets_sent': self.health_status['packets_sent'],
            'packets_received': self.health_status['packets_received'],
            'crc_errors': self.health_status['crc_errors'],
            'arduino_connected': self.health_status['arduino'],
            'ack_status': self.health_status['arduino_ack']
        }

    def process_logic(self, text: str) -> bool:
        """Advanced command processing with natural language understanding."""
        text_lower = text.lower().strip()
        self.command_history.append({'text': text, 'processed': True, 'time': time.time()})
        
        # Emergency commands
        if any(word in text_lower for word in ["emergency", "stop now", "halt immediately", "abort"]):
            self.emergency_stop("Voice command")
            return True
            
        # Movement commands (queued for async processing)
        if any(word in text_lower for word in ["forward", "avance", "go", "ahead"]):
            self.speak("Moving forward.", "high")
            return self.queue_command('F', 'movement')
        elif any(word in text_lower for word in ["back", "reverse", "arrière", "backward"]):
            self.speak("Moving backward.", "high")
            return self.queue_command('B', 'movement')
        elif any(word in text_lower for word in ["left", "gauche"]):
            self.speak("Turning left.", "high")
            return self.queue_command('L', 'movement')
        elif any(word in text_lower for word in ["right", "droite"]):
            self.speak("Turning right.", "high")
            return self.queue_command('R', 'movement')
        elif any(word in text_lower for word in ["stop", "halt", "arrête", "brake"]):
            self.speak("Stopping all movement.", "high")
            return self.queue_command('S', 'movement')
        elif any(word in text_lower for word in ["up", "rise", "monte"]):
            self.speak("Moving up.", "high")
            return self.queue_command('U', 'movement')
        elif any(word in text_lower for word in ["down", "descend", "baisse"]):
            self.speak("Moving down.", "high")
            return self.queue_command('D', 'movement')
            
        # Status commands
        elif any(word in text_lower for word in ["status", "état", "health", "report", "diagnostics"]):
            status = self.get_status_report()
            response = (
                f"System status: Arduino {'CONNECTED' if status['health']['arduino'] else 'DISCONNECTED'}, "
                f"ACK {'OK' if status['health']['arduino_ack'] else 'FAILED'}, "
                f"Distance: {status['distance_cm']}cm, "
                f"Packets: {status['packets_sent']} sent, {status['packets_received']} received, "
                f"{status['crc_errors']} errors, "
                f"Emergency: {'ACTIVE' if status['emergency_state'] else 'INACTIVE'}."
            )
            self.speak(response)
            
        elif any(word in text_lower for word in ["who are you", "qui es-tu", "identity", "name"]):
            self.speak("I am RobotBrain. Fully offline autonomous unit with secure packet communication and obstacle avoidance.")
            
        elif any(word in text_lower for word in ["test", "self-test", "diagnostic", "run test"]):
            self._run_self_test()
            
        elif any(word in text_lower for word in ["reset", "clear emergency", "resume"]):
            self.emergency_state = False
            self.speak("Emergency cleared. System resuming normal operation.")
            
        elif any(word in text_lower for word in ["shutdown", "exit", "bye", "goodbye"]):
            self.speak("Shutting down safely. Goodbye!")
            self.killer.kill_now = True
            return False
            
        elif any(word in text_lower for word in ["clear", "reset queue"]):
            self.speak("Clearing command queues.")
            while not self.command_queue.empty():
                try:
                    self.command_queue.get_nowait()
                    self.command_queue.task_done()
                except queue.Empty:
                    break
            self.health_status['obstacle_detected'] = False
            
        else:
            self.speak("Command not recognized. Try forward, back, left, right, stop, status, or emergency.")
            self.command_history[-1]['processed'] = False
        
        return True

    def _run_self_test(self):
        """Run comprehensive self-test."""
        self.speak("Running full system self-test with packet protocol...")
        
        tests = {
            'speech': self.speak("Speech test."),
            'arduino_connection': self.health_status['arduino'],
            'arduino_ack': self.health_status['arduino_ack'],
            'stt': self.health_status['stt'],
            'heartbeat': (time.time() - self.health_status['last_heartbeat']) < 5,
            'packet_rx': self.health_status['packets_received'] > 0,
            'sensor_data': self.health_status['last_sensor'] > 0
        }
        
        passed = sum(tests.values())
        total = len(tests)
        
        if passed == total:
            self.speak(f"All systems operational. {passed}/{total} tests passed.")
        else:
            failed = [name for name, result in tests.items() if not result]
            self.speak(f"Self-test complete. {passed}/{total} tests passed. Issues: {', '.join(failed)}")

    def _health_monitor(self):
        """Continuous health monitoring."""
        start_time = time.time()
        
        while not self.killer.kill_now:
            try:
                # Update uptime
                self.health_status['uptime'] = time.time() - start_time
                
                # Check Arduino connection
                if self.arduino:
                    self.health_status['arduino'] = self.arduino.is_open
                else:
                    self.health_status['arduino'] = False
                
                # Log detailed health status periodically
                if int(time.time()) % 60 == 0:  # Every 60 seconds
                    status = self.get_status_report()
                    logger.info(f"Health report: {status}")
                
                time.sleep(1)
            except Exception as e:
                logger.error(f"Health monitor error: {e}")
                time.sleep(5)

    def _audio_callback(self, indata, frames, time, status):
        """Audio input callback with error handling."""
        if status:
            logger.warning(f"Audio status: {status}")
        try:
            if not self.audio_queue.full():
                self.audio_queue.put(bytes(indata), block=False)
        except queue.Full:
            pass  # Drop audio if queue full

    def listen_loop(self):
        """Main listening loop - now non-blocking with async command processing."""
        logger.info("Starting audio listening loop...")
        
        while not self.killer.kill_now:
            try:
                self.is_listening = True
                self.health_status['audio'] = True
                
                with sd.RawInputStream(
                    samplerate=AUDIO_SAMPLE_RATE,
                    blocksize=AUDIO_BLOCK_SIZE,
                    dtype='int16',
                    channels=1,
                    callback=self._audio_callback
                ) as stream:
                    self.speak("Listening for commands. Secure packet protocol active.")
                    
                    while not self.killer.kill_now and self.is_listening:
                        try:
                            data = self.audio_queue.get(timeout=0.5)
                            if self.rec.AcceptWaveform(data):
                                result = json.loads(self.rec.Result())
                                text = result.get("text", "").strip()
                                
                                if text:
                                    logger.info(f"Recognized: '{text}'")
                                    # Queue voice command for async processing
                                    self.queue_command(text, 'voice')
                                        
                        except queue.Empty:
                            continue
                        except json.JSONDecodeError as e:
                            logger.debug(f"JSON decode error: {e}")
                            continue
                            
            except Exception as e:
                logger.error(f"Audio stream error: {e}")
                self.health_status['audio'] = False
                self.is_listening = False
                
                if self.killer.kill_now:
                    break
                    
                logger.info("Restarting audio stream in 3 seconds...")
                time.sleep(3)
        
        self.shutdown()

    def shutdown(self):
        """Clean shutdown sequence."""
        logger.info("Initiating shutdown sequence...")
        
        self.speak("System shutting down. Goodbye.")
        
        # Stop background threads
        self.heartbeat_running = False
        self.is_listening = False
        
        # Send final stop command
        if self.arduino and self.arduino.is_open:
            final_packet = Packet(ptype=PacketType.COMMAND, payload=b'S')
            self._send_packet_raw(final_packet)
            time.sleep(0.1)
            self.arduino.close()
        
        # Clear queues
        while not self.audio_queue.empty():
            try:
                self.audio_queue.get_nowait()
            except queue.Empty:
                break
        
        while not self.command_queue.empty():
            try:
                self.command_queue.get_nowait()
                self.command_queue.task_done()
            except queue.Empty:
                break
        
        logger.info("RobotBrain shutdown complete.")
        sys.exit(0)

def main():
    """Main entry point."""
    try:
        brain = RobotBrain()
        brain.speak("RobotBrain online. Secure packet protocol active. All systems nominal.")
        brain.listen_loop()
    except KeyboardInterrupt:
        logger.info("Keyboard interrupt received.")
    except Exception as e:
        logger.error(f"Fatal error: {e}")
    finally:
        if 'brain' in locals():
            brain.shutdown()

if __name__ == "__main__":
    main()
