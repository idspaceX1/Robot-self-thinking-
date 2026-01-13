// Arduino code with Watchdog and Packet Protocol
#include <NewPing.h>
#include <avr/wdt.h>

// Packet Protocol Constants
#define FRAME_START 0xAA

#define PKT_COMMAND   0x01
#define PKT_ACK       0x02
#define PKT_ERROR     0x03
#define PKT_SENSOR    0x10
#define PKT_HEARTBEAT 0x20
#define PKT_EMERGENCY 0x30

// Pin definitions
#define TRIGGER_PIN 12
#define ECHO_PIN 11
#define MAX_DISTANCE 200  // Maximum distance we want to ping for (in cm)

// Motor control pins
#define MOTOR_A1 2
#define MOTOR_A2 3
#define MOTOR_B1 4
#define MOTOR_B2 5

// Safety constants
#define OBSTACLE_THRESHOLD 10  // 10cm
#define HEARTBEAT_TIMEOUT 2000 // 2 seconds
#define MAX_PACKET_SIZE 64

// Global variables
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
unsigned long lastCommandTime = 0;
unsigned long lastHeartbeat = 0;
int obstacleDistance = 0;
bool emergencyState = false;
uint16_t packetCounter = 0;

// Motor control functions
void forward() {
    digitalWrite(MOTOR_A1, HIGH);
    digitalWrite(MOTOR_A2, LOW);
    digitalWrite(MOTOR_B1, HIGH);
    digitalWrite(MOTOR_B2, LOW);
}

void backward() {
    digitalWrite(MOTOR_A1, LOW);
    digitalWrite(MOTOR_A2, HIGH);
    digitalWrite(MOTOR_B1, LOW);
    digitalWrite(MOTOR_B2, HIGH);
}

void left() {
    digitalWrite(MOTOR_A1, LOW);
    digitalWrite(MOTOR_A2, HIGH);
    digitalWrite(MOTOR_B1, HIGH);
    digitalWrite(MOTOR_B2, LOW);
}

void right() {
    digitalWrite(MOTOR_A1, HIGH);
    digitalWrite(MOTOR_A2, LOW);
    digitalWrite(MOTOR_B1, LOW);
    digitalWrite(MOTOR_B2, HIGH);
}

void stopMotors() {
    digitalWrite(MOTOR_A1, LOW);
    digitalWrite(MOTOR_A2, LOW);
    digitalWrite(MOTOR_B1, LOW);
    digitalWrite(MOTOR_B2, LOW);
}

void moveUp() {
    // Implement up movement
}

void moveDown() {
    // Implement down movement
}

// CRC-8 function
uint8_t crc8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0;
    while (len--) {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; i++) {
            crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
        }
    }
    return crc;
}

// Send packet to BBB
void sendPacket(uint8_t type, const uint8_t *payload, uint8_t length) {
    uint8_t frame[3 + MAX_PACKET_SIZE + 1];
    uint8_t frameLength = 0;
    
    // Build frame
    frame[frameLength++] = FRAME_START;
    frame[frameLength++] = type;
    frame[frameLength++] = length;
    
    // Copy payload
    for (uint8_t i = 0; i < length; i++) {
        frame[frameLength++] = payload[i];
    }
    
    // Calculate and append CRC
    uint8_t crc = crc8(frame, frameLength);
    frame[frameLength++] = crc;
    
    // Send frame
    Serial.write(frame, frameLength);
}

// Send ACK packet
void sendACK(uint16_t cmdId = 0) {
    uint8_t payload[2];
    payload[0] = highByte(cmdId);
    payload[1] = lowByte(cmdId);
    sendPacket(PKT_ACK, payload, 2);
}

// Send error packet
void sendError(uint8_t errorCode) {
    uint8_t payload[1] = {errorCode};
    sendPacket(PKT_ERROR, payload, 1);
}

// Send sensor data
void sendSensorData() {
    updateDistance();
    
    uint8_t payload[2];
    payload[0] = highByte(obstacleDistance);
    payload[1] = lowByte(obstacleDistance);
    
    sendPacket(PKT_SENSOR, payload, 2);
}

// Send emergency packet
void sendEmergency() {
    sendPacket(PKT_EMERGENCY, NULL, 0);
}

// Update distance sensor
void updateDistance() {
    obstacleDistance = sonar.ping_cm();
    
    // If ping fails, set to max distance
    if (obstacleDistance == 0) {
        obstacleDistance = MAX_DISTANCE;
    }
}

// Process incoming command
void processCommand(char cmd, uint16_t cmdId) {
    updateDistance();
    
    // Safety check: stop if obstacle detected (unless it's a stop command)
    if (obstacleDistance < OBSTACLE_THRESHOLD && cmd != 'S') {
        stopMotors();
        sendError(0x01); // Obstacle error
        return;
    }
    
    // Process command
    switch(cmd) {
        case 'F': forward(); break;
        case 'B': backward(); break;
        case 'L': left(); break;
        case 'R': right(); break;
        case 'S': stopMotors(); break;
        case 'U': moveUp(); break;
        case 'D': moveDown(); break;
        case 'T': // Test command
            break;
        default:
            sendError(0x02); // Unknown command error
            return;
    }
    
    // Send ACK with command ID
    sendACK(cmdId);
    lastCommandTime = millis();
}

// Process incoming packet
void processPacket() {
    if (Serial.available() < 4) return;
    
    // Read frame start
    if (Serial.read() != FRAME_START) return;
    
    // Read header
    uint8_t type = Serial.read();
    uint8_t length = Serial.read();
    
    if (length > MAX_PACKET_SIZE) {
        // Discard invalid packet
        while (Serial.available() && Serial.read() != FRAME_START) {}
        return;
    }
    
    // Read payload
    uint8_t payload[MAX_PACKET_SIZE];
    for (uint8_t i = 0; i < length; i++) {
        if (!Serial.available()) return;
        payload[i] = Serial.read();
    }
    
    // Read CRC
    if (!Serial.available()) return;
    uint8_t crc_rx = Serial.read();
    
    // Verify CRC
    uint8_t frame[3 + MAX_PACKET_SIZE];
    frame[0] = FRAME_START;
    frame[1] = type;
    frame[2] = length;
    for (uint8_t i = 0; i < length; i++) {
        frame[3 + i] = payload[i];
    }
    
    if (crc8(frame, 3 + length) != crc_rx) {
        // CRC error
        sendError(0x03); // CRC error
        return;
    }
    
    // Process valid packet
    packetCounter++;
    
    if (type == PKT_COMMAND) {
        if (length >= 3) {
            // Extract command ID (first 2 bytes) and command
            uint16_t cmdId = (payload[0] << 8) | payload[1];
            char cmd = payload[2];
            processCommand(cmd, cmdId);
        }
    } else if (type == PKT_HEARTBEAT) {
        lastHeartbeat = millis();
        sendACK(); // Respond to heartbeat
    } else if (type == PKT_EMERGENCY) {
        emergencyState = true;
        stopMotors();
        sendEmergency(); // Echo emergency
    }
}

void setup() {
    // Initialize watchdog
    wdt_enable(WDTO_2S);
    
    // Initialize serial
    Serial.begin(115200);
    
    // Initialize motor pins
    pinMode(MOTOR_A1, OUTPUT);
    pinMode(MOTOR_A2, OUTPUT);
    pinMode(MOTOR_B1, OUTPUT);
    pinMode(MOTOR_B2, OUTPUT);
    
    // Initialize ultrasonic sensor
    pinMode(TRIGGER_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    
    // Stop motors initially
    stopMotors();
    
    // Wait for BBB to connect
    delay(2000);
    
    // Send initial ACK
    sendACK();
    
    wdt_reset();
}

void loop() {
    // Reset watchdog
    wdt_reset();
    
    // Process incoming packets
    processPacket();
    
    // Send sensor data periodically
    static unsigned long lastSensorSend = 0;
    if (millis() - lastSensorSend > 100) { // 10Hz
        sendSensorData();
        lastSensorSend = millis();
    }
    
    // Safety: stop if heartbeat is lost
    if (millis() - lastCommandTime > HEARTBEAT_TIMEOUT) {
        if (!emergencyState) {
            emergencyState = true;
            stopMotors();
            sendEmergency();
        }
    }
    
    // Small delay to prevent CPU spinning
    delay(5);
}
