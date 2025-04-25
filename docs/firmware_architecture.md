# Viztron Camera Module - Firmware Architecture

## Overview

The Viztron Camera Module firmware is designed for the Raspberry Pi Pico W platform, utilizing FreeRTOS for task management. The architecture follows a modular approach with clear separation of concerns between different functional components.

## System Architecture

### Hardware Components
- **Processor**: Raspberry Pi Pico W
- **Camera**: OV2640 2MP Camera Module
- **GPS**: Quectel L76-L GPS Module
- **Sensors**: PIR Motion Sensor, Ambient Light Sensor
- **Audio**: Microphone and speaker
- **Power**: 3.7V 2000mAh LiPo Battery with charging circuit
- **Connectivity**: Wi-Fi (built into Pico W)

### Software Architecture

The firmware is structured as a multi-task RTOS application with the following key components:

1. **Core System**
   - FreeRTOS kernel and task management
   - System initialization and configuration
   - Error handling and recovery mechanisms
   - Secure boot and firmware update system

2. **Task Management**
   - Camera Task: Frame acquisition and buffering
   - AI Task: Object detection and tracking
   - GPS Task: Location services and zone management
   - Audio Task: Sound processing and alerts
   - Communication Task: Network operations and data transmission
   - Power Management Task: Battery monitoring and power states

3. **Inter-Task Communication**
   - Message queues for asynchronous communication
   - Shared memory for frame data
   - Semaphores for resource synchronization
   - Event groups for state signaling

## Task Descriptions

### Camera Task
- Priority: High
- Responsibilities:
  - Initialize and configure OV2640 camera
  - Capture frames at 20+ FPS
  - Perform basic image preprocessing
  - Manage frame buffer
  - Control camera parameters based on lighting conditions

### AI Task
- Priority: High
- Responsibilities:
  - Run YOLOv5n object detection on captured frames
  - Implement ByteTrack algorithm for object tracking
  - Calculate real-world coordinates from pixel coordinates
  - Analyze motion patterns
  - Generate detection events

### GPS Task
- Priority: Medium
- Responsibilities:
  - Interface with Quectel L76-L GPS module
  - Parse NMEA sentences
  - Filter and smooth position data
  - Manage geofencing and zone-based detection
  - Provide position estimation when GPS is unavailable

### Audio Task
- Priority: Medium
- Responsibilities:
  - Process audio input from microphone
  - Generate alert sounds
  - Implement audio detection algorithms
  - Manage audio buffer

### Communication Task
- Priority: Medium
- Responsibilities:
  - Establish and maintain secure connection with Homebase
  - Implement H.264 video streaming
  - Transmit detection results and status information
  - Handle device discovery and pairing
  - Manage network reconnection

### Power Management Task
- Priority: Low
- Responsibilities:
  - Monitor battery level
  - Implement dynamic CPU scaling
  - Manage system power states
  - Control peripheral power
  - Implement wake-on-motion functionality

## Memory Management

### RAM Allocation
- Frame Buffer: 320x240x2 bytes (153.6 KB) for double buffering
- AI Model Working Memory: ~200 KB
- FreeRTOS Kernel: ~10 KB
- Task Stacks: ~5 KB per task
- Heap: Remaining memory

### Flash Allocation
- Firmware Code: ~200 KB
- AI Model Weights: ~8 MB (quantized YOLOv5n)
- Configuration Data: ~16 KB
- OTA Update Buffer: ~256 KB

## Power Management

### Power States
1. **Active Mode**
   - All systems operational
   - Full frame rate (20+ FPS)
   - Wi-Fi active
   - Power consumption: ~300mA

2. **Alert Mode**
   - Triggered by motion detection
   - Increased frame rate and processing
   - Active communication with Homebase
   - Power consumption: ~350mA

3. **Idle Mode**
   - Reduced frame rate (5 FPS)
   - AI processing on selected frames
   - Wi-Fi in power save mode
   - Power consumption: ~150mA

4. **Low Power Mode**
   - Camera in standby
   - Motion detection via PIR only
   - GPS and Wi-Fi disabled
   - Periodic wake-up for status updates
   - Power consumption: ~30mA

5. **Sleep Mode**
   - All systems off except PIR sensor
   - Wake on motion only
   - Power consumption: ~5mA

## Communication Protocol

### Homebase Communication
- TLS 1.3 encrypted WebSocket connection
- Binary protocol for efficiency
- Message types:
  - Authentication and pairing
  - Status updates
  - Detection events
  - Configuration updates
  - Video stream
  - Command and control

### Video Streaming
- H.264 encoding for efficiency
- Adaptive bitrate based on network conditions
- I-frame only mode for low bandwidth
- Event-triggered recording

## Security Architecture

### Secure Boot
- Signature verification of firmware
- Secure element for key storage
- Rollback protection

### Communication Security
- TLS 1.3 for all network communication
- Certificate-based authentication
- Secure key exchange

### Data Protection
- Encryption of sensitive data at rest
- Secure storage for credentials
- Anti-tampering mechanisms

## Error Handling and Recovery

### Error Detection
- Hardware watchdog timer
- Task deadlock detection
- Communication timeout monitoring
- Memory corruption detection

### Recovery Mechanisms
- Automatic task restart
- System reset on critical errors
- Fallback to safe mode
- Error logging for diagnostics

## Performance Considerations

### Optimizations
- SIMD instructions for image processing
- Fixed-point arithmetic for AI inference
- Memory pooling to reduce fragmentation
- Task scheduling optimization
- Power-aware processing

### Bottleneck Mitigation
- Double buffering for camera frames
- Parallel processing where possible
- Event-driven architecture to reduce polling
- Efficient inter-task communication

## Integration Points

### Homebase Integration
- API for device registration and authentication
- Protocol for status reporting and command reception
- Video streaming interface
- Configuration management

### Mobile App Integration
- Device setup and configuration
- Firmware update triggering
- Status monitoring
- Alert notifications
