# PETSA - OOP Architecture Documentation

## Overview
This document describes the refactored Object-Oriented Programming (OOP) architecture of the PETSA health monitoring system. The refactoring transforms the original monolithic Arduino code into a modular, maintainable, and testable codebase following industry-standard practices.

## Architecture Principles

### 1. **Separation of Concerns**
- Each class has a single, well-defined responsibility
- Hardware abstraction separates business logic from implementation details
- Clear boundaries between data collection, processing, and transmission

### 2. **Dependency Injection**
- Components receive dependencies through constructors
- Enables easy testing and mocking
- Reduces coupling between modules

### 3. **Interface-Based Design**
- Abstract interfaces define contracts
- Implementations can be swapped without affecting dependent code
- Facilitates unit testing and future extensions

### 4. **SOLID Principles**
- **S**ingle Responsibility: Each class has one reason to change
- **O**pen/Closed: Open for extension, closed for modification
- **L**iskov Substitution: Derived classes are substitutable for base classes
- **I**nterface Segregation: Clients depend only on interfaces they use
- **D**ependency Inversion: Depend on abstractions, not concretions

## Project Structure

```
petsa_01_slave/
├── include/                    # Interface definitions
│   ├── ISensor.h              # Base sensor interface
│   ├── ITemperatureSensor.h   # Temperature sensor interface
│   ├── IHeartRateSensor.h     # Heart rate sensor interface
│   ├── IDataProcessor.h       # Data processing interface
│   ├── IIoTClient.h          # IoT communication interface
│   ├── ILogger.h             # Logging interface
│   └── Config.h              # Configuration structures
├── src/
│   ├── sensors/              # Sensor implementations
│   │   ├── MLX90614TemperatureSensor.h/.cpp
│   │   └── MAX30105HeartRateSensor.h/.cpp
│   ├── communication/        # IoT communication
│   │   └── AzureIoTClient.h/.cpp
│   ├── data/                # Data processing
│   │   └── DataProcessor.h/.cpp
│   ├── logging/             # Logging implementation
│   │   └── SerialLogger.h/.cpp
│   ├── ui/                  # User interface
│   │   └── EventCardUI.h/.cpp
│   ├── config/              # Configuration management
│   │   └── Config.cpp
│   └── PETSAApplication.h/.cpp  # Main application controller
├── main_oop.ino             # New OOP-based main file
├── main.ino                 # Original monolithic file (for reference)
├── iot_configs.h            # Configuration constants
└── ARCHITECTURE.md          # This file
```

## Core Components

### 1. **Interfaces Layer** (`include/`)

#### ISensor
- Base interface for all sensors
- Defines common operations: initialize(), read(), isReady(), reset()
- Provides SensorReading structure with status and timestamp

#### ITemperatureSensor
- Extends ISensor for temperature-specific operations
- Provides TemperatureReading with ambient, object, and core temperatures
- Includes contact detection capabilities

#### IHeartRateSensor
- Extends ISensor for heart rate and SpO2 measurements
- Supports sleep/wake modes for power management
- Provides HeartRateReading with BPM, SpO2, and signal data

#### IDataProcessor
- Handles data collection, validation, and averaging
- Manages circular buffers for sensor readings
- Provides VitalSigns structure for processed data

#### IIoTClient
- Abstracts cloud communication
- Handles connection management and authentication
- Provides ConnectionInfo for status monitoring

#### ILogger
- Provides structured logging with levels
- Supports formatted output and timestamp management
- Enables different logging backends

### 2. **Implementation Layer** (`src/`)

#### Sensor Implementations
- **MLX90614TemperatureSensor**: Non-contact infrared temperature sensing with robust filtering
- **MAX30105HeartRateSensor**: Heart rate and SpO2 measurement with power management

#### Communication Layer
- **AzureIoTClient**: Full Azure IoT Hub integration with MQTT over TLS
- Automatic SAS token management and connection recovery
- Support for different power modes (Performance/Eco/Ultra-Eco)

#### Data Processing
- **DataProcessor**: Statistical processing with outlier rejection
- **DataValidator**: Input validation and range checking
- Configurable averaging windows and validation rules

#### User Interface
- **EventCardUI**: Rich console output with Unicode/emoji support
- **SerialLogger**: Timestamped logging with configurable levels
- Progress indicators and status displays

#### Application Controller
- **PETSAApplication**: Main orchestrator coordinating all components
- State machine for application lifecycle management
- Hardware initialization and error recovery

### 3. **Configuration Management**
- **Config**: Centralized configuration with validation
- Type-safe parameter management
- Support for different deployment scenarios

## Key Design Patterns

### 1. **Strategy Pattern**
- Different power management strategies (Performance/Eco/Ultra-Eco)
- Pluggable sensor implementations
- Configurable logging backends

### 2. **Observer Pattern**
- Event-driven sensor data collection
- Status change notifications
- Error event propagation

### 3. **Factory Pattern**
- Sensor creation based on configuration
- Power mode instantiation
- Logger backend selection

### 4. **State Machine Pattern**
- Application state management
- Graceful error handling and recovery
- Power state transitions

## Benefits of the OOP Refactoring

### 1. **Maintainability**
- Clear module boundaries reduce complexity
- Changes to one component don't affect others
- Easier to locate and fix bugs

### 2. **Testability**
- Interface-based design enables unit testing
- Dependency injection allows mocking
- Isolated components can be tested independently

### 3. **Extensibility**
- New sensors can be added by implementing interfaces
- Additional IoT backends can be integrated
- Different power modes can be implemented

### 4. **Reliability**
- Proper error handling at all levels
- Resource management with RAII
- Graceful degradation on component failures

### 5. **Performance**
- Efficient memory management
- Optimized I/O operations
- Power-aware design patterns

## Migration Guide

### From Original Code
1. **Sensor Integration**: Replace direct sensor calls with interface methods
2. **Data Flow**: Use DataProcessor instead of global arrays
3. **Communication**: Replace MQTT code with IIoTClient interface
4. **Logging**: Use ILogger instead of Serial.print statements
5. **Configuration**: Use Config class instead of #define constants

### Testing Strategy
1. **Unit Tests**: Test each component in isolation
2. **Integration Tests**: Test component interactions
3. **Hardware Tests**: Test with actual sensors
4. **System Tests**: End-to-end functionality testing

## Future Enhancements

### 1. **Additional Sensors**
- GPS location tracking
- Environmental sensors (humidity, pressure)
- Accelerometer for activity monitoring

### 2. **Enhanced Communication**
- Multiple cloud providers
- Local data storage with sync
- Mesh networking for multiple devices

### 3. **Advanced Analytics**
- Real-time anomaly detection
- Predictive health modeling
- Machine learning integration

### 4. **User Interface**
- Web-based configuration portal
- Mobile app integration
- Real-time dashboards

## Conclusion

The OOP refactoring transforms the PETSA system from a monolithic Arduino sketch into a professional, maintainable codebase. The new architecture provides a solid foundation for future enhancements while maintaining the original functionality and improving reliability, performance, and developer experience.