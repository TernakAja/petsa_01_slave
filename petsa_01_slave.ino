/**
 * PETSA - Personal Health Monitoring System (OOP Version)
 * 
 * A cattle health monitoring IoT system using ESP8266
 * Refactored with industry-standard OOP principles
 * 
 * Features:
 * - Clean separation of concerns with interfaces
 * - Dependency injection for testability
 * - Proper error handling and logging
 * - Configurable power management
 * - Extensible sensor framework
 * - Robust data processing and validation
 * 
 * Hardware:
 * - ESP8266 (NodeMCU/Wemos D1 Mini)
 * - MLX90614 infrared temperature sensor
 * - MAX30105 heart rate and SpO2 sensor
 * 
 * Cloud:
 * - Azure IoT Hub integration
 * - Secure MQTT over TLS
 * - Real-time data transmission
 */

#include "src/PETSAApplication.h"

PETSAApplication* app = nullptr;

void setup() {
    app = new PETSAApplication();
    
    if (!app->initialize()) {
        app->getLogger().critical("Application initialization failed - halting");
        while (true) {
            delay(1000);
            yield();
        }
    }
    
    app->getLogger().info("Setup completed successfully");
    app->getUI().cardHeader("READY", "EVENT");
    app->getUI().cardKV("status", "main loop starting");
    app->getUI().cardFooter();
}

void loop() {
    if (app) {
        app->loop();
    } else {
        delay(1000);
    }
}