#ifndef ITEMPERATURE_SENSOR_H
#define ITEMPERATURE_SENSOR_H

#include "ISensor.h"

struct TemperatureReading : public SensorReading {
    float ambientTemp;
    float objectTemp;
    bool inContact;
    
    TemperatureReading(float core = NAN, float ambient = NAN, float object = NAN, 
                      bool contact = false, SensorStatus stat = SensorStatus::NO_DATA)
        : SensorReading(core, stat), ambientTemp(ambient), objectTemp(object), inContact(contact) {}
};

class ITemperatureSensor : public ISensor {
public:
    virtual TemperatureReading readTemperature() = 0;
    virtual float getAmbientTemperature() = 0;
    virtual float getObjectTemperature() = 0;
    virtual bool isInContact() = 0;
};

#endif