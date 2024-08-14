/**
 * @file SensorBase.hpp
 * @brief Sensor object base class
 * 
 */

#ifndef SENSORBASE_HPP
#define SENSORBASE_HPP

class SensorBase {
public:
    virtual ~SensorBase() {}

    // Method to initialize the sensor
    virtual void initialize() = 0;

    // Method to read sensor data
    virtual float readData() = 0;

    // Method to calibrate the sensor
    virtual void calibrate() = 0;

    // Method to reset the sensor
    virtual void reset() = 0;
};

#endif // SENSORBASE_HPP