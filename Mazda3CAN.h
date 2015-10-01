#ifndef Mazda3CAN_H
#define Mazda3CAN_H

#include "Middleware.h"

class Mazda3CAN : public Middleware
{
public:
    int rpm; 
    int speed; // 100 * Km/h
    byte gear; // 0: Neutral, 1-5: 1st-5th, E: Reverse, F: Changing
    byte engTemp; // (T - 3.5 Â°C) * 4    
    bool dashboardOn; // true: ON, false: OFF
    bool engineOn; // true: ON, false: OFF
    unsigned long distance; // m * 5
    int mov; // Spostamento
    unsigned long fuel; // ???
    byte fuelLevel; // l * 4
    byte intTemp; // (T - 3.5 Â°C) * 4
    int steering;
    byte logMode;

    Mazda3CAN();

    void tick();
    Message process( Message msg );
    void commandHandler(byte* bytes, int length, Stream* activeSerial);

    char* getEngineTemp();
    char* getInternalTemp();    
    char* getDistance();
    char* getMovement();
    char* getFuel();
    char* getFuelLevel();
private:
    char _bufString[8];
    byte _distance;
    byte _fuel;
    byte _engineDashboard;
    unsigned long _nextLogTst;

    void updateEngineDashboard(byte status);
};


Mazda3CAN::Mazda3CAN() : 
    rpm(0), speed(0), gear(0), engTemp(86), dashboardOn(false), engineOn(false), distance(0L), mov(0),
    fuel(0L), fuelLevel(0), intTemp(86), steering(0), logMode(0)
{
    _distance = _fuel = _engineDashboard = 0;
    _nextLogTst = 0L; 
}


void Mazda3CAN::tick()
{
    if (logMode == 0 || !Serial || millis() < _nextLogTst) return;
    _nextLogTst = millis() + 100L;

    Serial.print(_nextLogTst);
    Serial.write(0x2C); // ","
    Serial.print(rpm);
    Serial.write(0x2C);
    Serial.print(speed);
    Serial.write(0x2C);
    Serial.print(gear, DEC);
    Serial.write(0x2C);
    Serial.print(getEngineTemp());
    Serial.print(dashboardOn? ",ON" : ",OFF");
    Serial.print(engineOn? ",ON" : ",OFF");
    Serial.write(0x2C);
    Serial.print(distance);
    Serial.write(0x2C);
    Serial.print(getDistance());
    Serial.write(0x2C);
    Serial.print(fuel);
    Serial.write(0x2C);
    Serial.print(fuelLevel, DEC);
    Serial.write(0x2C);
    Serial.print(getInternalTemp());
    Serial.write(0x2C);
    Serial.println(steering);
}


Message Mazda3CAN::process( Message msg )
{
    switch(msg.frame_id) {
        case 0x201: // RPM and vehicle speed
            if ((msg.frame_data[0] & 0x80) > 0) {
                // Dashboard OFF
                rpm = speed = 0;
            }
            else {
                rpm = (((int)msg.frame_data[0] << 8) + msg.frame_data[1]) & 0x7FFF;
                speed = (((int)msg.frame_data[4] << 8) + msg.frame_data[5]) & 0x7FFF;
            }
            break;

        case 0x231:
            gear = msg.frame_data[0] >> 4;//((msg.frame_data[6] & 0x40) > 0)? 0xF : (msg.frame_data[0] >> 4);
            break;

        case 0x420: // Engine temperature, distance, fuel and dashboard
            engTemp = msg.frame_data[0];
                        
            if (msg.frame_data[5] != _engineDashboard) updateEngineDashboard(msg.frame_data[5]);

            if (engineOn && msg.frame_data[1] != _distance) {
                int diff = msg.frame_data[1] + ((msg.frame_data[1] < _distance)? 256 : 0) - _distance;
                distance += (unsigned long)diff;
                if (gear == 0xE) mov -= diff; else mov += diff;
                _distance = msg.frame_data[1];
            }

            if (msg.frame_data[2] != _fuel) {
                if (msg.frame_data[2] == 0 && _fuel < 245) {
                    // Reset due to engine stopping
                    _fuel = 0;
                }
                else {
                    fuel += (unsigned long)(msg.frame_data[2] + ((msg.frame_data[2] < _fuel)? 256 : 0) - _fuel);
                    _fuel = msg.frame_data[2];                    
                }
            }        
            break;

        case 0x430:
            fuelLevel = msg.frame_data[0];
            break;

        case 0x433:
            intTemp = msg.frame_data[2];
            break;

        case 0x4DA: // Steering angle
            // TODO: save offset value in EEPROM
            steering = (msg.frame_data[0] == 0xFF)? 0 : ((int)msg.frame_data[0] << 8) + (int)msg.frame_data[1] - 32768;
            break;
    }
    return msg;
}


void Mazda3CAN::commandHandler(byte* bytes, int length, Stream* activeSerial)
{
    if (length > 0) {
        logMode = bytes[0];
        activeSerial->write(COMMAND_OK);
        activeSerial->write(NEWLINE);
    }
    else {
        activeSerial->write(COMMAND_ERROR);
    }
}


char* Mazda3CAN::getEngineTemp() 
{
    if (engTemp == 0xFF) {
        _bufString[0] = ' ';
        _bufString[1] = '-';
        _bufString[2] = ' ';
        _bufString[3] = 0x00;
        return _bufString;
    }
    return dtostrf(3.5 + (engTemp / 4), 3, 0, _bufString);
}


char* Mazda3CAN::getInternalTemp() 
{
    return dtostrf(3.5 + (intTemp / 4), 5, 1, _bufString);
}


void Mazda3CAN::updateEngineDashboard(byte status)
{
    switch(status) {
        case 0x10:
            dashboardOn = true;
            engineOn = false;
            break;                
        case 0x20:
            engineOn = dashboardOn = true;
            break;
        case 0x30: // Engine starting
        case 0x90:
            dashboardOn = true;
            engineOn = false;
            _distance = 0;
            break;  
        default:
            engineOn = dashboardOn = false;
            break;
    }
    _engineDashboard = status;
}


char* Mazda3CAN::getDistance() 
{    
    if (distance < 50000L)  {
        // If less than 10km displays in meter (if less than 100m displays also one decimal)
        dtostrf((float)distance / 5.0, 5, (distance < 500L)? 1 : 0, _bufString);
    }
    else {
        // If less than 100km displays also one decimal digit
        dtostrf((float)distance / 5000.0, 4, (distance < 500000L)? 1 : 0, _bufString);
        _bufString[4] = 0x4B; // "K"
    }
    _bufString[5] = 0x6D; // "m"
    _bufString[6] = 0x00;
    return _bufString;
}

char* Mazda3CAN::getMovement() 
{
    return dtostrf((float)mov / 5.0, 5, 1, _bufString);
}

char* Mazda3CAN::getFuel() 
{
    sprintf(_bufString, "%5d", fuel);
    return _bufString;
}

char* Mazda3CAN::getFuelLevel() 
{
    return dtostrf((float)fuelLevel / 4.0, 4, 1, _bufString);
}


#endif // Mazda3CAN_H
