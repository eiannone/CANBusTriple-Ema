#ifndef CBTButtons_H
#define CBTButtons_H

#include <avr/interrupt.h>
#include "Mazda3Lcd.h"

// Corrispondenza pin Arduino, canali ADC Atmega32u4:
// A4 -> ADC1
// A5 -> ADC0

// REFS1:0 = 01 -> Select VCC as voltage reference
// ADLAR   = 1  -> Right adjust result (so we can discard 2 LSB and read only first 8 bits)
// MUX4:0  = 0000 for ADC0 (pin A5) or 0001 for ADC1 (pin A4)
#define ADMUX_A4 B01100001 // VCC as voltage reference and ADC1 as conversion channel (pin A4)
#define ADMUX_A5 B01100000 // VCC as voltage reference and ADC0 as conversion channel (pin A5)

// ADEN  = 1 -> ADC Enabled
// ADSC  = 1 -> Start conversion 
// ADATE = 0 -> Auto Trigger disabled
// ADIF  = 0 -> (Interrupt flag)
// ADIE  = 1 -> ADC interrupt enabled
// ADPS  = 111 -> Division factor = 128
#define ADC_start() ( ADCSRA = B11001111 )


volatile byte a4_lev, a5_lev;
volatile unsigned long a4_msec, a5_msec;

class CBTButtons : public Middleware
{   
public:
    CBTButtons(Mazda3Lcd *mazda_lcd, int led, int relay_pin);
    void begin();
    void tick();

private:
    int _led;
    int _relay_pin;
    bool _relay_status;
    byte _curLev4;
    byte _curLev5;
    Mazda3Lcd* _lcd;

    void toggleRelay();
};

CBTButtons::CBTButtons(Mazda3Lcd *mazda_lcd, int led, int relay_pin) 
    : _curLev4(0), _curLev5(0), _led(led), _relay_pin(relay_pin)
{
    _lcd = mazda_lcd;
};

void CBTButtons::begin()
{
    pinMode(_relay_pin, OUTPUT);
    digitalWrite(_relay_pin, LOW);
    _relay_status = false;

    DIDR0 |= B00000011; // Disable digital inputs on pin ADC0 and ADC1
    ADMUX = ADMUX_A4;  // Select Vcc as voltage reference and ADC1 as conversion channel (pin A4)
    ADC_start();
}

byte getLevel(unsigned char val)
{
    if (val > 211) return 3;
    if (val > 137) return 2;
    if (val > 87)  return 1;
    return 0;
}

ISR(ADC_vect) {
    byte level = getLevel(ADCH);
    if (ADMUX == ADMUX_A4) { 
        // Reading from ADC1 (pin A4)
        if (level != a4_lev) {
            a4_lev = level;
            a4_msec = millis();
        }
        ADMUX = ADMUX_A5; // Switch to channel ADC0
    } else { 
        // Reading from ADC0 (pin A5)
        if (level != a5_lev) {
            a5_lev = level;
            a5_msec = millis();
        }
        ADMUX = ADMUX_A4; // Switch to channel ADC1
    }
    ADC_start();
}


void CBTButtons::tick()
{
    bool changed = false;
    if (a4_lev != _curLev4 && (millis() - a4_msec) > 50) {        
        _lcd->buttonInfo = (a4_lev & 0x2) != 0;
        _lcd->buttonClock = (a4_lev & 0x1) != 0;

        // Detect button pressed
        if (_lcd->buttonInfo && (_curLev4 & 0x2) == 0) _lcd->pushInfo();
        else if (_lcd->buttonClock && (_curLev4 & 0x1) == 0) _lcd->pushClock();
        _curLev4 = a4_lev;
        changed = true;
    }
    if (a5_lev != _curLev5 && (millis() - a5_msec) > 50) {
        // Detect button pressed
        if ((a5_lev & 0x1) != 0 && (_curLev5 & 0x1) == 0) toggleRelay();
        else if ((a5_lev & 0x2) != 0 && (_curLev5 & 0x2) == 0) _lcd->nextDisplayMode();
        _curLev5 = a5_lev;
        changed = true;
    }
    if (changed) digitalWrite(_led, (_curLev4 || _curLev5 || _relay_status)? HIGH : LOW);
}


void CBTButtons::toggleRelay()
{
    _relay_status = !_relay_status;
    digitalWrite(_relay_pin, _relay_status? HIGH : LOW);
}

#endif // CBTButtons_H