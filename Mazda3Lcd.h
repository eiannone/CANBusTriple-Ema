#ifndef Mazda3Lcd_H
#define Mazda3Lcd_H

#include <EEPROM.h>
#include <MessageQueue.h>
#include "Middleware.h"
#include "Mazda3CAN.h"
#include "Settings.h"

#define LCD_BUS_ID 2
#define N_DISPLAY_MODES 7
class Mazda3Lcd : public Middleware
{	
public:
    bool buttonInfo;
    bool buttonClock;

    Mazda3Lcd(Mazda3CAN *mazda_can, MessageQueue *writeQueue);
    void init(byte displayMode);
    void tick();
    void commandHandler(byte* bytes, int length, Stream* activeSerial);
    void pushInfo();
    void pushClock();
    void nextDisplayMode();
    void prevDisplayMode();
    void showMessage(const char * msg, const int msec);

private:
	unsigned long _nextDisplayTst;
    unsigned long _msgDisplayTst;
	char _lcdText[13];
    byte _canBuf[8];
    byte _displayMode;
    byte _lcdSymbols; // Byte 3 of msg 0x28F
    byte _lcdButtons; // Byte 5 of msg 0x28F        
	Mazda3CAN* _mazda;
	MessageQueue* _writeQueue;

    void generateLCDText();
    void pushMessage(const unsigned short msgId);
    char formatGear(const byte gear);
    void setDisplayMode(byte displayMode);
};

Mazda3Lcd::Mazda3Lcd(Mazda3CAN *mazda_can, MessageQueue *writeQueue) 
	: buttonInfo(false), buttonClock(false), _displayMode(0), _lcdSymbols(0), _lcdButtons(0x20)
{
	_mazda = mazda_can;
	_writeQueue = writeQueue;
}

void Mazda3Lcd::init(byte displayMode)
{
    _nextDisplayTst = _msgDisplayTst = 0;
    _lcdText[0] = 0x00;
    _displayMode = displayMode;
}

void Mazda3Lcd::tick()
{
    if (!_mazda->dashboardOn) return;
    unsigned long tst = millis();

	if (tst < _nextDisplayTst || (_displayMode == 0 && tst > _msgDisplayTst)) return;
    _nextDisplayTst = tst + 250L;

    if (tst < _msgDisplayTst) // Showing message
        _lcdSymbols = 0;
    else
        generateLCDText();
    
    _canBuf[0] = 0x80;
    memset((_canBuf + 1), 0, 7);
    _canBuf[3] = _lcdSymbols;
    _canBuf[4] = _lcdButtons;
    pushMessage(0x28F);

    _lcdButtons = 0x20;
    if (buttonInfo) _lcdButtons |= 0x08; // Imposta il bit 4
    if (buttonClock) _lcdButtons |= 0x10; // Imposta il bit 5

    _canBuf[0] = 0xC0;
    memcpy((_canBuf + 1), _lcdText, 7);
    pushMessage(0x290);

    _canBuf[0] = 0x85;
    memcpy((_canBuf + 1), (_lcdText + 5), 7);
    pushMessage(0x291);    
}

void Mazda3Lcd::commandHandler(byte* bytes, int length, Stream* activeSerial)
{
    if (length == 0) {
        activeSerial->write(COMMAND_ERROR);
        return;
    }

    if (bytes[0] == 0xFF) {
        // Riporta l'attuale modalità di visualizzazione
        activeSerial->print( F( "{\"event\":\"display-mode\",\"mode\":" ) );
        activeSerial->print(_displayMode);
        activeSerial->println( F( "}" ) );
        return;
    }

    if (bytes[0] > 0x3F) {
        // I primi due bit più significativi corrispondono ai pulsanti Clock e Info
        _lcdButtons |= (bytes[0] >> 3);
        _nextDisplayTst = 0; // Forza l'aggiornamento del display
    }
	else if (bytes[0] != _displayMode) {
        // Gli altri bit corrispondono alla modalità di visualizzazione
        setDisplayMode(bytes[0]);
    }
    activeSerial->write(COMMAND_OK);
    activeSerial->write(NEWLINE);    
}

void Mazda3Lcd::generateLCDText() 
{
    char* buf;

	switch(_displayMode) {
		case 1: // Barra RPM, marcia, velocità
			// Barra RPM
			_lcdText[0] = (_mazda->rpm > 800)?  0xBA : (_mazda->rpm > 600)?  ']' : '_';
			_lcdText[1] = (_mazda->rpm > 1200)? 0xBA : (_mazda->rpm > 1000)? ']' : '_';
			_lcdText[2] = (_mazda->rpm > 1600)? 0xBA : (_mazda->rpm > 1400)? ']' : '_';
			_lcdText[3] = (_mazda->rpm > 2000)? 0xBA : (_mazda->rpm > 1800)? ']' : '_';
			_lcdText[4] = (_mazda->rpm > 2400)? 0xBA : (_mazda->rpm > 2200)? ']' : '_';
			_lcdText[5] = (_mazda->rpm > 2800)? 0xBA : (_mazda->rpm > 2600)? ']' : '_';
			_lcdText[6] = (_mazda->rpm > 3000)? ']' : ' ';
			// Marcia
			_lcdText[7] = formatGear(_mazda->gear);
			// Velocità
			dtostrf((float)_mazda->speed / 100.0, 4, 0, _lcdText + 8);
			break;

		case 2: // Tachimetro
			sprintf(_lcdText, "%5d", _mazda->rpm);
			_lcdText[5] = _lcdText[6] = ' ';
			// Marcia
			_lcdText[7] = formatGear(_mazda->gear);
			// Velocità
			dtostrf((float)_mazda->speed / 100.0, 4, 0, _lcdText + 8);
			break;

		case 3: // T. motore e T. interna
            strcpy(_lcdText, "Tm    Ti");
            buf = _mazda->getEngineTemp();
            _lcdText[2] = buf[0];
            _lcdText[3] = buf[1];
            _lcdText[4] = buf[2];
            
            buf = _mazda->getInternalTemp();
            _lcdText[8] = buf[0];
            _lcdText[9] = buf[1];
            _lcdText[10] = buf[2];
            _lcdText[11] = buf[4];

            _lcdSymbols = 0x04; // Simbolo '.' tra 11° e 12° carattere
			break;

        case 4: // Volante e spostamento
            sprintf(_lcdText, "%5d", _mazda->steering);
            _lcdText[5] = _lcdText[6] = _lcdText[7] = ' ';

            buf = _mazda->getMovement();
            _lcdText[8] = buf[0];
            _lcdText[9] = buf[1];
            _lcdText[10] = buf[2];
            _lcdText[11] = buf[4];

            _lcdSymbols = 0x04; // Simbolo '.' tra 11° e 12° carattere
            break;

        case 5: // Distanza e carburante consumato
            memcpy(_lcdText, _mazda->getDistance(), 7);
            buf = _mazda->getFuel();
            for(int b = 0; b < 5; b++) _lcdText[7 + b] = buf[b];
            break;

        case 6: // Livello carburante
            strcpy(_lcdText, "Liv car    l");
            buf = _mazda->getFuelLevel();
            _lcdText[8] = buf[0];
            _lcdText[9] = buf[1];
            _lcdText[10] = buf[3];
            _lcdSymbols = 0x02; // Simbolo '.' tra 10° e 11° carattere
            break;

		default:
			strcpy(_lcdText, "  Emanuele  ");
	}
}

void Mazda3Lcd::pushMessage(const unsigned short msgId)
{
    Message msg;
    msg.length = 8;
    msg.frame_id = msgId;
    memcpy(msg.frame_data, _canBuf, 8);
    msg.busId = LCD_BUS_ID;
	msg.dispatch = true;
	_writeQueue->push(msg);
}

char Mazda3Lcd::formatGear(const byte gear) 
{
	if (gear == 0) return '-';
	else if (gear < 7) return 0xE0 + _mazda->gear;
	else if (gear == 0xE) return 'R';
	return gear;	
}

void Mazda3Lcd::pushInfo()
{
    _lcdButtons |= 0x08; // Imposta il bit 4
    _nextDisplayTst = 0; // Forza l'aggiornamento del display
}

void Mazda3Lcd::pushClock()
{
    _lcdButtons |= 0x10; // Imposta il bit 5
    _nextDisplayTst = 0; // Forza l'aggiornamento del display
}

void Mazda3Lcd::setDisplayMode(byte displayMode)
{
    _msgDisplayTst = millis() + 1500;
    sprintf(_lcdText, "   Modo %d   ", displayMode);
    cbt_settings.displayIndex = _displayMode = displayMode;
    EEPROM.write( offsetof(struct cbt_settings, displayIndex), _displayMode);
}

void Mazda3Lcd::nextDisplayMode()
{
    setDisplayMode((_displayMode + 1) % N_DISPLAY_MODES);
}

void Mazda3Lcd::prevDisplayMode()
{    
    if (_displayMode == 0) setDisplayMode(N_DISPLAY_MODES - 1);
    else setDisplayMode(_displayMode - 1);
}

void Mazda3Lcd::showMessage(const char * msg, const int msec)
{
    _msgDisplayTst = millis() + msec;
    strncpy(_lcdText, msg, 13);
}

#endif // Mazda3Lcd_H

