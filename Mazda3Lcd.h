#ifndef Mazda3Lcd_H
#define Mazda3Lcd_H

#include <EEPROM.h>
#include "Middleware.h"

class Mazda3Lcd : public Middleware
{	
public:
	byte displayMode;
    byte lcdButtons;
    Mazda3Lcd(Mazda3CAN *mazda_can, QueueArray<Message> *writeQueue);

    void tick();
    Message process( Message msg );
    void commandHandler(byte* bytes, int length, Stream* activeSerial);

private:
	unsigned long _nextDisplayTst;	
	char _lcdText[13];
	Mazda3CAN* _mazda;
	QueueArray<Message>* _writeQueue;

    void generateLCDText();
    void pushMessage(int msgId, byte* msgData);
    char formatGear(byte gear);
};

Mazda3Lcd::Mazda3Lcd(Mazda3CAN *mazda_can, QueueArray<Message> *writeQueue) 
	: displayMode(0), lcdButtons(0x20)
{
	_nextDisplayTst = 0;
	_lcdText[0] = 0x00;
	_mazda = mazda_can;
	_writeQueue = writeQueue;
}

void Mazda3Lcd::tick()
{
	if (displayMode == 0 || millis() < _nextDisplayTst) return;
    _nextDisplayTst = millis() + 250L;

    generateLCDText();
    
    byte buf[] = {0x80, 0, 0, 0, lcdButtons, 0, 0, 0};
    if (displayMode == 3 || displayMode == 4) buf[3] = 0x04; // Simbolo '.' tra 11° e 12° carattere
    else if (displayMode == 6) buf[3] = 0x02; // Simbolo '.' tra 10° e 11° carattere
    pushMessage(0x28F, buf);
    lcdButtons = 0x20;

    buf[0] = 0xC0;
    memcpy(buf + 1, _lcdText, 7);
    pushMessage(0x290, buf);

    buf[0] = 0x85;
    memcpy(buf + 1, _lcdText + 5, 7);
    pushMessage(0x291, buf);    
}

Message Mazda3Lcd::process( Message msg ) 
{ 
	return msg; 
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
        activeSerial->print(displayMode);
        activeSerial->println( F( "}" ) );
        return;
    }

    if (bytes[0] > 0x3F) {
        // I primi due bit più significativi corrispondono ai pulsanti Clock e Info
        lcdButtons |= (bytes[0] >> 3);
    }
	else if (bytes[0] != displayMode) {
        // Gli altri bit corrispondono alla modalità di visualizzazione
        cbt_settings.displayIndex = displayMode = bytes[0];
        EEPROM.write( offsetof(struct cbt_settings, displayIndex), displayMode);
    }
    activeSerial->write(COMMAND_OK);
    activeSerial->write(NEWLINE);    
}

void Mazda3Lcd::generateLCDText() 
{
    char* buf;

	switch(displayMode) {
		case 1: // Barra RPM, marcia, velocità
			// Barra RPM
			_lcdText[0] = (_mazda->rpm > 100)? 0xDB : '_';
			_lcdText[1] = (_mazda->rpm > 850)? 0xDB : '_';
			_lcdText[2] = (_mazda->rpm > 1400)? 0xDB : '_';
			_lcdText[3] = (_mazda->rpm > 1800)? 0xDB : '_';
			_lcdText[4] = (_mazda->rpm > 2150)? 0xDB : '_';
			_lcdText[5] = (_mazda->rpm > 2650)? 0xDB : '_';
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
            strcpy(_lcdText, "Mo    In");
            buf = _mazda->getEngineTemp();
            _lcdText[2] = buf[0];
            _lcdText[3] = buf[1];
            _lcdText[4] = buf[2];
            
            buf = _mazda->getInternalTemp();
            _lcdText[8] = buf[0];
            _lcdText[9] = buf[1];
            _lcdText[10] = buf[2];
            _lcdText[11] = buf[4];
			break;

        case 4: // Volante e spostamento
            sprintf(_lcdText, "%5d", _mazda->steering);
            _lcdText[5] = _lcdText[6] = _lcdText[7] = ' ';

            buf = _mazda->getMovement();
            _lcdText[8] = buf[0];
            _lcdText[9] = buf[1];
            _lcdText[10] = buf[2];
            _lcdText[11] = buf[4];
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
            break;

		default:
			strcpy(_lcdText, "  Emanuele  ");
	}
}

void Mazda3Lcd::pushMessage(int msgId, byte* msgData)
{
    Message msg;
	msg.busId = 2;
	msg.frame_id = msgId;    
    msg.length = 8;
	msg.dispatch = true;
	memcpy(msg.frame_data, msgData, 8);
	_writeQueue->push(msg); 
}

char Mazda3Lcd::formatGear(byte gear) 
{
	if (gear == 0) return '-';
	else if (gear < 7) return 0xE0 + _mazda->gear;
	else if (gear == 0xE) return'R';
	return gear;	
}

#endif // Mazda3Lcd_H

