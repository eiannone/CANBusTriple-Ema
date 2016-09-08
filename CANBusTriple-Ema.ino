/*
*  CANBus Triple
*  The Car Hacking Platform
*  https://canb.us
*  https://github.com/CANBus-Triple
*/

#include <avr/wdt.h>
#include <SPI.h>
#include <EEPROM.h>
#include <CANBus.h>
#include <MessageQueue.h>

#define BLUE_LED A0
#define RELAY_PIN A1

#define BUILDNAME "CANBus EMA"
#define BUILD_VERSION "0.5"

#define READ_BUFFER_SIZE 20
#define WRITE_BUFFER_SIZE 10


CANBus busses[] = {
    CANBus(CAN1SELECT, CAN1RESET, 1, "Bus 1"),
    CANBus(CAN2SELECT, CAN2RESET, 2, "Bus 2"),
    CANBus(CAN3SELECT, CAN3RESET, 3, "Bus 3")
};

#include "Middleware.h"
#include "Settings.h"
#include "SerialCommand.h"
#include "Mazda3CAN.h"
#include "Mazda3Lcd.h"
#include "CBTButtons.h"

Message readBuffer[READ_BUFFER_SIZE];
Message writeBuffer[WRITE_BUFFER_SIZE];
MessageQueue readQueue(READ_BUFFER_SIZE, readBuffer);
MessageQueue writeQueue(WRITE_BUFFER_SIZE, writeBuffer);

/*
*  Middleware Setup
*  http://docs.canb.us/firmware/main.html
*/
SerialCommand *serialCommand = new SerialCommand( &writeQueue );
Mazda3CAN *mazda3Can = new Mazda3CAN();
Mazda3Lcd *mazda3Lcd = new Mazda3Lcd(mazda3Can, &writeQueue);
CBTButtons *cbtButtons = new CBTButtons(mazda3Lcd, BLUE_LED, RELAY_PIN);

Middleware *activeMw[] = { serialCommand, mazda3Can, mazda3Lcd, cbtButtons };
int activeMwLength = (int)( sizeof(activeMw) / sizeof(activeMw[0]) );


void setup()
{
    Settings::init();
    delay(1);
    mazda3Lcd->init(cbt_settings.displayIndex);

    // Register additional serial command callback handlers
    serialCommand->registerCommand(0xA0, 1, mazda3Can);
    serialCommand->registerCommand(0xA1, 1, mazda3Lcd);

    Serial.begin( 115200 ); // USB
    Serial1.begin( 57600 ); // UART

    /*
    *  Power LED
    */
    DDRE |= B00000100;
    PORTE |= B00000100;

    /*
    *  BLE112 Init
    */
    pinMode( BT_SLEEP, OUTPUT );
    digitalWrite( BT_SLEEP, HIGH ); // Keep BLE112 Awake

    /*
    *  Boot LED
    */
    pinMode( BOOT_LED, OUTPUT );

    /*
    *  Blue LED
    */
    pinMode( BLUE_LED, OUTPUT );

    pinMode( CAN1INT_D, INPUT );
    pinMode( CAN2INT_D, INPUT );
    pinMode( CAN3INT_D, INPUT );
    pinMode( CAN1RESET, OUTPUT );
    pinMode( CAN2RESET, OUTPUT );
    pinMode( CAN3RESET, OUTPUT );
    pinMode( CAN1SELECT, OUTPUT );
    pinMode( CAN2SELECT, OUTPUT );
    pinMode( CAN3SELECT, OUTPUT );

    digitalWrite(CAN1RESET, LOW);
    digitalWrite(CAN2RESET, LOW);
    digitalWrite(CAN3RESET, LOW);


    // Setup CAN Busses
    for (int b = 0; b < 3; b++) {
        busses[b].begin(); // Resets bus and puts it in CONTROL mode
        busses[b].setClkPre(1);
        busses[b].baudConfig(cbt_settings.busCfg[b].baud);
        busses[b].setRxInt(true);
        
        if (b == 0) {
            // Filters on RX buffer 0 (high priority)
            busses[b].setFilterSingle(0, 0x430); // Fuel level
            busses[b].setFilterSingle(1, 0x231); // Gear
            busses[b].setMask(0, 0xFFFF); // Enable filter on buffer 0

            // Filters on RX buffer 1 (low priority)
            for(int f = 2; f <= 5; f++) busses[b].setFilterSingle(f, 0x4DA); // Steering wheel
            busses[b].setMask(1, 0xFFFF); // Enable filter on buffer 1
        }
        else if (b == 1) {
            // Filters on RX buffer 0 (high priority)
            busses[b].setFilterSingle(0, 0x201); // RPM and vehicle speed
            busses[b].setFilterSingle(1, 0x420); // Oil temp., distance, fuel consuming
            busses[b].setMask(0, 0xFFFF); // Enable filter on buffer 0

            // Filters on RX buffer 1 (low priority)
            for(int f = 2; f <= 5; f++) busses[b].setFilterSingle(f, 0x433); // Internal temperature
            busses[b].setMask(1, 0xFFFF); // Enable filter on buffer 1
        }

        busses[b].bitModify(RXB0CTRL, 0x04, 0x04); // Set buffer rollover enabled
        busses[b].setMode(cbt_settings.busCfg[b].mode);
    }

    // Start button listening
    cbtButtons->begin();

    for(int l = 0; l < 5; l++) {
        digitalWrite( BLUE_LED, HIGH );
        delay(100);
        digitalWrite( BLUE_LED, LOW );
        delay(100);
    }

    // wdt_enable(WDTO_1S);
}


/*
*  Main Loop
*/
void loop() 
{
    // Run all middleware ticks
    for(int i = 0; i <= activeMwLength - 1; i++) activeMw[i]->tick();

    if (digitalRead(CAN1INT_D) == 0) readBus(&busses[0]);
    if (digitalRead(CAN2INT_D) == 0) readBus(&busses[1]);

    // Process received CAN message through middleware
    if (!readQueue.isEmpty()) {
        Message msg = readQueue.pop();
        for(int i = 0; i < activeMwLength; i++) msg = activeMw[i]->process(msg);
        if (msg.dispatch) writeQueue.push(msg);
    }

    bool error = false;
    while(!writeQueue.isEmpty() && !error) {
        Message msg = writeQueue.pop();
	if (msg.dispatch) {
	    error = !sendMessage(msg, &busses[msg.busId - 1]);
            // When TX Failure, add back to queue
            if (error) writeQueue.push(msg);
        }
    }

    // Pet the dog
    // wdt_reset();

} // End loop()


/*
*  Load CAN Controller buffer and set send flag
*/
bool sendMessage( Message msg, CANBus * bus )
{
    int txBuf = bus->getNextTxBuffer();
    if (txBuf < 0 || txBuf > 2) return false; // All TX buffers full

    digitalWrite(BOOT_LED, HIGH);
    bus->loadFullFrame(txBuf, msg.length, msg.frame_id, msg.frame_data );
    bus->transmitBuffer(txBuf);
    digitalWrite(BOOT_LED, LOW );
    delay(1);
    return true;
}


/*
*  Read Can Controller Buffer
*/
void readBus( CANBus * bus )
{
    byte rx_status = 0x3;
    bool bufferAvailable = true;
    while((rx_status & 0x3) && bufferAvailable) {
        rx_status = bus->readStatus();
        if (rx_status & 0x1) 
            bufferAvailable = readMsgFromBuffer(bus, 0, rx_status);
        if ((rx_status & 0x2) && bufferAvailable) 
            bufferAvailable = readMsgFromBuffer(bus, 1, rx_status);
    }
}


bool readMsgFromBuffer(CANBus * bus, byte bufferId, byte rx_status)
{
    if (readQueue.isFull()) return false;
    Message msg;
    msg.busStatus = rx_status;
    msg.busId = bus->busId;
    msg.dispatch = false;
    bus->readFullFrame(bufferId, &msg.length, msg.frame_data, &msg.frame_id );  
    return readQueue.push(msg);
}
