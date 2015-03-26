/*
   Firmata.cpp - Firmata library v2.4.2 - 2015-3-16
   Copyright (C) 2006-2008 Hans-Christoph Steiner.  All rights reserved.

   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   See file LICENSE.txt for further informations on licensing terms.
   */

//******************************************************************************
//* Includes
//******************************************************************************
#include "easylogging++.h"

#include "Firmata.h"
#include "binary.h"

extern "C" {
#include <string.h>
#include <stdlib.h>
}

#define byte uint8_t

using namespace LibSerial;
using namespace std;

//******************************************************************************
//* Support Functions
//******************************************************************************

void FirmataClass::sendValueAsTwo7bitBytes(uint8_t value)
{
    FirmataStream << (value & B01111111); // LSB
    FirmataStream << (value >> 7 & B01111111); // MSB
}

void FirmataClass::startSysex(void)
{
    
    FirmataStream << (uint8_t)(START_SYSEX);
}

void FirmataClass::endSysex(void)
{
    LOG(DEBUG) << "Writing END_SYSEX " << END_SYSEX;
    FirmataStream << (uint8_t)(END_SYSEX);
}

//******************************************************************************
//* Constructors
//******************************************************************************

FirmataClass::FirmataClass()
{
    firmwareVersionCount = 0;
    firmwareVersionVector = 0;
}

//******************************************************************************
//* Public Methods
//******************************************************************************

/* begin method with default serial bitrate */
void FirmataClass::begin()
{
    begin("/dev/ttyACM0",SerialStreamBuf::BAUD_57600);
}

/* begin method for overriding default serial bitrate */
void FirmataClass::begin(string port, SerialStreamBuf::BaudRateEnum speed)
{
    FirmataStream.SetBaudRate(speed);
    FirmataStream.SetCharSize(SerialStreamBuf::CHAR_SIZE_8);
    FirmataStream.SetNumOfStopBits(1);
    FirmataStream.SetParity(SerialStreamBuf::PARITY_NONE);
    FirmataStream.SetFlowControl(SerialStreamBuf::FLOW_CONTROL_NONE);
    FirmataStream.Open(port);
}

// output the protocol version message to the serial port
void FirmataClass::printVersion(void)
{
    FirmataStream << (REPORT_VERSION);
    FirmataStream << (FIRMATA_MAJOR_VERSION);
    FirmataStream << (FIRMATA_MINOR_VERSION);
}

//------------------------------------------------------------------------------
// Serial Receive Handling

int FirmataClass::available(void)
{
    int peek =  FirmataStream.peek();
    if(peek != EOF)
    {
        return 1;
    }
    return 0;
}

void FirmataClass::processSysexMessage(void)
{
    switch (storedInputData[0]) { //first byte in buffer is command
        case REPORT_FIRMWARE:
            break;
        case STRING_DATA:
            if (currentStringCallback) {
                byte bufferLength = (sysexBytesRead - 1) / 2;
                byte i = 1;
                byte j = 0;
                while (j < bufferLength) {
                    // The string length will only be at most half the size of the
                    // stored input buffer so we can decode the string within the buffer.
                    storedInputData[j] = storedInputData[i];
                    i++;
                    storedInputData[j] += (storedInputData[i] << 7);
                    i++;
                    j++;
                }
                // Make sure string is null terminated. This may be the case for data
                // coming from client libraries in languages that don't null terminate
                // strings.
                if (storedInputData[j - 1] != '\0') {
                    storedInputData[j] = '\0';
                }
                (*currentStringCallback)((char *)&storedInputData[0]);
            }
            break;
        default:
            if (currentSysexCallback)
                (currentSysexCallback)(storedInputData[0], sysexBytesRead, storedInputData + 1);
    }
}

void FirmataClass::processInput(void)
{
    unsigned char inputData = 0;
    FirmataStream >> inputData; // this is 'int' to handle -1 when no data
    int command;

    // TODO make sure it handles -1 properly

    LOG(DEBUG) << "char: " << (unsigned int)inputData;

    if (parsingSysex) {
        if (inputData == END_SYSEX) {
            //stop sysex byte
            parsingSysex = false;
            //fire off handler function
            processSysexMessage();
        } else {
            //normal data byte - add to buffer
            storedInputData[sysexBytesRead] = inputData;
            sysexBytesRead++;
        }
    } else if ( (waitForData > 0) && (inputData < 128) ) {
        waitForData--;
        storedInputData[waitForData] = inputData;
        if ( (waitForData == 0) && executeMultiByteCommand ) { // got the whole message
            switch (executeMultiByteCommand) {
                case ANALOG_MESSAGE:
                    if (currentAnalogCallback) {
                        (currentAnalogCallback)(multiByteChannel,
                                (storedInputData[0] << 7)
                                + storedInputData[1]);
                    }
                    break;
                case DIGITAL_MESSAGE:
                    if (currentDigitalCallback) {
                        (*currentDigitalCallback)(multiByteChannel,
                                (storedInputData[0] << 7)
                                + storedInputData[1]);
                    }
                    break;
                case SET_PIN_MODE:
                    if (currentPinModeCallback)
                        (*currentPinModeCallback)(storedInputData[1], storedInputData[0]);
                    break;
                case REPORT_ANALOG:
                    if (currentReportAnalogCallback)
                        (*currentReportAnalogCallback)(multiByteChannel, storedInputData[0]);
                    break;
                case REPORT_DIGITAL:
                    if (currentReportDigitalCallback)
                        (*currentReportDigitalCallback)(multiByteChannel, storedInputData[0]);
                    break;
            }
            executeMultiByteCommand = 0;
        }
    } else {
        // remove channel info from command byte if less than 0xF0
        if (inputData < 0xF0) {
            command = inputData & 0xF0;
            multiByteChannel = inputData & 0x0F;
        } else {
            command = inputData;
            // commands in the 0xF* range don't use channel data
        }
        switch (command) {
            case ANALOG_MESSAGE:
            case DIGITAL_MESSAGE:
            case SET_PIN_MODE:
                waitForData = 2; // two data bytes needed
                executeMultiByteCommand = command;
                break;
            case REPORT_ANALOG:
            case REPORT_DIGITAL:
                waitForData = 1; // one data byte needed
                executeMultiByteCommand = command;
                break;
            case START_SYSEX:
                parsingSysex = true;
                sysexBytesRead = 0;
                break;
        }
    }
}

//------------------------------------------------------------------------------
// Serial Send Handling

// send an analog message
void FirmataClass::sendAnalog(byte pin, int value)
{
    // pin can only be 0-15, so chop higher bits
    FirmataStream << (ANALOG_MESSAGE | (pin & 0xF));
    sendValueAsTwo7bitBytes(value);
}

// send a single digital pin in a digital message
void FirmataClass::sendDigital(byte pin, int value)
{
    /* TODO add single pin digital messages to the protocol, this needs to
     * track the last digital data sent so that it can be sure to change just
     * one bit in the packet.  This is complicated by the fact that the
     * numbering of the pins will probably differ on Arduino, Wiring, and
     * other boards.  The DIGITAL_MESSAGE sends 14 bits at a time, but it is
     * probably easier to send 8 bit ports for any board with more than 14
     * digital pins.
     */

    // TODO: the digital message should not be sent on the serial port every
    // time sendDigital() is called.  Instead, it should add it to an int
    // which will be sent on a schedule.  If a pin changes more than once
    // before the digital message is sent on the serial port, it should send a
    // digital message for each change.

    //    if(value == 0)
    //        sendDigitalPortPair();
}


// send 14-bits in a single digital message (protocol v1)
// send an 8-bit port in a single digital message (protocol v2)
void FirmataClass::sendDigitalPort(byte portNumber, int portData)
{
    FirmataStream << (DIGITAL_MESSAGE | (portNumber & 0xF));
    FirmataStream << ((byte)portData % 128); // Tx bits 0-6
    FirmataStream << (portData >> 7);  // Tx bits 7-13
}


void FirmataClass::sendSysex(byte command, byte bytec, byte *bytev)
{
    byte i;
    startSysex();

    FirmataStream << (command);
    LOG(DEBUG) << "Writing command: " << command << endl;

    for (i = 0; i < bytec; i++) {
        sendValueAsTwo7bitBytes(bytev[i]);
    }
    endSysex();
}

void FirmataClass::sendString(byte command, const char *string)
{
    sendSysex(command, strlen(string), (byte *)string);
}


// send a string as the protocol string type
void FirmataClass::sendString(const char *string)
{
    sendString(STRING_DATA, string);
}

// expose the write method
void FirmataClass::write(byte c)
{
    FirmataStream << (c);
}
//
// generic callbacks
void FirmataClass::attach(byte command, callbackFunction newFunction)
{
    switch (command) {
        case ANALOG_MESSAGE: currentAnalogCallback = newFunction; break;
        case DIGITAL_MESSAGE: currentDigitalCallback = newFunction; break;
        case REPORT_ANALOG: currentReportAnalogCallback = newFunction; break;
        case REPORT_DIGITAL: currentReportDigitalCallback = newFunction; break;
        case SET_PIN_MODE: currentPinModeCallback = newFunction; break;
    }
}

/*void FirmataClass::attach(byte command, systemResetCallbackFunction newFunction)
{
    switch (command) {
        case SYSTEM_RESET: currentSystemResetCallback = newFunction; break;
    }
}*/

void FirmataClass::attach(byte command, stringCallbackFunction newFunction)
{
    switch (command) {
        case STRING_DATA: currentStringCallback = newFunction; break;
    }
}

/*void FirmataClass::attach(byte command, sysexCallbackFunction newFunction)
{
    currentSysexCallback = newFunction;
}*/

void FirmataClass::detach(byte command)
{
    switch (command) {
        case SYSTEM_RESET: currentSystemResetCallback = NULL; break;
        case STRING_DATA: currentStringCallback = NULL; break;
        case START_SYSEX: currentSysexCallback = NULL; break;
        default:
          attach(command, (callbackFunction)NULL);
    }
}
