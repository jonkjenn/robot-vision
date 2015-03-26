#include "arduinocomm.h"
using namespace std;

#ifndef __AVR_ATmega2560__
using namespace serial;
#endif


Arduinocomm::Arduinocomm()
{
    
#ifdef __AVR_ATmega2560__
    Serial.begin(115200);
#else
    mSerial = unique_ptr<Serial>(new Serial("/dev/ttyACM0",115200,Timeout::simpleTimeout(1)));
    /*serialstream.SetBaudRate(SerialStreamBuf::BAUD_115200);
    serialstream.SetCharSize(SerialStreamBuf::CHAR_SIZE_8);
    serialstream.SetNumOfStopBits(1);
    serialstream.SetParity(SerialStreamBuf::PARITY_NONE);
    serialstream.SetFlowControl(SerialStreamBuf::FLOW_CONTROL_NONE);
    serialstream.Open("/dev/ttyACM0");*/
#endif
}

#ifdef __AVR_ATmega2560__
void Arduinocomm::update()
{
    while(Serial.available())
    {
        unsigned int ret = Serial.readBytes(input_buffer,Serial.available());
        if(ret>0){process(ret);}
    }
}
#else

void Arduinocomm::update()
{
    while(mSerial->available())
    {
        unsigned int ret = mSerial->read((uint8_t *)input_buffer,MAX_BUFFER);

        if(ret>0){process(ret);}
    }
}
#endif

void Arduinocomm::process(unsigned int count)
{
    uint8_t val;
    for(unsigned int i=0;i<count;i++)
    {
        val = input_buffer[i];
        if(val > 127)//Command
        {
            switch(val)
            {
                case START_DATA:
                    packet_ready = false;
                    if(reading_packet)
                    {
                        packet_position = 0;
                    }
                    reading_packet = true;
#ifndef __AVR_ATmega2560__
                    LOG(DEBUG) << "Got packet start";
#endif
                    continue;
                case END_DATA:
#ifndef __AVR_ATmega2560__
                    LOG(DEBUG) << "Got packet end";
#endif
                    if(reading_packet)
                    {
                        packet_ready = true;
                        packet_size = packet_position;
                    }
                    reading_packet = false;
                    continue;
                default:
#ifndef __AVR_ATmega2560__
                    LOG(DEBUG) << "Corrupted packet?";
#endif
                    packet_position = 0;
                    packet_ready = false;
                    reading_packet = false;
                    continue;
            }
        }
        else
        {
            if(reading_packet)
            {
                packet_buffer[packet_position] = readbyte(i);
                packet_position++;
                i++;
            }
        }
    }
}

//Reads 2 bytes into 1 byte, converting from 7-bit packing to 8-bits.
uint8_t Arduinocomm::readbyte(unsigned int position)
{
    return input_buffer[position] + (input_buffer[position+1] >> 7);
}

uint16_t Arduinocomm::read_uint16(uint8_t (&packet)[MAX_BUFFER], unsigned int position)
{
    return packet[position] + (packet[position+1] << 8);
}

uint32_t Arduinocomm::read_uint32(uint8_t (&packet)[MAX_BUFFER], unsigned int position)
{
    return packet[position] + (packet[position+1] << 8) + (packet[position+2] << 16) + (packet[position+3] << 24);
}

#ifdef __AVR_ATmega2560__

void Arduinocomm::writecommand(uint8_t byte)
{
    Serial.write(byte);
}
//Writes 1 byte as 2 bytes, converting from 8-bit to 7-bit packing.
void Arduinocomm::writebyte(uint8_t byte)
{
    Serial.write(byte & 0x7F);
    Serial.write((byte & 0x80) >> 7);
}
#else
void Arduinocomm::writecommand(uint8_t byte)
{
    mSerial->write(&byte,1);
}

//Writes 1 byte as 2 bytes, converting from 8-bit to 7-bit packing.
void Arduinocomm::writebyte(uint8_t byte)
{
    uint8_t bytes[2] = {0,0};
    bytes[0] =  byte & 0x7F;
    bytes[1] = (byte & 0x80) >> 7;
    mSerial->write(bytes,2);
}
#endif

void Arduinocomm::writeuint32(uint32_t value)
{
    writebyte(value & 0xFF);
    writebyte((value >> 8) & 0xFF);
    writebyte((value >> 16) & 0xFF);
    writebyte((value >> 24) & 0xFF);
}

void Arduinocomm::driveForward(uint8_t speed, uint32_t duration)
{
    writebyte(DRIVE_DURATION);
    writebyte(speed);
    writeuint32(duration);
}
