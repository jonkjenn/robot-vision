#include "arduinocomm.h"
using namespace std;

using namespace serial;


Arduinocomm::Arduinocomm(string device, const unsigned int speed)
{
    mSerial = unique_ptr<Serial>(new Serial(device,speed,Timeout::simpleTimeout(1)));
    /*serialstream.SetBaudRate(SerialStreamBuf::BAUD_115200);
    serialstream.SetCharSize(SerialStreamBuf::CHAR_SIZE_8);
    serialstream.SetNumOfStopBits(1);
    serialstream.SetParity(SerialStreamBuf::PARITY_NONE);
    serialstream.SetFlowControl(SerialStreamBuf::FLOW_CONTROL_NONE);
    serialstream.Open("/dev/ttyACM0");*/
}

void Arduinocomm::update()
{
    if(input_position >= input_size){
        input_position = 0; 
        input_size = 0;
        //LOG(DEBUG) << "Checking for data, available: " << mSerial->available();
        if(mSerial->available() > 0)
        {
            unsigned int ret = mSerial->read((uint8_t *)input_buffer,MAX_BUFFER);

            if(ret>0){input_size = ret;process();}
        }
    }
    else{process();}
}

void Arduinocomm::process()
{
    uint8_t val;
    while(input_position < input_size)
    {
        val = input_buffer[input_position];
        if(val > 127)//Command
        {
            switch(val)
            {
                case START_DATA:
                    packet_ready = false;
                    packet_position = 0;
                    reading_packet = true;
                    packet_size = 0;
                    input_position++;
                    //LOG(DEBUG) << "Got packet start";
                    continue;
                case END_DATA:
                    //LOG(DEBUG) << "Got packet end";

                    for(int i=0;i<packet_position;i+=2)
                    {
                        packet_buffer[i/2] = readbyte(i);
                        //LOG(DEBUG) << "i: " << i/2 << " p: " << (unsigned int)packet_buffer[i/2];
                    }

                    if(reading_packet)
                    {
                        packet_ready = true;
                        packet_size = packet_position;
                    }
                    reading_packet = false;
                    input_position++;
                    return;
                default:
                    LOG(DEBUG) << "Corrupted packet?";
                    sendcustombyte(22);

                    packet_position = 0;
                    packet_ready = false;
                    reading_packet = false;
                    input_position++;
                    continue;
            }
        }
        else
        {
            if(reading_packet)
            {
                temp_buffer[packet_position] = input_buffer[input_position];
                packet_position++;
                input_position++;
            }
            else
            {
                input_position++;
            }
        }
    }
}

//Reads 2 bytes into 1 byte, converting from 7-bit packing to 8-bits.
uint8_t Arduinocomm::readbyte(unsigned int position)
{
    return temp_buffer[position] + (temp_buffer[position+1] << 7);
}

uint16_t Arduinocomm::read_uint16(uint8_t (&packet)[MAX_BUFFER], unsigned int position)
{
    return packet[position] + (packet[position+1] << 8);
}

uint32_t Arduinocomm::read_uint32(uint8_t (&packet)[MAX_BUFFER], unsigned int position)
{
    return packet[position] + (packet[position+1] << 8) + (packet[position+2] << 16) + (packet[position+3] << 24);
}

void Arduinocomm::writecommand(uint8_t byte)
{
    mSerial->write(&byte,1);
    //LOG(DEBUG) << "writecommand" << (unsigned int) byte;
    mSerial->flush();
}

//Writes 1 byte as 2 bytes, converting from 8-bit to 7-bit packing.
void Arduinocomm::writebyte(uint8_t byte)
{
    uint8_t bytes[2] = {0,0};
    bytes[0] =  byte & 0x7F;
    bytes[1] = (byte & 0x80) >> 7;
    int written = mSerial->write(bytes,2);
    //LOG(DEBUG) << "Writebyte: " << (unsigned int)bytes[0] << ", " << (unsigned int)bytes[1];
    mSerial->flush();
}

void Arduinocomm::writeuint32(uint32_t value)
{
    writebyte(value & 0x000000FF);
    writebyte((value >> 8) & 0x000000FF);
    writebyte((value >> 16) & 0x000000FF);
    writebyte((value >> 24) & 0x000000FF);
}

void Arduinocomm::sendcustombyte(uint8_t byte)
{
    writecommand(START_DATA);
    writebyte(DEBUG);
    writebyte(byte);
    writecommand(END_DATA);
}

void Arduinocomm::writeok()
{
    writecommand(START_DATA);
    writebyte(OK);
    writecommand(END_DATA);
}

//Speed from 0 to 180, 90 = stop,  180 max forward
void Arduinocomm::drive(uint8_t left, uint8_t right)
{
    writecommand(START_DATA);
    writebyte(DRIVE);
    writebyte(left);
    writebyte(right);
    writecommand(END_DATA);
}
