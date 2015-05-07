#include "arduinocomm.h"
using namespace std;

using namespace serial;


Arduinocomm::Arduinocomm(string device, const unsigned int speed)
{
    cout << "Starting serial" <<endl;
    auto deleter = [&](Serial* s){s->close();delete(s);};
    mSerial = unique_ptr<Serial, decltype(deleter)>(new Serial(device,speed,Timeout::simpleTimeout(1)),deleter);
    /*serialstream.SetBaudRate(SerialStreamBuf::BAUD_115200);
      serialstream.SetCharSize(SerialStreamBuf::CHAR_SIZE_8);
      serialstream.SetNumOfStopBits(1);
      serialstream.SetParity(SerialStreamBuf::PARITY_NONE);
      serialstream.SetFlowControl(SerialStreamBuf::FLOW_CONTROL_NONE);
      serialstream.Open("/dev/ttyACM0");*/
}

uint64_t ac_t = 0;
void Arduinocomm::update()
{
    if(input_position >= input_size){
        input_position = 0; 
        input_size = 0;
        //LOG(DEBUG) << "Checking for data, available: " << mSerial->available();

        unsigned int ret = mSerial->read((uint8_t *)input_buffer,MAX_BUFFER);
        //LOG(DEBUG) << "ret: " << ret << endl;
        //LOG(DEBUG) << "avail: " << nanos() - ac_t << endl;

        if(ret <= 0){return;}

        if(ret>0){input_size = ret;process();}
        //LOG(DEBUG) << "process " << nanos - ac_t << endl;
        //LOG(DEBUG) << "after avail: " << nanos() - ac_t << endl;
    }
    else{
        process();
    }
}

void Arduinocomm::process()
{
    uint8_t val;uint8_t crc ;uint8_t pb_pos = 0;
    int count = 0;
    while(input_position < input_size)
    {
        count++;
        if(count > 100){return;}

        //LOG(DEBUG) << "Stuck?" << endl;
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

                    pb_pos = 0;
                    for(int i=0;i<packet_position;i+=2)
                    {
                        packet_buffer[pb_pos++] = readbyte(i);
                        //LOG(DEBUG) << "pb_pos: " << pb_pos-1 << " p: " << (int)packet_buffer[pb_pos-1] << endl;
                    }

                    crc = CRC8((const uint8_t*)&temp_buffer,packet_position-2,0);

                    //LOG(DEBUG) << "crc calc:"  << (int)crc << " crc packet: " << (int)packet_buffer[pb_pos-1] << endl;

                    if(crc == packet_buffer[pb_pos-1] && reading_packet)
                    {
                        //LOG(DEBUG) << "crc matches" << endl;
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

//uint16_t Arduinocomm::read_uint16(uint8_t (&packet)[MAX_BUFFER], unsigned int position)
uint16_t Arduinocomm::read_uint16(unsigned int position)
{
    return packet_buffer[position] + (packet_buffer[position+1] << 8);
}

uint32_t Arduinocomm::read_uint32(uint8_t (&packet)[MAX_BUFFER], unsigned int position)
{
    return packet[position] + (packet[position+1] << 8) + (packet[position+2] << 16) + (packet[position+3] << 24);
}

void Arduinocomm::writecommand(uint8_t byte)
{
    bytes.push_back(byte);
    //mSerial->write(&byte,1);
    //LOG(DEBUG) << "writecommand" << (unsigned int) byte;
}

//Writes 1 byte as 2 bytes, converting from 8-bit to 7-bit packing.
void Arduinocomm::writebyte(uint8_t byte)
{
    /*uint8_t bytes[2] = {0,0};
    bytes[0] =  byte & 0x7F;
    bytes[1] = (byte & 0x80) >> 7;*/
    bytes.push_back(byte & 0x7F);
    bytes.push_back((byte & 0x80) >> 7);
    //int written = mSerial->write(bytes,2);
    //LOG(DEBUG) << "Writebyte: " << (unsigned int)bytes[0] << ", " << (unsigned int)bytes[1];
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
    //LOG(DEBUG) << "Driving: " << (int)left << " " << (int)right << endl;
    writecommand(START_DATA);
    writebyte(DRIVE);
    writebyte(left);
    writebyte(right);
    writebyte(CRC8(bytes,bytes.size(),1));
    writecommand(END_DATA);
    mSerial->write(bytes.data(),bytes.size());
    mSerial->flush();
    bytes.clear();
}

///CRC-8 - based on the CRC8 formulas by Dallas/Maxim
////code released under the therms of the GNU GPL 3.0 license
uint8_t Arduinocomm::CRC8(vector<uint8_t> &data, uint8_t len, uint8_t start) {
    return CRC8(data.data(),len,start);
}
uint8_t Arduinocomm::CRC8(const uint8_t *data, uint8_t len, uint8_t start) {
    //LOG(DEBUG) << "len " << (int)len<< " start " << (int)start <<   endl;
    uint8_t crc = 0x00;
    for(uint8_t i=start;i<len;i++)
    {
        uint8_t extract = data[i];
        //LOG(DEBUG) << (int)data[i] << endl;
        for (uint8_t tempI = 8; tempI; tempI--) {
            uint8_t sum = (crc ^ extract) & 0x01;
            crc >>= 1;
            if (sum) {
                crc ^= 0x8C;
            }
            extract >>= 1;
        }
    }
    //LOG(DEBUG) << "crc: " << (int)crc << endl;
    return crc;
}
