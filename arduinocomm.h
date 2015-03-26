//Header guard
#ifndef ARDUINOCOMM_H
#define ARDUINOCOMM_H

#ifdef __AVR_ATmega2560__
#include "Arduino.h"
#else
#include "easylogging++.h"
#include "SerialStream.h"
#endif

class Arduinocomm{
    private:
        static const uint8_t START_DATA = 0x80;
        static const uint8_t END_DATA = 0xFF;

#ifndef __AVR_ATmega2560__
        LibSerial::SerialStream serialstream;
#endif
        void writebyte(uint8_t);
        void writeuint32(uint32_t);
        uint8_t readbyte(unsigned int position);
        static const uint16_t MAX_BUFFER = 256;
        char input_buffer[MAX_BUFFER];
        char output_buffer[MAX_BUFFER];
        void process(unsigned int);

        void writecommand(uint8_t byte);

        uint16_t packet_length;
        bool reading_packet = false;

        bool waiting_ok = false;

        void handlepacket(uint8_t (&packet)[MAX_BUFFER], uint8_t);

    public:
        void update();
        Arduinocomm();
        void driveForward(uint8_t, uint32_t);
        void stop();

        uint8_t packet_buffer[MAX_BUFFER];
        uint8_t packet_position = 0;
        uint8_t packet_size = 0;
        bool packet_ready = false;

        uint32_t read_uint32(uint8_t (&packet)[MAX_BUFFER],unsigned int);
        uint16_t read_uint16(uint8_t (&packet)[MAX_BUFFER],unsigned int);
        //
        //Packet definitions
        static const uint8_t OK = 0x01;//Acknowledge packet
        static const uint8_t DRIVE_DURATION = 0x02;//Drive straight for duration

};

//End header guard
#endif
