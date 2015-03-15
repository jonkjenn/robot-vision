//Header guard
#ifndef ARDUINOCOMM_H
#define ARDUINOCOMM_H

#include <memory>
#include "serial/serial.h"
#include "easylogging++.h"

class Arduinocomm{
    public:
        void update();
        Arduinocomm();
    private:
        std::unique_ptr<serial::Serial> my_serial;
        uint8_t *buffer = new uint8_t[4];
};

//End header guard
#endif
