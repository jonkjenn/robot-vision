#include "arduinocomm.hpp"
using namespace std;


Arduinocomm::Arduinocomm()
{
  my_serial = unique_ptr<serial::Serial>(new serial::Serial{"/dev/ttyACM0", 115200, serial::Timeout::simpleTimeout(1000)});
  my_serial->setTimeout(serial::Timeout::max(), 1, 0, 1, 0);
}

void Arduinocomm::update()
{
    int result = my_serial->read(buffer,4);
    if(result>0)
    {
        LOG(DEBUG) << "Result: " << int(buffer[0]) << " " << int(buffer[1]) << " " << int(buffer[2]) << " " << int(buffer[3]) << endl;
    }
}
