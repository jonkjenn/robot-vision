/*
  QTRSensors.cpp - Arduino library for using Pololu QTR reflectance
    sensors and reflectance sensor arrays: QTR-1A, QTR-8A, QTR-1RC, and
    QTR-8RC.  The object used will determine the type of the sensor (either
    QTR-xA or QTR-xRC).  Then simply specify in the constructor which
    Arduino I/O pins are connected to a QTR sensor, and the read() method
    will obtain reflectance measurements for those sensors.  Smaller sensor
    values correspond to higher reflectance (e.g. white) while larger
    sensor values correspond to lower reflectance (e.g. black or a void).

    * QTRSensorsRC should be used for QTR-1RC and QTR-8RC sensors.
    * QTRSensorsAnalog should be used for QTR-1A and QTR-8A sensors.
*/

/*
 * Written by Ben Schmidel et al., October 4, 2010.
 * Copyright (c) 2008-2012 Pololu Corporation. For more information, see
 *
 *   http://www.pololu.com
 *   http://forum.pololu.com
 *   http://www.pololu.com/docs/0J19
 *
 * You may freely modify and share this code, as long as you keep this
 * notice intact (including the two links above).  Licensed under the
 * Creative Commons BY-SA 3.0 license:
 *
 *   http://creativecommons.org/licenses/by-sa/3.0/
 *
 * Disclaimer: To the extent permitted by law, Pololu provides this work
 * without any warranty.  It might be defective, in which case you agree
 * to be responsible for all resulting costs and damages.
 */

#include "QTRSensors.h"
#include "SimpleGPIO.h"
#include <cerrno>
#include <ctime>
#include <chrono>
#include <vector>
#include <iostream>
#include "utility.hpp"

using namespace std;
using namespace std::chrono;

// Base class data member initialization (called by derived class init())
void QTRSensors::init(vector<unsigned char> &pins,
  unsigned char emitterPin)
{
    calibratedMinimumOn.clear();
    calibratedMaximumOn.clear();
    calibratedMinimumOff.clear();
    calibratedMaximumOff.clear();

    /*if (_pins.size() == 0)
    {
        _pins = (unsigned char*)malloc(sizeof(unsigned char)*_numSensors);
        if (_pins.size() == 0)
            return;
    }*/

    /*_pins.clear();

    for (size_t i = 0; i < _pins.size(); i++)
    {
        _pins[i] = pins[i];
    }*/

    _pins = pins;

    _emitterPin = emitterPin;
}


// Reads the sensor values into an array. There *MUST* be space
// for as many values as there were sensors specified in the constructor.
// Example usage:
// unsigned int sensor_values[8];
// sensors.read(sensor_values);
// The values returned are a measure of the reflectance in abstract units,
// with higher values corresponding to lower reflectance (e.g. a black
// surface or a void).
void QTRSensors::read(vector<unsigned int> &sensor_values, unsigned char readMode)
{
    vector<unsigned int> off_values(QTR_MAX_SENSORS);

    if(readMode == QTR_EMITTERS_ON || readMode == QTR_EMITTERS_ON_AND_OFF)
        emittersOn();
    else
        emittersOff();

    readPrivate(sensor_values);
    emittersOff();

    if(readMode == QTR_EMITTERS_ON_AND_OFF)
    {
        readPrivate(off_values);

        for(size_t i=0;i<_pins.size();i++)
        {
            sensor_values.push_back(_maxValue - off_values[i]);
        }
    }
}


// Turn the IR LEDs off and on.  This is mainly for use by the
// read method, and calling these functions before or
// after the reading the sensors will have no effect on the
// readings, but you may wish to use these for testing purposes.
void QTRSensors::emittersOff()
{
    if (_emitterPin == QTR_NO_EMITTER_PIN)
        return;
    gpio_export(_emitterPin);
    gpio_set_dir(_emitterPin, OUTPUT_PIN);
    gpio_set_value(_emitterPin, LOW);
    gpio_unexport(_emitterPin);
    delayMicroseconds(200);
}

void QTRSensors::emittersOn()
{
    if (_emitterPin == QTR_NO_EMITTER_PIN)
        return;
    gpio_export(_emitterPin);
    gpio_set_dir(_emitterPin, OUTPUT_PIN);
    gpio_set_value(_emitterPin, HIGH);
    gpio_unexport(_emitterPin);
    delayMicroseconds(200);
}

// Resets the calibration.
void QTRSensors::resetCalibration()
{
    unsigned char i;
    for(size_t i=0;i<_pins.size();i++)
    {
        if(!calibratedMinimumOn.empty())
            calibratedMinimumOn.assign(i, _maxValue);
        if(!calibratedMinimumOff.empty())
            calibratedMinimumOff.assign(i, _maxValue);
        if(!calibratedMaximumOn.empty())
            calibratedMaximumOn.assign(i,0);
        if(!calibratedMaximumOff.empty())
            calibratedMaximumOff.assign(i,0);
    }
}

// Reads the sensors 10 times and uses the results for
// calibration.  The sensor values are not returned; instead, the
// maximum and minimum values found over time are stored internally
// and used for the readCalibrated() method.
void QTRSensors::calibrate(unsigned char readMode)
{
    if(readMode == QTR_EMITTERS_ON_AND_OFF || readMode == QTR_EMITTERS_ON)
    {
        calibrateOnOrOff(calibratedMinimumOn,
                         calibratedMaximumOn,
                         QTR_EMITTERS_ON);
    }


    if(readMode == QTR_EMITTERS_ON_AND_OFF || readMode == QTR_EMITTERS_OFF)
    {
        calibrateOnOrOff(calibratedMinimumOff,
                         calibratedMaximumOff,
                         QTR_EMITTERS_OFF);
    }
}

void QTRSensors::calibrateOnOrOff(vector<unsigned int> &calibratedMinimum,
                                        vector<unsigned int> &calibratedMaximum,
                                        unsigned char readMode)
{
    vector<unsigned int> sensor_values;
    vector<unsigned int> max_sensor_values(_pins.size());
    vector<unsigned int> min_sensor_values(_pins.size());

    if(calibratedMaximum.empty())
    {
        calibratedMaximum.reserve(_pins.size());
        for(size_t i=0;i<_pins.size();i++)
        {
            calibratedMaximum[i] = 0;
        }
    }


    if(calibratedMinimum.empty())
    {
        //*calibratedMinimum = (unsigned int*)malloc(sizeof(unsigned int)*_numSensors);
        calibratedMinimum.reserve(_pins.size());

        for(size_t i=0;i<_pins.size();i++)
        {
            calibratedMinimum[i] = _maxValue;
        }
    }

    int j;
    for(j=0;j<10;j++)
    {
        read(sensor_values,readMode);
        for(size_t i=0;i<_pins.size();i++)
        {
            // set the max we found THIS time
            if(j == 0 || max_sensor_values[i] < sensor_values[i])
                max_sensor_values[i] = sensor_values[i];

            // set the min we found THIS time
            if(j == 0 || min_sensor_values[i] > sensor_values[i])
                min_sensor_values[i] = sensor_values[i];
        }
    }

    // record the min and max calibration values
    for(size_t i=0;i<_pins.size();i++)
    {
        if(min_sensor_values[i] > calibratedMaximum[i])
        {
            calibratedMaximum[i] = min_sensor_values[i];
        }
        if(max_sensor_values[i] < (calibratedMinimum[i]))
        {
            calibratedMinimum[i] = max_sensor_values[i];
        }
    }
}


// Returns values calibrated to a value between 0 and 1000, where
// 0 corresponds to the minimum value read by calibrate() and 1000
// corresponds to the maximum value.  Calibration values are
// stored separately for each sensor, so that differences in the
// sensors are accounted for automatically.
void QTRSensors::readCalibrated(vector<unsigned int> &sensor_values, unsigned char readMode)
{
    int i;

    // if not calibrated, do nothing
    if(readMode == QTR_EMITTERS_ON_AND_OFF || readMode == QTR_EMITTERS_OFF)
        if(calibratedMinimumOff.empty() || !calibratedMaximumOff.empty())
            return;
    if(readMode == QTR_EMITTERS_ON_AND_OFF || readMode == QTR_EMITTERS_ON)
        if(calibratedMinimumOn.empty() || calibratedMaximumOn.empty())
            return;

    // read the needed values
    read(sensor_values,readMode);

    for(size_t i=0;i<_pins.size();i++)
    {
        unsigned int calmin,calmax;
        unsigned int denominator;

        // find the correct calibration
        if(readMode == QTR_EMITTERS_ON)
        {
            calmax = calibratedMaximumOn[i];
            calmin = calibratedMinimumOn[i];
        }
        else if(readMode == QTR_EMITTERS_OFF)
        {
            calmax = calibratedMaximumOff[i];
            calmin = calibratedMinimumOff[i];
        }
        else // QTR_EMITTERS_ON_AND_OFF
        {

            if(calibratedMinimumOff[i] < calibratedMinimumOn[i]) // no meaningful signal
                calmin = _maxValue;
            else
                calmin = calibratedMinimumOn[i] + _maxValue - calibratedMinimumOff[i]; // this won't go past _maxValue

            if(calibratedMaximumOff[i] < calibratedMaximumOn[i]) // no meaningful signal
                calmax = _maxValue;
            else
                calmax = calibratedMaximumOn[i] + _maxValue - calibratedMaximumOff[i]; // this won't go past _maxValue
        }

        denominator = calmax - calmin;

        signed int x = 0;
        if(denominator != 0)
            x = (((signed long)sensor_values[i]) - calmin)
                * 1000 / denominator;
        if(x < 0)
            x = 0;
        else if(x > 1000)
            x = 1000;
        sensor_values[i] = x;
    }

}


// Operates the same as read calibrated, but also returns an
// estimated position of the robot with respect to a line. The
// estimate is made using a weighted average of the sensor indices
// multiplied by 1000, so that a return value of 0 indicates that
// the line is directly below sensor 0, a return value of 1000
// indicates that the line is directly below sensor 1, 2000
// indicates that it's below sensor 2000, etc.  Intermediate
// values indicate that the line is between two sensors.  The
// formula is:
//
//    0*value0 + 1000*value1 + 2000*value2 + ...
//   --------------------------------------------
//         value0  +  value1  +  value2 + ...
//
// By default, this function assumes a dark line (high values)
// surrounded by white (low values).  If your line is light on
// black, set the optional second argument white_line to true.  In
// this case, each sensor value will be replaced by (1000-value)
// before the averaging.
int QTRSensors::readLine(vector<unsigned int> &sensor_values,
    unsigned char readMode, unsigned char white_line)
{
    unsigned char i, on_line = 0;
    unsigned long avg; // this is for the weighted total, which is long
                       // before division
    unsigned int sum; // this is for the denominator which is <= 64000
    static int last_value=0; // assume initially that the line is left.

    readCalibrated(sensor_values, readMode);

    avg = 0;
    sum = 0;

    for(size_t i = 0;i<_pins.size();i++)
    {
        int value = sensor_values[i];
        if(white_line)
            value = 1000-value;

        // keep track of whether we see the line at all
        if(value > 200) {
            on_line = 1;
        }

        // only average in values that are above a noise threshold
        if(value > 50) {
            avg += (long)(value) * (i * 1000);
            sum += value;
        }
    }

    if(!on_line)
    {
        // If it last read to the left of center, return 0.
        if(last_value < (_pins.size()-1)*1000/2)
            return 0;

        // If it last read to the right of center, return the max.
        else
            return (_pins.size()-1)*1000;

    }

    last_value = avg/sum;

    return last_value;
}



// Derived RC class constructors
QTRSensorsRC::QTRSensorsRC()
{
    calibratedMinimumOn.clear();
    calibratedMaximumOn.clear();
    calibratedMinimumOff.clear();
    calibratedMaximumOff.clear();
    _pins.clear();
}

QTRSensorsRC::QTRSensorsRC(vector<unsigned char> &pins,
  unsigned int timeout, unsigned char emitterPin)
{
    calibratedMinimumOn.clear();
    calibratedMaximumOn.clear();
    calibratedMinimumOff.clear();
    calibratedMaximumOff.clear();
    _pins.clear();

    init(pins, timeout, emitterPin);
}


// The array 'pins' contains the Arduino pin number for each sensor.

// 'numSensors' specifies the length of the 'pins' array (i.e. the
// number of QTR-RC sensors you are using).  numSensors must be
// no greater than 16.

// 'timeout' specifies the length of time in microseconds beyond
// which you consider the sensor reading completely black.  That is to say,
// if the pulse length for a pin exceeds 'timeout', pulse timing will stop
// and the reading for that pin will be considered full black.
// It is recommended that you set timeout to be between 1000 and
// 3000 us, depending on things like the height of your sensors and
// ambient lighting.  Using timeout allows you to shorten the
// duration of a sensor-reading cycle while still maintaining
// useful analog measurements of reflectance

// 'emitterPin' is the Arduino pin that controls the IR LEDs on the 8RC
// modules.  If you are using a 1RC (i.e. if there is no emitter pin),
// or if you just want the emitters on all the time and don't want to
// use an I/O pin to control it, use a value of 255 (QTR_NO_EMITTER_PIN).
void QTRSensorsRC::init(vector<unsigned char> &pins,
    unsigned int timeout, unsigned char emitterPin)
{
    QTRSensors::init(pins, emitterPin);

    _maxValue = timeout;
}


// Reads the sensor values into an array. There *MUST* be space
// for as many values as there were sensors specified in the constructor.
// Example usage:
// unsigned int sensor_values[8];
// sensors.read(sensor_values);
// ...
// The values returned are in microseconds and range from 0 to
// timeout (as specified in the constructor).
void QTRSensorsRC::readPrivate(vector<unsigned int> &sensor_values)
{
    unsigned char i;

    if (_pins.size() == 0)
        return;

    for(size_t i=0;i<_pins.size();i++)
    {
        sensor_values.push_back(_maxValue);

        gpio_export(_pins[i]);
        gpio_set_dir(_pins[i], OUTPUT_PIN);
        gpio_set_value(_pins[i], HIGH);
        gpio_unexport(_pins[i]);
    }

    delayMicroseconds(10);              // charge lines for 10 us

    for(size_t i=0;i<_pins.size();i++)
    {
        gpio_export(_pins[i]);
        gpio_set_dir(_pins[i], INPUT_PIN);
        gpio_set_value(_pins[i], LOW);
        gpio_unexport(_pins[i]);
    }

    //unsigned long startTime = micros();
    high_resolution_clock::time_point startTime = high_resolution_clock::now();

    while (duration_cast<microseconds>(high_resolution_clock::now() - startTime).count() < _maxValue)
    {
        //unsigned int time = micros() - startTime;
        unsigned int time = duration_cast<microseconds>(high_resolution_clock::now() - startTime).count();
        //unsigned int time = (unsigned int)dur.count();

        LOG(DEBUG) << "Pin time: " << time;

        for(size_t i=0;i<_pins.size();i++)
        {
            unsigned int val;
            gpio_export(_pins[i]);
            gpio_set_dir(_pins[i], INPUT_PIN);
            gpio_get_value(_pins[i], &val) ;
            gpio_unexport(_pins[i]);
            if (val == LOW && time < sensor_values[i])
                sensor_values[i] = time;
        }
    }
}



/*// Derived Analog class constructors
QTRSensorsAnalog::QTRSensorsAnalog()
{
    calibratedMinimumOn = 0;
    calibratedMaximumOn = 0;
    calibratedMinimumOff = 0;
    calibratedMaximumOff = 0;
    _pins.clear();
}

QTRSensorsAnalog::QTRSensorsAnalog(unsigned char* pins,
  unsigned char numSensors, unsigned char numSamplesPerSensor,
  unsigned char emitterPin)
{
    calibratedMinimumOn = 0;
    calibratedMaximumOn = 0;
    calibratedMinimumOff = 0;
    calibratedMaximumOff = 0;
    _pins.clear();
    _pins = 0;

    init(pins, numSensors, numSamplesPerSensor, emitterPin);
}*/


// the array 'pins' contains the Arduino analog pin assignment for each
// sensor.  For example, if pins is {0, 1, 7}, sensor 1 is on
// Arduino analog input 0, sensor 2 is on Arduino analog input 1,
// and sensor 3 is on Arduino analog input 7.

// 'numSensors' specifies the length of the 'analogPins' array (i.e. the
// number of QTR-A sensors you are using).  numSensors must be
// no greater than 16.

// 'numSamplesPerSensor' indicates the number of 10-bit analog samples
// to average per channel (i.e. per sensor) for each reading.  The total
// number of analog-to-digital conversions performed will be equal to
// numSensors*numSamplesPerSensor.  Note that it takes about 100 us to
// perform a single analog-to-digital conversion, so:
// if numSamplesPerSensor is 4 and numSensors is 6, it will take
// 4 * 6 * 100 us = ~2.5 ms to perform a full readLine().
// Increasing this parameter increases noise suppression at the cost of
// sample rate.  The recommended value is 4.

// 'emitterPin' is the Arduino pin that controls the IR LEDs on the 8RC
// modules.  If you are using a 1RC (i.e. if there is no emitter pin),
// or if you just want the emitters on all the time and don't want to
// use an I/O pin to control it, use a value of 255 (QTR_NO_EMITTER_PIN).
/*void QTRSensorsAnalog::init(unsigned char* pins,
    unsigned char numSensors, unsigned char numSamplesPerSensor,
    unsigned char emitterPin)
{
    QTRSensors::init(pins, emitterPin);

    _numSamplesPerSensor = numSamplesPerSensor;
    _maxValue = 1023; // this is the maximum returned by the A/D conversion
}*/


// the destructor frees up allocated memory
/*QTRSensors::~QTRSensors()
{
}*/
