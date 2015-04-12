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
#include <stdlib.h>
#include <Arduino.h>



// Base class data member initialization (called by derived class init())
void QTRSensors::init(unsigned char *pins, unsigned char numSensors,
  unsigned char emitterPin)
{
    /*calibratedMinimumOn=0;
    calibratedMaximumOn=0;
    calibratedMinimumOff=0;
    calibratedMaximumOff=0;*/

    if (numSensors > QTR_MAX_SENSORS)
        _numSensors = QTR_MAX_SENSORS;
    else
        _numSensors = numSensors;

    if (_pins == 0)
    {
        _pins = (unsigned char*)malloc(sizeof(unsigned char)*_numSensors);
        if (_pins == 0)
            return;
    }

    unsigned char i;
    for (i = 0; i < _numSensors; i++)
    {
        _pins[i] = pins[i];
    }

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
void QTRSensors::read(unsigned char readMode)
{
    _readMode = readMode;

    if(readMode == QTR_EMITTERS_ON || readMode == QTR_EMITTERS_ON_AND_OFF)
        emittersOn();
    else
        emittersOff();

    readPrivate();

}

void QTRSensors::HandleReadComplete()
{
    //unsigned int off_values[QTR_MAX_SENSORS];
    //unsigned char i;

    emittersOff();

    /*if(_readMode == QTR_EMITTERS_ON_AND_OFF)
    {
        readPrivate(off_values);

        for(i=0;i<_numSensors;i++)
        {
            sensor_values[i] += _maxValue - off_values[i];
        }
    }*/
    if(step){step = 4;}
}


// Turn the IR LEDs off and on.  This is mainly for use by the
// read method, and calling these functions before or
// after the reading the sensors will have no effect on the
// readings, but you may wish to use these for testing purposes.
void QTRSensors::emittersOff()
{
    if (_emitterPin == QTR_NO_EMITTER_PIN)
        return;
    //pinMode(_emitterPin, OUTPUT);
    //PORTB &= ~_BV(_emitterPin);
    //digitalWrite(_emitterPin, LOW);
    //delayMicroseconds(200);
    DDRC |= B10000000;
    PORTC &= B01111111;
}

void QTRSensors::emittersOn()
{
    if (_emitterPin == QTR_NO_EMITTER_PIN)
        return;
    //pinMode(_emitterPin, OUTPUT);
    //digitalWrite(_emitterPin, HIGH);
    //PORTC |= _emitterPin;
    
    DDRC |= B10000000;
    PORTC |= B10000000;
    //delayMicroseconds(200);
}

// Resets the calibration.
void QTRSensors::resetCalibration()
{
    unsigned char i;
    for(i=0;i<_numSensors;i++)
    {
        if(calibratedMinimumOn)
            calibratedMinimumOn[i] = _maxValue;
        if(calibratedMinimumOff)
            calibratedMinimumOff[i] = _maxValue;
        if(calibratedMaximumOn)
            calibratedMaximumOn[i] = 0;
        if(calibratedMaximumOff)
            calibratedMaximumOff[i] = 0;
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
        calibrateOnOrOff(&calibratedMinimumOn,
                         &calibratedMaximumOn,
                         QTR_EMITTERS_ON);
    }


    if(readMode == QTR_EMITTERS_ON_AND_OFF || readMode == QTR_EMITTERS_OFF)
    {
        calibrateOnOrOff(&calibratedMinimumOff,
                         &calibratedMaximumOff,
                         QTR_EMITTERS_OFF);
    }
    Serial.println("Calibrated");
}

void QTRSensors::calibrateOnOrOff(unsigned int **calibratedMinimum,
                                        unsigned int **calibratedMaximum,
                                        unsigned char readMode)
{
    int i;
    uint16_t sensor_values[16];
    uint16_t max_sensor_values[16];
    uint16_t min_sensor_values[16];

    // Allocate the arrays if necessary.
    if(*calibratedMaximum == 0)
    {
        *calibratedMaximum = (uint16_t *)malloc(sizeof(uint16_t)*_numSensors);

        // If the malloc failed, don't continue.
        if(*calibratedMaximum == 0)
            return;

        // Initialize the max and min calibrated values to values that
        // will cause the first reading to update them.

        for(i=0;i<_numSensors;i++)
            (*calibratedMaximum)[i] = 0;
    }
    if(*calibratedMinimum == 0)
    {
        *calibratedMinimum = (uint16_t *)malloc(sizeof(uint16_t )*_numSensors);

        // If the malloc failed, don't continue.
        if(*calibratedMinimum == 0)
            return;

        for(i=0;i<_numSensors;i++)
            (*calibratedMinimum)[i] = _maxValue;
    }

    int j;
    for(j=0;j<10;j++)
    {
        read(readMode);
        for(i=0;i<_numSensors;i++)
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
    for(i=0;i<_numSensors;i++)
    {
        if(min_sensor_values[i] > (*calibratedMaximum)[i])
            (*calibratedMaximum)[i] = min_sensor_values[i];
        if(max_sensor_values[i] < (*calibratedMinimum)[i])
            (*calibratedMinimum)[i] = max_sensor_values[i];
    }
}


// Returns values calibrated to a value between 0 and 1000, where
// 0 corresponds to the minimum value read by calibrate() and 1000
// corresponds to the maximum value.  Calibration values are
// stored separately for each sensor, so that differences in the
// sensors are accounted for automatically.
void QTRSensors::readCalibrated(unsigned char readMode)
{
    _readMode = readMode;

    // if not calibrated, do nothing
    //if(readMode == QTR_EMITTERS_ON_AND_OFF || readMode == QTR_EMITTERS_OFF)
    //    if(!calibratedMinimumOff || !calibratedMaximumOff)
            //return;
    //if(readMode == QTR_EMITTERS_ON_AND_OFF || readMode == QTR_EMITTERS_ON)
    //    if(!calibratedMinimumOn || !calibratedMaximumOn)
    //        return;

    // read the needed values
    read(readMode);

}

int ctest = 0;
void QTRSensors::HandleReadCalibratedResults()
{
    uint16_t calmin,calmax;
    uint16_t denominator;
    uint16_t x = 0;

    ctest++;

    /*if(ctest%100==0){
        for(int i=0;i<_numSensors;i++)
        {
            Serial.print(String(_sensor_values[i]) + " ");
        }
        Serial.println("");
    }*/

    for(int i=0;i<_numSensors;i++)
    {
        // find the correct calibration
        //if(_readMode == QTR_EMITTERS_ON)
        //{
        calmax = calibratedMaximumOn[i];
        calmin = calibratedMinimumOn[i];
        //}
        /*
        else if(_readMode == QTR_EMITTERS_OFF)
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
        }*/

        denominator = calmax - calmin;

        //Serial.println("Sv: " + String(sensor_values[i]));

        if(denominator != 0)
        {
            if(_sensor_values[i] < calmin)
            {
                x = 0;
            }
            else
            {
                x = ((float)(_sensor_values[i] - calmin)  * (1000.0 / denominator));
            }
        }

        if(x > 1000)
            x = 1000;
        _sensor_values[i] = x;

        //Serial.println("s");
        //Serial.println(i);
        //Serial.println(x);

    }



     /*if(ctest%1==0){
        for(int i=0;i<_numSensors;i++)
        {
            Serial.print(String(_sensor_values[i]));
            Serial.print('\t');
        }
        Serial.println();
    }*/

    //if(step){step = 5;}
    HandleReadLineResults();

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
void QTRSensors::readLine(uint16_t *sensor_values,
    unsigned char readMode, unsigned char white_line, unsigned int *position, int *result_ready, uint16_t *calibrated_min, uint16_t *calibrated_max)
{
    calibratedMaximumOn = calibrated_max;
    calibratedMinimumOn = calibrated_min;
    _result_ready = result_ready;
    *_result_ready = 0;
    _position = position;
    _white_line = white_line;
    _sensor_values = sensor_values;

    step = 1;

    readCalibrated(readMode);
}

unsigned long hravg; // this is for the weighted total, which is long
                   // before division
uint16_t hrsum; // this is for the denominator which is <= 64000
uint16_t hrlast_value=0; // assume initially that the line is left.

void QTRSensors::HandleReadLineResults()
{
    bool on_line = false;
    hravg = 0;
    hrsum = 0;
    uint16_t value = 0;

    for(uint8_t i=0;i<_numSensors;i++) {
        value = _sensor_values[i];
        if(_white_line)
            value = 1000-value;

        // keep track of whether we see the line at all
        if(value > 200) {
            on_line = true;
        }

        // only average in values that are above a noise threshold
        if(value > 100) {
            hravg += (unsigned long)(value) * (unsigned long)(i * 1000);
            hrsum += value;
        }
    }

    if(!on_line)
    {
        // If it last read to the left of center, return 0.
        if(hrlast_value < (_numSensors-1)*500)
            hrlast_value = 0;

        // If it last read to the right of center, return the max.
        else
            hrlast_value =  (_numSensors-1)*1000;
    }
    else
    {
        hrlast_value = hravg/hrsum;
    }

    //Serial.println(hrlast_value);

    *_position = hrlast_value;
    *_result_ready = 1;
}

QTRSensors::QTRSensors(unsigned char* pins,
  unsigned char numSensors, unsigned int timeout, unsigned char emitterPin, Drive *driver)
{
    calibratedMinimumOn = 0;
    calibratedMaximumOn = 0;
    calibratedMinimumOff = 0;
    calibratedMaximumOff = 0;
    _pins = 0;
    _driver = driver;

    init(pins, numSensors, timeout, emitterPin);
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
void QTRSensors::init(unsigned char* pins,
    unsigned char numSensors, unsigned int timeout, unsigned char emitterPin)
{
    QTRSensors::init(pins, numSensors, emitterPin);

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
void QTRSensors::readPrivate()
{
    //unsigned long start = micros();
    unsigned char i;

    if (_pins == 0)
        return;

    for(i = 0; i < _numSensors; i++)
    {
        _sensor_values[i] = _maxValue;
        //pinMode(_pins[i], OUTPUT);      // drive sensor line high
        //digitalWrite(_pins[i], HIGH);   // make sensor line an output
    }

    DDRA = B11111111;//Set register A ports for output
    PORTA = B11111111;//Set set register A ports to HIGH
    //pinMode(22,OUTPUT);
    //digitalWrite(22,HIGH);
    readPrivate_start =micros();

    delayMicroseconds(5);// charge lines for 10 us

    readPrivate2();
    //if(debug){Serial.println("readPrivate duration: " + String(micros() - start));}
}

void QTRSensors::readPrivate2()
{
    //unsigned long time = micros() - readPrivate_start;
    //if(debug){Serial.println("readPrivate2 time: " + String(time));}

    DDRA = B00000000;//Set register A ports for INPUT
    PORTA = B00000000;//Disable pull ups
    //pinMode(22,INPUT);
    //digitalWrite(22,LOW);
    //PORTA = B00000000;//Set set register A ports to LOW
    /*for(int i = 0; i < _numSensors; i++)
    {
        pinMode(_pins[i], INPUT);       // make sensor line an input
        digitalWrite(_pins[i], LOW);        // important: disable internal pull-up!
    }*/

    step = 2;
    readPrivate3();
}

unsigned long rpt3 = 0;
int ftest = 0;
void QTRSensors::readPrivate3()
{
    ftest++;
    //Serial.println("PINA: " + String(PINA));
    //rpt3 = micros() - readPrivate_start;
    readPrivate_start = micros();
    rpt3 = 0;
    //unsigned long start_readrprivate3 = micros();
    //if(debug){Serial.println("readPrivate3 time1 "); Serial.println(time);}
    //if(debug){Serial.println("readPrivate3 time2 "); Serial.println(time);}
    uint8_t completed = 0;
    do
    {
        for (int i = 0; i < _numSensors; i++)
        {
            //if (digitalRead(_pins[i]) == LOW && time < sensor_values[i])
            //Serial.println(PINA);
            if (_sensor_values[i] == calibratedMaximumOn[i] && bitRead(PINA,i) == 0 && rpt3 < _sensor_values[i])
            {
                //if(debug){Serial.println("Sensors i:" + String(i) + " Time: " + String(time));}

//                Serial.println(time);
                _sensor_values[i] = rpt3;
                completed++;
            }
            //if(debug){Serial.println("pina:"  + String(PINA) + " pin: " + String(bitRead(PINA,i)) + " sv: " + String(sensor_values[i]));}
        }
        //Serial.println("ns: " + String((int)_numSensors) + " com:" + String(completed));
        if(completed == _numSensors){break;}
        if(_driver != NULL){_driver->update();}
        rpt3 = micros() - readPrivate_start;
    }while(rpt3 <_maxValue);

    /*if(ftest%1==0)
    //if(_sensor_values[0]>950)
    {
        Serial.println(rpt3);
        for(int i=0;i<_numSensors;i++)
        {
            Serial.print(_sensor_values[i]);
            Serial.print('\t');
        }
        Serial.println();
    }*/
    /*else
    {
       ctest++;
        if(ctest%100 ==0){
            for(int i=0;i<_numSensors;i++)
            {
                Serial.print(String(_sensor_values[i]) + " ");
            }
            Serial.println(" ");
        }
        step = 3;
    }*/
    if(debug)
    {
        for(int i=0;i<_numSensors;i++)
        {
            if(_sensor_values[i] < 2500 && _sensor_values[i] > low_values[i])
            {
                low_values[i] = _sensor_values[i];

                for(int j=0;j<_numSensors;j++)
                {
                    Serial.print(low_values[j]);
                    Serial.print(" ");
                }
                    Serial.println("");
            }
        }
    }
    //if(debug){Serial.println("readPrivate3 internal time: " + String(micros() - start_readrprivate3));}
    step = 3;
}

void QTRSensors::update()
{
    unsigned long start = micros();
    switch(step)
    {
        case 1:
            readPrivate2(); //Trigger lines for reading
            //if(debug){Serial.println("case 1, duration : " + String(micros() - start));}
            break;
        case 2:
            readPrivate3();//Actually read data
            //if(debug){Serial.println("case 2, duration : " + String(micros() - start));}
            break;
        case 3:
            HandleReadComplete();//Turns emitters off and other stuff
            //if(debug){Serial.println("case 3, duration : " + String(micros() - start));}
            break;
        case 4:
            HandleReadCalibratedResults();
            //if(debug){Serial.println("case 4, duration : " + String(micros() - start));}
            //Serial.println(micros()-start);
            break;
        case 5:
            HandleReadLineResults();
            //if(debug){Serial.println("case 5, duration : " + String(micros() - start));}
            break;
    }
}


// the destructor frees up allocated memory
QTRSensors::~QTRSensors()
{
    if (_pins)
        free(_pins);
    if(calibratedMaximumOn)
        free(calibratedMaximumOn);
    if(calibratedMaximumOff)
        free(calibratedMaximumOff);
    if(calibratedMinimumOn)
        free(calibratedMinimumOn);
    if(calibratedMinimumOff)
        free(calibratedMinimumOff);
}
