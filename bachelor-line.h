#include "PID_v1.h"
#include <stdio.h>
#include "drive.h"

int16_t GetMedian(int16_t *daArray,int size);

class LineFollower{

    private:
    // This example is designed for use with eight QTR-1RC sensors or the eight sensors of a
    // QTR-8RC module.  These reflectance sensors should be connected to digital inputs 3 to 10.
    // The QTR-8RC's emitter control pin (LEDON) can optionally be connected to digital pin 2, 
    // or you can leave it disconnected and change the EMITTER_PIN #define below from 2 to 
    // QTR_NO_EMITTER_PIN.

    // The setup phase of this example calibrates the sensor for ten seconds and turns on
    // the LED built in to the Arduino on pin 13 while calibration is going on.
    // During this phase, you should expose each reflectance sensor to the lightest and 
    // darkest readings they will encounter.
    // For example, if you are making a line follower, you should slide the sensors across the
    // line during the calibration phase so that each sensor can get a reading of how dark the
    // line is and how light the ground is.  Improper calibration will result in poor readings.
    // If you want to skip the calibration phase, you can get the raw sensor readings
    // (pulse times from 0 to 2500 us) by calling qtrrc.read(sensorValues) instead of
    // qtrrc.readLine(sensorValues).

    // The main loop of the example reads the calibrated sensor values and uses them to
    // estimate the position of a line.  You can test this by taping a piece of 3/4" black
    // electrical tape to a piece of white paper and sliding the sensor across it.  It
    // prints the sensor values to the serial monitor as numbers from 0 (maximum reflectance) 
    // to 1000 (minimum reflectance) followed by the estimated location of the line as a number
    // from 0 to 5000.  1000 means the line is directly under sensor 1, 2000 means directly
    // under sensor 2, etc.  0 means the line is directly under sensor 0 or was last seen by
    // sensor 0 before being lost.  5000 means the line is directly under sensor 5 or was
    // last seen by sensor 5 before being lost.

#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low

        // sensors 0 through 7 are connected to digital pins 3 through 10, respectively
//        QTRSensors *qtrrc;
        uint16_t sensorValues[NUM_SENSORS];
        //Servo ST1,ST2;//ST1 left motor, ST2 right motor

        //unsigned int preCalibratedMin[] = {800, 588, 488, 532, 536, 536, 580, 812};
        uint16_t preCalibratedMin[8] = {750, 550, 500, 500, 450, 400, 400, 450};
        uint16_t preCalibratedMax[8] = {2500,2500,2500,2500,2500,2500,2500,2500};

        float out_pid_SetPoint,out_pid_Input,out_pid_Output;
        float out_consKp=1.0, out_consKi=0.0, out_consKd=0.0, out_pid_weight = 1.0;
        float inn_pid_SetPoint,inn_pid_Input,inn_pid_Output;
        float inn_consKp=2.0, inn_consKi=0.0, inn_consKd=0.0, inn_pid_weight = 1.0;
        PID *outerPID;
        PID *innerPID;

        unsigned int maxPower = 110;
        unsigned int stopPower = 90;//stand still
        unsigned int minPower = 20;
        unsigned int power_range = maxPower - minPower;

        void do_turn(int direction);

        uint16_t previous_position = 0;
        uint16_t position = 0;

        unsigned int previous_update = 0;

        std::shared_ptr<Drive> _driver = nullptr;

        const uint8_t debug = 1;
        int result_ready = -1;

        uint8_t startpos_count = 0;
        bool collected_startpos = false;

        const uint16_t ir_width = 65; //65mm, 65000um

        const float ir_modifier = ir_width/7.0; //0-7000 being the position value range, reduced to 7 because ir_width is mm

        const uint16_t ir_center = 3500;
        
        int16_t dist_center = 0;
        int16_t prev_dist_center = 0;
        int16_t delta_dist_center = 0;
        uint32_t diff= 0;

        bool enabled = true;

        int16_t sum_delta_dist = 0;

        uint8_t stopcount = 0;

        int leftright = 0;

        int16_t prev_delta[4] = {0,0,0,0};
        int16_t part_tmp[4] = {0,0,0,0};

        public:
            ~LineFollower();
            void update(unsigned int);
            void setup(std::shared_ptr<Drive> driver);
};
