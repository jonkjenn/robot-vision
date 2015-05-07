#include "gyroscope.hpp"

using namespace std;
using namespace serial;

gyroscope::gyroscope(const string &device, uint32_t speed)
{
    auto deleter = [&](Serial* s){s->close();delete(s);};
    mSerial = unique_ptr<Serial,decltype(deleter)>(new Serial(device,speed,Timeout::simpleTimeout(1)),deleter);
}

void gyroscope::start(float degrees)
{
    //lock_guard<mutex> lock(gyromutex);
    goal_rotation = degrees * DEG_RAD_RATIO;
    total_rotation = 0;
    distance_rotation = abs(goal_rotation)*RAD_DEG_RATIO -  abs(total_rotation)*RAD_DEG_RATIO;
}

//
// Example variable, by declaring them static they're persistent
// and will thus track the system state
//static int mode = MAV_MODE_UNINIT; /* Defined in mavlink_types.h, which is included by mavlink.h */

/**
 * @brief Receive communication packets and handle them
 *
 * This function decodes packets on the protocol level and also handles
 * their value by calling the appropriate functions.
 */
uint64_t g_t = 0;
uint64_t sum = 0;
void gyroscope::update()
{
    mavlink_message_t msg;
    mavlink_status_t status;

    // COMMUNICATION THROUGH EXTERNAL UART PORT (XBee serial)
    //
    g_t = nanos();
    //int count = mSerial->available();
    int read = mSerial->read(gyro_temp,256);
    //cout << "105 :   " << nanos() - g_t <<endl;
    int i=0;
    while(i<read)
    {
        // Try to get a new message
        if(mavlink_parse_char(MAVLINK_COMM_0, gyro_temp[i], &msg, &status)) {
            // Handle message
            //cout << "GYRO msg: " << (int)msg.msgid << endl;

            switch(msg.msgid)
            {
                case 105:

                    mavlink_highres_imu_t sensors;
                    mavlink_msg_highres_imu_decode(&msg, &sensors);

                    if(prevtime == 0){prevtime = sensors.time_usec;break;}
                    if(abs(sensors.zgyro) < 0.005){
                        //cout << "105 :   " << nanos() - g_t <<endl;
                        return;
                    }

                    gyro_dur = sensors.time_usec - prevtime;

                    if(((sensors.fields_updated & 0x20) >> 5) == 1)
                    {
                        //lock_guard<mutex> lock(gyromutex);
                        current_rotation = sensors.zgyro;
                        //LOG(DEBUG) << "Current rot: " << current_rotation << endl;;
                        /*LOG(DEBUG) << "Rotation: " << sensors.zgyro * gyro_dur * 1e-6 * RAD_DEG_RATIO;
                        LOG(DEBUG) << "Total rotation: " << total_rotation * RAD_DEG_RATIO;
                        LOG(DEBUG) << "Duration: " << gyro_dur;
                        LOG(DEBUG) << "time_usec: " << sensors.time_usec;
                        LOG(DEBUG) << "prevtime: " << prevtime;
                        LOG(DEBUG) << "Rotation speed: " << sensors.zgyro;
                        LOG(DEBUG) << "Goal " << goal_rotation;*/

                        total_rotation += sensors.zgyro * gyro_dur * 1e-6;
                        //total_rotation += sensors.zgyro * gyro_dur * 1e-6;
                        distance_rotation = abs(goal_rotation)*RAD_DEG_RATIO -  abs(total_rotation)*RAD_DEG_RATIO;
                        prevtime = sensors.time_usec;
                        return;
                    }

                    prevtime = sensors.time_usec;


                    break;

                    /*case MAVLINK_MSG_ID_SET_MODE:
                      {
                    //mode = mavlink_msg_set_mode_get_mode(&msg);
                    }
                    break;
                    case MAVLINK_MSG_ID_ACTION:
                    Serial.println("Got mavlink");
                    Serial.println(MAVLINK_MSG_ID_ACTION);
                    break;
                    default:
                    //Do nothing
                    break;*/
            }
        }

        // And get the next one
        i++;
    }

    // Update global packet drops counter
    packet_drops += status.packet_rx_drop_count;
}

float gyroscope::get_current_rotation()
{
    //lock_guard<mutex> lock(gyromutex);
    return current_rotation;
}

float gyroscope::get_total_rotation()
{
    //lock_guard<mutex> lock(gyromutex);
    return total_rotation;
}

float gyroscope::get_distance_rotation()
{
    //lock_guard<mutex> lock(gyromutex);
    return distance_rotation;
}

gyroscope::~gyroscope()
{
    mSerial->close();
    mSerial = nullptr;
}
