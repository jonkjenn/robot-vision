#include "gyroscope.hpp"

using namespace std;
using namespace serial;

gyroscope::gyroscope(const string &device, uint32_t speed)
{
    mSerial = unique_ptr<Serial>(new Serial(device,speed,Timeout::simpleTimeout(100)));
}

void gyroscope::start(float degrees)
{
    goal_rotation = degrees * DEG_RAD_RATIO;
    total_rotation = 0;
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

uint8_t gyro_temp;
uint32_t gyro_dur;
void gyroscope::update()
{
    mavlink_message_t msg;
    mavlink_status_t status;

    // COMMUNICATION THROUGH EXTERNAL UART PORT (XBee serial)
    //
    int count = 0;
    while(mSerial->available())
    {
        count++;
        mSerial->read(&gyro_temp,1);
        // Try to get a new message
        if(mavlink_parse_char(MAVLINK_COMM_0, gyro_temp, &msg, &status)) {
            // Handle message
            LOG(DEBUG) << "GYRO msg: " << msg.msgid;

            switch(msg.msgid)
            {
                case 105:
                    mavlink_highres_imu_t sensors;
                    mavlink_msg_highres_imu_decode(&msg, &sensors);

                    if(prevtime == 0){prevtime = sensors.time_usec;continue;}
                    if(abs(sensors.zgyro) < 0.005){
                        return;
                    }

                    gyro_dur = sensors.time_usec - prevtime;

                    if(((sensors.fields_updated & 0x20) >> 5) == 1)
                    {
                        LOG(DEBUG) << "Rotation: " << sensors.zgyro * gyro_dur * 1e-6 * RAD_DEG_RATIO;
                        LOG(DEBUG) << "Total rotation: " << total_rotation * RAD_DEG_RATIO;
                        total_rotation += sensors.zgyro * gyro_dur * 1e-6;
                        distance_rotation = abs(goal_rotation) -  abs(total_rotation);
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
    }

    // Update global packet drops counter
    packet_drops += status.packet_rx_drop_count;
}
