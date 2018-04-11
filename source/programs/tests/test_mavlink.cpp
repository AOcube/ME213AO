#include "configCosmos.h"
#include "device/serial/serialclass.h"

#include <iostream>
using namespace std;

#include <mavlink.h>

int main() {

  // Initialize the required buffers
  mavlink_message_t msg;
  mavlink_status_t status;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len;
  int bytes_sent;

  // Example variable, by declaring them static they're persistent
  // and will thus track the system state
  static int packet_drops = 0;
  static int mode = 0; /* Defined in mavlink_types.h, which is included by mavlink.h */

  cout << "Open serial port /dev/ttyO1, status: " ;
  Serial serial("/dev/ttyO1", 115200, 8, 0, 1);
  cout << serial.get_error() << endl;

  //  /*Send Heartbeat */
  //  mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_HELICOPTER, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
  //  len = mavlink_msg_to_send_buffer(buf, &msg);
  //  //    bytes_sent = sendto(sok.cudp, (const char *)buf, len, 0, (struct sockaddr*)&sok.caddr, sizeof(struct sockaddr_in));
  //  __mavlink_heartbeat_t te;
  //  cout << mavlink_msg_heartbeat_get_type(&msg) << endl;

  //  __mavlink_scaled_pressure2_t hello;
  //  cout << hello.temperature << endl;

  //  mavlink_highres_imu_t imu;
  //  mavlink_msg_highres_imu_decode(&msg, &imu);

  //  printf("Got message HIGHRES_IMU (spec: https://pixhawk.ethz.ch/mavlink/#HIGHRES_IMU)\n");
  //  printf("\t time: %llu\n", imu.time_usec);
  //  printf("\t acc  (NED):\t% f\t% f\t% f (m/s^2)\n", imu.xacc, imu.yacc, imu.zacc);
  //  printf("\t gyro (NED):\t% f\t% f\t% f (rad/s)\n", imu.xgyro, imu.ygyro, imu.zgyro);
  //  printf("\t mag  (NED):\t% f\t% f\t% f (Ga)\n", imu.xmag, imu.ymag, imu.zmag);
  //  printf("\t baro: \t %f (mBar)\n", imu.abs_pressure);
  //  printf("\t altitude: \t %f (m)\n", imu.pressure_alt);
  //  printf("\t temperature: \t %f C\n", imu.temperature);
  //  printf("\n");

  mavlink_system_t mavlink_system;

  mavlink_system.sysid = 20;                   ///< ID 20 for this airplane
  mavlink_system.compid = MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process

  // Define the system type, in this case an airplane
  uint8_t system_type = MAV_TYPE_FIXED_WING;
  uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;

  uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
  uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
  uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight



  // Pack the message
  mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid, &msg, system_type, autopilot_type, system_mode, custom_mode, system_state);

  // Copy the message to the send buffer
  len = mavlink_msg_to_send_buffer(buf, &msg);

  int send_status = serial.SendBuffer(buf, len);

  cout << "send status " << send_status << endl;

  //serial.put_string("tone_alarm stop\n");

  // receive


  // COMMUNICATION THROUGH EXTERNAL UART PORT (XBee serial)

  while(1)
    {

      uint8_t c = serial.get_char();
      //      cout << c << endl;
      // Try to get a new message
      if(mavlink_parse_char(MAVLINK_COMM_1, c, &msg, &status)) {
          // Handle message
          //          printf("Received packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d", msg.sysid, msg.compid, msg.len, msg.msgid);
          //          cout << endl;

          switch(msg.msgid)
            {
            case MAVLINK_MSG_ID_HEARTBEAT:
              {
                // E.g. read GCS heartbeat and go into
                // comm lost mode if timer times out
              }
              break;
            case MAVLINK_MSG_ID_COMMAND_LONG:
              // EXECUTE ACTION
              break;

            case MAVLINK_MSG_ID_ATTITUDE:
              //                cout << "attitude " << endl;

              mavlink_attitude_t attitude;
              mavlink_msg_attitude_decode(&msg, &attitude);
              cout << "[roll pitch]: " <<
                      attitude.roll*180/3.1415 << " " <<
                      attitude.pitch *180/3.1415 << endl;


              break;

            case MAVLINK_MSG_ID_HIGHRES_IMU:
              mavlink_highres_imu_t imu;
              mavlink_msg_highres_imu_decode(&msg, &imu);

              printf("Got message HIGHRES_IMU");
              printf("\t time: %llu\n", imu.time_usec);
              printf("\t acc  (NED):\t% f\t% f\t% f (m/s^2)\n", imu.xacc, imu.yacc, imu.zacc);
              printf("\t gyro (NED):\t% f\t% f\t% f (rad/s)\n", imu.xgyro, imu.ygyro, imu.zgyro);
              printf("\t mag  (NED):\t% f\t% f\t% f (Ga)\n", imu.xmag, imu.ymag, imu.zmag);
              printf("\t baro: \t %f (mBar)\n", imu.abs_pressure);
              printf("\t altitude: \t %f (m)\n", imu.pressure_alt);
              printf("\t temperature: \t %f C\n", imu.temperature);
              printf("\n");
              cout << endl;
              break;

            default:
              //Do nothing
              break;
            }
        }

      // And get the next one
      //      COSMOS_SLEEP(0.001);
    }

  // Update global packet drops counter
  packet_drops += status.packet_rx_drop_count;



  return 0;
}
