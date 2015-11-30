/**
 *
 *  \file
 *  \brief      Main entry point for GP9 driver. Handles serial connection
 *              details, as well as all ROS message stuffing, parameters,
 *              topics, etc.
 *  \author     Mike Purvis <mpurvis@clearpathrobotics.com> (original code for UM6)
 *  \copyright  Copyright (c) 2013, Clearpath Robotics, Inc.
 *  \author     Alex Brown <rbirac@cox.net>		    (adapted to UM7)
 *  \copyright  Copyright (c) 2015, Alex Brown.
 *  \author     Damian Manda <damian.manda@noaa.gov>  (adapted to GP9)
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. OR ALEX BROWN BE LIABLE 
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * Please send comments, questions, or patches to Alex Brown  rbirac@cox.net
 *
 */

#include "geometry_msgs/Vector3Stamped.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/NavSatFix.h"
#include "serial/serial.h"            // must install serial library from apt-get
#include "std_msgs/Float32.h"
#include "std_msgs/Header.h"
#include "gp9/comms.h"
#include "gp9/registers.h"
#include "gp9/Reset.h"
#include <string>
#include <math.h>

float covar[9];     // orientation covariance values
const char VERSION[10] = "0.0.3";   // gp9_driver version

// Don't try to be too clever. Arrival of this message triggers
// us to publish everything we have.
const uint8_t TRIGGER_PACKET = DREG_QUAT_AB;

/**
 * Function generalizes the process of writing an XYZ vector into consecutive
 * fields in GP9 registers.
 */
template<typename RegT>
void configureVector3(gp9::Comms* sensor, const gp9::Accessor<RegT>& reg,
    std::string param, std::string human_name)
{
  if (reg.length != 3)
  {
    throw std::logic_error("configureVector3 may only be used with 3-field registers!");
  }

  if (ros::param::has(param))
  {
    double x, y, z;
    ros::param::get(param + "/x", x);
    ros::param::get(param + "/y", y);
    ros::param::get(param + "/z", z);
    ROS_INFO_STREAM("Configuring " << human_name << " to ("
                    << x << ", " << y << ", " << z << ")");
    reg.set_scaled(0, x);
    reg.set_scaled(1, y);
    reg.set_scaled(2, z);
    if (sensor->sendWaitAck(reg))
    {
      throw std::runtime_error("Unable to configure vector.");
    }
  }
}

/**
 * Function generalizes the process of commanding the GP9 via one of its command
 * registers.
 */
template<typename RegT>
void sendCommand(gp9::Comms* sensor, const gp9::Accessor<RegT>& reg, std::string human_name)
{
  ROS_INFO_STREAM("Sending command: " << human_name);
  if (!sensor->sendWaitAck(reg))
  {
    throw std::runtime_error("Command to device failed.");
  }
}


/**
 * Send configuration messages to the GP9, critically, to turn on the value outputs
 * which we require, and inject necessary configuration parameters.
 */
void configureSensor(gp9::Comms* sensor)
{
  gp9::Registers r;

  uint32_t comm_reg = (BAUD_115200 << COM_BAUD_START) | COM_GPS_DATA_ENABLED;
  r.communication.set(0, comm_reg);
  if (!sensor->sendWaitAck(r.communication))
  {
    throw std::runtime_error("Unable to set CREG_COM_SETTINGS.");
  }

  bool output_raw;
  ros::param::param<bool>("~output_raw", output_raw, false);
  uint32_t raw1_rate;
  if (output_raw) {
    raw1_rate = (10 << RATE1_RAW_ACCEL_START)| (10 << RATE1_RAW_GYRO_START); 
  } else {
    raw1_rate = 0;
  }
  r.comrate1.set(0, raw1_rate);
  if (!sensor->sendWaitAck(r.comrate1))
  {
    throw std::runtime_error("Unable to set CREG_COM_RATES1.");
  }

  uint32_t raw_rate = (0 << RATE2_ALL_RAW_START);
  r.comrate2.set(0, raw_rate);
  if (!sensor->sendWaitAck(r.comrate2))
  {
    throw std::runtime_error("Unable to set CREG_COM_RATES2.");
  }


  uint32_t proc_rate_indiv = (10 << RATE3_PROC_ACCEL_START) | (10 << RATE3_PROC_GYRO_START) 
	| (2 << RATE3_PROC_MAG_START) | (0 << RATE3_PROC_PRESS_START);
  r.comrate3.set(0, proc_rate_indiv);
  if (!sensor->sendWaitAck(r.comrate3))
  {
    throw std::runtime_error("Unable to set CREG_COM_RATES3.");
  }

  uint32_t proc_rate = (0 << RATE4_ALL_PROC_START) | (1 << RATE4_PROC_TEMP_START);
  r.comrate4.set(0, proc_rate);
  if (!sensor->sendWaitAck(r.comrate4))
  {
    throw std::runtime_error("Unable to set CREG_COM_RATES4.");
  }

  uint32_t misc_rate = (2 << RATE5_EULER_START) | (10 << RATE5_POSITION_START)
           | (0 << RATE5_VELOCITY_START) | (10 << RATE5_QUAT_START);
  r.comrate5.set(0, misc_rate);
  if (!sensor->sendWaitAck(r.comrate5))
  {
    throw std::runtime_error("Unable to set CREG_COM_RATES5.");
  }

  uint32_t health_rate = (0 << RATE6_POSE_START) | (5 << RATE6_HEALTH_START)
	| (10 << RATE6_VARIANCE_START);  // note:  5 gives 2 hz rate for health
  r.comrate6.set(0, health_rate);
  if (!sensor->sendWaitAck(r.comrate6))
  {
    throw std::runtime_error("Unable to set CREG_COM_RATES6.");
  }

  // r.home_north.set(0, (float)m_dLatOrigin);
  // if (!sensor->sendWaitAck(r.home_north))
  // {
  //   throw std::runtime_error("Unable to set CREG_HOME_NORTH.");
  // }

  // r.home_east.set(0, (float)m_dLonOrigin);
  // if (!sensor->sendWaitAck(r.home_east))
  // {
  //   throw std::runtime_error("Unable to set CREG_HOME_EAST.");
  // }


  // Options available using parameters)
  uint32_t filter_config_reg = 0;  // initialize all options off

  // Optionally disable mag updates in the sensor's EKF.
  bool mag_updates;
  ros::param::param<bool>("~mag_updates", mag_updates, true);
  if (mag_updates)
  {
    filter_config_reg |= MAG_UPDATES_ENABLED;
  }
  else
  {
    ROS_WARN("Excluding magnetometer updates from EKF.");
  }

  // Optionally disable accelerometer updates in the sensor's EKF.
  bool acc_updates;
  ros::param::param<bool>("~acc_updates", acc_updates, true);
  if (acc_updates)
  {
    filter_config_reg |= ACC_UPDATES_ENABLED;
  }
  else
  {
    ROS_WARN("Excluding accelerometer updates from EKF.");
  }

  // Optionally disable gps updates in the sensor's EKF.
  bool gps_updates;
  ros::param::param<bool>("~gps_updates", gps_updates, true);
  if (gps_updates)
  {
    filter_config_reg |= GPS_UPDATES_ENABLED;
  }
  else
  {
    ROS_WARN("Excluding GPS updates from EKF.");
  }

  r.filter_config.set(0, filter_config_reg);
  if (!sensor->sendWaitAck(r.filter_config))
  {
    throw std::runtime_error("Unable to set CREG_FILTER_SETTINGS.");
  }

  // Optionally disable performing a zero gyros command on driver startup.
  bool zero_gyros;
  ros::param::param<bool>("~zero_gyros", zero_gyros, true);
  if (zero_gyros) sendCommand(sensor, r.cmd_zero_gyros, "zero gyroscopes");
}


bool handleResetService(gp9::Comms* sensor,
    const gp9::Reset::Request& req, const gp9::Reset::Response& resp)
{
  gp9::Registers r;
  if (req.zero_gyros) sendCommand(sensor, r.cmd_zero_gyros, "zero gyroscopes");
  if (req.reset_ekf) sendCommand(sensor, r.cmd_reset_ekf, "reset EKF");
  if (req.set_home_pos) sendCommand(sensor, r.cmd_set_home_pos, "set home position");
  //if (req.set_mag_ref) sendCommand(sensor, r.cmd_set_mag_ref, "set magnetometer reference");
  return true;
}

/**
 * Uses the register accessors to grab data from the IMU, and populate
 * the ROS messages which are output.
 */
void publishMsgs(gp9::Registers& r, ros::NodeHandle* n, const std_msgs::Header& header)
{
  static ros::Publisher imu_pub = n->advertise<sensor_msgs::Imu>("imu/data", 1, false);
  static ros::Publisher imu_raw_pub = n->advertise<sensor_msgs::Imu>("imu/raw", 1, false);
  static ros::Publisher mag_pub = n->advertise<geometry_msgs::Vector3Stamped>("imu/mag", 1, false);
  static ros::Publisher mag_pub_2 = n->advertise<sensor_msgs::MagneticField>("imu/mag_msg", 1, false);
  static ros::Publisher rpy_pub = n->advertise<geometry_msgs::Vector3Stamped>("imu/rpy", 1, false);
  static ros::Publisher temp_pub = n->advertise<std_msgs::Float32>("imu/temperature", 1, false);
  static ros::Publisher temp2_pub = n->advertise<std_msgs::Float32>("imu/temperature2", 1, false);
  static ros::Publisher pos_pub = n->advertise<sensor_msgs::NavSatFix>("gps/fix", 1, false);

  if (imu_pub.getNumSubscribers() > 0)
  {
    sensor_msgs::Imu imu_msg;
    imu_msg.header = header;

    // IMU outputs [w,x,y,z], convert to [x,y,z,w] & transform to ROS axes
    imu_msg.orientation.x = r.quat.get_scaled(2);
    imu_msg.orientation.y = r.quat.get_scaled(1);
    imu_msg.orientation.z = -r.quat.get_scaled(3);
    imu_msg.orientation.w = r.quat.get_scaled(0);

    // Covariance of attitude.  set to constant default or parameter values
    imu_msg.orientation_covariance[0] = r.quat_var.get_scaled(2);
    imu_msg.orientation_covariance[1] = 0;
    imu_msg.orientation_covariance[2] = 0;
    imu_msg.orientation_covariance[3] = 0;
    imu_msg.orientation_covariance[4] = r.quat_var.get_scaled(1);
    imu_msg.orientation_covariance[5] = 0;
    imu_msg.orientation_covariance[6] = 0;
    imu_msg.orientation_covariance[7] = 0;
    imu_msg.orientation_covariance[8] = r.quat_var.get_scaled(3);

    // Angular velocity.  transform to ROS axes
    imu_msg.angular_velocity.x = r.gyro.get_scaled(1);
    imu_msg.angular_velocity.y = r.gyro.get_scaled(0);
    imu_msg.angular_velocity.z = -r.gyro.get_scaled(2);

    //These should be possible to figure out from specs
    //0.0000008 from testing
    imu_msg.angular_velocity_covariance[0] = 0.002;
    imu_msg.angular_velocity_covariance[1] = 0;
    imu_msg.angular_velocity_covariance[2] = 0;
    imu_msg.angular_velocity_covariance[3] = 0;
    imu_msg.angular_velocity_covariance[4] = 0.002;
    imu_msg.angular_velocity_covariance[5] = 0;
    imu_msg.angular_velocity_covariance[6] = 0;
    imu_msg.angular_velocity_covariance[7] = 0;
    imu_msg.angular_velocity_covariance[8] = 0.002;

    // Linear accel.  transform to ROS axes
    imu_msg.linear_acceleration.x =  r.accel.get_scaled(1);
    imu_msg.linear_acceleration.y =  r.accel.get_scaled(0);
    imu_msg.linear_acceleration.z = -r.accel.get_scaled(2);

    //0.00005 from testing
    imu_msg.linear_acceleration_covariance[0] = 0.002;
    imu_msg.linear_acceleration_covariance[1] = 0;
    imu_msg.linear_acceleration_covariance[2] = 0;
    imu_msg.linear_acceleration_covariance[3] = 0;
    imu_msg.linear_acceleration_covariance[4] = 0.002;
    imu_msg.linear_acceleration_covariance[5] = 0;
    imu_msg.linear_acceleration_covariance[6] = 0;
    imu_msg.linear_acceleration_covariance[7] = 0;
    imu_msg.linear_acceleration_covariance[8] = 0.002;

    imu_pub.publish(imu_msg);
  }

  if (imu_raw_pub.getNumSubscribers() > 0)
  {
    sensor_msgs::Imu imu_msg;
    imu_msg.header = header;

    // IMU outputs [w,x,y,z], convert to [x,y,z,w] & transform to ROS axes
    imu_msg.orientation.x = r.quat.get_scaled(2);
    imu_msg.orientation.y = r.quat.get_scaled(1);
    imu_msg.orientation.z = -r.quat.get_scaled(3);
    imu_msg.orientation.w = r.quat.get_scaled(0);

    // Covariance of attitude.  set to constant default or parameter values
    imu_msg.orientation_covariance[0] = r.quat_var.get_scaled(2);
    imu_msg.orientation_covariance[1] = 0;
    imu_msg.orientation_covariance[2] = 0;
    imu_msg.orientation_covariance[3] = 0;
    imu_msg.orientation_covariance[4] = r.quat_var.get_scaled(1);
    imu_msg.orientation_covariance[5] = 0;
    imu_msg.orientation_covariance[6] = 0;
    imu_msg.orientation_covariance[7] = 0;
    imu_msg.orientation_covariance[8] = r.quat_var.get_scaled(3);

    // Angular velocity.  transform to ROS axes
    imu_msg.angular_velocity.x = r.gyro_raw.get_scaled(1);
    imu_msg.angular_velocity.y = r.gyro_raw.get_scaled(0);
    imu_msg.angular_velocity.z = -r.gyro_raw.get_scaled(2);

    //These should be possible to figure out from specs
    imu_msg.angular_velocity_covariance[0] = 0.0000008;
    imu_msg.angular_velocity_covariance[1] = 0;
    imu_msg.angular_velocity_covariance[2] = 0;
    imu_msg.angular_velocity_covariance[3] = 0;
    imu_msg.angular_velocity_covariance[4] = 0.0000008;
    imu_msg.angular_velocity_covariance[5] = 0;
    imu_msg.angular_velocity_covariance[6] = 0;
    imu_msg.angular_velocity_covariance[7] = 0;
    imu_msg.angular_velocity_covariance[8] = 0.0000008;

    // Linear accel.  transform to ROS axes
    imu_msg.linear_acceleration.x =  r.accel_raw.get_scaled(1);
    imu_msg.linear_acceleration.y =  r.accel_raw.get_scaled(0);
    imu_msg.linear_acceleration.z = -r.accel_raw.get_scaled(2);

    imu_msg.linear_acceleration_covariance[0] = 0.00005;
    imu_msg.linear_acceleration_covariance[1] = 0;
    imu_msg.linear_acceleration_covariance[2] = 0;
    imu_msg.linear_acceleration_covariance[3] = 0;
    imu_msg.linear_acceleration_covariance[4] = 0.00005;
    imu_msg.linear_acceleration_covariance[5] = 0;
    imu_msg.linear_acceleration_covariance[6] = 0;
    imu_msg.linear_acceleration_covariance[7] = 0;
    imu_msg.linear_acceleration_covariance[8] = 0.00005;

    imu_raw_pub.publish(imu_msg);
  }

  // Magnetometer.  transform to ROS axes
  if (mag_pub.getNumSubscribers() > 0)
  {
    geometry_msgs::Vector3Stamped mag_msg;
    mag_msg.header = header;
    mag_msg.vector.x =  r.mag.get_scaled(1);
    mag_msg.vector.y =  r.mag.get_scaled(0);
    mag_msg.vector.z = -r.mag.get_scaled(2);
    mag_pub.publish(mag_msg);
  }

  if (mag_pub_2.getNumSubscribers() > 0)
  {
    sensor_msgs::MagneticField mag_msg;
    mag_msg.header = header;
    mag_msg.magnetic_field.x =  r.mag.get_scaled(0);
    mag_msg.magnetic_field.y = -r.mag.get_scaled(1);
    mag_msg.magnetic_field.z = -r.mag.get_scaled(2);
    mag_pub_2.publish(mag_msg);
  }

  // Euler attitudes.  transform to ROS axes
  if (rpy_pub.getNumSubscribers() > 0)
  {
    geometry_msgs::Vector3Stamped rpy_msg;
    rpy_msg.header = header;
    rpy_msg.vector.x =  r.euler.get_scaled(1);
    rpy_msg.vector.y =  r.euler.get_scaled(0);
    rpy_msg.vector.z = -r.euler.get_scaled(2);
    rpy_pub.publish(rpy_msg);
  }

  // Temperature
  if (temp_pub.getNumSubscribers() > 0)
  {
    std_msgs::Float32 temp_msg;
    temp_msg.data = r.temperature1.get_scaled(0);
    temp_pub.publish(temp_msg);
  }

  // Temperature2
  if (temp2_pub.getNumSubscribers() > 0)
  {
    std_msgs::Float32 temp2_msg;
    temp2_msg.data = r.temperature2.get_scaled(0);
    temp2_pub.publish(temp2_msg);
  }

  if (pos_pub.getNumSubscribers() > 0)
  {
    sensor_msgs::NavSatFix fix_msg;
    sensor_msgs::NavSatStatus status_msg;

    std_msgs::Header map_header;
    map_header.frame_id = "map";
    map_header.stamp = ros::Time::now();
    fix_msg.header = map_header;

    //Comonents of the status message
    uint32_t health_reg = r.health.get(0);
    //ROS_INFO("Health Register: %i", health_reg);
    uint8_t gps_status = (health_reg >> HEALTH_GPS_ST_START) & HEALTH_GPS_ST_MASK;
    //ROS_INFO("GPS_STATUS = %i", gps_status);
    if (gps_status == 2) {
      status_msg.status = status_msg.STATUS_FIX;
    } else if (gps_status == 3) {
      status_msg.status = status_msg.STATUS_SBAS_FIX;
    } else {
      status_msg.status = status_msg.STATUS_NO_FIX;
    }
    status_msg.service = status_msg.SERVICE_GPS;

    //the fix message
    fix_msg.status = status_msg;
    fix_msg.latitude = r.latitude.get_scaled(0);
    fix_msg.longitude = r.longitude.get_scaled(0);
    fix_msg.altitude = r.gps_altitude.get_scaled(0);

    //compute an approximate covariance
    float HDOP = (float)((health_reg >> HEALTH_HDOP_START) & HEALTH_HDOP_MASK);
    HDOP = HDOP / 10;
    //ROS_INFO("HDOP = %f", HDOP);
    float hvar = pow(HDOP * 3, 2);
    fix_msg.position_covariance[0] = hvar;
    fix_msg.position_covariance[1] = 0;
    fix_msg.position_covariance[2] = 0;
    fix_msg.position_covariance[3] = 0;
    fix_msg.position_covariance[4] = hvar;
    fix_msg.position_covariance[5] = 0;
    fix_msg.position_covariance[6] = 0;
    fix_msg.position_covariance[6] = 0;
    fix_msg.position_covariance[8] = 25;	//estimate

    fix_msg.position_covariance_type = fix_msg.COVARIANCE_TYPE_APPROXIMATED; 

    pos_pub.publish(fix_msg);
  }
}


/**
 * Node entry-point. Handles ROS setup, and serial port connection/reconnection.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "gp9_driver");

  // Load parameters from private node handle.
  std::string port;
  int32_t baud;
  ros::param::param<std::string>("~port", port, "/dev/ttyUSB0");
  ros::param::param<int32_t>("~baud", baud, 115200);

  serial::Serial ser;
  ser.setPort(port);
  ser.setBaudrate(baud);
  serial::Timeout to = serial::Timeout(50, 50, 0, 50, 0);
  ser.setTimeout(to);

#ifdef DEBUGG
  std::cout<<"The baud rate for serial communication is: "<< baud<< "\n";
  std::cout<<"The port for serial communication is: "<< port<< "\n";
#endif

  ros::NodeHandle n;

  std_msgs::Header header;
  ros::param::param<std::string>("~frame_id", header.frame_id, "imu_link");

  // Initialize covariance. The GP9 sensor does not provide covariance values so,
  //   by default, this driver provides a covariance array of all zeros indicating
  //   "covariance unknown" as advised in sensor_msgs/Imu.h.
  // This param allows the user to specify alternate covariance values if needed.

  std::string covariance;
  char cov[200];
  char *ptr1;

  ros::param::param<std::string>("~covariance", covariance, "0 0 0 0 0 0 0 0 0");
  snprintf(cov, sizeof(cov), "%s", covariance.c_str());

  char* p = strtok_r(cov, " ", &ptr1);           // point to first value
  for (int iter = 0; iter < 9; iter++)
  {
    if (p) covar[iter] = atof(p);                // covar[] is global var
    else  covar[iter] = 0.0;
    p = strtok_r(NULL, " ", &ptr1);              // point to next value (nil if none)
  }

  // Real Time Loop
  bool first_failure = true;
  while (ros::ok())
  {
    try
    {
      ser.open();
      ROS_DEBUG("Serial open no exception thorwn");
    }
    catch(const serial::IOException& e)
    {
        ROS_DEBUG("gp9_driver ver %s unable to connect to port.", VERSION);
    }
    if (ser.isOpen())
    {
      ROS_INFO("gp9_driver ver %s connected to serial port.", VERSION);
      first_failure = true;
      try
      {
        //Set DTR (pin 4) for driving voltage level converter
        //ser.setDTR(true);
        gp9::Comms sensor(&ser);
        configureSensor(&sensor);
        gp9::Registers registers;
        ros::ServiceServer srv = n.advertiseService<gp9::Reset::Request, gp9::Reset::Response>(
            "reset", boost::bind(handleResetService, &sensor, _1, _2));

        while (ros::ok())
        {
          // triggered by arrival of last message packet
          if (sensor.receive(&registers) == TRIGGER_PACKET)
          {
            header.stamp = ros::Time::now();
            publishMsgs(registers, &n, header);
            ros::spinOnce();
          }
        }
      }
      catch(const std::exception& e)
      {
        if (ser.isOpen()) ser.close();
        ROS_ERROR_STREAM(e.what());
        ROS_INFO("Attempting reconnection after error.");
        ros::Duration(1.0).sleep();
      }
    }
    else
    {
      ROS_WARN_STREAM_COND(first_failure, "Could not connect to serial device "
          << port << ". Trying again every 1 second.");
      first_failure = false;
      ros::Duration(1.0).sleep();
    }
  }
}
