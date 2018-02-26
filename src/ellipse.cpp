#include <fstream>

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include <sensor_msgs/TimeReference.h>

#include <sbgEComLib.h>
#include <sbgEComIds.h>

#include <sbg_driver/SolutionStatus.h>

using namespace std;

geometry_msgs::PoseStamped pose_msg;
geometry_msgs::Vector3Stamped pose_errors_msg;
bool new_pose_msg = false;
sensor_msgs::NavSatFix nav_msg;
geometry_msgs::Vector3Stamped nav_errors_msg;
bool new_nav_msg = false;
sensor_msgs::TimeReference time_ref;
bool new_timeref = false;
sbg_driver::SolutionStatus status_msg;
bool new_status = false;

class JsonGenerator {
public:
  JsonGenerator(void) {
  }

  void openFile(const string &filename) {
    file.open(filename.c_str());
  }

  void genDict(const sbg_driver::SolutionStatus &status, const SbgLogEkfQuatData &quat_data) {
    file << "{ \"type\":\"quat_data\", \"timeStamp\":" << quat_data.timeStamp << ", ";
    genDict(status);
    file << ", \"quaternion\":[" << quat_data.quaternion[1] << ", " << quat_data.quaternion[2] << ", " << quat_data.quaternion[3] << ", " << quat_data.quaternion[0] << "]";
    file << ", \"error\":[" << quat_data.eulerStdDev[0] << ", " << quat_data.eulerStdDev[1] << ", " << quat_data.eulerStdDev[2] << "] }";
    file << endl;
  }

  void genDict(const sbg_driver::SolutionStatus &status, const SbgLogEkfNavData &nav_data) {
    file << "{ \"type\":\"nav_data\", \"timeStamp\":" << nav_data.timeStamp << ", ";
    genDict(status);
    file << ", \"position\":[" << nav_data.position[0] << ", " << nav_data.position[1] << ", " << nav_data.position[2] << "]";
    file << ", \"error\":[" << nav_data.positionStdDev[0] << ", " << nav_data.positionStdDev[1] << ", " << nav_data.positionStdDev[2] << "] }";
    file << endl;
  }

  void genDict(const SbgLogUtcData &utc_data) {
    file << "{ \"type\":\"utc_data\", \"timeStamp\":" << utc_data.timeStamp << ", \"gpsTimeOfWeek\":" << utc_data.gpsTimeOfWeek * 1e-3 << " }";
    file << endl;
  }

protected:

  void genDict(const sbg_driver::SolutionStatus &status) {
    file << "\"status\": { \"solution_mode\":\"" << status_msg.solution_mode << "\""
        << ", \"attitude_valid\":" << (bool)status_msg.attitude_valid
        << ", \"heading_valid\":" << (bool)status_msg.heading_valid
        << ", \"velocity_valid\":" << (bool)status_msg.velocity_valid
        << ", \"position_valid\":" << (bool)status_msg.position_valid
        << ", \"vert_ref_used\":" << (bool)status_msg.vert_ref_used
        << ", \"mag_ref_used\":" << (bool)status_msg.mag_ref_used
        << ", \"gps1_vel_used\":" << (bool)status_msg.gps1_vel_used
        << ", \"gps1_pos_used\":" << (bool)status_msg.gps1_pos_used
        << ", \"gps1_course_used\":" << (bool)status_msg.gps1_course_used
        << ", \"sol_align_valid\":" << (bool)status_msg.sol_align_valid << " }";
  }

private:
  ofstream file;
};

void setStatus(const uint16 status, const ros::Time ros_now) {
  status_msg.stamp = ros_now;

  SbgEComSolutionMode mode = sbgEComLogEkfGetSolutionMode(status);
  switch(mode) {
  case SBG_ECOM_SOL_MODE_UNINITIALIZED:
    status_msg.solution_mode = "UNINITIALIZED";
    break;
  case SBG_ECOM_SOL_MODE_VERTICAL_GYRO:
    status_msg.solution_mode = "VERTICAL_GYRO";
    break;
  case SBG_ECOM_SOL_MODE_AHRS:
    status_msg.solution_mode = "AHRS";
    break;
  case SBG_ECOM_SOL_MODE_NAV_VELOCITY:
    status_msg.solution_mode = "NAV_VELOCITY";
    break;
  case SBG_ECOM_SOL_MODE_NAV_POSITION:
    status_msg.solution_mode = "NAV_POSITION";
    break;
  }

  // SBG_ECOM_SOL_ATTITUDE_VALID         (0x00000001u << 4)      /*!< Set to 1 if attitude data is reliable (Roll/Pitch error < 0,5deg). */
  status_msg.attitude_valid = (status & SBG_ECOM_SOL_ATTITUDE_VALID) != 0;

  // SBG_ECOM_SOL_HEADING_VALID          (0x00000001u << 5)      /*!< Set to 1 if geading data is reliable (Heading error < 1deg). */
  status_msg.heading_valid = (status & SBG_ECOM_SOL_HEADING_VALID) != 0;

  // SBG_ECOM_SOL_VELOCITY_VALID         (0x00000001u << 6)      /*!< Set to 1 if velocity data is reliable (velocity error < 1.5 m/s). */
  status_msg.velocity_valid = (status & SBG_ECOM_SOL_VELOCITY_VALID) != 0;

  // SBG_ECOM_SOL_POSITION_VALID         (0x00000001u << 7)      /*!< Set to 1 if position data is reliable (Position error < 10m). */
  status_msg.position_valid = (status & SBG_ECOM_SOL_POSITION_VALID) != 0;

  // SBG_ECOM_SOL_VERT_REF_USED          (0x00000001u << 8)      /*!< Set to 1 if vertical reference is used in solution (data used and valid since 3s). */
  status_msg.vert_ref_used = (status & SBG_ECOM_SOL_VERT_REF_USED) != 0;

  // SBG_ECOM_SOL_MAG_REF_USED           (0x00000001u << 9)      /*!< Set to 1 if magnetometer is used in solution (data used and valid since 3s). */
  status_msg.mag_ref_used = (status & SBG_ECOM_SOL_MAG_REF_USED) != 0;

  // SBG_ECOM_SOL_GPS1_VEL_USED          (0x00000001u << 10)     /*!< Set to 1 if GPS1 velocity is used in solution (data used and valid since 3s). */
  status_msg.gps1_vel_used = (status & SBG_ECOM_SOL_GPS1_VEL_USED) != 0;

  // SBG_ECOM_SOL_GPS1_POS_USED          (0x00000001u << 11)     /*!< Set to 1 if GPS1 Position is used in solution (data used and valid since 3s). */
  status_msg.gps1_pos_used = (status & SBG_ECOM_SOL_GPS1_POS_USED) != 0;

  // SBG_ECOM_SOL_GPS1_COURSE_USED       (0x00000001u << 12)     /*!< Set to 1 if GPS1 Course is used in solution (data used and valid since 3s). */
  status_msg.gps1_course_used = (status & SBG_ECOM_SOL_GPS1_COURSE_USED) != 0;

  // SBG_ECOM_SOL_ALIGN_VALID            (0x00000001u << 27)     /*!< Set to 1 if sensor alignment and calibration parameters are valid */
  status_msg.sol_align_valid = (status & SBG_ECOM_SOL_ALIGN_VALID) != 0;

  new_status = true;
}

/*!
 *  Callback definition called each time a new log is received.
 *  \param[in]  pHandle                 Valid handle on the sbgECom instance that has called this callback.
 *  \param[in]  msgClass                Class of the message we have received
 *  \param[in]  msg                   Message ID of the log received.
 *  \param[in]  pLogData                Contains the received log data as an union.
 *  \param[in]  pUserArg                Optional user supplied argument.
 *  \return                       SBG_NO_ERROR if the received log has been used successfully.
 */
SbgErrorCode onLogReceived(SbgEComHandle *pHandle, SbgEComClass msgClass, SbgEComMsgId msg, const SbgBinaryLogData *pLogData, void *pUserArg)
{
  ros::Time ros_now = ros::Time::now();
  JsonGenerator *generator = (JsonGenerator *)pUserArg;

  switch (msg){
    case SBG_ECOM_LOG_EKF_QUAT:
      pose_msg.pose.orientation.x = pLogData->ekfQuatData.quaternion[1];
      pose_msg.pose.orientation.y = pLogData->ekfQuatData.quaternion[2];
      pose_msg.pose.orientation.z = pLogData->ekfQuatData.quaternion[3];
      pose_msg.pose.orientation.w = pLogData->ekfQuatData.quaternion[0];
      pose_msg.header.stamp = ros_now;
      pose_errors_msg.vector.x = pLogData->ekfQuatData.eulerStdDev[0];
      pose_errors_msg.vector.y = pLogData->ekfQuatData.eulerStdDev[1];
      pose_errors_msg.vector.z = pLogData->ekfQuatData.eulerStdDev[2];
      pose_errors_msg.header.stamp = ros_now;
      new_pose_msg = true;
      setStatus(pLogData->ekfQuatData.status, ros_now);
      if(generator != NULL) {
        generator->genDict(status_msg, pLogData->ekfQuatData);
      }
      break;

    case SBG_ECOM_LOG_EKF_NAV:
      nav_msg.latitude  = pLogData->ekfNavData.position[0];
      nav_msg.longitude = pLogData->ekfNavData.position[1];
      nav_msg.altitude  = pLogData->ekfNavData.position[2];
      nav_msg.header.stamp = ros_now;
      nav_errors_msg.vector.x = pLogData->ekfNavData.positionStdDev[0];
      nav_errors_msg.vector.y = pLogData->ekfNavData.positionStdDev[1];
      nav_errors_msg.vector.z = pLogData->ekfNavData.positionStdDev[2];
      nav_errors_msg.header.stamp = ros_now;
      new_nav_msg = true;
      setStatus(pLogData->ekfNavData.status, ros_now);
      if(generator != NULL) {
        generator->genDict(status_msg, pLogData->ekfNavData);
      }
      break;

    case SBG_ECOM_LOG_UTC_TIME:
      time_ref.header.stamp = ros_now;
      time_ref.time_ref = ros::Time(pLogData->utcData.gpsTimeOfWeek * 1e-3);
      new_timeref = true;
      if(generator != NULL) {
        generator->genDict(pLogData->utcData);
      }
      break;

    default:
      break;
  }
  return SBG_NO_ERROR;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sbg_ellipse");

  ros::NodeHandle n;
  ros::Publisher gps_pub = n.advertise<sensor_msgs::NavSatFix>("imu_nav", 10);
  ros::Publisher gps_err_pub = n.advertise<geometry_msgs::Vector3Stamped>("imu_nav_errors", 10);
  ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("imu_pose", 10);
  ros::Publisher pose_err_pub = n.advertise<geometry_msgs::Vector3Stamped>("imu_pose_errors", 10);
  ros::Publisher timeref_pub = n.advertise<sensor_msgs::TimeReference>("imu_timeref", 10);
  ros::Publisher status_pub = n.advertise<sbg_driver::SolutionStatus>("imu_status", 10);

  std::string uart_port;
  int uart_baud_rate;
  string output_filename;

  n.param<string>("/sbg_ellipse/uart_port", uart_port, "/dev/ttyUSB0");
  n.param<int>("/sbg_ellipse/uart_baud_rate", uart_baud_rate, 115200);
  n.param<string>("/sbg_ellipse/output_file", output_filename, "");

    // ********************* Initialize the SBG  *********************
  SbgEComHandle       comHandle;
  SbgInterface        sbgInterface;
  SbgEComDeviceInfo   deviceInfo;
  SbgErrorCode        errorCode;

  errorCode = sbgInterfaceSerialCreate(&sbgInterface, uart_port.c_str(), uart_baud_rate);
  if (errorCode != SBG_NO_ERROR){ROS_WARN("sbgInterfaceSerialCreate Error");}

  errorCode = sbgEComInit(&comHandle, &sbgInterface); // Init the SBG
  if (errorCode != SBG_NO_ERROR){ROS_WARN("sbgEComInit Error");}

  errorCode = sbgEComCmdGetInfo(&comHandle, &deviceInfo); // Get device info
  if (errorCode != SBG_NO_ERROR){ROS_WARN("sbgEComCmdGetInfo Error");}

  ROS_INFO("CONNEXTION SET-UP");

  // ****************************** SBG Config ******************************
  // ToDo: improve configuration capabilities

  errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_QUAT, SBG_ECOM_OUTPUT_MODE_DIV_5);
  if (errorCode != SBG_NO_ERROR){ROS_WARN("sbgEComCmdOutputSetConf SBG_ECOM_LOG_EKF_QUAT Error");}

  errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_NAV, SBG_ECOM_OUTPUT_MODE_DIV_5);
  if (errorCode != SBG_NO_ERROR){ROS_WARN("sbgEComCmdOutputSetConf SBG_ECOM_LOG_EKF_NAV Error");}

  errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_UTC_TIME, SBG_ECOM_OUTPUT_MODE_DIV_5);
  if (errorCode != SBG_NO_ERROR){ROS_WARN("sbgEComCmdOutputSetConf SBG_ECOM_LOG_UTC_TIME Error");}

  // SAVE AND REBOOT
  errorCode = sbgEComCmdSettingsAction(&comHandle, SBG_ECOM_SAVE_SETTINGS);
  if (errorCode != SBG_NO_ERROR){ROS_WARN("sbgEComCmdSettingsAction Error");}

  ROS_INFO("CONFIGURATION DONE");

  // ************************** SBG Callback for data ************************
  JsonGenerator generator;
  bool test = false;
  if(output_filename.empty()) {
    sbgEComSetReceiveLogCallback(&comHandle, onLogReceived, NULL);
  } else {
    generator.openFile(output_filename);
    sbgEComSetReceiveLogCallback(&comHandle, onLogReceived, (void *)&generator);
    ROS_INFO_STREAM("OUTPUT_FILE: " << output_filename);
  }

  ROS_INFO("START RECEIVING DATA");

  nav_msg.header.frame_id = "imu_base";
  pose_msg.header.frame_id = "imu_base";

  tf::TransformBroadcaster tf_broadcaster;
  tf::StampedTransform tf_msg;
  tf_msg.frame_id_ = "imu_base";
  tf_msg.child_frame_id_ = "imu";

  ros::Rate loop_rate(80);
  while (ros::ok())
  {
    int errorCode = sbgEComHandle(&comHandle);

    if(new_nav_msg){
      gps_pub.publish(nav_msg);
      gps_err_pub.publish(nav_errors_msg);
      new_nav_msg = false;
    }

    if(new_timeref) {
      timeref_pub.publish(time_ref);
      new_timeref = false;
    }

    if(new_status) {
      status_pub.publish(status_msg);
      new_status = false;
    }

    if(new_pose_msg){
      pose_pub.publish(pose_msg);
      pose_err_pub.publish(pose_errors_msg);

      tf_msg.stamp_ = pose_msg.header.stamp;
      tf_msg.setOrigin(tf::Vector3(pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z));
      tf_msg.setRotation(tf::Quaternion(pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w));
      tf_broadcaster.sendTransform(tf_msg);

      new_pose_msg = false;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
