#include <fstream>
#include <cmath>

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include <sensor_msgs/TimeReference.h>
#include <image_transport/image_transport.h>
#include <eigen3/Eigen/Eigen>

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

bool should_discard_heading;

class JsonGenerator {
public:
  JsonGenerator(void) {
  }

  void openFile(const string &filename) {
    file.open(filename.c_str());
  }

  void genDict(const sbg_driver::SolutionStatus &status, const uint32 timeStamp,
      const geometry_msgs::Quaternion &quaternion, const geometry_msgs::Vector3 eulerStdDev) {
    file << "{ \"type\":\"quat_data\", \"timeStamp\":" << timeStamp << ", ";
    genDict(status);
    file << ", \"quaternion\":[" << quaternion.x << ", " << quaternion.y << ", " << quaternion.z << ", " << quaternion.w << "]";
    file << ", \"error\":[" << eulerStdDev.x << ", " << eulerStdDev.y << ", " << eulerStdDev.z << "] }";
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

void discard_heading(float *w, float *x, float *y, float *z) {
  Eigen::Matrix3f R = Eigen::Quaternionf(*w, *x, *y, *z).matrix();
  float angle = acos(R(2,2));                   // dot product [0,0,1] and R*[0,0,1]
  Eigen::Vector3f axis(-R(1,2), R(0,2), 0);     // cross product [0,0,1] and R*[0,0,1]
  axis.normalize();
  Eigen::AngleAxisf aa_noheading(angle, axis);
  Eigen::Quaternionf q_noheading(aa_noheading);
  *w = q_noheading.w();
  *x = q_noheading.x();
  *y = q_noheading.y();
  *z = q_noheading.z();
}

void rp_to_quaternion(const float roll_angle, const float pitch_angle, geometry_msgs::Quaternion &orientation) {
  Eigen::Matrix3f roll, pitch;
  float sr = sin(roll_angle);
  float cr = cos(roll_angle);
  roll << 1.0, 0.0, 0.0,
          0.0,  cr, -sr,
          0.0,  sr,  cr;
  float sp = sin(pitch_angle);
  float cp = cos(pitch_angle);
  pitch <<  cp, 0.0,  sp,
           0.0, 1.0, 0.0,
           -sp, 0.0,  cp;
  Eigen::Quaternionf quaternion(pitch * roll);
  orientation.x = quaternion.x();
  orientation.y = quaternion.y();
  orientation.z = quaternion.z();
  orientation.w = quaternion.w();
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
    /*case SBG_ECOM_LOG_EKF_QUAT:
      if(discard_heading) {
        discard_heading(pLogData->ekfQuatData.quaternion,
            pLogData->ekfQuatData.quaternion+1,
            pLogData->ekfQuatData.quaternion+2,
            pLogData->ekfQuatData.quaternion+3);
      }
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
        generator->genDict(status_msg, pLogData->ekfQuatData.timeStamp, pose_msg.pose.orientation, pose_errors_msg.vector);
      }
      break;*/

    case SBG_ECOM_LOG_EKF_EULER:
      rp_to_quaternion(pLogData->ekfEulerData.euler[0],
          pLogData->ekfEulerData.euler[1], pose_msg.pose.orientation);

      pose_msg.header.stamp = ros_now;
      pose_errors_msg.vector.x = pLogData->ekfEulerData.eulerStdDev[0];
      pose_errors_msg.vector.y = pLogData->ekfEulerData.eulerStdDev[1];
      pose_errors_msg.vector.z = pLogData->ekfEulerData.eulerStdDev[2];
      pose_errors_msg.header.stamp = ros_now;
      new_pose_msg = true;
      setStatus(pLogData->ekfEulerData.status, ros_now);
      if(generator != NULL) {
        generator->genDict(status_msg, pLogData->ekfEulerData.timeStamp, pose_msg.pose.orientation, pose_errors_msg.vector);
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

void publish_info_panel(image_transport::Publisher &display_pub, geometry_msgs::Vector3Stamped pose_errors_msg);

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
  image_transport::ImageTransport it(n);
  image_transport::Publisher display_pub = it.advertise("info_display", 10);

  std::string uart_port;
  int uart_baud_rate;
  string output_filename;


  n.param<string>("/sbg_ellipse/uart_port", uart_port, "/dev/ttyUSB0");
  n.param<int>("/sbg_ellipse/uart_baud_rate", uart_baud_rate, 115200);
  n.param<string>("/sbg_ellipse/output_file", output_filename, "");
  n.param<bool>("/sbg_ellipse/discard_heading", should_discard_heading, true);

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

  errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_EULER, SBG_ECOM_OUTPUT_MODE_DIV_2);
  if (errorCode != SBG_NO_ERROR){ROS_WARN("sbgEComCmdOutputSetConf SBG_ECOM_LOG_EKF_QUAT Error");}

  errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_NAV, SBG_ECOM_OUTPUT_MODE_DIV_2);
  if (errorCode != SBG_NO_ERROR){ROS_WARN("sbgEComCmdOutputSetConf SBG_ECOM_LOG_EKF_NAV Error");}

  errorCode = sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_UTC_TIME, SBG_ECOM_OUTPUT_MODE_DIV_2);
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

      publish_info_panel(display_pub, pose_errors_msg);

      new_pose_msg = false;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
