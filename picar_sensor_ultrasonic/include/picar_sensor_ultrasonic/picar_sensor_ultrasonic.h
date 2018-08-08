#ifndef PICAR_SENSOR_ULTRASONIC_H_
#define PICAR_SENSOR_ULTRASONIC_H_


#include <ros/ros.h>
#include <pigpiod_if2.h>
#include <time.h>

// Include ROS-Msgs
// Publisher
#include <sensor_msgs/Range.h>
#include <sensor_msgs/LaserScan.h>
// Subscriber
#include <geometry_msgs/Twist.h>

#define HC_SR04_READ_TIMEOUT 100000 //us -> 100ms
#define HIGH 1
#define LOW 0
#define MAX_TIME_USEC 240
#define PI2 = 1.57079632679

static const int TRIGGER = 18, ECHO = 27, SERVO = 17, 
                 SERVO_MIN = 501, SERVO_MAX = 2500, SERVO_STEP_SIZE = 11, 
                 CLUSTERING_COUNT = 3;
static const float CLUSTERING_MAX_DIST = 0.05, VELOCITY_THRESHOLD = 0.01;

/**
 * @class 
 * @brief Server that handles all the path segments
 */
class SensorUltrasonic
{
public:
  /**
   * @brief Constructor
   */
  SensorUltrasonic();

  /**
   * @brief Destructor
   */
  ~SensorUltrasonic();

  void configure();
  void update();

  // Input/Output of Subscriber/Publisher
  void readInput();
  void writeOutput();

  // Subscriber callback functions
  void topicCallback_robotTwist(const geometry_msgs::Twist& msg);

  static void alertEcho(int temp, uint32_t gpio, uint32_t level, uint32_t tick);
  static void triggerSensorReading();
  static bool moveServo();
  static void clusterPoints();
  static float getMean();


  // Additional Functions

protected:
  // Nodehandle
  ros::NodeHandle n_;

  // Publisher
  static ros::Publisher pub_sensor_ultrasonic_;

  // Subscriber
  ros::Subscriber sub_robot_twist_;

  // Publishing messages
  static sensor_msgs::Range out_sensor_ultrasonic_;
  static sensor_msgs::LaserScan out_sensor_ultrasonic_scan_;

  // Subscribed messages
  geometry_msgs::Twist in_robot_twist_;

  // Usual membervariables
  static uint32_t servoPos_, sensorStartTick_;
  static std::vector<float> lastSensorReadings_;
  static int pi_;
};

uint32_t SensorUltrasonic::servoPos_, SensorUltrasonic::sensorStartTick_;
std::vector<float> SensorUltrasonic::lastSensorReadings_;
int SensorUltrasonic::pi_;

ros::Publisher SensorUltrasonic::pub_sensor_ultrasonic_;
sensor_msgs::Range SensorUltrasonic::out_sensor_ultrasonic_;
sensor_msgs::LaserScan SensorUltrasonic::out_sensor_ultrasonic_scan_;

#endif
























