#include "../include/picar_sensor_ultrasonic/picar_sensor_ultrasonic.h"


SensorUltrasonic::SensorUltrasonic()
{
  // Initialize publisher
  SensorUltrasonic::pub_sensor_ultrasonic_ = n_.advertise<sensor_msgs::LaserScan>("picar/sensors/ultrasonic", 1);

  // Initialize subscriber
  sub_robot_twist_ = n_.subscribe("/cmd_vel", 1, &SensorUltrasonic::topicCallback_robotTwist, this);

  // initialize output
  SensorUltrasonic::out_sensor_ultrasonic_scan_.header.seq = 0.;
  SensorUltrasonic::out_sensor_ultrasonic_scan_.header.frame_id = "sensor_ultrasonic";
  SensorUltrasonic::out_sensor_ultrasonic_scan_.angle_min = 0.1;
  SensorUltrasonic::out_sensor_ultrasonic_scan_.angle_max = 3.04159;
  SensorUltrasonic::out_sensor_ultrasonic_scan_.angle_increment = 0.0174533;
  SensorUltrasonic::out_sensor_ultrasonic_scan_.time_increment = 0.02;
  SensorUltrasonic::out_sensor_ultrasonic_scan_.scan_time = 2;
  SensorUltrasonic::out_sensor_ultrasonic_scan_.range_min = 0.02;
  SensorUltrasonic::out_sensor_ultrasonic_scan_.range_max = 4.0;

  // initialize sensor
  if ((pi_ = pigpio_start(NULL, NULL)) < 0) 
    ROS_WARN_STREAM("GPIO could not be initialized!");

  if(set_mode(pi_, TRIGGER, PI_OUTPUT) != 0)
    ROS_WARN_STREAM("PIN 'TRIGGER' could not be initialized!");
  
  if(set_mode(pi_, ECHO, PI_INPUT) != 0)
    ROS_WARN_STREAM("PIN 'ECHO' could not be initialized!");


  SensorUltrasonic::sensorStartTick_ = 0;

  if(callback(pi_, ECHO, EITHER_EDGE, SensorUltrasonic::alertEcho) < 0)
    ROS_WARN_STREAM("CB 'ECHO' could not be initialized!");

  // initialize servo
  if(set_mode(pi_, SERVO, PI_OUTPUT) != 0)  
    ROS_WARN_STREAM("PIN 'SERVO' could not be initialized!");

  SensorUltrasonic::servoPos_ = SERVO_MAX;
  set_servo_pulsewidth(pi_, SERVO, servoPos_);

  ROS_INFO_STREAM("Sensor Ultrasonic successfully initialized.");
}

SensorUltrasonic::~SensorUltrasonic()
{

}

void SensorUltrasonic::configure()
{
//  ROS_INFO_STREAM("Configuration successfully completed.");
}

// update function called by node
void SensorUltrasonic::update()
{
  lastSensorReadings_.clear();
  SensorUltrasonic::out_sensor_ultrasonic_scan_.ranges.clear();
  
  if (in_robot_twist_.linear.x <= VELOCITY_THRESHOLD
   && in_robot_twist_.linear.y <= VELOCITY_THRESHOLD
   && in_robot_twist_.linear.z <= VELOCITY_THRESHOLD
   && in_robot_twist_.angular.x <= VELOCITY_THRESHOLD
   && in_robot_twist_.angular.y <= VELOCITY_THRESHOLD
   && in_robot_twist_.angular.z <= VELOCITY_THRESHOLD)
  {
    //ROS_INFO_STREAM("SCAN!");
    gpio_trigger(pi_, TRIGGER, 10, HIGH);
  }
  else
    ROS_INFO_STREAM("NO SCAN!");

  //getDistance();
  //writeOutput();
}



// Input/Output of Subscriber/Publisher

void SensorUltrasonic::readInput()
{

}

void SensorUltrasonic::writeOutput()
{

}



// Subscriber callback functions

void SensorUltrasonic::topicCallback_robotTwist(const geometry_msgs::Twist& msg)
{
  in_robot_twist_ = msg;
}


void SensorUltrasonic::alertEcho(int temp, uint32_t gpio, uint32_t level, uint32_t tick)
{
  //ROS_INFO_STREAM("GPIO " << gpio << " became " << level << " at " << tick << " and temp " << temp);
  if (level == HIGH)      // First loop
  {
    SensorUltrasonic::sensorStartTick_ = tick;
  }
  else if (level == LOW)  // Second loop
  {
    float travelTime = (float)(tick - SensorUltrasonic::sensorStartTick_);
    
    ROS_DEBUG_STREAM("Distance (m):  " << travelTime*0.017150);

    lastSensorReadings_.push_back(travelTime*0.00017150);

    // Add scan to pointcloud
    if (lastSensorReadings_.size()==CLUSTERING_COUNT)
    {
      clusterPoints();
      float meanDistance = getMean();
      out_sensor_ultrasonic_scan_.ranges.push_back(meanDistance);
      lastSensorReadings_.clear();

      // Move Servo
      // Publish scan if finished
      if (moveServo())
      {
        // move servo back to start position
        SensorUltrasonic::servoPos_ = SERVO_MAX;
        set_servo_pulsewidth(pi_, SERVO, servoPos_);

        // publish data
        SensorUltrasonic::out_sensor_ultrasonic_scan_.header.stamp = ros::Time::now();
        SensorUltrasonic::pub_sensor_ultrasonic_.publish(out_sensor_ultrasonic_scan_);
        return;
      }
    }

    // Trigger next sensor reading
    gpio_trigger(pi_, TRIGGER, 10, HIGH);
    //triggerSensorReading();

  }
}

void SensorUltrasonic::triggerSensorReading()
{
  gpio_write(pi_, TRIGGER, HIGH);
  usleep(10);
  gpio_write(pi_, TRIGGER, LOW);
}

// returns true if end is reached
bool SensorUltrasonic::moveServo()
{
  if ((servoPos_ - SERVO_STEP_SIZE) < SERVO_MIN)
    return true;

  servoPos_ -= SERVO_STEP_SIZE;

  ROS_DEBUG_STREAM("Moving Servo to: " << servoPos_);
  
  set_servo_pulsewidth(pi_, SERVO, servoPos_);

  return false;
}

void SensorUltrasonic::clusterPoints()
{
  if (lastSensorReadings_.size() <= 1)
    return;
  bool rangeInCluster[CLUSTERING_COUNT] = {0};
  bool clusterFound = false;
  for (int i=0;i<lastSensorReadings_.size();i++)
    for (int k=i+1;k<lastSensorReadings_.size();k++)
      if (fabs(lastSensorReadings_.at(i)-lastSensorReadings_.at(k)) <= CLUSTERING_MAX_DIST)
        rangeInCluster[i] = rangeInCluster[k] = clusterFound = true;
  
  if (!clusterFound)
  {
    lastSensorReadings_.clear();
    lastSensorReadings_.push_back(out_sensor_ultrasonic_scan_.range_max-0.01);
  }
  else
    for (int i=lastSensorReadings_.size()-1;i>=0;i--)
      if (rangeInCluster[i] == false)
        lastSensorReadings_.erase(lastSensorReadings_.begin()+i);
}

float SensorUltrasonic::getMean()
{
  if (lastSensorReadings_.size() == 1)
    return lastSensorReadings_.at(0);

  float mean = 0;

  for (int i=0;i<lastSensorReadings_.size();i++)
    mean += lastSensorReadings_.at(i);

  return mean/lastSensorReadings_.size();
}



// Additional Functions




















