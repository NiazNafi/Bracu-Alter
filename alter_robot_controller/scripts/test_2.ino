#include <ros.h>
#include <std_msgs/String.h>

// Motor Pins (Arduino Uno)
#define RIGHT_PWM 3
#define RIGHT_DIR 2
#define LEFT_PWM 5
#define LEFT_DIR 4

ros::NodeHandle nh;

// Single debug publisher
std_msgs::String debug_msg;
ros::Publisher debug("motor_debug", &debug_msg);

void setMotor(int pwmPin, int dirPin, int speed, bool forward) {
  // Constrain speed to 0-255 and set outputs
  speed = constrain(speed, 0, 255);
  analogWrite(pwmPin, speed);
  digitalWrite(dirPin, forward ? HIGH : LOW);
}

void handleCommand(const std_msgs::String &cmd_msg) {
  const char* cmd = cmd_msg.data;
  
  // Parse speed value (appears after ':')
  char* colon = strchr(cmd, ':');
  if (!colon) return;
  int speed = atoi(colon + 1);

  // Process command
  if (strstr(cmd, "right_forward")) {
    setMotor(RIGHT_PWM, RIGHT_DIR, speed, true);
    debug_msg.data = "Right motor forward";
  }
  else if (strstr(cmd, "right_backward")) {
    setMotor(RIGHT_PWM, RIGHT_DIR, speed, false);
    debug_msg.data = "Right motor backward";
  }
  else if (strstr(cmd, "left_forward")) {
    setMotor(LEFT_PWM, LEFT_DIR, speed, true);
    debug_msg.data = "Left motor forward";
  }
  else if (strstr(cmd, "left_backward")) {
    setMotor(LEFT_PWM, LEFT_DIR, speed, false);
    debug_msg.data = "Left motor backward";
  }
  else if (strstr(cmd, "stop")) {
    setMotor(RIGHT_PWM, RIGHT_DIR, 0, true);
    setMotor(LEFT_PWM, LEFT_DIR, 0, true);
    debug_msg.data = "All motors stopped";
  }

  nh.loginfo(debug_msg.data);
  debug.publish(&debug_msg);
}

ros::Subscriber<std_msgs::String> sub("motor_cmd", &handleCommand);

void setup() {
  // Initialize motor pins
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);
  pinMode(LEFT_DIR, OUTPUT);
  
  // Stop motors initially
  setMotor(RIGHT_PWM, RIGHT_DIR, 0, true);
  setMotor(LEFT_PWM, LEFT_DIR, 0, true);

  // ROS setup
  nh.initNode();
  nh.advertise(debug);
  nh.subscribe(sub);
  
  nh.loginfo("Motor controller ready");
}

void loop() {
  nh.spinOnce();
  delay(10);
}
