#include <ros.h>
#include <std_msgs/String.h>
#define drive_r_right 2
#define drive_l_right 3
#define drive_r_left 4
#define drive_l_left 5
#define flipper_for_r_right 6
#define flipper_for_l_right 7
#define flipper_for_r_left 8
#define flipper_for_l_left 9
#define flipper_back_r_right 10
#define flipper_back_l_right 11
#define flipper_back_r_left 12
#define flipper_back_l_left 13

ros::NodeHandle  nh;

std_msgs::String serialprint;

ros::Publisher pub("r_a",&serialprint);
int pwmValue = 180;

void callBackFunction(const std_msgs::String &cmd_msg){
  
  serialprint.data=cmd_msg.data;
  String command=cmd_msg.data;
  if (command == "forwards") {
    analogWrite(drive_r_right, pwmValue);
    analogWrite(drive_r_left, pwmValue);
    analogWrite(drive_l_right, 0);
    analogWrite(drive_l_left, 0);
  } else if (command == "backwards") {
    analogWrite(drive_r_right, 0);
    analogWrite(drive_r_left, 0);
    analogWrite(drive_l_right, pwmValue);
    analogWrite(drive_l_left, pwmValue);
  }
  else if (command == "left") {
    analogWrite(drive_r_right,pwmValue);
    analogWrite(drive_l_right, 0);
    analogWrite(drive_r_left, 0);
    analogWrite(drive_l_left, pwmValue);
  } else if (command == "right") {
    analogWrite(drive_r_right,0);
    analogWrite(drive_l_right, pwmValue);
    analogWrite(drive_r_left, pwmValue);
    analogWrite(drive_l_left, 0);
  }

  else if (command == "all_flipper_down") {
    analogWrite(flipper_for_r_left, 0);
    analogWrite(flipper_for_l_left, pwmValue);
    analogWrite(flipper_back_r_left, 0);
    analogWrite(flipper_back_l_left, pwmValue);
    analogWrite(flipper_for_r_right, 0);
    analogWrite(flipper_for_l_right, pwmValue);
    analogWrite(flipper_back_r_right, 0);
    analogWrite(flipper_back_l_right, pwmValue);
    //Serial.println("All flippers Down");
  }
   else if (command == "all_flipper_up" ) {
    analogWrite(flipper_for_r_left, pwmValue);
    analogWrite(flipper_for_l_left, 0);
    analogWrite(flipper_for_r_right, pwmValue);
    analogWrite(flipper_for_l_right, 0);
    analogWrite(flipper_back_r_left, pwmValue);
    analogWrite(flipper_back_l_left, 0);
    analogWrite(flipper_back_r_right, pwmValue);
    analogWrite(flipper_back_l_right, 0);
    //Serial.println("All flippers in UP position");
  }
    // Control flipper1, flipper2, flipper3 and flipper4
  else if (command == "flipper_forward_up") {
    analogWrite(flipper_for_r_left, pwmValue);
    analogWrite(flipper_for_l_left, 0);
    analogWrite(flipper_for_r_right, pwmValue);
    analogWrite(flipper_for_l_right, 0);
    //Serial.println("Flipper forward up");
  } else if (command == "flipper_forward_down") {
    analogWrite(flipper_for_r_left, 0);
    analogWrite(flipper_for_l_left, pwmValue);
    analogWrite(flipper_for_r_right, 0);
    analogWrite(flipper_for_l_right, pwmValue);
    //Serial.println("Flipper forward Down");
  } else if (command == "flipper_backward_down") {
    analogWrite(flipper_back_r_left, 0);
    analogWrite(flipper_back_l_left, pwmValue);
    analogWrite(flipper_back_r_right,0);
    analogWrite(flipper_back_l_right, pwmValue);
    //Serial.println("Flipper back in Down");
  } else if (command == "flipper_backward_up") {
    analogWrite(flipper_back_r_left, pwmValue);
    analogWrite(flipper_back_l_left, 0);
    analogWrite(flipper_back_r_right,pwmValue);
    analogWrite(flipper_back_l_right, 0);
    //Serial.println("Flipper back up");
  } 
  else if (command == "flipper_right_forward_up") {
    analogWrite(flipper_for_r_right, pwmValue);
    analogWrite(flipper_for_l_right, 0);
    //Serial.println("Flipper forware right up");
  } else if (command == "flipper_right_forward_down") {
    analogWrite(flipper_for_r_right, 0);
    analogWrite(flipper_for_l_right, pwmValue);
    //Serial.println("Flipper forware right down");
  } 
  else if (command == "flipper_left_forward_up") {
    analogWrite(flipper_for_r_left, pwmValue);
    analogWrite(flipper_for_l_left, 0);
    //Serial.println("Flipper forward left up");
  } else if (command == "flipper_left_forward_down") {
    analogWrite(flipper_for_r_left, 0);
    analogWrite(flipper_for_l_left, pwmValue);
    //Serial.println("Flipper forward left down");
  }

  // Control flipper3 and flipper4
  else if (command == "flipper_left_backward_up") {
    analogWrite(flipper_back_r_left, pwmValue);
    analogWrite(flipper_back_l_left, 0);
    //Serial.println("Flipper backward left up");
  } else if (command == "flipper_left_backward_down") {
    analogWrite(flipper_back_r_left, 0);
    analogWrite(flipper_back_l_left, pwmValue);
    //Serial.println("Flipper backward left down");
  } 
  else if (command == "flipper_right_backward_up") {
    analogWrite(flipper_back_r_right, pwmValue);
    analogWrite(flipper_back_l_right, 0);
    //Serial.println("Flipper backward right up");
  } else if (command == "flipper_right_backward_down") {
    analogWrite(flipper_back_r_right, 0);
    analogWrite(flipper_back_l_right, pwmValue);
    //Serial.println("Flipper backward right down");
  }
 else{
    digitalWrite(drive_r_right, LOW);
    digitalWrite(drive_l_right, LOW);
    digitalWrite(drive_r_left, LOW);
    digitalWrite(drive_l_left, LOW);
    serialprint.data=" Stop ";
    analogWrite(flipper_for_r_left, LOW);
    analogWrite(flipper_for_l_left, LOW);
    analogWrite(flipper_back_r_left, LOW);
    analogWrite(flipper_back_l_left, LOW);
    analogWrite(flipper_for_r_right, LOW);
    analogWrite(flipper_for_l_right, LOW);
    analogWrite(flipper_back_r_right, LOW);
    analogWrite(flipper_back_l_right, LOW);
    
    }

  pub.publish(&serialprint);
    
 
  }
  
ros::Subscriber<std_msgs::String> sub("command",&callBackFunction);

void setup() {
  
  nh.initNode();
  
  nh.advertise(pub);
  
  nh.subscribe(sub);
}

void loop() {
  
 nh.spinOnce();
 
 delay(1);
}
