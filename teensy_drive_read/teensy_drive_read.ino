#include <ros.h>                // header files sourced from  Step 3
#include <std_msgs/Bool.h>      
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <race/drive_values.h>
ros::NodeHandle  nh;


boolean flagStop = false;     // These values were cacluated for the specific Teensy microcontroller using an oscilloscope. 
int pwm_center_value = 9830;  //  15% duty cycle - corresponds to zero velocity, zero steering
int pwm_lowerlimit = 6554;    //  10% duty cycle - corresponds to max reverse velocity, extreme left steering
int pwm_upperlimit = 13108;   //  20% duty cycle - corresponds to max forward velocity, extreme right steering

std_msgs::Int32 str_msg;          // creater a ROS Publisher called chatter of type str_msg
ros::Publisher chatter("chatter", &str_msg);
std_msgs::Int32 pwm_speed_msg;
ros::Publisher speed_pub("pwm_speed_read", &pwm_speed_msg);
std_msgs::Int32 pwm_angle_msg;
ros::Publisher angle_pub("pwm_angle_read", &pwm_angle_msg);

int kill_pin = 2;     // This is the GPIO pin for emergency stopping.
unsigned long duration = 0;

void messageDrive( const race::drive_values& pwm ) 
{
//  Serial.print("Pwm drive : ");
//  Serial.println(pwm.pwm_drive);
//  Serial.print("Pwm angle : ");
//  Serial.println(pwm.pwm_angle);
  
  if(flagStop == false)
  {
    str_msg.data = pwm.pwm_drive;
    chatter.publish( &str_msg );

    if(pwm.pwm_drive < pwm_lowerlimit)  // Pin 5 is connected to the ESC..dive motor
    {
      analogWrite(5,pwm_lowerlimit);    //  Safety lower limit        
    }
    else if(pwm.pwm_drive > pwm_upperlimit)
    {
      analogWrite(5,pwm_upperlimit);    //  Safety upper limit
    }
    else
    {
      analogWrite(5,pwm.pwm_drive);     //  Incoming data                    
    }

    if(pwm.pwm_angle < pwm_lowerlimit) // Pin 6 is connected to the steering servo.
    {
      analogWrite(6,pwm_lowerlimit);    //  Safety lower limit        
    }
    else if(pwm.pwm_angle > pwm_upperlimit)
    {
      analogWrite(6,pwm_upperlimit);    //  Safety upper limit
    }
    else
    {
      analogWrite(6,pwm.pwm_angle);     //  Incoming data                    
    }

  }
  else
  {
    analogWrite(5,pwm_center_value);
    analogWrite(6,pwm_center_value);    
  }
}

void messageEmergencyStop( const std_msgs::Bool& flag )
{
  flagStop = flag.data;
  if(flagStop == true)
  {
    analogWrite(5,pwm_center_value);
    analogWrite(6,pwm_center_value);    
  }
}


ros::Subscriber<race::drive_values> sub_drive("drive_pwm", &messageDrive );   // Subscribe to drive_pwm topic sent by Jetson
ros::Subscriber<std_msgs::Bool> sub_stop("eStop", &messageEmergencyStop );  // Subscribe to estop topic sent by Jetson

void setup() {
  // Need to produce PWM signals so we need to setup the PWM registers. This setup happens next.
  analogWriteFrequency(5, 100); //  freq at which PWM signals is generated at pin 5.
  analogWriteFrequency(6, 100); 
  analogWriteResolution(16); // Resolution for the PWM signal
  analogWrite(5,pwm_center_value); // Setup zero velocity and steering.
  analogWrite(6,pwm_center_value);
  pinMode(13,OUTPUT); // Teensy's onboard LED pin. 
  digitalWrite(13,HIGH); // Setup LED.
  pinMode(kill_pin,INPUT); // Set emergency pin to accept inputs.
  pinMode(20, INPUT);
  //pinMode(19, INPUT);
  Serial.begin(115200);
//  digitalWrite(2,LOW);

  nh.initNode();  // intialize ROS node
  nh.subscribe(sub_drive); // start the subscribers.
  nh.subscribe(sub_stop);

  nh.advertise(chatter);  // start the publisher..can be used for debugging.
  nh.advertise(speed_pub);
  nh.advertise(angle_pub);
}

void loop() {
  duration = pulseIn(kill_pin, HIGH, 30000);  // continuously monitor the kill pin.
  while(duration > 1900) // stop if kill pin activated..setup everything to zero. 
  {
    duration = pulseIn(kill_pin, HIGH, 30000);
    analogWrite(5,pwm_center_value);
    analogWrite(6,pwm_center_value);        
  }
  // put your main code here, to run repeatedly:
  int pwm_drive, pwm_angle;
  pwm_drive = pulseIn(20, HIGH);
  pwm_angle = pulseIn(19, HIGH);
  //std::stringstream ss;
  //ss << "Drive: " << pwm_drive <<" Angle: " << pwm_angle;
  pwm_speed_msg.data = pwm_drive;
  speed_pub.publish( &pwm_speed_msg );
  pwm_angle_msg.data = pwm_angle;
  angle_pub.publish( &pwm_angle_msg );
  nh.spinOnce();
  /*
  if(Serial.available())
  {
    int spd = Serial.read();
    if(spd>127) {
      spd = spd-128;
      spd = map(spd,0,100,410,820);
      analogWrite(5,spd);  
    }
    else {
      //angle servo
      spd = map(spd,0,100,410,820);
      analogWrite(6,spd);
    }
    
  } 
  */ 
}
