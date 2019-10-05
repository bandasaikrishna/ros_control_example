#include <ros.h>
#include <rospy_tutorials/Floats.h>
#define encodPinA1      2                       // Quadrature encoder A pin
#define encodPinB1      8                       // Quadrature encoder B pin
#define M1              9                       // PWM outputs to motor driver module
#define M2              10

ros::NodeHandle  nh;

double pos = 0, vel= 0, output = 0, temp=0;
unsigned long lastTime,now,lasttimepub;
volatile long encoderPos = 0,last_pos=0;
rospy_tutorials::Floats joint_state;

void set_angle_cb( const rospy_tutorials::Floats& cmd_msg){
  output= cmd_msg.data[0]; 
}


ros::Subscriber<rospy_tutorials::Floats> sub("/joints_to_aurdino", set_angle_cb);
ros::Publisher pub("/joint_states_from_arduino", &joint_state);

void setup(){
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
  pinMode(encodPinA1, INPUT_PULLUP);                  // quadrature encoder input A
  pinMode(encodPinB1, INPUT_PULLUP);                  // quadrature encoder input B
  attachInterrupt(0, encoder, FALLING);               // update encoder position
  TCCR1B = TCCR1B & 0b11111000 | 1;                   // set 31KHz PWM to prevent motor noise  
}

void loop(){
  pos = (encoderPos*360)/2200 ;
  now = millis();
  int timeChange = (now - lastTime);
  if(timeChange>=500 )
  {
      temp = (360.0*1000*(encoderPos-last_pos)) /(2200.0*(now - lastTime));
      if ((encoderPos < -2 || encoderPos > 2) && temp >= -60 && temp <=60 ) // to gaurd encoderPos at boundary i.e., after max limit it will rest. Then lastPos will be greater than encoderpos
          vel =temp;
      lastTime=now;
      last_pos=encoderPos;
  }

  pwmOut(output);
  
  if ((now - lasttimepub)> 100)
  {
    joint_state.data_length=2;
    joint_state.data[0]=pos;
    joint_state.data[1]=vel;
    pub.publish(&joint_state);
    lasttimepub=now;
  }

  nh.spinOnce();

}

void encoder()  {                                     // pulse and direction, direct port reading to save cycles

  if (encoderPos > 2250 || encoderPos < -2250)
    encoderPos=0; 
  if (PINB & 0b00000001)    encoderPos++;             // if(digitalRead(encodPinB1)==HIGH)   count ++;
  else                      encoderPos--;             // if(digitalRead(encodPinB1)==LOW)   count --;
}

void pwmOut(float out) {                                
  if (out > 0) {
    analogWrite(M2, out);                             // drive motor CW
    analogWrite(M1, 0);
  }
  else {
    analogWrite(M2, 0);
    analogWrite(M1, abs(out));                        // drive motor CCW
  }
}
