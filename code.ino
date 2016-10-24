// Library
#include <NewPing.h>

// CONSTANTS

// only print to the serial console every 1/4 second
const int serialPeriod = 250;
unsigned long timeSerialDelay = 0;

// a period of 20ms = a frequency of 50Hz
const int loopPeriod = 20;
unsigned long timeLoopDelay = 5;

// specify the trig & echo pins used for the ultrasonic sensors
const int trig_pin = 14;
const int echo_pin = 15;
#define MAX_DISTANCE 200

unsigned int sensor_distance;
int sensor_duration;
int mode = 0;

// define the states
#define DRIVE_FORWARD 0
#define TURN 1

// define the states
#define SEARCH_FORWARD 2
#define SEARCH_LEFT 3
#define SEARCH_RIGHT 4

// 0 = search forward (DEFAULT), 1 = search left, 2 = search right
int state2 = SEARCH_FORWARD;

// 0 = drive forward (DEFAULT), 1 = turn left
int state = DRIVE_FORWARD;

void setup()
{
  Serial.begin(9600);
  
  // ultrasonic sensor pin configurations
  pinMode(trig_pin, OUTPUT);
  pinMode(echo_pin, INPUT);
  
  // right motor
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  
  //left motor
  pinMode(11,OUTPUT);
  pinMode(10,OUTPUT);
}

void loop()
{
    while (Serial.available() > 0) 
    {
        char ch = Serial.read();
        if (ch == 'o') 
        { 
          mode = 1;
          while (mode == 1)
          {
            // prints debugging messages to the serial console
            debugOutput();
            
            if(millis() - timeLoopDelay >= loopPeriod)
            {
              // read and store the measured distances
              readUltrasonicSensors();
              
              robot_state();
              
              timeLoopDelay = millis();
              
              char h = Serial.read();
              if(h == 'o')
              {
                stopped();
                Serial.println(h);
                break;
              }
            }
          } 
        } 
        
        else if(ch == 't')
        {
          mode = 2;
          Serial.println(mode);
          while (mode == 2)
          {
            if (Serial.available() > 0) 
            {
                char ch = Serial.read();
                Serial.print("Received: ");
                Serial.println(ch);
                if (ch == 'f') 
                {
                    // drive forward in normal speed
          
                    // right motor movement
                    analogWrite(5,255);
                    digitalWrite(6,LOW);
                    
                    // left motor movement
                    analogWrite(10,235);
                    digitalWrite(11,LOW);
                }
                
                else if(ch == 'r')
                {
                  // right motor movement
                  analogWrite(6,255);
                  digitalWrite(5,LOW);
                  
                  // left motor movement
                  analogWrite(10,255);
                  digitalWrite(11,LOW);
                   
                }
                
                else if(ch == 'l')
                { 
                  // right motor movement
                  analogWrite(5,255);
                  digitalWrite(6,LOW);
                  
                  // left motor movement
                  analogWrite(11,255);
                  digitalWrite(10,LOW);
                }
                
                else if(ch == 'b')
                {
                  // right motor movement
                  analogWrite(6,255);
                  digitalWrite(5,LOW);
                    
                  // left motor movement
                  analogWrite(11,236);
                  digitalWrite(10,LOW);
                  
                }
                
                //char q = Serial.read();
                else if(ch == 't')
                {
                  stopped();
                  Serial.println(ch);
                  break;
                }
            }   
          }
        }
        
        else if(ch == 'x')
        {
          mode = 3;
          Serial.println(mode);
          while (mode == 3)
          {
            // prints debugging messages to the serial console
            debugOutput();
            
            if(millis() - timeLoopDelay >= loopPeriod)
            {
              // read and store the measured distances
              readUltrasonicSensors();
              
              robot_state2();
              
              timeLoopDelay = millis();
              
              char k = Serial.read();
              if(k == 'x')
              {
                stopped();
                Serial.println(k);
                break;
              }
            }
          } 
      }
    }  
}

// alter through the states of the robot
void robot_state()
{
  if(state == DRIVE_FORWARD) // no obstacles detected
  {
    // if there's nothing in front of us
    if(sensor_distance > 20 || sensor_distance == 0)
    {
      // drive forward
      drive_forward();
    }
    
    else // there's an object in front of us
    {
      state = TURN;
    }
  
  }
  else if(state == TURN) // obstacle detected -- turn left
  {
    unsigned long turning_time = 650; // time it takes to turn 90 degrees
    
    unsigned long starting_turn_time = millis(); // save the time that we started turning
    
    // stop for 0.1 s
    stopped();
    delay(300);
    
    while((millis()- starting_turn_time) < turning_time) // stay in this loop until turning_time has elapsed
    {
    // turn
    turn();
    }
    
    state = DRIVE_FORWARD;
  }
}

void robot_state2()
{
  if(state2 == SEARCH_FORWARD)
  {
    
    if(sensor_distance < 30 && sensor_distance > 10)
    {
      //move closer
      drive_forward();    
    }  
    
    else if(sensor_distance <= 10)
    {
      // if near object, stop
      stopped();
    }
    
    else 
    {
      state2 = SEARCH_LEFT;
    }  
  }
  
  if(state2 == SEARCH_LEFT)
  {
    //turn left to start searching
    unsigned long turning_time = 400; // time it takes to turn 
    unsigned long starting_turn_time = millis(); // save the time that we started turning
      
    while((millis()- starting_turn_time) < turning_time) // stay in this loop until turning_time has elapsed
    {
      // turn
      readUltrasonicSensors();
      turn_left();
      
      if(sensor_distance < 30)
      {
        state2 = SEARCH_FORWARD;
        break;
      }  
    
      else
      {
        state2 = SEARCH_RIGHT;
      }
     }
    }
 
  
  if(state2 == SEARCH_RIGHT)
  {
    
    // turn right to begin searching
    unsigned long turning_time2 = 1000; // time it takes to turn 90 degrees
    unsigned long starting_turn_time2 = millis(); // save the time that we started turning
      
    while((millis()- starting_turn_time2) < turning_time2) // stay in this loop until turning_time has elapsed
    {
      // turn
      readUltrasonicSensors();
      turn_right();
      if(sensor_distance < 30)
      {
        state2 = SEARCH_FORWARD;
        break;
      }  
    
      else
      {
        unsigned long turning_time3 = 800; // time it takes to turn 
        unsigned long starting_turn_time3 = millis(); // save the time that we started turning
          
        while((millis()- starting_turn_time3) < turning_time3) // stay in this loop until turning_time has elapsed
        {
          // turn
          readUltrasonicSensors();
          turn_left();
          delay(100);
          drive_forward();
        }
        
        state2 = SEARCH_FORWARD;
      }         
    }
  } 
}


void readUltrasonicSensors()
{
NewPing DistanceSensor(trig_pin, echo_pin, MAX_DISTANCE);
sensor_distance = DistanceSensor.ping_cm();
}


void debugOutput()
{
if((millis() - timeSerialDelay) > serialPeriod)
{
Serial.print("Ultrasonic Sensor Distance: ");
Serial.print(sensor_distance);
Serial.print("cm");
Serial.println();

timeSerialDelay = millis();
}
}

void drive_forward()
{
// drive forward in normal speed

// right motor movement
analogWrite(5,255);
digitalWrite(6,LOW);

// left motor movement
analogWrite(10,236);
digitalWrite(11,LOW);
}

void turn()
{

// right motor movement
analogWrite(5,255);
digitalWrite(6,LOW);

// left motor movement
analogWrite(11,255);
digitalWrite(10,LOW);
}

void stopped()
{
// right motor switch off
digitalWrite(6,LOW);
digitalWrite(5,LOW);

// left motor switch off
digitalWrite(10,LOW);
digitalWrite(11,LOW);
}

void turn_right()
{

  // right motor movement
  analogWrite(5,125);
  digitalWrite(6,LOW);
  
  // left motor movement
  analogWrite(11,100);
  digitalWrite(10,LOW);
}

void turn_left()
{

  // right motor movement
  analogWrite(6,125);
  digitalWrite(5,LOW);
  
  // left motor movement
  analogWrite(10,125);
  digitalWrite(11,LOW);
}

