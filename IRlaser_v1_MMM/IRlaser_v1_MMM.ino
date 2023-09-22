#include <Wire.h>
#include "Adafruit_MPR121.h"

//capicitance sensor
Adafruit_MPR121 cap = Adafruit_MPR121();

//pins
int IRlaser = 33; // pin for IRlaser
//int laser     = 36;   // we'll use this to trigger SLM
int IRlaserOut = 27;
//int videoOut = 40;

//int pinSpeaker = 22;
  
//variables to change;
unsigned long TTLPulseLength = 1000; //length of pulse, in ms
unsigned long IRlaserLength = 1000; //length of opening, in ms
//int lickCutoff = 5;
long ITImin = 50; //in seconds default 20 60
long ITImax = 120; //in seconds default is 25 90
//int ProbStim = 0; //out of 100
int ProbIRlaser = 100; //probability of reward type
int tone_to_controller_delay = 000; // in ms
// tone cue -----------------
static unsigned long CSplus_tone_freq = 000; // tone played during CS+ / laser stim period (Hz)
static unsigned long CSminus_tone_freq = 000; // tone played during CS- / laser stim period (Hz)
static unsigned long tone_duration = 000; // tone duration (starts with begining of access time)

//for status of outputs
int IRlaserOn = 0;
//int laserOn = 0;
int controllerOn = 0;
int reading = 0;
//int lickCount = 0;
unsigned long IRlaserTTL; //time of onset
//unsigned long laserTTL; //time of onset
unsigned long controllerTTL; //time of onset
unsigned long currentTime;
unsigned long ts_tone_to_controller = 0;
long NextTrial = 0; //milliseconds
//int NextStim = 0; //will be rand below
int NextIRlaser = 0; //will be rand below
int play_tone = 1;
int start_tone_to_ctrl_delay = 0; // boolean to start delay between tone and controller 
int start_controller = 0; // boolean of if tone and delay completed, so controller can go off

//for capacitance sensor
uint16_t lasttouched = 0;
uint16_t currtouched = 0;

void setup() {
  // put your setup code here, to run once:
  
Serial.begin(9600);

  pinMode(IRlaser, OUTPUT);
//  pinMode(laser, OUTPUT);
  pinMode(IRlaserOut, OUTPUT);
  pinMode(videoOut, OUTPUT);
  pinMode(pinSpeaker, OUTPUT);

  //get random values
  randomSeed(analogRead(0));

  

  if (!cap.begin(0x5A)) {
    Serial.println("MPR121 not found, check wiring?");
    while (1);
  }
  Serial.println("MPR121 found!");
  digitalWrite(videoOut,LOW); //video should not be recording yet

  //type anything in the serial window to start the session
  while (Serial.available() <= 0);
  digitalWrite(videoOut,HIGH);
  Serial.println("Session starting");
  NextTrial = random((ITImin*1000), (ITImax*1000)) + millis();
  NextIRlaser = random(1,100);

}



void loop() {

  if (millis() >= NextTrial && play_tone==1) {
    if (NextIRlaser <= ProbIRlaser) {
      tone(pinSpeaker, CSplus_tone_freq, tone_duration); //play tone when controller turns on
      
      ts_tone_to_controller = tone_duration + tone_to_controller_delay + millis();
      start_tone_to_ctrl_delay = 1;
      play_tone = 0;
    }

    else if (NextIRlaser >= ProbIRlaser) {
      tone(pinSpeaker, CSminus_tone_freq, tone_duration); //play tone when controller does not turn on
            
      ts_tone_to_controller = tone_duration + tone_to_controller_delay + millis();
      start_tone_to_ctrl_delay = 1;
      play_tone = 0;
    }
  }

  if (start_tone_to_ctrl_delay==1 && millis() >= ts_tone_to_controller) {
    start_controller = 1;
  }
  
  //deliver reward if enough time has past
  if (start_controller==1) {
    if (NextIRlaser <= ProbIRlaser) {

      digitalWrite(IRlaser,HIGH); //turns on controller
      controllerTTL=millis();
      controllerOn=1;
      
      digitalWrite(IRlaserOut,HIGH); //TTL event
      IRlaserTTL=millis();
      IRlaserOn=1;

      //check if stim this trial
//      if (NextStim <= ProbStim){
//        //laser TTL signal
//        digitalWrite(laser,HIGH);
//        laserTTL=millis();
//        laserOn=1;
//        Serial.print(millis()); Serial.println(" SLM Stim delivered");
//      }
      Serial.print(millis()); Serial.println(" IR laser stim delivered");
    }
      
     else if (NextIRlaser >= ProbIRlaser) {
      
      //check if stim this trial
//      if (NextStim <= ProbStim){
//        //laser TTL signal
//        digitalWrite(laser,HIGH);
//        laserTTL=millis();
//        laserOn=1;
//        Serial.print(millis()); Serial.println(" SLM Stim delivered");
//    }
    Serial.print(millis()); Serial.println(" No IR laser stim delivered");
     }
      
      NextTrial = random((ITImin*1000), (ITImax*1000)) + millis();
//      NextStim = random(1,100);
      NextIRlaser = random(1,100);

      //reset flags for next trial
      start_tone_to_ctrl_delay = 0;
      start_controller = 0;
      play_tone = 1;
            
  }
     

  //check if time has elapsed
  currentTime = millis();
  
  //turn laser TTL off
//  if (laserOn==1){
//    if ((currentTime-laserTTL)>=TTLPulseLength){
//      digitalWrite(laser, LOW);
//      laserOn=0;
//
//      Serial.print(millis()); Serial.println(" Laser TTL off");
//    }
//  }

  //turn IR laser TTL off
  if (IRlaserOn==1){
    if ((currentTime-IRlaserTTL)>=TTLPulseLength){
      digitalWrite(IRlaserOut, LOW);
      IRlaserOn=0;

      //Serial.print(millis()); Serial.println(" Event TTL off");
    }
  }


  //turn controller off
  if (controllerOn==1){
    if ((currentTime-controllerTTL)>=IRlaserLength){
    
      digitalWrite(IRlaser, LOW);
      controllerOn=0;
      
      Serial.print(millis()); Serial.println(" Controller off");  


    }
  }

}
