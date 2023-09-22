/*
-General notes--------------------------
* This program delivers a fixed number of airpuffs for a set duration of time at a variable ITI, with TTLs sent to Synapse.

-serial event ids------------------------------
---parameters---
- 127 trial_count

- 128 min_iti
- 129 max_iti

- 116 sol_duration
- 117 num_sol
- 118 inter_sol_time



---dynamic parameters--- 
[[2 line method: first row shows ts in 2nd column second row shows current value in 2nd colum]]
- none

---events---
- 1   start_session
- 14  sol_offset

- 31  lick
- 11  break_engaged
- 12  break_disengaged
- 13  spout_extended
- 15  spout_retracted
- 0   end_session

- 222 trial_end
*/

// dependencies
  #include <Wire.h>
  #include "Adafruit_MPR121.h"
  #include <Servo.h>

// session parameters *************************************************************************************************************
 // general parameters -------
  
  static byte trial_count = 15;
  static byte trial_ids [] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}; // channel ids-- 1: cs+, 0: cs-
  byte trial_id;


  static boolean session_serial_initiate = 1;  // 0: starts without serial input, 1: waits for serial input before starting

  // baseline period before first trial
  static int baseline_time = 5000;            // delay time until first trial
  
  // iti range (uniform random)
  static unsigned long min_iti = 5000;           // minimum iti (ms) 
  static unsigned long max_iti = 10000;           // maximum iti (ms)

  // air puff solenoid 
  static byte sol_duration = 100;          // duration solenoid is open during delivery
  static byte num_sol = 1;                // number of solenoid opening access period
  static int inter_sol_time = 0;        // interval between each solenoid opening (measured from end of previous until start of next)

// arduino pins ----------------------------------------------------------------------------
  // inputs ----------------------------
  
  // outputs ---------------------------
  static byte pinSol = 13;

  // ttls for external time stamps
  static byte pinSol_ttl = 3;

// flags -------------------------------------------------------------------------------------
  // variables---
  //boolean solOpen;
  //boolean frame;
  //boolean lick;
  boolean first_loop = 1;     // sets the starting condition to first_loop = true, so first_loop setup section will run


// counters ---------------------------------------------------------------------------------
  // variables---
  volatile byte count_sol;                 // counter for number of solenoid openings per access period
  volatile byte count_trial;               // counter for number of trials

// time stamps & timers ---------------------------------------------------------------------
  // variables---
  volatile unsigned long ts; 
  unsigned long ts_start;
  unsigned long ts_trial_start;
  unsigned long ts_sol_offset;
  unsigned long ts_sol_onset;
  unsigned long ts_sol_ttl_off;
  unsigned long ts_trial_end;

  unsigned long session_end_ts;

// parameters---
  static int ttl_duration = 2; // duration of tttl pulses for external time stamps (ms)

  
// _______________________________________________________________________________________________________________________________________________________________________________
/// setup ________________________________________________________________________________________________________________________________________________________________________
void setup() {
  delay(100);
  Serial.begin(115200);
  delay(100);
  randomSeed(analogRead(0)); 

  // define outputs
  pinMode(pinSol, OUTPUT);
  pinMode(pinSol_ttl, OUTPUT);

  digitalWrite(pinSol, LOW);
  digitalWrite(pinSol_ttl, LOW);

  // wait for serial command before initating session---------------------------------------------------------------------
  if(session_serial_initiate){
    delay(100);
    while (Serial.available() <= 0) {} // wait for serial input to start session
    delay(100);
  }

 // save start time
  ts_start=millis();  
}

void loop() {
 // background functions (run constantly independent of task events)--------------------------------------------
 // generate timestamp ---------------------------
  ts=millis()-ts_start;

 // close solenoids---------------------------
  if(ts>=ts_sol_offset && ts_sol_offset!=0){                 // if time is after solenoid offset time
    digitalWrite(pinSol, LOW);                               // close solenoid
    Serial.print(14); Serial.print(" "); Serial.println(ts); // print sol offset
    ts_sol_offset = 0;    // reset solenoid offset time to close if statement
    }

 // turn off ttls for external time stamps ------------------------
 // solenoid---
  if(ts>=ts_sol_ttl_off && ts_sol_ttl_off!=0){
    digitalWrite(pinSol_ttl,LOW);  // write ttl low
    ts_sol_ttl_off = 0;            // reset off time to close if statement
  }

 // session initialization (runs once at start) -----------------------------------------------------------------
  if(first_loop){
    Serial.print(1);   Serial.print(" "); Serial.println(ts);                         // print start session
    Serial.print(127); Serial.print(" "); Serial.println(trial_count);                // print session duration

    Serial.print(128); Serial.print(" "); Serial.println(min_iti);                    // print min_iti
    Serial.print(129); Serial.print(" "); Serial.println(max_iti);                    // print max_iti
    
    Serial.print(116); Serial.print(" "); Serial.println(sol_duration);                // print sol_duration
    Serial.print(117); Serial.print(" "); Serial.println(num_sol);                     // print num_sol
    Serial.print(118); Serial.print(" "); Serial.println(inter_sol_time);              // print inter_sol_time

    // set and print first trial id
    trial_id = trial_ids[0];
    Serial.print(222);      Serial.print(" "); Serial.println(ts);                     // print first trial id
    Serial.print(trial_id); Serial.print(" "); Serial.println(ts);
    
    // set time for first delivery
    ts_trial_start = ts + baseline_time + random(min_iti, max_iti);           // calculate first trial start
    count_trial = 0;          // added 9/20/23, not sure if necessary
    count_sol = 0;            // added 9/20/23, not sure if necessary
    
    first_loop = 0;

  }


 // schedule of air puff delivery --------------------------------------------------------------
  if(ts >= ts_trial_start && ts_trial_start != 0){
    ts_trial_end = ts + (num_sol * (sol_duration + inter_sol_time));        // set time for when trial ends and variables reset
    ts_sol_onset = ts;                                                      // set time for when solenoid access occurs
    ts_trial_start = 0;

  }

 // start air puff trial --------------------------------------------------------------
 // open solenoid num_sol times ---------------------------
 if(ts >= ts_sol_onset && ts_sol_onset != 0 &&  count_sol < num_sol){
    if(trial_id == 1){ // if cs+
      digitalWrite(pinSol,HIGH);          // open solenoid
      ts_sol_offset = ts + sol_duration;  // set solenoid close time
      
      Serial.print(18); Serial.print(" "); Serial.println(ts); // print sol onset
      
      digitalWrite(pinSol_ttl,HIGH);      // turn on ttl for solenoid onset 
      ts_sol_ttl_off = ts + ttl_duration; // set ttl offset time
    }

    ts_sol_onset = ts + inter_sol_time + sol_duration; // set next solenoid onset time
    count_sol++;                                       // increase counter for solenoid
  }
 
 // end air puff trial --------------------------------------------------------------
 if(ts >= ts_trial_end && ts_trial_end != 0){
    ts_trial_end = 0;
    // reset various variables and counters
    ts_sol_onset = 0;
    count_sol = 0;

    count_trial = count_trial + 1;
    trial_id = trial_ids[count_trial];
    Serial.print(222);      Serial.print(" "); Serial.println(ts);
    Serial.print(trial_id); Serial.print(" "); Serial.println(ts);

    ts_trial_start = ts + random(min_iti, max_iti);           // calculate next trial start

  }

  // session termination ---------------------------------------------------------------------------------------------------
 // end session after trials exceed trial_count
  if(count_trial >= trial_count && session_end_ts == 0){
    session_end_ts = ts + min_iti-1000; // set end of session to 1s shorter than minimum ITI
  }
  
  if(ts > session_end_ts && session_end_ts != 0){
    fun_end_session();
    Serial.print(99); Serial.print(" "); Serial.println(ts);    // print end of session  
  }
}


/// end session _________________________________________________________________________________________________________________________________________________________
void fun_end_session() {
  Serial.print(0); Serial.print(" "); Serial.println(ts);    // print end of session                 
  while(1){}                               //  Stops executing the program
}
