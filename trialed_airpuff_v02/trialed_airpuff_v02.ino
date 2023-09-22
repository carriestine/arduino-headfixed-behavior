/*
-General notes--------------------------
v02 
- uses ChatGPT written method of randomly calculating a trial_order (which replaces the trial_ids array that had to be manually assigned) 
- based on a desired fraction (csPlusRatio) of cs+ trials
-Todo------------------------------------------
- add in additional optional delay from tone to spout
  - remove subtract opposite direction

- include opto manipulations during different stages of the task 
  - open-loop with a set stim_tm_step
  - closed-loop during access period
  - closed-loop after access period.  
  
- clean up active / inactive / right / left code? 
  - currently downsampled rotation is recorded as active / inactive, but it may benifit us to record left / right instead
  - of two minds, doesn't really matter so long as we are tracking the contingency and we are careful to analyze correctly

long term todo...
- add parameters via serial?
- setup website with scripts / hardware list

- write out description of rotary encoder code
- write up description of trial control timing code

-serial event ids------------------------------
---parameters---
- 127 trial_count
- 104 session_retract
- 105 session_serial_initiate

- 128 min_iti
- 129 max_iti

- 115 access_time
- 116 sol_duration
- 117 num_sol
- 118 inter_sol_time

- 121 tone_freq_cs_plus
- 122 tone_duration


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

*/

// dependencies
  #include <Wire.h>
  #include "Adafruit_MPR121.h"
  #include <Servo.h>

// session parameters *************************************************************************************************************
 // general parameters -------
  
  static byte trial_count = 50;
  //static byte trial_ids[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // channel ids-- 0: cs+, 1: cs-
  static float csPlusRatio = 1; // defines fraction of cs+ trials
  byte trial_id;

 
  static boolean session_retract = 0;          // 0: no retractable spout, 1: retractable spout
  static boolean session_serial_initiate = 1;  // 0: starts without serial input, 1: waits for serial input before starting

 // iti range (uniform random)
  static unsigned long min_iti = 20000;           // minimum iti (ms) 
  static unsigned long max_iti = 30000;           // maximum iti (ms)

 // access & solenoid --------
  static int tone_pre_access_time = 3000; // tone duration prior to access initiation 
  static int access_time = 0;          // total time before spout is retracted 
  
  static byte sol_duration = 100;          // duration solenoid is open during delivery
  static byte num_sol = 1;                // number of solenoid opening access period
  static int inter_sol_time = 0;        // interval between each solenoid opening (measured from end of previous until start of next)
 
 // tone cue -----------------
  boolean playTone = false; // true: tone will preceed solenoid, false: disables tone
  static unsigned long tone_freq_cs_plus  = 8000;  // tone played during access period / laser stim period (Hz)
  static unsigned long tone_freq_cs_minus = 4000;  // tone played during access period / laser stim period (Hz)
  
  static int tone_duration = tone_pre_access_time; // tone duration (starts with begining of access time)
  

// arduino pins ----------------------------------------------------------------------------
 // inputs ----------------------------
  static byte pinLickometer = 6;   
  
 // outputs ---------------------------
  static byte pinServo_break = 38;
  static byte pinServo_retract = 36;
  static byte pinSol = 8;
  static byte pinSpeaker = 12;
  static byte pinImageStart = 41;     
  static byte pinImageStop = 42;  

  // ttls for external time stamps
  static byte pinSol_ttl = 52;
  static byte pinLickometer_ttl = 46;

//capicitance sensor variables
  Adafruit_MPR121 cap = Adafruit_MPR121();
  uint16_t lasttouched = 0;
  uint16_t currtouched = 0;
  // note: code is setup to use sensor position 1

// servo break variables / parameters ------------------------------------------------------
  Servo servo_break;  // create servo object to control a servo
  static byte servo_break_engaged_deg = 22;
  static byte servo_break_disengaged_deg = 0;
  unsigned long detach_servo_break_ts = 0;
  static int detach_servo_break_step = 100; // time in ms to allow for the servo to travel

// servo retractable spout variables / parameters ------------------------------------------
  Servo servo_retract;
  static byte servo_retract_extended_deg = 180;
  static byte servo_retract_retracted_deg = 120;
  unsigned long detach_servo_retract_ts = 0;
  static int detach_servo_retract_step = 100; // time in ms to allow for the servo to travel
  unsigned long ts_servo_retract_retracted;
  unsigned long ts_pre_access_start = 0;

// flags -------------------------------------------------------------------------------------
 // variables---
  boolean solOpen;
  boolean frame;
  boolean lick;
  boolean first_loop = 1;

// counters ---------------------------------------------------------------------------------
 // variables---
  volatile byte count_sol;                 // counter for number of solenoid openings per access period
  volatile byte count_trial;               // counter for number of trials

// time stamps & timers ---------------------------------------------------------------------
 // variables---
  volatile unsigned long ts; 
  unsigned long ts_start;
  unsigned long ts_access_start; 
  unsigned long ts_sol_offset;
  unsigned long ts_sol_onset;
  unsigned long ts_sol_ttl_off;
  unsigned long ts_lickomter_ttl_off; 

  volatile unsigned long break_spout_delay; // random delay length between break and access period
  
  unsigned long session_end_ts;

 // parameters---
  static int ttl_duration = 2; // duration of tttl pulses for external time stamps (ms)
  
 // Calculate the number of CS+ and CS- trials based on the ratio
  int numCsPlusTrials = static_cast<int>(ceil(trial_count * csPlusRatio)); // round up number of cs+ trials to create a whole number
  int numCsMinusTrials = trial_count - numCsPlusTrials;
  
  int trial_order[trial_count];

  // Fill the trial order array with CS+ and CS- identifiers
  for (int i = 0; i < trial_count; i++) {
    if (numCsPlusTrials > 0) {
      trial_order[i] = 0; // CS+
      numCsPlusTrials--;
    } else {
      trial_order[i] = 1; // CS-
    }
  }

  // Randomly shuffle the trial order array
  for (int i = 0; i < trial_count; i++) {
    int randomIndex = random(i, trial_count);
    // Swap elements to shuffle
    int temp = trial_order[i];
    trial_order[i] = trial_order[randomIndex];
    trial_order[randomIndex] = temp;
  }


// _______________________________________________________________________________________________________________________________________________________________________________
/// setup ________________________________________________________________________________________________________________________________________________________________________
void setup() {
  Serial.begin(115200);
  randomSeed(analogRead(0)); 

 // engage servo break prior to session start
  servo_break.attach(pinServo_break); 
  Serial.print(11); Serial.print(" "); Serial.println(ts);      
  servo_break.write(servo_break_engaged_deg);

 // fully extend spout prior to session start if using liquid reinforcer
  servo_retract.attach(pinServo_retract); 
  Serial.print(13); Serial.print(" "); Serial.println(ts);      
  servo_retract.write(servo_retract_extended_deg);

 // delay with enough time for servos to move then detach
  delay(1500);
  servo_break.detach();
  servo_retract.detach();

 // define inputs --------------------------------
  pinMode(pinLickometer, INPUT);
  
 // define outputs
  pinMode(pinSol, OUTPUT);
  pinMode(pinSpeaker, OUTPUT);
  pinMode(pinImageStart, OUTPUT);
  pinMode(pinImageStop, OUTPUT);

  pinMode(pinSol_ttl, OUTPUT);
  pinMode(pinLickometer_ttl, OUTPUT);

  // setup capative touch sesnsor ---------------
  if (!cap.begin(0x5A)) {                   // if the sensor is not detected
    Serial.println("MPR121 not detected!"); // print warning (and crash python program)
    while (1);
  }

 // wait for serial command before initating session---------------------------------------------------------------------
  if(session_serial_initiate){
    while (Serial.available() <= 0) {} // wait for serial input to start session
  }

 // save start time and send ttl to initate scope
  ts_start=millis();  
  digitalWrite(pinImageStart, HIGH);
  delay(100);
  digitalWrite(pinImageStart, LOW);
}


/// void loop ____________________________________________________________________________________________________________________________________________________________________
// _______________________________________________________________________________________________________________________________________________________________________________
void loop() {
// background functions (run constantly independent of task events)--------------------------------------------
 // generate timestamp ---------------------------
  ts=millis()-ts_start;

 // detach break servo ---------------------------
  if(ts >= detach_servo_break_ts && detach_servo_break_ts!=0){
    servo_break.detach();
    detach_servo_break_ts = 0;
  }

 // detach spout servo ---------------------------
  if(ts >= detach_servo_retract_ts && detach_servo_retract_ts!=0){
    servo_retract.detach();
    detach_servo_retract_ts = 0;
  }

 // close solenoids---------------------------
  if(ts>=ts_sol_offset && ts_sol_offset!=0){                 // if time is after solenoid offset time
    digitalWrite(pinSol, LOW);                               // close solenoid
    Serial.print(14); Serial.print(" "); Serial.println(ts); // print sol offset
    ts_sol_offset = 0;    // reset solenoid offset time to close if statement
    }

 // turn off ttls for external time stamps ------------------------
 // lick---
  if(ts>=ts_lickomter_ttl_off && ts_lickomter_ttl_off!=0){
    digitalWrite(pinLickometer_ttl,LOW); // write ttl low
    ts_lickomter_ttl_off = 0;            // reset off time to close if statement
  }
  
 // solenoid---
  if(ts>=ts_sol_ttl_off && ts_sol_ttl_off!=0){
    digitalWrite(pinSol_ttl,LOW);  // write ttl low
    ts_sol_ttl_off = 0;            // reset off time to close if statement
  }

  
// session initialization (runs once at start) -----------------------------------------------------------------
  if(first_loop){
    Serial.print(1);   Serial.print(" "); Serial.println(ts);             // print start session
    Serial.print(127); Serial.print(" "); Serial.println(trial_count);            // print session duration
    Serial.print(104); Serial.print(" "); Serial.println(session_retract);             // print session_retract

    Serial.print(128); Serial.print(" "); Serial.println(min_iti);                   // print min_delay
    Serial.print(129); Serial.print(" "); Serial.println(max_iti);                   // print max_delay
    
    Serial.print(115); Serial.print(" "); Serial.println(access_time);                 // print access_time
    Serial.print(116); Serial.print(" "); Serial.println(sol_duration);                // print sol_duration
    Serial.print(117); Serial.print(" "); Serial.println(num_sol);                     // print num_sol
    Serial.print(118); Serial.print(" "); Serial.println(inter_sol_time);              // print inter_sol_time

    Serial.print(221); Serial.print(" "); Serial.println(tone_freq_cs_plus);                   // print tone_freq_cs_plus
    Serial.print(220); Serial.print(" "); Serial.println(tone_freq_cs_minus);                  // print tone_freq_cs_minus
    
    Serial.print(122); Serial.print(" "); Serial.println(tone_duration);               // print tone_duration
    

   // retract spout
    if(session_retract){ 
      servo_retract.attach(pinServo_retract); 
      Serial.print(15); Serial.print(" "); Serial.println(ts);      
      servo_retract.write(servo_retract_retracted_deg);
    }
    
   // allow time for servo travel and detach
    delay(500); 
    servo_retract.detach();


   // set and print first trial id
    trial_id = trial_order[0];
    Serial.print(222);      Serial.print(" "); Serial.println(ts);
    Serial.print(trial_id); Serial.print(" "); Serial.println(ts);

   // set time for first delivery
    ts_pre_access_start = ts + random(min_iti, max_iti);           // calculate first trial start

   first_loop = 0;

  }

// licking-----------------------------------------------------------------------------------------------------
 // check state of sensor to see if it is currently touched
  currtouched = cap.touched(); 

 // check to see if touch onset occured
  for (uint8_t i=1; i<2; i++) { // for each sensor (change the maximum i if more touch sensors are added)
    if ((currtouched & _BV(i)) && !(lasttouched & _BV(i)) ) { // if touched now but not previously
      lick=1;                                                 // flag lick
    }
  }

 // save current state for comparision with next state
  lasttouched = currtouched; 

 // programed consequences to licking
  if (lick==1){ // if lick has occured
      Serial.print(31); Serial.print(" "); Serial.println(ts); // print lick onset
      digitalWrite(pinLickometer_ttl,HIGH);                    // turn on ttl for external ts
      ts_lickomter_ttl_off = ts + ttl_duration;                // set ttl offset time

      lick=0; // reset lick flag to close if statement
  }

 // schedule of magazine training --------------------------------------------------------------
 // play tone prior to access period----
  if(ts >= ts_pre_access_start && ts_pre_access_start != 0){
    if(trial_id == 0 && playTone){tone(pinSpeaker, tone_freq_cs_plus,  tone_duration);} // start cs+ tone
    if(trial_id == 1 && playTone){tone(pinSpeaker, tone_freq_cs_minus, tone_duration);} // start cs- tone

    ts_access_start = ts + tone_pre_access_time; // set time for when the access time / aversive conditioning 
    ts_pre_access_start = 0;
    
  }
 // start access period------------
  if(ts >= ts_access_start && ts_access_start != 0){
    if(session_retract){fun_servo_retract_extended();}  // extend spout
    ts_servo_retract_retracted = ts + access_time;      // set time for retract spout
    ts_sol_onset = ts;                                  // set solenoid onset time to current time
    ts_access_start = 0;
    
  }

 // open solenoid num_sol times----
  if(ts >= ts_sol_onset && ts_sol_onset != 0 &&  count_sol < num_sol){
    if(trial_id == 0){ // if cs+
      digitalWrite(pinSol,HIGH);          // open solenoid
      ts_sol_offset = ts + sol_duration;  // set solenoid close time
      
      Serial.print(18); Serial.print(" "); Serial.println(ts); // print sol onset
      
      digitalWrite(pinSol_ttl,HIGH);      // turn on ttl for solenoid onset 
      ts_sol_ttl_off = ts + ttl_duration; // set ttl offset time
    }

    ts_sol_onset = ts + inter_sol_time + sol_duration; // set next solenoid onset time
    count_sol++;                                       // increase counter for solenoid
  }

 // end access period--------------
  if(ts >= ts_servo_retract_retracted && ts_servo_retract_retracted != 0){
    if(session_retract){fun_servo_retract_retracted();} // retract spout
    ts_servo_retract_retracted = 0;                     // reset retract time to close if statement

   // reset various variables and counters
    ts_sol_onset = 0;
    count_sol = 0;
    
    count_trial = count_trial + 1;
    trial_id = trial_order[count_trial];
    Serial.print(222);      Serial.print(" "); Serial.println(ts);
    Serial.print(trial_id); Serial.print(" "); Serial.println(ts);

    ts_pre_access_start = ts + random(min_iti, max_iti);           // calculate first trial start
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


// _______________________________________________________________________________________________________________________________________________________________________________
/// functions & interupts_________________________________________________________________________________________________________________________________________________________

// servos ----------------------------------------------------------------
void fun_servo_break_engage(){ //-----------------------------------------
  servo_break.attach(pinServo_break);  
  servo_break.write(servo_break_engaged_deg);
  Serial.print(11); Serial.print(" "); Serial.println(ts);            
  detach_servo_break_ts = ts + detach_servo_break_step;
}

void fun_servo_break_disengage(){ //--------------------------------------
  servo_break.attach(pinServo_break);  
  servo_break.write(servo_break_disengaged_deg);
  Serial.print(12); Serial.print(" "); Serial.println(ts);            
  detach_servo_break_ts = ts + detach_servo_break_step;
}

void fun_servo_retract_extended(){ //-------------------------------------
  servo_retract.attach(pinServo_retract);  
  servo_retract.write(servo_retract_extended_deg);
  Serial.print(13); Serial.print(" "); Serial.println(ts);            
  detach_servo_retract_ts = ts + detach_servo_retract_step;
}

void fun_servo_retract_retracted(){ //------------------------------------
  servo_retract.attach(pinServo_retract);  
  servo_retract.write(servo_retract_retracted_deg);
  Serial.print(15); Serial.print(" "); Serial.println(ts);            
  detach_servo_retract_ts = ts + detach_servo_retract_step;
}

/// end session -------------------------------------------------------------------------------------------
void fun_end_session() {
  digitalWrite(pinImageStop, HIGH);
  servo_break.attach(pinServo_break);  
  servo_break.write(servo_break_engaged_deg);
  Serial.print(11); Serial.print(" "); Serial.println(ts);     

  servo_retract.attach(pinServo_retract);  
  servo_retract.write(servo_retract_retracted_deg);
  Serial.print(15); Serial.print(" "); Serial.println(ts); 
  
  delay(500);
  
  digitalWrite(pinImageStop, LOW); 
  servo_break.detach();  
  servo_retract.detach();  
  
  Serial.print(0); Serial.print(" "); Serial.println(ts);    // print end of session                 
  while(1){}                               //  Stops executing the program
}

  
 
