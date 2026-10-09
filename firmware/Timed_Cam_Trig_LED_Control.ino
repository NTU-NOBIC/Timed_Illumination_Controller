/*
Timed Illumination Controller
Arduino Teensy 4.0 MCU as master to drive TRIGGER OUT signal to Hamamatsu ORCHA Flash Camera EXT TRIGGER input to control the camera exposure in sync with two LED control outputs.
  - Ch-A (active HIGH)
  - Ch-B (active HIGH)
  - only one channel active at a time. (Not possible to turn on both channel at the same time)
  - MCU detects if camera is responding to TRIGGER_OUT signal by monitoring TRIGGER READY signal (CAM_TRIG_READY) from Camera
  - A frame count is incremented when a valid CAM_TRIG_READY is detected
  - t1-t8 and ts timing, total cycle no, and active channel can be saved into EEPROM
  - ch-A and ch-B can be manually turned-on and off, one at a time.
  - UART Baudrate: 115200
*/

#include <IntervalTimer.h>
#include <EEPROM.h>

#define DEVICE_NAME "SCELSE/NOBIC - Timed Illumination Controller, ver 1.0.0 (Arduino, Teensy 4.0)"
//#define DEBUG_PIN_OUT 2     // Pin 2, output pin, software debugging use only
#define CAM_TRIG_OUT 9      // Pin 9, output pin, Trigger Signal to camera EXT TRIGGER input to start capturing image
#define CAM_TRIG_READY 10   // Pin 10, input Pin, Camera TRIGGER_READY signal from camera
#define GATE_A LED_BUILTIN  // Pin 13, digital output, ch-A MOSFET GATE, active HIGH  (used by on-board LED as well)
#define GATE_B 14           // Pin 14, digital output, ch-B MOSFET GATE, active HIGH
#define TRIG_RDY_AFTER_EXPO 15000  // Processing Time (in us) of camera after exposure, ie CAM_TRIG_READY goes HIGH again after Exposure ends. 10ms - 14ms
#define CAM_TRIG_READY_WAIT_TIMEOUT 15000000 // Start of test, 15s wait time for CAM_TRIG_READY to go HIGH

/* Time intervals definition:
      <----------------- Cycle Duration (Cycle m) ---------------------><-- (Cycle m+1) ....
         Frame n                   Frame (n+1)
      +----------+             +----------------+                      +-
      |    t1    |      t2     |       t3       |          t4          |
      |    xx ms |      xx ms  |       xx ms    |          xx ms       |
 _____|          |_____________|                |______________________|
 <--- LED Ch-A ON ----> ^ <------ LED Ch-B ON ------->     ^      <-----
 < t5 >          < t6 > | < t7 >                < t8 >     |      < t5 >
             t_wait1 ---+                        t_wait2---+
Users specify timing in ms, but timing variables (unsigned long) are in us for better timing control.
unsigned long is 32-bit for Teency ==> max time = 4,294,967,295us = 71.5min
*/
volatile unsigned long t1 = 10000;           // 10ms
volatile unsigned long t2 = 15000;           // 15ms
volatile unsigned long t3 = 40000;           // 40ms
volatile unsigned long t4 = 100000;          // 100ms
volatile unsigned long t5 = 15;              // 0ms;  put t5=15us for t5=0ms setting to cater for MCU response time
volatile unsigned long t6 = 0;               // 0ms;  put t6= 0us for t6=0ms setting
volatile unsigned long t7 = 15;              // 0ms;  put t7=15us for t7=0ms setting to cater for MCU response time
volatile unsigned long t8 = 0;               // 0ms;  put t8= 0us for t8=0ms setting
volatile unsigned long ts = 10000;           // 10ms; wait time for end of TRIG_RDY signal before switch off the LED
volatile unsigned long t_cycle = t1 + t2 + t3 + t4;
volatile long t_wait1 = t2 - t6 - t7;
volatile long t_wait2 = t4 - t8 - t5;
volatile unsigned long cycle_Total = 1000;               // Total No of loop to run  under start_test_Exposure_LED_timed().  One loop could be 1 (if t1=0 or t3=0) or 2 images taken.
unsigned long cycleNo = 0;
unsigned long previousMicros = 0;               // variable to handle overflow of micros() function.
volatile bool ch = false;                       // Toggle between ch-A (false) and ch-B (true)
bool streamingMode = false;                     // streaming mode status: 0=off, 1: on
char msgbuf[4096];                              // line buffer to print messages
IntervalTimer pulseTimer;                       // use of timer to detect pulse width
IntervalTimer cam_ExpoTimer;                    // use of timer to time if camera starts/stops exposure
volatile unsigned long pulseStartTime = 0;      // start time of falling edge, in micro seconds

// Refer 'void init_for_new_test()' for refreshed values of following variables before new test
volatile bool on_gr_red[] = {0, 0};    // Enable LEDs {Ch-A, Ch-B}. 0=Disable, 1=Enable
volatile long frameNo = 0;             // count of valid pulse on CAM_TRIG_READY pin, calculating successful captured frame by camera
unsigned long trig_rdy_width_min[] = {0, 0};  // Minimum Pulse width (us) on CAM_TRIG_READY pin for Ch-A and Ch-B
unsigned long trig_rdy_width_max[] = {0, 0};  // Maximum Pulse width (us) on CAM_TRIG_READY pin for Ch-A and Ch-B
volatile bool measuringTrigRdy = false;       // refer handleCamTrigReadyFalling() on definition of this flag
bool strmTrigRdyFalling = false;              // Streaming Mode, flag for CAM_TRIG_READY signal falling edge successful detection
// end of 'void init_for_new_test()' variable list


// Extract and print hours, minutes, and seconds in hhmmss.xxx
void print_hhmmss(unsigned long duration_ms, unsigned long totalLoops) {
    double total_duration_s = duration_ms * totalLoops / 1000;
    int hours = (int)(total_duration_s / 3600);
    int minutes = (int)((total_duration_s - (hours * 3600)) / 60);
    double seconds = ( (duration_ms * totalLoops) % 60000) / 1000.0;
    sprintf(msgbuf, "%02d:%02d:%05.3f s", hours, minutes, seconds);
    Serial.print(msgbuf);
}

/* 
CAM_TRIG_READY signal from Camera vs 'mmeasuringTrigRdy' flag and 'frameNo' count.
      |  frameNo = n   |         frameNo = n+1         |
------+                +-------+                       +---------------
      |measuringTrigRdy|       |                       |
=false|   =true        | =false|        =true          |  =false
      |________________|       |_______________________|
              --->~10ms<---                   --->~10ms<---

TRIG_OUT signal to EXT TRIGGER input of camera
      +----------+             +----------------+
      |          |             |                |
      |    t1    |      t2     |       t3       |          t4          
 _____|          |_____________|                |______________________

Definition: Presence of rising edge of CAM_TRIG_READY after TRIG_OUT is defined as camera has responded and completed one frame of image capture.
  . measuringTrigRdy=true starts at falling edge of TRIGGER READY signal   ==> Camera sets TRIGGER READY signal low.
  . measuringTrigRdy=true means TRIG_OUT signal to camera is active.  Camera has been triggered to take image.
  . measuringTrigRdy=true ends ~10ms (10ms is camera processing time) after falling edge of TRIG_OUT signal,
                       ie ends ~10ms after end of exposure.                ==> Camera sets TRIGGER READY signal high
  . absence of this rising edge within trig_rdy_width_min and trig_rdy_width_max means camera is not responding to TRIG_OUT
*/

// ISR to handle the falling edge of CAM_TRIG_READYtrig_rdy_width_min
void handleCamTrigReadyFalling() {
  if (!measuringTrigRdy) {
    measuringTrigRdy = true;
    pulseStartTime = micros();
    attachInterrupt(digitalPinToInterrupt(CAM_TRIG_READY), handleCamTrigReadyRising, RISING);
  }
}

// ISR to handle the rising edge of CAM_TRIG_READY
void handleCamTrigReadyRising() {
  unsigned long pulseEndTime = micros();
  unsigned long pulseDuration = pulseEndTime - pulseStartTime;
  // Detect camera has captured the frame, ie if the pulse length is within trig_rdy_width_min and trig_rdy_width_max
  if (pulseDuration >= trig_rdy_width_min[ch] && pulseDuration <= trig_rdy_width_max[ch]) {
      frameNo++;
  } else {  // Show abnormaly timing only if test is running.
      Serial.print("\nAlert.  End of Camera Trig Ready signal is arriving late.\n");
      sprintf(msgbuf, "Pulse Duration, min, max : %lu, %lu, %lu\n", pulseDuration, trig_rdy_width_min[ch], trig_rdy_width_max[ch]);
      Serial.print(msgbuf);
  }
  // Reset for the next pulse measurement;  Indication of camera has captured this frame
  measuringTrigRdy = false;
  attachInterrupt(digitalPinToInterrupt(CAM_TRIG_READY), handleCamTrigReadyFalling, FALLING);
}

void handleCamTrigReadyFalling_Streaming() {
  strmTrigRdyFalling = true;
  frameNo++;
  on_gr_red[ch] = 1;

}

void gateA_on() {
    digitalWrite(GATE_B, LOW);   // guarantee B is OFF first
    digitalWrite(GATE_A, HIGH);
}

void gateA_off() {
    digitalWrite(GATE_A, LOW);
}

void gateB_on() {
    digitalWrite(GATE_A, LOW);   // guarantee A is OFF first
    digitalWrite(GATE_B, HIGH);
}

void gateB_off() {
    digitalWrite(GATE_B, LOW);
}

void saveToEEPROM(unsigned int memblock) {
  int address = memblock * 100;

  unsigned long temp_t1, temp_t2, temp_t3, temp_t4, temp_t5;
  unsigned long temp_t6, temp_t7, temp_t8, temp_ts, temp_cycle_Total;
  bool tempCh;

  noInterrupts();
  temp_t1 = t1;
  temp_t2 = t2;
  temp_t3 = t3;
  temp_t4 = t4;
  temp_t5 = t5;
  temp_t6 = t6;
  temp_t7 = t7;
  temp_t8 = t8;
  temp_ts = ts;
  temp_cycle_Total = cycle_Total;
  tempCh = ch;
  interrupts();

  EEPROM.put(address, temp_t1); address += sizeof(temp_t1);
  EEPROM.put(address, temp_t2); address += sizeof(temp_t2);
  EEPROM.put(address, temp_t3); address += sizeof(temp_t3);
  EEPROM.put(address, temp_t4); address += sizeof(temp_t4);
  EEPROM.put(address, temp_t5); address += sizeof(temp_t5);
  EEPROM.put(address, temp_t6); address += sizeof(temp_t6);
  EEPROM.put(address, temp_t7); address += sizeof(temp_t7);
  EEPROM.put(address, temp_t8); address += sizeof(temp_t8);
  EEPROM.put(address, temp_ts); address += sizeof(temp_ts);
  EEPROM.put(address, temp_cycle_Total); address += sizeof(temp_cycle_Total);
  EEPROM.put(address, tempCh); address += sizeof(tempCh);
}
void readFromEEPROM(unsigned int memblock) {
  int address = memblock * 100;

  unsigned long temp_t1, temp_t2, temp_t3, temp_t4, temp_t5;
  unsigned long temp_t6, temp_t7, temp_t8, temp_ts, temp_cycle_Total;
  bool tempCh;

  EEPROM.get(address, temp_t1);           address += sizeof(temp_t1);
  EEPROM.get(address, temp_t2);           address += sizeof(temp_t2);
  EEPROM.get(address, temp_t3);           address += sizeof(temp_t3);
  EEPROM.get(address, temp_t4);           address += sizeof(temp_t4);
  EEPROM.get(address, temp_t5);           address += sizeof(temp_t5);
  EEPROM.get(address, temp_t6);           address += sizeof(temp_t6);
  EEPROM.get(address, temp_t7);           address += sizeof(temp_t7);
  EEPROM.get(address, temp_t8);           address += sizeof(temp_t8);
  EEPROM.get(address, temp_ts);           address += sizeof(temp_ts);
  EEPROM.get(address, temp_cycle_Total);  address += sizeof(temp_cycle_Total);
  EEPROM.get(address, tempCh);            address += sizeof(tempCh);

  noInterrupts();
  t1 = temp_t1;
  t2 = temp_t2;
  t3 = temp_t3;
  t4 = temp_t4;
  t5 = temp_t5;
  t6 = temp_t6;
  t7 = temp_t7;
  t8 = temp_t8;
  ts = temp_ts;
  cycle_Total = temp_cycle_Total;
  ch = tempCh;
  interrupts();
}

// Function to read an unsigned long (32-bit) value from EEPROM
unsigned long readUnsignedLong(unsigned int address) {
  unsigned long value = 0;
  for (int i = 0; i < 4; i++) {
    value |= ((unsigned long)EEPROM.read(address + i)) << (8 * i);
  }
  return value;
}


// Print current status;  '?' command
void print_currentStatus() {
    sprintf(msgbuf, "\n\
Current Status: (type 'help' to list available commands)\n\
      Active Channel : %s\n\
      Channel A      : %s\n\
      Channel B      : %s\n",\
      ch ? "ch B" : "ch A",\
      on_gr_red[0] ? "on" : "off",\
      on_gr_red[1] ? "on" : "off");
    Serial.print(msgbuf);
    show_timing_setting();
}

// Reset/Re-initialised to start run a new test
void init_for_new_test() {
  on_gr_red[0] = 0;
  on_gr_red[1] = 0;
  gateA_off();
  gateB_off();
  frameNo = 0;
  trig_rdy_width_min[0] = t1;                                 // Minimum Pulse width (us) on CAM_TRIG_READY pin for Ch-A
  trig_rdy_width_max[0] = t1 + TRIG_RDY_AFTER_EXPO + 2000;    // Maximum Pulse width (us) on CAM_TRIG_READY pin for Ch-A
  trig_rdy_width_min[1] = t3;                                 // Minimum Pulse width (us) on CAM_TRIG_READY pin for Ch-B
  trig_rdy_width_max[1] = t3 + TRIG_RDY_AFTER_EXPO + 2000;    // Maximum Pulse width (us) on CAM_TRIG_READY pin for Ch-B
  measuringTrigRdy = false;
  strmTrigRdyFalling = false;
}


void setup() {
  // Set UART baudrate to 115200
  Serial.begin(115200);

  // Configure GATE_A pin as output
  pinMode(GATE_A, OUTPUT);  
  digitalWrite(GATE_A, LOW);    // LED OFF initially

  // Configure GATE_B pin as output
  pinMode(GATE_B, OUTPUT); 
  digitalWrite(GATE_B, LOW);    // LED OFF initially

  // Configure CAM_TRIG_READY pin as input with pull-up resistor, and attach falling edge to ISR handleCamTrigReadyFalling() when required
  pinMode(CAM_TRIG_READY, INPUT_PULLUP);
  
  // Trigger Out signal to camera, active HIGH
  pinMode(CAM_TRIG_OUT, OUTPUT);
  digitalWrite(CAM_TRIG_OUT, LOW);

  // Initialise a bunch of variables
  init_for_new_test();

  // Load default timing from EEPROM
  readFromEEPROM(0);
  
  // Say Hello to the world
  sprintf(msgbuf, "\n%s\n", DEVICE_NAME);
  Serial.print(msgbuf);
}


// Streaming mode - ISR will set strmTrigRdyFalling (to true) on falling edge detection of CAM_TRIG_READY.
// As long as strmTrigRdyFalling continues to be set, the timer is refreshed every ts+2550 us and LED is kept on, frameNo countinng continues.  
// If the timer expires, the LED is off and Total frameNo is printed out and the loop is auto-reinitialised and standby for next new test.

// Time out on presence of CAM_TRIG_READY signal (EXPOSURE), turn off LED
void timerCallback() {
  printf("\nEnd of streaming, LED turned OFF\n");
  on_gr_red[ch] = 0;
  cam_ExpoTimer.end();
  frameNo--;  // Last falling edge of TRIG_OUT_READY is end of streaming, not new frame count.
  if (frameNo == 0) {
    Serial.print("  Time out waiting for response from Camera.\n");
    Serial.print("  Likely exposure timing (ts) in microcontroller does not match with MicroManager setting.  Use 'ts <value>' command to set correctly the exposure timing\n");
  }
  sprintf(msgbuf, "LED ON time Total Frames: %ld (Note: Different from the total images saved by the camera)\n", frameNo);
  Serial.print(msgbuf);

  // Reset back the frameNo, and standby for next streaming cycles
  frameNo = 0;  // Reset LED On time Total Frames
  Serial.print("\n\n\nStandby to turn ON LED when camera runs...\n");
  Serial.print("       0: ");
}

// "stm"  Continuous streaming (Camera exposure control is set by MicroManager/Camera)
// CAM_TRIG_READY pin is used to detect camera image capturing activity to turn ON/OFF the LED.
// Camera trigger source is configured as internal trigger in this mode, with it's output pin configured as "EXPOSURE" functionality

void set_streaming_mode() {
  // initialise all variables
  init_for_new_test();

  // Attached ISR - CAM_TRIG_READY pin (EXPOSURE signal)
  attachInterrupt(digitalPinToInterrupt(CAM_TRIG_READY), handleCamTrigReadyFalling_Streaming, FALLING);

  // Status for users attention
  Serial.print("\nContinuous Streaming Mode\n");
  Serial.print("(Use 'stm off' command to stop auto-LED operation, or stop the streaming mode from MicroManager)\n");
  sprintf(msgbuf, "  Current active channel : ch %c\n", ch ? 'B' : 'A');
  Serial.print(msgbuf);
  sprintf(msgbuf, "      Exposure time (ts) : %lu ms\n", ts/1000);
  Serial.print(msgbuf);
  Serial.print("\nStandby to turn ON LED when camera runs...\n");
  Serial.print("       0: ");
}


// "r"  Run test
void start_test_Exposure_LED_timed() {
  unsigned long startTime;
  bool ch_before_test;
  long frame_expected_total;
  bool cam_trig_ready_status;

  // attach ISR - CAM_TRIG_READY pin (TRIGGER_READY signal)
  attachInterrupt(digitalPinToInterrupt(CAM_TRIG_READY), handleCamTrigReadyFalling, FALLING);


  // initialise all variables
  ch_before_test = ch;
  init_for_new_test();

  // comments to users
  //show_timing_setting();
  Serial.print("\nLegend:\n  '.' : EXT_TRIGGER signal to Camera generated\n  'A' : Frame captured, Ch-A fluorescence\n  'B' : Frame captured, Ch-B fluorescence\n");
  Serial.print("Press 'q' and <Enter> to abort.\n");
  Serial.print("Timed Exposure cycles, waiting for camera...\n");

  // wait for the camera to be ready, ie CAM_TRIG_OUT == HIGH
  startTime = micros();
  cam_trig_ready_status = digitalRead(CAM_TRIG_READY);
  while ( (cam_trig_ready_status == LOW) && (micros() - startTime < CAM_TRIG_READY_WAIT_TIMEOUT) ) {
    //  check if user enters 'q' to abort while waiting for camera
    if (Serial.available()) {
      String str = Serial.readStringUntil('\n');
      char* cmd = (char*)str.c_str();
      if (strcmp(cmd, "q") == 0)  {
        Serial.print("\nUser aborted.\n");
        // Detach ISR
        detachInterrupt(digitalPinToInterrupt(CAM_TRIG_READY));
        ch = ch_before_test;  // restore back the active channel before start of test
        return;
      }
      else {  // provide hint if user enters other unrelated command
        Serial.print("\nPress 'q' and <Enter> to abort.\n");
      }
    }
    delay(10);
    // read CAM_TRIG_READY status 
    cam_trig_ready_status = digitalRead(CAM_TRIG_READY);
  }
  if (cam_trig_ready_status == LOW) {
    Serial.print("\nTime out while waiting for Camera to get ready to capture image.  Aborted.\n");
    detachInterrupt(digitalPinToInterrupt(CAM_TRIG_READY));
    gateA_off();
    gateB_off();
    ch = ch_before_test;
    return;
  }

  for (cycleNo = 0; cycleNo < cycle_Total; cycleNo++) {
    startTime = micros();
    if (startTime < previousMicros) startTime += 4294967295;  // Handle overflow
    previousMicros = startTime;
    
    if (cycleNo %25 == 0){   // header (cycle no) and new line every 50 cycles, else just print a dot
      sprintf(msgbuf,"\n%8lu: ", cycleNo);
      Serial.print(msgbuf);
    }

    if (t1 != 0) {
      // Step 1: starts, GATE_A high
      gateA_on();

      // Step 2: t5 later, CAM_TRIG_OUT high;
      while (micros() - startTime < t5) {
        if (micros() < previousMicros) startTime += 4294967295; // Handle overflow
        previousMicros = micros();
      }
      if (measuringTrigRdy)  {   // Camera has not completed nor responded to earlier CAM_TRIG_OUT signal
        if (frameNo == 0) {
          Serial.print("\nError: Camera is not responding to 'EXT_TRIGGER' signal.\n       Likely camera is set to 16-bit with low t2/t4 values or,\n       or use 8-bit with higher t2/t4 values.\n       Test Aborted.\n");
        } else {
          Serial.print("\nError: Total cycle not completed and camera is not responding to 'EXT TRIGGER' signal. Test Aborted.\n");
          if (t3 ==0) frameNo--;
        }
        break;
      }
      digitalWrite(CAM_TRIG_OUT, HIGH);     // current frame, trigger CAM_TRIG_OUT of Ch-A.
      if (t3 != 0) {                                // Not single frame per cycle (t 10/11/20/30)
        Serial.print((frameNo != 0) ? "B." : ".");  // indicates <previous> frame captured successfully     
      } else {                                      // Single frame per cycle (t 10/11/0/0)
        Serial.print((frameNo != 0) ? "A." : ".");  // indicates <previous> frame captured successfully
      }
      ch = 0;   // Ch-A
  
      // Step 3: t1 later, CAM_TRIG_OUT low
      while (micros() - startTime < t5 + t1) {
        if (micros() < previousMicros) startTime += 4294967295; // Handle overflow
        previousMicros = micros();
      }
      digitalWrite(CAM_TRIG_OUT, LOW);

      // Step 4: t6 later, GATE_A low
      while (micros() - startTime < t5 + t1 + t6) {
        if (micros() < previousMicros) startTime += 4294967295; // Handle overflow
        previousMicros = micros();
      }
      gateA_off();

      // Wait for t_wait1ch 
      while (micros() - startTime < t5 + t1 + t6 + t_wait1) {
        if (micros() < previousMicros) startTime += 4294967295; // Handle overflow
        previousMicros = micros();
      }
    }  // end of "if (t1 != 0)"

    if (t3 != 0) {
      // Step 5: t_wait1 later, GATE_B high
      gateB_on();


      // Step 6: t7 later, CAM_TRIG_OUT high;
      while (micros() - startTime < t5 + t1 + t6 + t_wait1 + t7) {
        if (micros() < previousMicros) startTime += 4294967295; // Handle overflow
        previousMicros = micros();
      }
      if ((frameNo == 0) && (t1 != 0)){
        Serial.print("\nError: Camera is not responding to 'EXT_TRIGGER' signal.\n       Likely camera is set to 16-bit with low t2/t4 values or,\n       or use 8-bit with higher t2/t4 values.\n       Test Aborted.\n");
        break;
      }
      if (measuringTrigRdy) {   // Camera has not completed nor responded to earlier CAM_TRIG_OUT signal
        Serial.print("\nError: Total cycle not completed and camera is not responding to 'EXT TRIGGER' signal. Test Aborted.\n");
        if (t1 == 0) frameNo--;
        break;
      }
      digitalWrite(CAM_TRIG_OUT, HIGH);
      if (t1 != 0) {                                // Not single frame per cycle (t 10/11/20/30)
        Serial.print((frameNo != 0) ? "A." : ".");  // indicates <previous> frame captured successfully  
      } else {                                      // Single frame per cycle (t 0/0/10/11)
        Serial.print((frameNo != 0) ? "B." : ".");  // indicates <previous> frame captured successfully
      }
      ch = 1;    // Ch-B
  
      // Step 7: t3 later, set CAM_TRIG_OUT low
      while (micros() - startTime < t5 + t1 + t6 + t_wait1 + t7 + t3) {
        if (micros() < previousMicros) startTime += 4294967295; // Handle overflow
        previousMicros = micros();
      }
      digitalWrite(CAM_TRIG_OUT, LOW);


      // Step 8: t8 later, set GATE_B low
      while (micros() - startTime < t5 + t1 + t6 + t_wait1 + t7 + t3 + t8) {     
        if (micros() < previousMicros) startTime += 4294967295; // Handle overflow
        previousMicros = micros();
      }
      gateB_off();

      // Wait for t_wait2
      while (micros() - startTime < t5 + t1 + t6 + t_wait1 + t7 + t3 + t8 + t_wait2) {  
        if (micros() < previousMicros) startTime += 4294967295; // Handle overflow
        previousMicros = micros();
      }
    }  // end of "if (t3 != 0)

    if (frameNo == 0) {
      Serial.print("\nError: Camera is not responding to 'EXT TRIGGER' signal.  Likely Camera is not put to acquisition mode. Test Aborted.\n");
      break;
    }

    //  check if user enters 'q' to abort
    if (Serial.available()) {
      String str = Serial.readStringUntil('\n');
      char* cmd = (char*)str.c_str();
      if (strcmp(cmd, "q") == 0)  {
        Serial.print("\nUser aborted.\n");
        sprintf(msgbuf, "Last Cycle No: %lu\n",cycleNo);
        Serial.print(msgbuf);
        delay(t4/1000 + 50);
        sprintf(msgbuf, "Last Frame No: %ld\n",frameNo);
        Serial.print(msgbuf);
        // Detach ISR
        detachInterrupt(digitalPinToInterrupt(CAM_TRIG_READY));
        ch = ch_before_test;  // restore back the active channel before start of test
        return;
      }
      else {  // provide hint if user enters other unrelated command
        Serial.print("\nPress 'q' and <Enter> to abort.\n");
      }
    }
  }
  // CycleNo loop completed

  // Switch off all LED's (in case of abnormality)
  gateA_off();
  gateB_off();

  // Compute the expected total frame no
  frame_expected_total = ( (t1 == 0) || (t3 == 0) )? cycle_Total : cycle_Total * 2;

// checks on Last FrameNo;  Not applicable if the cycle terminated half way due to MicroManager abort, or uM set total image count to lower than FrameNo count.
  delay((t3 != 0)? t4/1000 + 1 : t2/1000 + 1);      // Pause for a while, allowing the last FrameNo to complete.
  if (frameNo < frame_expected_total) {   // if not abort condition
      if (cycleNo == cycle_Total) Serial.print("\nError: Camera is not responding to 'EXT TRIGGER' signal on last frame/cycle. Test Aborted.\n");
  } else if (t3 != 0) {                                // Not single frame per cycle (t 10/11/20/30)
    Serial.print("B");  // indicates <previous> frame captured successfully     
  } else {                                      // Single frame per cycle (t 10/11/0/0)
    Serial.print("A");  // indicates <previous> frame captured successfully
  }
  sprintf(msgbuf, "\nTotal cycles completed: %lu/%lu\n", cycleNo, cycle_Total);
  Serial.print(msgbuf);
  sprintf(msgbuf, "Total Frames completed: %ld\n", frameNo);
  Serial.print(msgbuf);

  // Detach ISR 
  detachInterrupt(digitalPinToInterrupt(CAM_TRIG_READY));
  ch = ch_before_test;  // restore back the active channel before start of test
}

/*
      <----------------- Cycle Duration (Cycle m) ---------------------><-- (Cycle m+1) ....
         Frame n                   Frame (n+1)
      +----------+             +----------------+                      +-
      |    t1    |      t2     |       t3       |          t4          |
      |    xx ms |      xx ms  |       xx ms    |          xx ms       |
 _____|          |_____________|                |______________________|
 <--- LED Ch-A ON ----> ^ <------ LED Ch-B ON ------->     ^      <-----
 < t5 >          < t6 > | < t7 >                < t8 >     |      < t5 >
             t_wait1 ---+                        t_wait2---+
*/
void show_timing_setting() {
  Serial.print("\n");
  Serial.print("      <----------------- Cycle Duration (Cycle m) ---------------------><-- (Cycle m+1) ....\n");
  Serial.print("         Frame n                   Frame (n+1)\n");
  Serial.print("      +----------+             +----------------+                      +-\n");
  Serial.print("      |    t1    |      t2     |       t3       |          t4          |\n");
  sprintf(msgbuf, "      |%6lu ms |  %6lu ms  |   %6lu ms    |     %7lu ms       |\n", t1/1000, t2/1000, t3/1000, t4/1000);
  Serial.print(msgbuf);
  Serial.print(" _____|          |_____________|                |______________________|\n");
  Serial.print(" <--- LED Ch-A ON ----> ^ <------ LED Ch-B ON ------->     ^      <-----\n");
  Serial.print(" < t5 >          < t6 > | < t7 >                < t8 >     |      < t5 >\n");
  Serial.print("             t_wait1 ---+                        t_wait2---+\n");

  if ( (t1 == 0) || (t3 == 0) ){
    sprintf(msgbuf, "\n   Total Cycles : %lu (MicroManager Image Count = %lu)\n", cycle_Total, cycle_Total);
  } 
  else {
    sprintf(msgbuf, "\n   Total Cycles : %lu (MicroManager Image Count = %lu)\n", cycle_Total, cycle_Total*2);
  }
  Serial.print(msgbuf);

  sprintf(msgbuf, "             t1 : %7lu ms\n", t1/1000);
  Serial.print(msgbuf);

  sprintf(msgbuf, "             t2 : %7lu ms\n", t2/1000);
  Serial.print(msgbuf);

  sprintf(msgbuf, "             t3 : %7lu ms\n", t3/1000);
  Serial.print(msgbuf);

  sprintf(msgbuf, "             t4 : %7lu ms\n", t4/1000);
  Serial.print(msgbuf);

  sprintf(msgbuf, "             t5 :  %6lu ms (LED #1 pre turn-on time before t1)\n", t5/1000);
  Serial.print(msgbuf);

  sprintf(msgbuf, "             t6 :  %6lu ms (LED #1 post turn-on time after t1)\n", t6/1000);
  Serial.print(msgbuf);

  sprintf(msgbuf, "             t7 :  %6lu ms (LED #2 pre turn-on time before t3)\n", t7/1000);
  Serial.print(msgbuf);

  sprintf(msgbuf, "             t8 :  %6lu ms (LED #2 post turn-on time after t3)\n", t8/1000);
  Serial.print(msgbuf);

  sprintf(msgbuf, "             ts :  %6lu ms (streaming mode exposure time)\n", ts/1000);
  Serial.print(msgbuf);

  sprintf(msgbuf, " Cycle Duration : %7lu ms (t1+t2+t3+t4)\n", (t1+t2+t3+t4)/1000);
  Serial.print(msgbuf);

  sprintf(msgbuf, " LED #1 On time :  %6lu ms (t1+t5+t6)\n", (t5+t1+t6)/1000);
  Serial.print(msgbuf);

  sprintf(msgbuf, " LED #2 On time :  %6lu ms (t3+t7+t8)\n", (t7+t3+t8)/1000);
  Serial.print(msgbuf);

  if (t1 == 0) {
    sprintf(msgbuf, "        t_wait1 :       0 ms\n");
    Serial.print(msgbuf);

    sprintf(msgbuf, "        t_wait2 :  %6ld ms (t4-t8-t7); Ch-B LED #2 OFF to Ch-B LED #2 ON and vice versa\n", lround(t_wait2/1000.0));
    Serial.print(msgbuf);
  } else if (t3 == 0) {
    sprintf(msgbuf, "        t_wait1 :  %6ld ms (t2-t6-t5); Ch-A LED #1 OFF to Ch-A LED #1 ON and vice versa\n", lround(t_wait1/1000.0));
    Serial.print(msgbuf);

    sprintf(msgbuf, "        t_wait2 :       0 ms\n");
    Serial.print(msgbuf);
  } else {
    sprintf(msgbuf, "        t_wait1 :  %6ld ms (t2-t6-t7); Ch-A LED #1 OFF to Ch-B LED #2 ON\n", lround(t_wait1/1000.0));
    Serial.print(msgbuf);

    sprintf(msgbuf, "        t_wait2 :  %6ld ms (t4-t8-t5); Ch-B LED #2 OFF to Ch-A LED #1 ON\n", lround(t_wait2/1000.0));
    Serial.print(msgbuf);    
  }

  Serial.print("Total Test Time : ");
  print_hhmmss((t1+t2+t3+t4) / 1000, cycle_Total);
  Serial.print(" (hh:mm:ss; Cycle Duration * Total Cycles)\n");
}

//  Listen and fetch command if available on serial port
void listen_command() {
  if (Serial.available()) {
    String str = Serial.readStringUntil('\n');
    char* cmd = (char*)str.c_str();
    parse_command(cmd);
  }
}

void parse_command(char* cmd) {
  long t_wait_temp;
  char* tok;
  tok = strtok(cmd, " \n\r");  // space, newline, and carriage return as delimiters
  // "" or " " - no action.
  if (tok == NULL) {
    sprintf(msgbuf, "Empty command\n");
    Serial.print(msgbuf);
    return;  // Return early if no command is found
  }

  // "help" - list available command
  if (strcmp(tok, "help") == 0) {
    sprintf(msgbuf, "\nCommands:\n\
        ?   : Show current status and settings\n\
        ch? : Show current active channel\n\
        on? : Read channel status (on or off)\n\
        t?  : Read timing related settings\n\
        ts? : Read streaming mode exposure time\n\
        c?  : Show total cycles to run\n\
      stm?  : Show streaming mode - either on/off\n\
  c <value> : Set the Total Cycles to run.  Max value = 2,147,483,647 (2^31 - 1)\n\
         r  : Run preset camera exposure and LED ON/OFF cycles\n\
     stm on : Turn on  Streaming mode (LED is auto turned ON/OFF when camera runs)\n\
    stm off : Turn off Streaming mode\n\
 ts <value> : Set streaming mode exposure time in ms (ts >= 10ms to allocate\n\
              sufficient time to save images in streaming mode)\n\
  s <value> : Save to EEPROM memory 0-10, '0' as bootup default setting (eg: 's 0', 's 5').\n\
              Saved settings are t1-t8 and ts timing, Total Cycles, active channel\n\
  l <value> : load from EEPROM memory 0-10 (eg: 'l 0', 'l 1', 'l 5').\n\
              Loaded settings are t1-t8 and ts timing, Total Cycles, active channel\n\
t <t1>/<t2>/<t3>/<t4> : Set timing t1, t2, t3 and t4 in ms\n\
                        (eg: 't 10/11/40/100', 't 10/11/0/0', 't 0/0/40/11')\n\
p <t5>/<t6>/<t7>/<t8> : Set pre/post-exposure timing t5, t6, t7 and t8 in ms\n\
                        (eg: 'p 5/5/5/5', 'p 0/0/0/0', 'p 5/0/5/0')\n\
Manual LED operations :\n\
        A   : Set active channel to Ch-A; Ch-B is off\n\
        B   : Set active channel to Ch-B; Ch-A is off\n\
        on  : Turn on  LED of active channel\n\
        off : Turn off LED of active channel\n");
    Serial.print(msgbuf);
  
  // "?" - print current status
  } else if (strcmp(tok, "?") == 0) {
      print_currentStatus();
  
  // "ch?" - print current active channel
  } else if (strcmp(tok, "ch?") == 0) {
      Serial.print(ch ? "\nCurrent active channel: ch B\n" : "\nCurrent active channel: ch A\n"); // channel true is B, false is A

  // "on?" - print current status (on or off) of both channel
  } else if (strcmp(tok, "on?") == 0 || strcmp(tok, "off?") == 0) {
      Serial.print(on_gr_red[0] ? "\n  Channel A: on\n" : "\n  Channel A: off\n");
      Serial.print(on_gr_red[1] ? "  Channel B: on\n" : "  Channel B: off\n");

  // "A", "B" - set ch A or ch B as active channel
  } else if (strcmp(tok, "A") == 0 || strcmp(tok, "B") == 0) {
      ch = (strcmp(tok, "B") == 0); // channel B is ch=1
      on_gr_red[!ch] = 0;           // switch off unselected channel
      Serial.print(ch ? "\nChannel B set as active channel, and Channel A set to off\n" : "\nChannel A set as active channel, and Channal B set to off\n"); 

  // "on", "off" - set active channel to on or off
  } else if (strcmp(tok, "on") == 0 || strcmp(tok, "off") == 0) {
      on_gr_red[ch] = (strcmp(tok, "on") == 0);
      sprintf(msgbuf, "\nChannel A : %s\nChannel B : %s\n",\
            on_gr_red[0] ?   "on" : "off",
            on_gr_red[1] ?   "on" : "off");
      Serial.print(msgbuf);

  // "c <value>" - set total cycle to run
  } else if (strcmp(tok, "c") == 0) {
      tok = strtok(NULL, " \n\r");
      if (tok == NULL) {
        Serial.print("\nMissing total cycle value\n");
        return;
      }
      unsigned long val = strtol(tok, NULL, 10);
      cycle_Total = val;
      sprintf(msgbuf, "\nTotal Cycle is set to %lu\n", cycle_Total);
      Serial.print(msgbuf);

  // "t?" - print timing settings
  } else if (strcmp(tok, "t?") == 0) {
      show_timing_setting();

  // "t <t1>/<t2>/<t3>/<t4>" - Set t1, t2, t3 and t4 timing
  } else if (strcmp(tok, "t") == 0) {
      unsigned long val1, val2, val3, val4;
      tok = strtok(NULL, " ");
      if (tok == NULL) {
        Serial.print("\nMissing t1/t2/t3/t4 timing values\n");
        return;
      } else {
          sprintf(msgbuf, "\n%s %s\n", cmd, tok);
          Serial.print(msgbuf);
      }
  // <t1> timing
      char* subtok = strtok(tok, "/");
      if (subtok == NULL) {
        Serial.print("\nIncorrect or missing t1 timing values\n");
        return;
      }
      val1 = atoi(subtok);
      if ((val1 < 2) && (val1 != 0) ){
        Serial.print("\nInvalid t1 setting (must be >=2ms;  put 0ms to disable Ch-A)\n");
        return;
      } 
  // <t2> timing
      subtok = strtok(NULL, "/");
      if (subtok == NULL) {
        Serial.print("\nIncorrect or missing t2 timing values\n");
        return;
      }
      val2 = atoi(subtok);
      t_wait_temp = (val2 * 1000) - t6 - t7;
      if (val1 != 0) { // ignore t2<11ms and t_wait1<0 constraits if t1=0
        if (val2 < 11) {
          Serial.print("\nInvalid t2 setting (must be >=11ms, and >='t6+t7')\n");
          return;
        } else if (t_wait_temp < 0 ) {  // t_wait1 < 0
            sprintf(msgbuf, "\nInvalid t2 timing of %lu ms!!  t2 must be >='t6+t7'\n", val2);
            Serial.print(msgbuf);
            return;
        }
      }
  // <t3> timing
      subtok = strtok(NULL, "/");
      if (subtok == NULL) {
        Serial.print("\nIncorrect or missing t3 timing values\n");
        return;
      }
      val3 = atoi(subtok);
      if ((val3 < 2) && (val3 != 0) ){
        Serial.print("\nInvalid t3 setting (must be >=2ms;  put 0ms to disable Ch-B)\n");
        return;
      } 
  // <t4> timing
      subtok = strtok(NULL, "/");
      if (subtok == NULL) {
        Serial.print("\nIncorrect or missing t4 timing values\n");
        return;
      }
      val4 = atoi(subtok);
      t_wait_temp = (val4 * 1000) - t8 - t5;
      if (val3 != 0) { // ignore t4<11ms and t_wait2<0 constraits if t3=0
        if (val4 < 11) {   
          Serial.print("\nInvalid t4 setting (must be >=11ms, and >='t8+t5')\n");
          return;
        } else if (t_wait_temp < 0 ) {  // t_wait1 < 0
            sprintf(msgbuf, "\nInvalid t4 timing of %lu ms!!  t4 must be >='t8+t5'\n", val4);
            Serial.print(msgbuf);
            return;
        }
      }
      t1 = val1 * 1000;
      t2 = val2 * 1000;
      t3 = val3 * 1000;
      t4 = val4 * 1000;
      if (t1 == 0) {         // if Ch-A not used, pre and post timings (t5, t6) are not relevant.
        t5 = 0;
        t6 = 0;
      } else if (t5 == 0) {  // if t1!=0 and t5=0, need to add back 15us to t5.
          t5 = 15;
      }
      if (t3 == 0) {        // if Ch-B not used, pre and post timings (t7, t8) are not relevant.
        t7 = 0;
        t8 = 0;
      } else if (t7 == 0) {  // if t3!=0 and t7=0, need to add back 15us to t7.
          t7 = 15;
      }
      t_cycle = t1 + t2 + t3 + t4;
      if (t1 == 0) {              // if Ch-A is not used, skip t_wait1 and compute t_wait2 based on Ch-B timings only
        t_wait1 = 0;
        t_wait2 = t4 - t8 - t7;       // use Ch-B timings
      } else if (t3 == 0) {       // if Ch-B is not used, skip t_wait2 and compute t_wait1 based on Ch-A timings only
          t_wait1 = t2 - t5 - t6;     // use Ch-A timings
          t_wait2 = 0;
      } else {                    // Both Ch-A and Ch-B are used, compute t_wait1 and t_wait2 based on respective channel timings
          t_wait1 = t2 - t6 - t7;     // use Ch-A timings
          t_wait2 = t4 - t8 - t5;     // use Ch-B timings
      }
      show_timing_setting();

  // "p <t5>/<t6>/<t7>/<t8>" - Set t5, t6, t7 and t8 timing
  } else if (strcmp(tok, "p") == 0) {
      unsigned long val5, val6, val7, val8;
      tok = strtok(NULL, " ");
      if (tok == NULL) {
        Serial.print("\nMissing t5/t6/t7/t8 timing values\n");
        return;
      } else {
          sprintf(msgbuf, "\n%s %s\n", cmd, tok);
          Serial.print(msgbuf);
      }
  // <t5> timing
      char* subtok = strtok(tok, "/");
      if (subtok == NULL) {
        Serial.print("\nIncorrect or missing t5 timing values\n");
        return;
      }
      val5 = atoi(subtok);
      if (t1 == 0) {  // t5=0 if t1=0
        val5 = 0;
      } else if (t3 == 0) {
        t_wait_temp = t2 - t6 - (val5 * 1000) ;
        if (t_wait_temp < 0 ) { 
            sprintf(msgbuf, "\nInvalid t5 timing of %lu ms!!  t5 must be <=%lu ms ('t2-t6')\n", val5, (t2-t6)/1000);
            Serial.print(msgbuf);
            return;
        }
      } else {       // check other timings related to t5
        t_wait_temp = t4 - t8 - (val5 * 1000) ;
        if (t_wait_temp < 0 ) {  // t_wait2 < 0
            sprintf(msgbuf, "\nInvalid t5 timing of %lu ms!!  t5 must be <=%lu ms ('t4-t8')\n", val5, (t4-t8)/1000);
            Serial.print(msgbuf);
            return;
        }
      }
  // <t6> timing
      subtok = strtok(NULL, "/");
      if (subtok == NULL) {
        Serial.print("\nIncorrect or missing t6 timing values\n");
        return;
      }
      val6 = atoi(subtok);
      if (t1 == 0) {  // t6=0 if t1=0
        val6 = 0;
      } else if (t3 == 0) {
        t_wait_temp = t2 - t5 - (val6 * 1000) ;
        if (t_wait_temp < 0 ) { 
            sprintf(msgbuf, "\nInvalid t6 timing of %lu ms!!  t6 must be <=%lu ms ('t2-t5')\n", val5, (t2-t5)/1000);
            Serial.print(msgbuf);
            return;
        }
      } else {     // check other timings related to t6
        t_wait_temp = t2 - t7 - (val6 * 1000) ;
        if (t_wait_temp < 0 ) {  // t_wait1 < 0
            sprintf(msgbuf, "\nInvalid t6 timing of %lu ms!!  t6 must be <=%lu ms ('t2-t7')\n", val6, (t2-t7)/1000);
            Serial.print(msgbuf);
            return;
        }
      }
  // <t7> timing
      subtok = strtok(NULL, "/");
      if (subtok == NULL) {
        Serial.print("\nIncorrect or missing t7 timing values\n");
        return;
      }
      val7 = atoi(subtok);
      if (t3 == 0) { // t6=0 if t3=0
        val7 = 0;
      } else if (t1 == 0) {
        t_wait_temp = t4 - t8 - (val7 * 1000) ;
        if (t_wait_temp < 0 ) { 
            sprintf(msgbuf, "\nInvalid t7 timing of %lu ms!!  t7 must be <=%lu ms ('t4-t8')\n", val5, (t4-t8)/1000);
            Serial.print(msgbuf);
            return;
        }
      } else {     // check other timings related to t7
        t_wait_temp = t2 - t6 - (val7 * 1000) ;
        if (t_wait_temp < 0 ) {  // t_wait1 < 0
            sprintf(msgbuf, "\nInvalid t7 timing of %lu ms!!  t7 must be <=%lu ms ('t2-t6')\n", val7, (t2-t6)/1000);
            Serial.print(msgbuf);
            return;
        }
      }
  // <t8> timing
      subtok = strtok(NULL, "/");
      if (subtok == NULL) {
        Serial.print("\nIncorrect or missing t8 timing values\n");
        return;
      }
      val8 = atoi(subtok);
      if (t3 == 0) { // t8=0 if t3=0
        val8 = 0;
      } else if (t1 == 0) {
        t_wait_temp = t4 - t7 - (val8 * 1000) ;
        if (t_wait_temp < 0 ) { 
            sprintf(msgbuf, "\nInvalid t8 timing of %lu ms!!  t8 must be <=%lu ms ('t4-t7')\n", val5, (t4-t7)/1000);
            Serial.print(msgbuf);
            return;
        }
      } else {     // check other timings related to t8
          t_wait_temp = t4 - t5 - (val8 * 1000) ;
          if (t_wait_temp < 0 ) {  // t_wait2 < 0
              sprintf(msgbuf, "\nInvalid t8 timing of %lu ms!!  t8 must be <= %lu ms ('t4-t5')\n", val8, (t4-t5)/1000);
              Serial.print(msgbuf);
              return;
          }
      }
      // Recompute the related timings
      if (t1 != 0) t5 = (val5 == 0) ? 15 : val5 * 1000; // if Ch-A is used and user enters 0ms, set t5=15us.
      t6 = val6 * 1000;
      if (t3 != 0) t7 = (val7 == 0) ? 15 : val7 * 1000; // if Ch-B is used and user enters 0ms, set t7=15us.
      t8 = val8 * 1000;
      t_cycle = t1 + t2 + t3 + t4;
      if (t1 == 0) {              // if Ch-A is not used, skip t_wait1 and compute t_wait2 based on Ch-B timings only
        t_wait1 = 0;
        t_wait2 = t4 - t8 - t7;       // use Ch-B timings
      } else if (t3 == 0) {       // if Ch-B is not used, skip t_wait2 and compute t_wait1 based on Ch-A timings only
          t_wait1 = t2 - t5 - t6;     // use Ch-A timings
          t_wait2 = 0;
      } else {                    // Both Ch-A and Ch-B are used, compute t_wait1 and t_wait2 based on respective channel timings
          t_wait1 = t2 - t6 - t7;     // use Ch-A timings
          t_wait2 = t4 - t8 - t5;     // use Ch-B timings
      }
      show_timing_setting();

  // "ts <value>" - set ts timing
  } else if (strcmp(tok, "ts") == 0) {
      tok = strtok(NULL, " \n\r");
      if (tok == NULL) {
        Serial.print("\nMissing ts value\n");
        return;
      }
      long val = strtol(tok, NULL, 10);
      if (val < 10) {
          Serial.print("\nInvalid ts setting (must be >=10ms)\n");
          return;
      }
      ts = val * 1000;
      sprintf(msgbuf, "\nts is set to %lu ms\n", ts/1000);
      Serial.print(msgbuf);

  // "ts?" - read ts timing
  } else if (strcmp(tok, "ts?") == 0) {
      sprintf(msgbuf, "\nts = %lu ms\n", ts/1000);
      Serial.print(msgbuf);

  // "c?" - print total cycles
  } else if (strcmp(tok, "c?") == 0) {
      sprintf(msgbuf, "\nTotal Cycles : %lu\n", cycle_Total);
      Serial.print(msgbuf);

  // "stm?" - print total cycles
  } else if (strcmp(tok, "stm?") == 0) {
      Serial.print(streamingMode ?
                 "\nStreaming mode: ON\n" :
                 "\nStreaming mode: OFF\n");

  // "r" - reset or re-initialise setting, ready to start new test.  Print current status
  } else if (strcmp(tok, "r") == 0) {
    start_test_Exposure_LED_timed();

  // "stm on"  - Set to continuous streaming mode - LED is auto turned ON/OFF when camera runs
  // "stm off" - Off streaming mode - LED is turn OFF
  } else if (strcmp(tok, "stm") == 0) {
    tok = strtok(NULL, " \n\r");
    if (tok == NULL) {
      Serial.print("\nMissing 'on' or 'off'\n");
      return;
    }
    if (strcmp(tok, "on") == 0) {
      streamingMode = true;
      set_streaming_mode();
    } else if (strcmp(tok, "off") == 0) {
      streamingMode = false;
      Serial.print("\nStreaming mode off\n");
      // Detach ISR
      detachInterrupt(digitalPinToInterrupt(CAM_TRIG_READY));
      // Turn off all LED's
      on_gr_red[0] = 0;
      on_gr_red[1] = 0;
    } else {
      sprintf(msgbuf, "\nUnknown option '%s' (should be either 'on' or 'off')\n", tok);
      Serial.print(msgbuf);
    }

  // "s <value>" - save settings to EEPROM memory location 0-10: tl-t8 and ts timing, total cycle no
  } else if (strcmp(tok, "s") == 0) {
      tok = strtok(NULL, " \n\r");
      if (tok == NULL) {
        Serial.print("\nMissing EEPROM memory location (0-10)\n");
        return;
      }
      unsigned int memblock = strtol(tok, NULL, 10);
      if ( (memblock >= 0) && (memblock <= 10) ){
        saveToEEPROM(memblock);
        sprintf(msgbuf, "\n\nSettings are save to EEPROM memory location '%d': tl-t8 and ts timing, total cycle no.\n", memblock);
        Serial.print(msgbuf);
        readFromEEPROM(memblock);
        delay(200);
        show_timing_setting();
      } else {
        sprintf(msgbuf, "\nInvalid EEPROM memory location '%d'.  Use valid range of 0-10\n", memblock);
        Serial.print(msgbuf);
      }

  // "l <value>" - load settings from EEPROM memory location 0-10:  tl-t8 and ts timing, total cycle no
  } else if (strcmp(tok, "l") == 0) {
      tok = strtok(NULL, " \n\r");
      if (tok == NULL) {
        Serial.print("\nMissing EEPROM memory location (0-10)\n");
        return;
      }
      unsigned int memblock = strtol(tok, NULL, 10);
      if ( (memblock >= 0) && (memblock <= 10) ){
        readFromEEPROM(memblock);
        sprintf(msgbuf, "\n\nSettings are loaded from EEPROM memory location '%d': tl-t8 and ts timing, total cycle no.\n", memblock);
        Serial.print(msgbuf);
        show_timing_setting();
      } else {
        sprintf(msgbuf, "\nInvalid EEPROM memory location '%d'.  Use valid range of 0-10\n", memblock);
        Serial.print(msgbuf);
      }


  // others - unknown commands
  } else {
    sprintf(msgbuf, "\nUnknown command.  Type 'help' to list available commands\n");
    Serial.print(msgbuf);
  }
}


void loop() {
  listen_command(); // Listen and process remote control command, if any

  // only the selected channel can turn ON, and the other channel is forced LOW.
  digitalWrite(GATE_A, (ch == 0 && on_gr_red[0]) ? HIGH : LOW);
  digitalWrite(GATE_B, (ch == 1 && on_gr_red[1]) ? HIGH : LOW);

// Streaming mode - ISR will set strmTrigRdyFalling (to true) on falling edge detection of CAM_TRIG_READY.
// As long as strmTrigRdyFalling continues to be set, the timer is refreshed every ts+100000 us and LED is kept on, frameNo countinng continues.  
// If the timer expires, the LED is off and Total frameNo is printed out and the loop is auto-reinitialised and standby for next new test.
  if (strmTrigRdyFalling) {    // print out indication successful frame detected, and which LED is used
    if (frameNo %100 == 0) {  
      sprintf(msgbuf,"\n%8ld: ", frameNo);
      Serial.print(msgbuf);
    } else {
      Serial.print((ch) ? "B" : "A");
    }
    strmTrigRdyFalling = false;
    cam_ExpoTimer.begin(timerCallback, ts + 100000); // re-initialise timer for next frame
  }
}
