// ----------------------------------------------------
//     Hydraulic Control Board: Pump Control Code
// ----------------------------------------------------
// Author: Allyson Elise Chen
// Date Updated: 2025-11-17
//
// Description: 
//    Closed-loop control of three pumps using RPM feedback. Each pump is driven by a sinusoidal PWM signal. 
//
// Control Strategy: 
//    - First-order Infinite Impulse Response (IIR) filter
//    - Feedback (Proportional Gain) + Feedforward Control
//
// Hardware:
//    - ESP32 microcontroller
//    - 3 Pumps w/ ESC control
//    - 3 RPM Sensors 
//
// Notes: 
//    - Sinusoidal PWM signal parameters (amplitude, frequency, phase) can be adjusted
//    - Control Parameters (Kp, alpha, scaling factor) can be tuned 

// ======== LIBRARY ========
#include <ESP32Servo.h>

// ======== PUMP ESC OBJECTS ========
Servo ESC0; // pump #1 
Servo ESC1; // pump #2
Servo ESC2; // pump #3

// ======== SERIAL COMUNICATION ========
const int baudRate = 9600; // constant integer to set the baud rate for serial monitor

// ======== PARAMETERS: PWM SIGNAL [SINUSOID] ========
float pi = 3.14159;
float scale_amp = 0.5; // scaling factor for amplitude of the first period of the PWM signal 

// PUMP #1 
float amp0 = 175;                // amplitude of sinusoid (1000 is max, 0 is min)
float norm_amp0 = amp0;          // stored reference amplitude of sinusoid (1000 is max, 0 is min)
float mod_amp0 = amp0*scale_amp; // scaled amplitude for first period
float freq0 = 0.25;              // frequency [Hz] 
float phi0 = 0;                  // phase [deg]

// PUMP #2
float amp1 = 175;                // amplitude of sinusoid (1000 is max, 0 is min)
float norm_amp1 = amp1;          // stored reference amplitude of sinusiod (1000 is max, 0 is min)
float mod_amp1 = amp1*scale_amp; // scaled amplitude for first period
float freq1 = 0.25;              // frequency [Hz]
float phi1 = 120;                // phase [deg]

// PUMP #3
float amp2 = 175;                // amplitude of sinusoid (1000 is max, 0 is min)
float norm_amp2 = amp2;          // stored reference amplitude of sinusoid (1000 is max, 0 is min)
float mod_amp2 = amp2*scale_amp; // scaled amplitude for first period 
float freq2 = 0.25;              // frequency [Hz]
float phi2 = 240;                // phase [deg]

// ======== INITIALIZE PWM SIGNALS ========
// PWM SIGNAL
int pump0Pwm = 0;       // pump #1 
int pump1Pwm = 0;       // pump #2 
int pump2Pwm = 0;       // pump #3

// REFERENCE PWM SIGNAL 
float pump0Pwm_ref = 0; // pump #1
float pump1Pwm_ref = 0; // pump #2
float pump2Pwm_ref = 0; // pump #3

// ======== INITIALIZE RPM SENSORS ========
float numpoles = 2;           // number of magnets in the motor

// RPM SENSOR #1 
#define INTERRUPT_PIN_0 8     // interrupt pin
volatile int interruptCount0; // pulse count 
float rpm0 = 0;               // computed rpm 

// RPM SENSOR #2 
#define INTERRUPT_PIN_1 7     // interrupt pin 
volatile int interruptCount1; // pulse count 
float rpm1 = 0;               // computed rpm 

// RPM SENSOR #3
#define INTERRUPT_PIN_2 6     // interrupt pin
volatile int interruptCount2; // pulse count
float rpm2 = 0;               // computed rpm 

// ======== CONTROL VARIABLES/PARAMETERS ========
// RPM -> PWM SCALING FACTOR 
float scaling_factor = 160;

// FILTERED RPM SENSOR READINGS 
float filtered_rpm0 = 0; // pump #1
float filtered_rpm1 = 0; // pump #2
float filtered_rpm2 = 0; // pump #3

// ERROR
float e_0 = 0;  // pump #1
float e_1 = 0;  // pump #2
float e_2 = 0;  // pump #3

// CONTROLLER PROPORTIONAL GAIN
float Kp = 1.5;

// FIRST-ORDER INFINITE IMPULSE RESPONSE (IIR) FILTER COEFFICIENT
float alpha = 0.2;

// ======== VARIABLE FOR ESC INITIALIZATION SWEEP ========
int pos = 0;

// ======== TIMING VARIABLES ========
int first_pass = 0; // flag for first loop (used for start time)
float start_t = 0;  // start time of the loop (s)
float t = 0;        // internal time variable for generating reference PWM signal

// ======== INITIALIZE ESCS ========
void initESCs(){

  // Sends maximum PWM signal to arm ESCs [should hear initialization beeps]
  ESC0.write(180);
  ESC1.write(180);
  ESC2.write(180);

  delay(1000); // wait 1 second to ensure ESCs recieved high signal 

  // Gradually decreases PWM from max to min [180 -> 0]
  for (pos = 180; pos >= 0; pos -= 1) {
    ESC0.write(pos); // update ESC0 with current position
    ESC1.write(pos); // update ESC1 with current position
    ESC2.write(pos); // update ESC2 with current position 
    delay(5);        // waits 5 ms for ESC to respond smoothly 
  }

  // Sets ESC to neutral position [pumps off] 
  ESC0.write(90);
  ESC1.write(90);
  ESC2.write(90);
  delay(500);       // waits 500 ms for ESC to stablize at neutral
}

void setup() {
  // Initialize the ESCs - attach them to PWM pins & define pulse width range [pin, min pulse width, max pulse width in microseconds]
  ESC0.attach(11,1000,2000); // pump #1 
  ESC1.attach(10,1000,2000); // pump #2
  ESC2.attach(9 ,1000,2000); // pump #3

  // Initialize serial communication 
  Serial.begin(baudRate);

  // Sends arming sequence to each ESC to prepare pumps for operation 
  initESCs();

  // RPM SENSOR SETUP - configures interrupt pins for each pump's RPM sensor 
  pinMode(INTERRUPT_PIN_0, INPUT); // pump #1
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_0), interruptFired0, CHANGE);
  pinMode(INTERRUPT_PIN_1, INPUT); // pump #2
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_1), interruptFired1, CHANGE);
  pinMode(INTERRUPT_PIN_2, INPUT); // pump #3
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_2), interruptFired2, CHANGE);
}

void loop() {
  // ======== INITIALIZE START TIME ========
  if (first_pass == 0) {
    start_t = millis()/1000.0; // convert milliseconds to seconds 
    first_pass = 1;            // mark that initialization is complete 
  }

  // ======== ELAPSED TIME ========
  t = millis()/1000.0 - start_t;
  Serial.print(t,4);
  Serial.print("\t");

  // ======== FIRST PERIOD AMPLITUDE MODIFICATION ========
  // For the 1st half period of each sinusoid, apply modified amplitude to gradually ramp up the PWM signal. After the half-period, switch to the stored reference amplitude for steady-state operation. 
  // if (t < (1/freq0)/2) {
  //   amp0 = mod_amp0;
  //   amp1 = mod_amp1;
  //   amp2 = mod_amp2;
  // }
  // else {
  //   amp0 = norm_amp0; 
  //   amp1 = norm_amp1;
  //   amp2 = norm_amp2; 
  // }

  // ======== REFERENCE PWM SIGNAL ========
  // PUMP #1
  pump0Pwm_ref = amp0*sin(2*pi*freq0*t + phi0*pi/180.0); // generate sinusoid
  pump0Pwm_ref = pump0Pwm_ref*90.0/1000;                 // scale to PWM range 

  // PUMP #2
  pump1Pwm_ref = amp1*sin(2*pi*freq1*t + phi1*pi/180.0); // generate sinusoid
  pump1Pwm_ref = pump1Pwm_ref*90.0/1000;                 // scale to PWM range

  // PUMP #3
  pump2Pwm_ref = amp2*sin(2*pi*freq2*t + phi2*pi/180.0); // generate sinusoid
  pump2Pwm_ref = pump2Pwm_ref*90.0/1000;                 // scale to PWM range

  // ======== PRINT REFERENCE PWM SIGNAL ========
  Serial.print(pump0Pwm_ref); // pump #1 
  Serial.print("\t");
  Serial.print(pump1Pwm_ref); // pump #2 
  Serial.print("\t");
  Serial.print(pump2Pwm_ref); // pump #3
  Serial.print("\t");

  // ======== WRITE PWM SIGNAL ========
  ESC0.write(pump0Pwm); // pump #1 
  ESC1.write(pump1Pwm); // pump #2
  ESC2.write(pump2Pwm); // pump #3

  // ======== CHECK RPM READINGS ========
  checkRPM0(); // measure rpm for pump #1 
  checkRPM1(); // measure rpm for pump #2
  checkRPM2(); // measure rpm for pump #3

  // ======== ADJUSTS RPM SIGN BASED ON PUMP PWM DIRECTION ========
  // If PWM > 90: pump running forward -> RPM stays position 
  // If PWM < 90: pump running reverse -> RPM becomes negative 
  if (pump0Pwm > 90) {
    rpm0 = rpm0;  // forward direction -> RPM remains positive 
  }
  if (pump0Pwm < 90) {
    rpm0 = -rpm0; // reverse direction -> invert RPM 
  }
  if (pump1Pwm > 90) {
    rpm1 = rpm1;  // forward direction
  }
  if (pump1Pwm < 90) {
    rpm1 = -rpm1; // reverse direction
  }
  if (pump2Pwm > 90) {
    rpm2 = rpm2;  // forward direction
  }
  if (pump2Pwm < 90) {
    rpm2 = -rpm2; // reverse direction
  }

  // ======== FIRST-ORDER INFINITE IMPULSE RESPONSE (IIR) FILTER: RPM SENSOR READINGS ========
  filtered_rpm0 = alpha*rpm0 + (1-alpha)*filtered_rpm0; // pump #1 filtered rpm 
  filtered_rpm1 = alpha*rpm1 + (1-alpha)*filtered_rpm1; // pump #2 filtered rpm 
  filtered_rpm2 = alpha*rpm2 + (1-alpha)*filtered_rpm2; // pump #3 filtered rpm 

  // ======== PRINT FILTERED RPM VALUES (SCALED TO PWM-EQUIVALENT UNITS) ========
  // Used to monitor real-time pump speeds and verify controller performance 
  Serial.print(filtered_rpm0/scaling_factor);   // pump #1 scaled rpm 
  Serial.print("\t");                           
  Serial.print(filtered_rpm1/scaling_factor);   // pump #2 scaled rpm 
  Serial.print('\t');                           
  Serial.println(filtered_rpm2/scaling_factor); // pump #3 scaled rpm 

  // ======== COMPUTE CONTROL ERROR  ========
  // Error = desired PWM command minus measured pump RPM (scaled)
  e_0 = (pump0Pwm_ref - filtered_rpm0/scaling_factor); // pump #1 
  e_1 = (pump1Pwm_ref - filtered_rpm1/scaling_factor); // pump #2 
  e_2 = (pump2Pwm_ref - filtered_rpm2/scaling_factor); // pump #3

  // ======== SCALE CONTROL ERRORS ========
  // Convert the calculated error into actual PWM range used by the ESC [-90,90] 
  e_0 = e_0*90.0/40.0; // pump #1
  e_1 = e_1*90.0/40.0; // pump #2
  e_2 = e_2*90.0/40.0; // pump #3
  
  // ======== PROPORTIONAL CONTROLLER ========
  pump0Pwm = Kp*e_0; // pump #1 
  pump1Pwm = Kp*e_1; // pump #2
  pump2Pwm = Kp*e_2; // pump #3

  // ======== ENFORCES UPPER & LOWER BOUNDS ON PWM SIGNAL [-90,90] ========
  if (pump0Pwm > 90) {
    pump0Pwm = 90;
  }
  if (pump0Pwm < -90) {
    pump0Pwm = -90;
  }
  if (pump1Pwm > 90) {
    pump1Pwm = 90;
  }
  if (pump1Pwm < -90) {
    pump1Pwm = -90;
  }
  if (pump2Pwm > 90) {
    pump2Pwm = 90;
  }
  if (pump2Pwm < -90) {
    pump2Pwm = -90;
  }

  // ======== FEEDFORWARD CONTROLLER ========
  // Adds reference PWM to the proportional control output
  pump0Pwm = pump0Pwm + pump0Pwm_ref; // pump #1 
  pump1Pwm = pump1Pwm + pump1Pwm_ref; // pump #2
  pump2Pwm = pump2Pwm + pump2Pwm_ref; // pump #3

  // ======== MAPS PWM [-90,90] TO ESC [0,180] ========
  pump0Pwm = map(pump0Pwm,-90,90,0,180); // pump #1 
  pump1Pwm = map(pump1Pwm,-90,90,0,180); // pump #2 
  pump2Pwm = map(pump2Pwm,-90,90,0,180); // pump #3 
}

// ======== CHECK RPM READINGS ========
void checkRPM0() { // pump #1 rpm measurement
  // Reset pulse counter 
  noInterrupts();
  interruptCount0 = 0;
  interrupts();

  delay(40); // sampling window [~40 ms = 25 samples/sec]

  // Read pulse count 
  noInterrupts();
  int critical_rpm0 = interruptCount0;
  interrupts();

  // Convert pulse count to rpm 
  rpm0 = ((critical_rpm0)*(60))/(numpoles)*25; // 25 = 1 / 0.04 s 
}

void checkRPM1() { // pump #2 rpm measurement
  // Reset pulse counter 
  noInterrupts();
  interruptCount1 = 0;
  interrupts();

  delay(40); // sampling window [~40 ms = 25 samples/sec]

  // Read pulse count 
  noInterrupts() ;
  int critical_rpm1 = interruptCount1;
  interrupts();

  // Convert pulse count to rpm 
  rpm1 = ((critical_rpm1)*(60))/(numpoles)*25; // 25 = 1 / 0.04 s 
}

void checkRPM2() { // pump #3 rpm measurement
  // Reset pulse counter
  noInterrupts();
  interruptCount2 = 0;
  interrupts();

  delay(40); // sampling window [~40 ms = 25 samples/sec]

  // Read pulse count 
  noInterrupts();
  int critical_rpm2 = interruptCount2; 
  interrupts();

  // Convert pulse count to rpm 
  rpm2 = ((critical_rpm2)*(60))/(numpoles)*25; // 25 = 1 / 0.04 s 
}

// ======== PULSE COUNTER FOR RPM SENSORS ========
void interruptFired0() { // pump #1 rpm
  interruptCount0++;     // increment pulse count 
}

void interruptFired1() { // pump #2 rpm
  interruptCount1++;     // increment pulse count 
}

void interruptFired2() { // pump #3 rpm 
  interruptCount2++;     // increment pulse count
}
