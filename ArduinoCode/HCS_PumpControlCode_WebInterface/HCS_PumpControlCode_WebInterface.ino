// -------------------------------------------------------------------
//     Hydraulic Control Board: Pump Control Code w/ Web Interface
// -------------------------------------------------------------------
// Author: Allyson Elise Chen
// Date Updated: 2025-11-17
//
// Description: 
//    Closed-loop control of three pumps using RPM feedback. Each pump is driven by a sinusoidal PWM signal. 
//
// Web Interface: 
//    1. Demo Controls
//      a. Initialize [INITIALIZE/OFF] 
//      b. Open-Loop Control [RUN/OFF]
//      c. Closed-Loop Control [RUN/OFF]
//    2. General Controls 
//      a. Pump #1 
//        i. [RUN/OFF]
//        ii. Slider [-90,90] PWM
//        iii. Priming [RUN/OFF]
//      b. Pump #2
//        i. [RUN/OFF]
//        ii. Slider [-90,90] PWM
//        iii. Priming [RUN/OFF]
//      c. Pump #3
//        i. [RUN/OFF]
//        ii. Slider [-90,90] PWM
//        iii. Priming [RUN/OFF]
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

/*********
  setting up ESP32 Access Point:
  https://randomnerdtutorials.com/esp32-access-point-ap-web-server/
*********/

// ======== LIBRARY ========
#include <ESP32Servo.h>
#include <WiFi.h>

// ======== SERIAL COMUNICATION ========
const int baudRate = 115200; // constant integer to set the baud rate for serial monitor

// ======== WEB INTERFACE - ESP32 ACCESS POINT ========
// NETWORK CREDENTIALS
const char* ssid     = "ESP32-Access-Point";
const char* password = "123456789";

// WEB SERVER PORT NUMBER
WiFiServer server(80);

// STORES HTTP REQUEST
String header;

// CURRENT OUTPUT STATE
String open_outputState = "OFF";
String closed_outputState = "OFF";
String pump1_state = "OFF";
String pump2_state = "OFF";
String pump3_state = "OFF";
String priming_pump1_state = "OFF";
String priming_pump2_state = "OFF";
String priming_pump3_state = "OFF";
String initialize = "OFF";

// INTERNAL CONTROL FLAGS  
bool open_control_run = false;
bool closed_control_run = false;
bool priming_pump1_run = false;
bool priming_pump2_run = false; 
bool priming_pump3_run = false; 

// SLIDER VALUES - ESC PWM COMMAND VALUES
int pump1_slider = 90;
int pump2_slider = 90;
int pump3_slider = 90;

// ======== PUMP ESC OBJECTS ========
Servo ESC0; // pump #1 
Servo ESC1; // pump #2
Servo ESC2; // pump #3

// ======== PARAMETERS: PWM SIGNAL [SINUSOID] ========
float pi = 3.14159;
float scale_amp = 0.5;           // scaling factor for amplitude of the first period of the PWM signal 

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
int pump0Pwm = 0;             // pump #1 
int pump1Pwm = 0;             // pump #2 
int pump2Pwm = 0;             // pump #3

// REFERENCE PWM SIGNAL 
float pump0Pwm_ref = 0;       // pump #1
float pump1Pwm_ref = 0;       // pump #2
float pump2Pwm_ref = 0;       // pump #3

// PRIMING PWM SIGNAL
float priming_pump1_input = 0; // pump #1
float priming_pump2_input = 0; // pump #2
float priming_pump3_input = 0; // pump #3

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
float e_0 = 0; // pump #1 
float e_1 = 0; // pump #2
float e_2 = 0; // pump #3

// CONTROLLER PROPORTIONAL GAIN
float Kp = 1.5;

// FIRST-ORDER INFINITE IMPULSE RESPONSE (IIR) FILTER COEFFICIENT
float alpha = 0.2;

// OPEN-LOOP GAIN
float OL_scaling = 1.2;

// ======== VARIABLE FOR ESC INITIALIZATION SWEEP ========
int pos = 0;

// ======== TIMING VARIABLES ========
int first_pass = 0;    // flag for first loop (used for start time)
float start_t = 0;     // start time of the loop (s)
float t = 0;           // internal time variable for generating reference PWM signal

void setup() {
  // Initialize the ESCs - attach them to PWM pins & define pulse width range [pin, min pulse width, max pulse width in microseconds]
  ESC0.attach(11,1000,2000); // pump #1 
  ESC1.attach(10,1000,2000); // pump #2
  ESC2.attach(9,1000,2000);  // pump #3

  // Initialize serial communication
  Serial.begin(baudRate);

  // Sends arming sequence to each ESC to prepare pumps for operation 
  initESCs();

  Serial.println("Setup Complete");

  // RPM SENSOR SETUP - configures interrupt pins for each pump's RPM sensor 
  pinMode(INTERRUPT_PIN_0, INPUT); // pump #1 
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_0), interruptFired0, CHANGE);
  pinMode(INTERRUPT_PIN_1, INPUT); // pump #2 
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_1), interruptFired1, CHANGE);
  pinMode(INTERRUPT_PIN_2, INPUT); // pump #3
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_2), interruptFired2, CHANGE);

  // CONNECT TO WI-FI NETWORK WITH SSID AND PASSWORD
  Serial.print("Setting AP (Access Point)…");
  WiFi.softAP(ssid, password);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  
  server.begin();
}

void loop(){
  // ======== LISTENS FOR INCOMING CLIENTS ========
  WiFiClient client = server.available();

  // ======== OPEN-LOOP CONTROL ========
  if (open_control_run == true) {
    // ======== CONTROLLER PROPORTIONAL GAIN ========
    Kp = 0;

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
    ESC0.write(pump0Pwm*OL_scaling); // pump #1 
    ESC1.write(pump1Pwm*OL_scaling); // pump #2
    ESC2.write(pump2Pwm*OL_scaling); // pump #3 

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
    filtered_rpm0 = alpha*rpm0 + (1-alpha)*filtered_rpm0;
    filtered_rpm1 = alpha*rpm1 + (1-alpha)*filtered_rpm1;
    filtered_rpm2 = alpha*rpm2 + (1-alpha)*filtered_rpm2;

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

  // ======== CLOSED LOOP CONTROL ========
  if (closed_control_run == true) {
    // ======== CONTROLLER PROPORTIONAL GAIN ========
    Kp = 1.5;

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
    pump0Pwm = Kp*e_0;  // pump #1
    pump1Pwm = Kp*e_1;  // pump #2
    pump2Pwm = Kp*e_2;  // pump #3

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

  // ======== PRIMING PUMP #1 ========
  if (priming_pump1_run == true) {
    t = millis()/1000.0;
    priming_pump1_input = 300*sin(2*pi*freq0*t + phi0*pi/180.0); // generate sinusoid
    priming_pump1_input = priming_pump1_input*90/1000;           // scale to PWM range
    priming_pump1_input = map(priming_pump1_input,-90,90,0,180); // maps PWM [-90,90] to ESC [0,180]
    Serial.println(priming_pump1_input);                         // print pump input 
    ESC0.write(priming_pump1_input);                             // write PWM signal 
  }

  // ======== PRIMING PUMP #2 ========
  if (priming_pump2_run == true) {
    t = millis()/1000.0;
    priming_pump2_input = 300*sin(2*pi*freq0*t + phi0*pi/180.0); // generate sinusoid
    priming_pump2_input = priming_pump2_input*90/1000;           // scale to PWM range
    priming_pump2_input = map(priming_pump2_input,-90,90,0,180); // maps PWM [-90,90] to ESC [0,180]
    Serial.println(priming_pump2_input);                         // print pump input 
    ESC1.write(priming_pump2_input);                             // write PWM signal
  }

  // ======== PRIMING PUMP #3 ========
  if (priming_pump3_run == true) {
    t = millis()/1000.0;
    priming_pump3_input = 300*sin(2*pi*freq0*t + phi0*pi/180.0); // generate sinusoid
    priming_pump3_input = priming_pump3_input*90/1000;           // scale to PWM range
    priming_pump3_input = map(priming_pump3_input,-90,90,0,180); // maps PWM [-90,90] to ESC [0,180]
    Serial.println(priming_pump3_input);                         // print pump input 
    ESC2.write(priming_pump3_input);                             // write PWM signal
  }

  if (client) {                             // If a new client connects,
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected

      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
            
            // ======== WEB UI: OPEN-LOOP CONTROL/CLOSED-LOOP CONTROL ========
            // OPEN-LOOP CONTROL
            if (header.indexOf("GET /open_control/on") >= 0) {
              // User request: enable open-loop mode
              Serial.println("OPEN CONTROL PUMP ON");
              open_outputState = "ON";
              open_control_run = true;
            } else if (header.indexOf("GET /open_control/off") >= 0) {
              // User request: disable open-loop mode
              Serial.println("OPEN CONTROL PUMP OFF");
              open_outputState = "OFF";
              open_control_run = false;
              ESC0.write(90);
              ESC1.write(90);
              ESC2.write(90);
              Serial.println("Open Loop Control off");

            // CLOSED-LOOP CONTROL
            } else if (header.indexOf("GET /closed_control/on") >= 0) {
              // User request: enable open-loop mode
              Serial.println("CLOSED CONTROL PUMP ON");
              closed_outputState = "ON";
              closed_control_run  = true; 
            } else if (header.indexOf("GET /closed_control/off") >= 0) {
              // User request: disable open-loop mode
              Serial.println("CLOSED CONTROL PUMP OFF");
              closed_outputState = "OFF";
              closed_control_run  = false; 
              ESC0.write(90);
              ESC1.write(90);
              ESC2.write(90);
              Serial.println("Closed Loop Control off");
            }

            // ======== RESET INITIALIZATION FOR OPEN/CLOSED LOOP CONTROL ========
            if (header.indexOf("GET /initialize/on") >=0) {
              Serial.println("Initialized");
              first_pass = 0; 
            } else if (header.indexOf("GET /initialize/off") >=0) {
              Serial.println("Not Initialized");
            } 
            
            // ======== PUMP #1 ========
            // HANDLES WEB REQUESTS FOR TURNING PUMP #1 ON/OFF
            if (header.indexOf("GET /pump1/on") >=0) {
              // User request: Turn Pump #1 ON 
              Serial.print("PUMP 1 ON - ");
              Serial.println(pump1_slider);
              pump1_state = "ON";
              ESC0.write(pump1_slider);
            } else if ((header.indexOf("GET /pump1/off") >=0) && (closed_outputState == "OFF") && (open_outputState == "OFF") && (priming_pump1_state == "OFF")) {
              // User request: Turn Pump #2 OFF
              Serial.println("PUMP 1 OFF");
              pump1_state = "OFF";
              ESC0.write(90);
            } 

            // HANDLES WEB REQUESTS FOR ADJUSTING PUMP #1 PWM
            if(header.indexOf("GET /slider1?value=") >= 0) {
              Serial.println("SLIDER");
              // Extract slider value
              int pos1 = header.indexOf('=');
              int pos2 = header.indexOf('&');
              String valueStr = header.substring(pos1+1, pos2);
              pump1_slider = valueStr.toInt();
              Serial.print("PUMP 1 ");
              Serial.print(pump1_state);
              Serial.print(" - ");
              Serial.println(pump1_slider);
              // If Pump #1 is ON, write PWM signal
              if(pump1_state == "ON") {
                ESC0.write(pump1_slider);
              }
            }

            // HANDLES WEB REQUESTS FOR PRIMING PUMP #1 
            if (header.indexOf("GET /priming_pump1/on") >=0) {
              // User request: Prime Pump #1 ON 
              Serial.println("PRIMING - PUMP 1 ON");
              priming_pump1_state = "ON";
              priming_pump1_run = true; 
            } else if ((header.indexOf("GET /priming_pump1/off") >=0) && (pump1_state == "OFF") && (closed_outputState == "OFF") && (open_outputState == "OFF")) {
              // User request: Prime Pump #1 OFF 
              Serial.println("PRIMING - PUMP 1 OFF");
              priming_pump1_state = "OFF";
              priming_pump1_run = false;
              ESC0.write(90);
            } 
            
            // ======== PUMP #2 ========
            // HANDLES WEB REQUESTS FOR TURNING PUMP #2 ON/OFF
            if (header.indexOf("GET /pump2/on") >=0) {
              // User request: Turn Pump #2 ON 
              Serial.print("PUMP 2 ON - ");
              Serial.println(pump2_slider);
              pump2_state = "ON";
              ESC1.write(pump2_slider); 
            } else if ((header.indexOf("GET /pump2/off") >=0) && (closed_outputState == "OFF") && (open_outputState == "OFF") && (priming_pump2_state == "OFF")) {
              // User request: Turn Pump #2 OFF 
              Serial.println("PUMP 2 OFF");
              pump2_state = "OFF";
              ESC1.write(90);
            } 

            // HANDLES WEB REQUESTS FOR ADJUSTING PUMP #2 PWM 
            if(header.indexOf("GET /slider2?value=") >= 0) {
              Serial.println("SLIDER");
              // Extract slider value
              int pos1 = header.indexOf('=');
              int pos2 = header.indexOf('&');
              String valueStr = header.substring(pos1+1, pos2);
              pump2_slider = valueStr.toInt();
              Serial.print("PUMP 2 ");
              Serial.print(pump2_state);
              Serial.print(" - ");
              Serial.println(pump2_slider);
              // If Pump #2 is ON, write PWM signal
              if(pump2_state == "ON") {
                ESC1.write(pump2_slider);
              }
            }

            // HANDLES WEB REQUESTS FOR PRIMING PUMP #2 
            if (header.indexOf("GET /priming_pump2/on") >=0) {
              // User request: Prime Pump #2 ON 
              Serial.println("PRIMING - PUMP 2 ON");
              priming_pump2_state = "ON";
              priming_pump2_run = true; 
            } else if ((header.indexOf("GET /priming_pump2/off") >=0) && (pump2_state == "OFF") && (closed_outputState == "OFF") && (open_outputState == "OFF")) {
              // User request: Prime Pump #2 OFF 
              Serial.println("PRIMING - PUMP 2 OFF");
              priming_pump2_state = "OFF";
              priming_pump2_run = false;
              ESC1.write(90);
            } 

            // ======== PUMP #3 ========
            // HANDLES WEB REQUESTS FOR TURNING PUMP #3 ON/OFF
            if (header.indexOf("GET /pump3/on") >=0) {
              // User request: Turn Pump #3 ON 
              Serial.print("PUMP 3 ON - ");
              Serial.println(pump3_slider);
              pump3_state = "ON";
              ESC2.write(pump3_slider);
            } else if ((header.indexOf("GET /pump3/off") >=0) && (closed_outputState == "OFF") && (open_outputState == "OFF") && (priming_pump3_state == "OFF")) {
              // User request: Turn Pump #3 OFF
              Serial.println("PUMP 3 OFF");
              pump3_state = "OFF";
              ESC2.write(90);
            } 

            // HANDLES WEB REQUESTS FOR ADJUSTING PUMP #3 PWM 
            if(header.indexOf("GET /slider3?value=") >= 0) {
              Serial.println("SLIDER");
              // Extract slider value
              int pos1 = header.indexOf('=');
              int pos2 = header.indexOf('&');
              String valueStr = header.substring(pos1+1, pos2);
              pump3_slider = valueStr.toInt();
              Serial.print("PUMP 3 ");
              Serial.print(pump3_state);
              Serial.print(" - ");
              Serial.println(pump3_slider);
              // If Pump #3 is ON, write PWM signal
              if(pump3_state == "ON") {
                ESC2.write(pump3_slider);
              }
            }

            // HANDLES WEB REQUESTS FOR PRIMING PUMP #3 
            if (header.indexOf("GET /priming_pump3/on") >=0) {
              // User request: Prime Pump #3 ON
              Serial.println("PRIMING - PUMP 3 ON");
              priming_pump3_state = "ON";
              priming_pump3_run = true; 
            } else if ((header.indexOf("GET /priming_pump3/off") >=0) && (pump3_state == "OFF") && (closed_outputState == "OFF") && (open_outputState == "OFF")) {
              // User request: Prime Pump #3 OFF
              Serial.println("PRIMING - PUMP 3 OFF");
              priming_pump3_state = "OFF";
              priming_pump3_run = false;
              ESC2.write(90);
            } 

            // HTML Web Page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            
            // CSS
            client.println("<style>");
            client.println("html { font-family: Times New Roman; display: inline-block; margin: 0px auto; text-align: center; }");
            client.println(".section { margin: 40px 0; padding: 20px; background: #f5f5f5; border-radius: 10px; }");
            client.println(".controls-row { display: flex; justify-content: center; gap: 20px; margin: 20px 0; }");
            client.println(".pump-control { flex: 1; max-width: 300px; padding: 15px; background: white; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }");
            client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 24px; margin: 2px; cursor: pointer; border-radius: 5px; width: 200px; }");
            client.println(".button2 { background-color: #555555; }");
            client.println(".slider-container { margin: 20px 0; }");
            client.println(".slider { width: 90%; height: 25px; margin: 10px 0; }");
            client.println("h1 { color: #333; margin-bottom: 40px; }");
            client.println("h2 { color: #444; margin-bottom: 20px; }");
            client.println("h3 { color: #666; margin: 10px 0; }");
            client.println(".value-display { font-size: 18px; color: #444; margin: 10px 0; }");
            client.println("</style>");
            
            // JavaScript
            client.println("<script>");
            client.println("function updateSlider(number, value) {");
            client.println("  document.getElementById(`pump${number}Value`).textContent = value;");
            client.println("  var xhr = new XMLHttpRequest();");
            client.println("  xhr.open('GET', `/slider${number}?value=${value}&`, true);");
            client.println("  xhr.send();");
            client.println("}");
            // client.println("</script>");
            // client.println("</head>");

            client.println("function toggleDevice(device, action) {");
            client.println("  var xhr = new XMLHttpRequest();");
            client.println("  xhr.onreadystatechange = function() {");
            client.println("    if (this.readyState == 4 && this.status == 200) {");
            client.println("      var button = document.querySelector(`[data-device='${device}']`);");
            client.println("      if (action === 'on') {");
            client.println("        button.innerHTML = 'OFF';");
            client.println("        button.setAttribute('onclick', `toggleDevice('${device}', 'off')`);");
            client.println("        button.classList.add('button2');");
            client.println("      } else {");
            client.println("        button.innerHTML = 'RUN';");
            client.println("        button.setAttribute('onclick', `toggleDevice('${device}', 'on')`);");
            client.println("        button.classList.remove('button2');");
            client.println("      }");
            client.println("    }");
            client.println("  };");
            client.println("  xhr.open('GET', `/${device}/${action}`, true);");
            client.println("  xhr.send();");
            client.println("}");
            client.println("</script>");
            
            // WEB PAGE BODY 
            client.println("<body><h1>Hydraulic Control Board</h1>");

              // DEMO CONTROL SECTION
              client.println("<div class=\"section\">");
              client.println("<h2>Demo Controls</h2>");
              if (initialize == "OFF") {
                    client.println("<button class=\"button\" data-device=\"initialize\" onclick=\"toggleDevice('initialize', 'on')\">INITIALIZE</button>");
                } else {
                    client.println("<button class=\"button button2\" data-device=\"initialize\" onclick=\"toggleDevice('initialize', 'off')\">OFF</button>");
                }
              client.println("<div class=\"controls-row\">");
            
                // OPEN LOOP CONTROL 
                client.println("<div class=\"pump-control\">");
                client.println("<h3>Open Loop</h3>");
                if (open_outputState == "OFF") {
                    client.println("<button class=\"button\" data-device=\"open_control\" onclick=\"toggleDevice('open_control', 'on')\">RUN</button>");
                } else {
                    client.println("<button class=\"button button2\" data-device=\"open_control\" onclick=\"toggleDevice('open_control', 'off')\">OFF</button>");
                }
                client.println("</div>");
              
                // CLOSED LOOP CONTROL 
                client.println("<div class=\"pump-control\">");
                client.println("<h3>Closed Loop</h3>");
                if (closed_outputState == "OFF") {
                    client.println("<button class=\"button\" data-device=\"closed_control\" onclick=\"toggleDevice('closed_control', 'on')\">RUN</button>");
                } else {
                    client.println("<button class=\"button button2\" data-device=\"closed_control\" onclick=\"toggleDevice('closed_control', 'off')\">OFF</button>");
                }
                client.println("</div>");
                client.println("</div>"); // End controls-row
                client.println("</div>"); // End section

              // GENERAL CONTROL SECTION 
              client.println("<div class=\"section\">");
              client.println("<h2>General Control</h2>");
              client.println("<div class=\"controls-row\">");

                // PUMP #1 CONTROL 
                client.println("<div class=\"pump-control\">");
                client.println("<h3>Pump #1</h3>");
                if (pump1_state == "OFF") {
                    client.println("<button class=\"button\" data-device=\"pump1\" onclick=\"toggleDevice('pump1', 'on')\">RUN</button>");
                } else {
                    client.println("<button class=\"button button2\" data-device=\"pump1\" onclick=\"toggleDevice('pump1', 'off')\">OFF</button>");
                }
                client.println("<div class=\"slider-container\">");
                client.println("<input type=\"range\" min=\"0\" max=\"180\" value=\"" + String(pump1_slider) + "\" class=\"slider\" onchange=\"updateSlider(1, this.value)\">");
                client.println("<p class=\"value-display\">PWM: <span id=\"pump1Value\">" + String(pump1_slider) + "</span></p>");
                client.println("</div>");

                client.println("<h3>Priming</h3>");
                if (priming_pump1_state == "OFF") {
                    client.println("<button class=\"button\" data-device=\"priming_pump1\" onclick=\"toggleDevice('priming_pump1', 'on')\">RUN</button>");
                } else {
                    client.println("<button class=\"button button2\" data-device=\"priming_pump1\" onclick=\"toggleDevice('priming_pump1', 'off')\">OFF</button>");
                }
                client.println("</div>");

                // PUMP #2 CONTROL 
                client.println("<div class=\"pump-control\">");
                client.println("<h3>Pump #2</h3>");
                if (pump2_state == "OFF") {
                    client.println("<button class=\"button\" data-device=\"pump2\" onclick=\"toggleDevice('pump2', 'on')\">RUN</button>");
                } else {
                    client.println("<button class=\"button button2\" data-device=\"pump2\" onclick=\"toggleDevice('pump2', 'off')\">OFF</button>");
                }
                client.println("<div class=\"slider-container\">");
                client.println("<input type=\"range\" min=\"0\" max=\"180\" value=\"" + String(pump2_slider) + "\" class=\"slider\" onchange=\"updateSlider(2, this.value)\">");
                client.println("<p class=\"value-display\">PWM: <span id=\"pump2Value\">" + String(pump2_slider) + "</span></p>");
                client.println("</div>");

                client.println("<h3>Priming</h3>");
                if (priming_pump2_state == "OFF") {
                    client.println("<button class=\"button\" data-device=\"priming_pump2\" onclick=\"toggleDevice('priming_pump2', 'on')\">RUN</button>");
                } else {
                    client.println("<button class=\"button button2\" data-device=\"priming_pump2\" onclick=\"toggleDevice('priming_pump2', 'off')\">OFF</button>");
                }
                client.println("</div>");

                // PUMP #3 CONTROL 
                client.println("<div class=\"pump-control\">");
                client.println("<h3>Pump #3</h3>");
                if (pump3_state == "OFF") {
                    client.println("<button class=\"button\" data-device=\"pump3\" onclick=\"toggleDevice('pump3', 'on')\">RUN</button>");
                } else {
                    client.println("<button class=\"button button2\" data-device=\"pump3\" onclick=\"toggleDevice('pump3', 'off')\">OFF</button>");
                }
                client.println("<div class=\"slider-container\">");
                client.println("<input type=\"range\" min=\"0\" max=\"180\" value=\"" + String(pump3_slider) + "\" class=\"slider\" onchange=\"updateSlider(3, this.value)\">");
                client.println("<p class=\"value-display\">PWM: <span id=\"pump3Value\">" + String(pump3_slider) + "</span></p>");
                client.println("</div>");

                client.println("<h3>Priming</h3>");
                if (priming_pump3_state == "OFF") {
                    client.println("<button class=\"button\" data-device=\"priming_pump3\" onclick=\"toggleDevice('priming_pump3', 'on')\">RUN</button>");
                } else {
                    client.println("<button class=\"button button2\" data-device=\"priming_pump3\" onclick=\"toggleDevice('priming_pump3', 'off')\">OFF</button>");
                }
                client.println("</div>");

              client.println("</div>"); // End controls-row
              client.println("</div>"); // End section

            client.println("</body></html>");

            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
}

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
  delay(500);        // waits 500 ms for ESC to stablize at neutral

}

// ======== CHECK RPM READINGS ========
void checkRPM0() { // pump #1 rpm measurement
  // Reset pulse counter
  noInterrupts() ;
  interruptCount0 = 0;
  interrupts() ;

  delay(40); // sampling window [~40 ms = 25 samples/sec]

  // Read pulse count
  noInterrupts() ;
  int critical_rpm0 = interruptCount0;
  interrupts() ;
  // Convert pulse count to rpm 
  rpm0 = ((critical_rpm0)*(60))/(numpoles)*25;
}

void checkRPM1() { // pump #2 rpm measurement
  // Reset pulse counter
  noInterrupts() ;
  interruptCount1 = 0;
  interrupts() ;

  delay(40); // sampling window [~40 ms = 25 samples/sec]

  // Read pulse count
  noInterrupts() ;
  int critical_rpm1 = interruptCount1;
  interrupts() ;
  // Convert pulse count to rpm 
  rpm1 = ((critical_rpm1)*(60))/(numpoles)*25;
}

void checkRPM2() { // pump #3 rpm measurement
  // Reset pulse counter
  noInterrupts() ;
  interruptCount2 = 0; 
  interrupts() ;

  delay(40); // sampling window [~40 ms = 25 samples/sec]

  // Read pulse count
  noInterrupts() ;
  int critical_rpm2 = interruptCount2; 
  interrupts() ;

  // Convert pulse count to rpm 
  rpm2 = ((critical_rpm2)*(60))/(numpoles)*25;
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