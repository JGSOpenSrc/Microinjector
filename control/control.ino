#include<Wire.h>
#include<Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

/* The amount of time, in seconds, that the injector will wait
   after an injection before retracting the liquid. */
#define TIMEOUT    10

/* I2C bus speed, is modified in setup() */
#define I2C_FREQ 400000L

/* Buffer length used for storing / sending messages over UART
  and for doing general string comparisons*/
#define BUFFER_LEN 256

/* Defines for IO port names*/
#define S1 9
#define S2 10
#define SOLENOID_ENERGIZE 8
#define BOUNCE_DELAY 100
#define TTL_INCREMENT_PIN 5
#define IR_SENSOR_PIN A0

#define WITHDRAW_STEP_COUNT 1200

// Motor constants
static const int motor_speed = 600;
static const int steps_per_rev = 200;
// ADC thresholds for the IR liquid sensor
static const int high_to_low = 400;
static const int low_to_high = 500;

/* Enumeration of possible states */
enum STATE{
  halt,
  reset_pos,
  wait,
  inject,
  withdraw
};

/* Enumeration of possible positions */
enum POSITION{
  zero_stroke,
  mid_stroke,
  full_stroke
};

Adafruit_MotorShield AFMS = Adafruit_MotorShield();             // Create the motor shield object with the default I2C address.

Adafruit_StepperMotor *motor = AFMS.getStepper(steps_per_rev, 1);// Connect a stepper motor with 200 steps per revolution (1.8 degree)
                                                                // to motor port #2 (M3 and M4).

bool stopflag = false;                                          // Emertency stop flag that is set by the forward limit switch. Prevents damage to injector.
bool reset = true;                                              // Reset flag used to initialize the position of the injector.
bool is_empty;                                                  // Flag used for detecting


int steps = 0;                                                  // Number of steps left for the actuator to complete before its current command is finished
int volume = 0;                                                 // Volume of fluid requested for injection by serial communication interface.
enum POSITION injector_pos;                                     // General position of the injector at power-on reset. Can be zero-stroke, mid-stroke, or full-stroke.
enum STATE injector_state;                                      // Control variable. Can be halt, reset_pos, wait, or inject
char input_buffer[BUFFER_LEN];                                  // Serial input buffer, used to store serial data and to parse commands.
char output_buffer[BUFFER_LEN];                                 // Serial output buffer, used to store messages which will be sent out to the serial port.

void setup() {

                                                                // initialize message buffer
  int i;
  for(i = 0; i < BUFFER_LEN; i++){
    output_buffer[i] = 0;
  }
                                                                // setup serial port
  Serial.begin(115200);

  /* Set up pin modes for safety / limit switches */
  pinMode(S1, INPUT_PULLUP);                                    // pole 1 of safety switch 1
  delay(10);                                                    // For some reason this delay is necessary
  pinMode(S2, INPUT_PULLUP);                                    // Configure the S2 with an internal pullup.
  delay(10);
  pinMode(SOLENOID_ENERGIZE, OUTPUT);                           // Set solenoid mosfet driver to output
  digitalWite(SOLENOID_ENERGIZE, LOW);

  /* Determine the liquid level */
  /*int avg;
  for(i = 0; i < 10; i++){
    avg += analogRead(IR_SENSOR_PIN);
  } avg = avg / 10;

  sprintf(output_buffer, "Average value read on IR sensor: %d", avg);
  Serial.println(output_buffer);

  if(avg > low_to_high) {
    is_empty = true;
    Serial.println("Initially, liquid not detected by sensor.");
  }
  else if (avg < high_to_low){
    Serial.println("Initially, liquid detected by sensor.");
  }
  else {
    is_empty = false;
  }*/
  
  int value = analogRead(IR_SENSOR_PIN);
  sprintf(output_buffer, "IR sensor reads %d", value);
  Serial.println(output_buffer);
  if(value > low_to_high) is_empty = true;

  /* Determine the initial state of the micoinjector */
  if (!digitalRead(S1) && digitalRead(S2)){                     // Actuator is initally in zero_stroke position
    injector_pos = zero_stroke;
    injector_state = halt;
    Serial.println("Acuator initial position is zero-stroke.");
  }
  else if (digitalRead(S1) && digitalRead(S2)){                // Actuator is initially in full_stroke position
    injector_pos = mid_stroke;
    injector_state = halt;
    Serial.println("Actuator initial position is mid-stroke");
  }
  else if (digitalRead(S1) && !digitalRead(S2)){                 // Actuator is initially in mid_stroke
    injector_pos = full_stroke;
    injector_state = halt;
    Serial.println("Actuator initial position is full-stroke");
  }
  else{
    Serial.println("Actuator initial position unknown...");
    sprintf(output_buffer, "S1 %d S2 %d", digitalRead(S1), digitalRead(S2));
    Serial.println(output_buffer);
    Serial.println("There is a problem with the limit switches.");
    Serial.println("Fix the switches and then reset or reprogram.");
    while(1);
  }
  
  Serial.println("You must prime the actuator by pressing the front switch to continue.");

  /* Setup of the TTL injection trigger,
  this requires a digital pin, and configuration of a hardware counter. */
  pinMode(TTL_INCREMENT_PIN, INPUT_PULLUP);
                                                                // Initialize Counter 1, used to track TTL pulses on the TTL_INCREMENT_PIN
  TCCR1A = 0x00;                                                // Configures Counter 1 to run in normal mode, with no output flags.
  TCCR1B = (1<<CS02)|(1<<CS01);                                 // Set the counter clock source to the TTL_INCREMENT_PIN falling edge.
  TCNT1 = 0;                                                    // Reset the counter output register.

  AFMS.begin();                                                 // Initialize motor
  motor->setSpeed(motor_speed);
  
  stopflag = false;

  TWBR = ((F_CPU / I2C_FREQ) - 16) / 2;                         // Change the I2C frequency
}

void loop() {

  if(halt == injector_state){                                   // injector is in the halt state - waiting for switch signal.

    switch(injector_pos){

      case zero_stroke:                                          // When in the zero-stroke position, waiting for S2 to be tripped

      if (!digitalRead(S2)){

        while(!digitalRead(S2)){
          delay(BOUNCE_DELAY);
        }

        delay(BOUNCE_DELAY);
        TCNT1 = 0;
        injector_state = wait;                                  // S2 tripped -> go to waiting state (injector is ready to inject)
        Serial.println("waiting for user input...");
      }
      break;

      case mid_stroke:                                          // When in mid storke position, waiting for S1 or S2 to trip

      if (!digitalRead(S1)){

        while(!digitalRead(S1)){
          delay(BOUNCE_DELAY);                                  // Delays are for filtering switch bounce
        }

        delay(BOUNCE_DELAY);
        injector_state = reset_pos;                             // S1 tripped -> reset injector position to zero-stroke
        Serial.println("Resetting actuator position...");
      }

      else if (!digitalRead(S2)){

        while(!digitalRead(S2)){
          delay(BOUNCE_DELAY);                                  // Delays are for filtering switch bounce.
        }

        delay(BOUNCE_DELAY);

        TCNT1 = 0;

        injector_state = wait;                                  // S2 tripped -> go to waiting state (injector is ready to inject)
        Serial.println("waiting for user input...");
      }
      break;

      case full_stroke:
      /* When in full-stroke position, wait for S1 to trip, then reset position */
      if(!digitalRead(S1)){

        while(!digitalRead(S1)){
          delay(BOUNCE_DELAY);                                  // Delays are for filtering switch bounce.
        }

        delay(BOUNCE_DELAY);

        injector_state = reset_pos;                             // S1 tripped -> reset injector position to zero-stroke
        Serial.println("Resetting actuator position...");
      }
      break;

      default:
        Serial.println("STATE DEFAULT :: FATAL ERROR");
        while(1);
    }
  }

  else if (reset_pos == injector_state){                        // reset_pos state. The injector steps backward until it is in zero-stroke position.

    if (!digitalRead(S1)){
      motor->release();
      injector_pos = zero_stroke;
      injector_state = halt;                                    // S1 tripped -> position is now zero-stroke. Go to halt state.
      Serial.println("halt!");
    }

    else {

      if (digitalRead(S2)){
        injector_pos = mid_stroke;                              // S2 tripped -> position is mid-stroke.
      }

      motor->step(1, BACKWARD, SINGLE);
    }
  }

  else if (wait == injector_state){                             // wait state. The injector is waiting for an injection commmand form the user.

    if (!check_liquid()) {
      delay(1000 * TIMEOUT);

      if(!check_liquid()){                                      // if the liquid is still present, withdraw it.
        injector_state = withdraw;
        steps = WITHDRAW_STEP_COUNT;
        Serial.println("Withdrawing...");
      }
    }
    else {
      if(check_liquid() && get_user_input()){
        injector_state = inject;
        Serial.println("injecting..");
      }
    }
  }

  else if (inject == injector_state){                           // inject state. The injector is currently stepping forward.

    /* If S2 is depressed, then the syringe is empty. */
    if (!digitalRead(S2)){
      motor->release();
      injector_pos = full_stroke;
      injector_state = halt;                                    // S2 tripped -> position is full-stroke. Go to halt state.
      Serial.println("End of actuator reached; please reset the position by pressing rear switch.");
    }

    /* If the liquid is detected, cease injecting and wait */
    else if (!check_liquid()){
      motor->release();
      injector_state = wait;
      int voltage = analogRead(0);
      sprintf(output_buffer, "IR sensor reads %d", voltage);
      Serial.println(output_buffer);
    }

    else {
      motor->step(1, FORWARD, SINGLE);
      if(digitalRead(S1)){
        injector_pos = mid_stroke;                              // S1 high -> position is mid-stroke.
      }
    }
  }

  else if (withdraw == injector_state){                         // withdraw state. The injecgtor is currently stepping backwards

    /* if S1 is depressed, then the syringe cannot step back any more */
    if(!digitalRead(S1)){
      motor->release();
      injector_pos = zero_stroke;
      injector_state = halt;
      Serial.println("End of actuator reached; you must prime the actuator by pressing the front switch to continue.");
    }

    /* Liquid is no longer detected, cease withdrawing and wait */
    else if (0 == steps){
      motor->release();
      injector_state = wait;
      int voltage = analogRead(0);
      sprintf(output_buffer, "IR sensor reads %d", voltage);
      Serial.println(output_buffer);
      if(check_liquid()) Serial.println("waiting for user input...");
    }

    /* Step backward to withdraw liquid */
    else {
        motor->step(1, BACKWARD, SINGLE);
        steps -= 1;
        /* If S1 is pressed, the injector cannot step backward any more. */
        if(!digitalRead(S1)){
          motor->release();
          injector_pos = zero_stroke;
          injector_state = halt;
        }
    }
  }
}

/*  Reads the value from the IR sensor, and based upon the value
    read and the current value of is_empty, operates on is_empty and returns its value.

    If there is liquid detected by the sensor, this will return false. If there
    is no liquid detected, it will return true.
 */
bool check_liquid(){
  int val = analogRead(IR_SENSOR_PIN);

  is_empty = is_empty && !(val < high_to_low) || !is_empty && (val > low_to_high);

  return is_empty;
}

/*
  get_user_input

  Checks the serial port for an injection command, and also checks the hardware
  counter for a TTL pulse (either can be implemented). Returns true if an injection
  command has been received.
*/
bool get_user_input(){

  bool inject = false;

  int rc = read_line(input_buffer, BUFFER_LEN);

  if(0 == rc){
    /* Check to see if the string is a recognized command. */
    if(0 == strcmp(input_buffer, "inject")){

      inject = true;

      sprintf(output_buffer,
              "Injection command received");
      Serial.println(output_buffer);
    }

    else {
      /* The string received was no good */
      inject = false;

      sprintf(output_buffer,
              "ERROR invalid command received");
      Serial.println(output_buffer);
    }
  }

  /* Check the counter for a TTL pulse, which singals an injection request */
  else if(0 < TCNT1) {

    TCNT1--;
    inject = true;

    sprintf(output_buffer,
            "TTL pulse received");
    Serial.println(output_buffer);
  }

  return inject;
}

/* read_line

  A quick and dirty method for retrieving data segmented by newline characters
  from the serial port. Writes the data to the buffer provided.
*/

int read_line(char* buff, int buff_len){
  int rc;

  char* strptr = buff;

  bool overflow = false;

  if (Serial.available()){
    do {
      
      *strptr = Serial.read();
      strptr++;
  
      if(strptr > (buff + buff_len)){
        overflow = true;
        break;
      }
      
      delay(1);

    } while (*(strptr - 1) != '\n');
  }

  /* In case of buffer overflow, return 1*/
  if(overflow){
    rc = 1;
  }

  /* In case of no data available, return -1*/
  else if (strptr == buff) {
    rc = -1;
  }

  /* No errors, clean up and return */
  else {
    *(strptr-1) = '\0';
    rc = 0;
  }

  return rc;
}
