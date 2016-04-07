#include<Wire.h>
#include<Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

/* I2C bus speed, is modified in setup() */
#define I2C_FREQ 400000L

/* Buffer length used for storing / sending messages over UART
  and for doing general string comparisons*/
#define BUFFER_LEN 256

/* Defines for IO port names*/
#define S1 9
#define S2 10
#define BOUNCE_DELAY 100
#define TTL_INCREMENT_PIN 5

/* Constants for calculation */
// volume injected after ttl pulse
static const float ttl_step_volume = 10;                          // Adjust according to desired application
// number of motor steps per microliter of fluid injected
static const float steps_per_ul = ((float)965) / ((float)25);     // Adjust according to empirical data (must calibrate)
// millimeters of carriage motion per motor revolution
static const float mm_per_rev = 1.25;                             // Adjust according to lead screw thread pitch
static const float steps_per_rev = 200;                           // Adjust according to stepper motor used
static const int motor_speed = 600;                               // Adjust according to application, actual speed is capped by I2C bus frequency

/* Enumeration of possible states */
enum STATE{
  halt,
  reset_pos,
  wait,
  inject
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

int steps = 0;                                                  // Number of steps left for the actuator to complete before its current command is finished
int volume = 0;                                                 // Volume of fluid requested for injection by serial communication interface.
enum POSITION injector_pos;                                     // General position of the injector at power-on reset. Can be zero-stroke, mid-stroke, or full-stroke.
enum STATE injector_state;                                      // Control variable. Can be halt, reset_pos, wait, or inject
char input_buffer[BUFFER_LEN];                                  // Serial input buffer, used to store serial data and to parse commands.
char output_buffer[BUFFER_LEN];                                 // Serial output buffer, used to store messages which will be sent out to the serial port.
char* cmd;                                                      // String pointer, used to parse serial commands
char* value;                                                    // String pointer, used to parse serial commands

void setup() {

                                                                // initialize message buffer
  int i;
  for(i = 0; i < BUFFER_LEN; i++){
    output_buffer[i] = 0;
  }
                                                                // setup serial port
  Serial.begin(115200);

                                                                //Setup pinmodes and interrupt handlers
  pinMode(S1, INPUT_PULLUP);                                    // pole 1 of safety switch 1
  delay(10);                                                    // For some reason this delay is necessary
  pinMode(S2, INPUT_PULLUP);                                    // Configure the S2 with an internal pullup.

                                                                // Determining the initial state of the actuator...
  if (!digitalRead(S1) && digitalRead(S2)){                     // Actuator is initally in zero_stroke position
    injector_pos = zero_stroke;
    injector_state = halt;
    Serial.println("INITIAL POSITION ZERO-STROKE");
  }
  else if (digitalRead(S1) && !digitalRead(S2)){                // Actuator is initially in full_stroke position
    injector_pos = full_stroke;
    injector_state = halt;
    Serial.println("INITIAL POSITION FULL-STROKE");
  }
  else if (digitalRead(S1) && digitalRead(S2)){                 // Actuator is initially in mid_stroke
    injector_pos = mid_stroke;
    injector_state = halt;
    Serial.println("INITIAL POSITION MID-STROKE");
  }
  else{
    Serial.println("INITIAL STATE UNKNOWN!");
    sprintf(output_buffer, "S1 %d S2 %d", digitalRead(S1), digitalRead(S2));
    Serial.println(output_buffer);
    while(1);
  }

  pinMode(TTL_INCREMENT_PIN, INPUT_PULLUP);                     // Setup of the TTL increment pin used to trigger injections via pulses
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

      case full_stroke:                                         // When in the full-stroke position, wait for S1 to be tripped
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
        Serial.println("STATE DEFAULT");
      break;
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
    if (steps > 0) {
      injector_state = inject;                                  // Steps >0 -> go to the inject state.
      Serial.println("injecting..");
    } else {
      getUserInput();                                           // Check for user input, enqueue any injection commands.
    }
  }

  else if (inject == injector_state){                           // inject state. The injector is currently stepping forward.
    if (!digitalRead(S2)){
      motor->release();
      injector_pos = full_stroke;
      injector_state = halt;                                    // S2 tripped -> position is full-stroke. Go to halt state.
      steps = 0;
      Serial.println("halt!");
    }

    else if (0 >= steps){
      motor->release();
      injector_state = wait;                                    // Steps complete. Go to the wait state.
      Serial.println("waiting for user input...");
    }

    else {
      motor->step(1, FORWARD, SINGLE);                          // Step the injector forward 1 and decrement steps
      steps--;
      if(digitalRead(S1)){
        injector_pos = mid_stroke;                              // S1 high -> position is mid-stroke.
      }

    }
  }
}

void getUserInput(){
  if(Serial.available()){                                       // Check for a new serial command
  parseCommand(&cmd, &value);

    if(NULL!=cmd && NULL!=value){                               // Execute parseCommand to check if the command is valid.

      if(0==strcmp(cmd, "step")){
        steps = atoi(value);
        sprintf(output_buffer,
                "Command received, stepping %d units", steps);
        Serial.println(output_buffer);                          // send a response to the user
      }

      else if(0==strcmp(cmd, "volume")){                        // if the command is for a specific volume, calculate the required number of steps.
        volume = atoi(value);
        if(0!=volume){
          volume = atoi(value);
          steps = (int)((float)volume * steps_per_ul);
          sprintf(output_buffer,                                // Send the number of required steps to the user.
                  "Command received, stepping %d units",
                   steps);
          Serial.println(output_buffer);
        }

        else {                                                // Command input is invalid, notify user of error.
          sprintf(output_buffer,
                  "ERROR invalid command received");
          Serial.println(output_buffer);
        }
      }

      else {                                                  // Command input is invalid, notify user of error.
        sprintf(output_buffer,
                "ERROR invalid command received");
        Serial.println(output_buffer);
      }
    }

    else {                                                    // Command input is invalid, notify user of error.
      sprintf(output_buffer,
              "ERROR invalid command received");
      Serial.println(output_buffer);
    }
  }

  else if(0 < TCNT1) {                                        // Check COUNTER1 for a TTL pulse, which indicates 1 injection requested.
    TCNT1--;
    steps = (int)((float)ttl_step_volume * (float)steps_per_ul);
    sprintf(output_buffer,
            "TTL pulse received, stepping %d units", steps);  // Notify the user that the pulse was received.
    Serial.println(output_buffer);
  }
}
/*
 * parseCommand(char**, char**) reads data from the serial buffer into the input buffer.
 * The data in the inputk buffer is parsed for the ' ' delimiting character (a space).
 * The input buffer is then separated into two strings, which are pointed at by cmd and value.
 *
 * cmd:   a pointer the the command parameter. Will be NULL if the command is invaled.
 * value: a pointer to the value parameter. Will be NULL if the command is invalid.
 *
 * Valid commands have the form COMMAND:VALUE where COMMAND and VALUE are strings of length greater then zero.
 */

void  parseCommand(char** cmd, char** value){
  char* strptr = input_buffer;

  while(Serial.available()){                                    // read the message from the serial buffer into the message buffer
     *strptr = Serial.read();
     strptr++;
     delay(5);
  }
  *strptr = '\0';

  int len;                                                      // parse the message into its COMMAND:VALUE pair
  len = strlen(input_buffer);
  strptr = (char*)memchr(input_buffer, ' ', len);

  if(NULL == strptr){                                           // no occurrance of delimiter means invalid command, return NULL
    *cmd = NULL;
    *value = NULL;
    return;
  }

  if(strlen(strptr)==len){                                      // no occurance of COMMAND, return NULL
    *cmd = NULL;
    *value = NULL;
    return;
  }

  if(strlen(strptr)==1){                                        // no occurance of VALUE, return NULL
    *cmd = NULL;
    *value = NULL;
    return;
  }
  *strptr = '\0';

  *cmd = input_buffer;                                         // return COMMAND:VALUE pair as two separate strings in the message buffer
  *value = (strptr + 1);
}
