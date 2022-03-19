/*
 * closed_loop.ino
 *
 * EE16B Fall 2016
 * John Maidens, Emily Naviasky & Nathaniel Mailoa
 *
 * EE16B Fall 2017
 * Andrew Blatner
 *
 */
#include <MspFlash.h>

#define LEFT_MOTOR                  P2_0
#define LEFT_ENCODER                P6_2
#define RIGHT_MOTOR                 P1_5
#define RIGHT_ENCODER               P6_3

#define RUN_TIME                    (20*1000)
#define SAMPLING_INTERVAL           100
#define SAMPLE_LEN                  (RUN_TIME/SAMPLING_INTERVAL)
#define WRITE                       1
#define PUSH_START                  PUSH1

#define JOLT_STEPS                  2

#define flash1 SEGMENT_D
#define flash2 SEGMENT_B

int step_num = 0;
volatile boolean do_loop = 0; // timer signal to increment timestep

int do_write = 1;
int do_read = 1;

int16_t deltaArr[SAMPLE_LEN] = {0};
uint16_t lpos[SAMPLE_LEN] = {0};
uint16_t rpos[SAMPLE_LEN] = {0};
uint8_t lpwm[SAMPLE_LEN] = {0};
uint8_t rpwm[SAMPLE_LEN] = {0};

typedef struct encoder {
  int pin;
  int pos;
  bool level;
  int avg;
} encoder_t;

encoder_t left_encoder = {LEFT_ENCODER, 0, LOW, 0};
encoder_t right_encoder = {RIGHT_ENCODER, 0, LOW, 0};

/*---------------------------*/
/*      CODE BLOCK CON1      */
/*     From open_loop.ino    */
/*       with changes        */
/*---------------------------*/

float theta_left = ;
float theta_right = ;
float beta_left = ;
float beta_right = ;
float v_star =;

// PWM inputs to jolt the car straight
int left_jolt = ;
int right_jolt = ;

// Control gains
float f_left =;
float f_right =;

/*---------------------------*/
/*      CODE BLOCK CON2      */
/*---------------------------*/

float driveStraight_left(float v_star, float delta) {
  return;
}

float driveStraight_right(float v_star, float delta) {
  return;
}

/*---------------------------*/
/*      CODE BLOCK CON3      */
/*---------------------------*/

float delta_ss = 0;

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/

void setup(void) {
  Serial.begin(38400);

  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(LEFT_ENCODER, INPUT);
  pinMode(RIGHT_MOTOR, OUTPUT);
  pinMode(RIGHT_ENCODER, INPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(PUSH_START, INPUT_PULLUP);


  write_pwm(0, 0); // Turn off motors
  delay(2000); // Wait 2 seconds to put down car
  
  while (digitalRead(PUSH_START) && WRITE) {

  }
  reset_blinker(); // Blink lights to indicate car is running
  setTimer(); // Set timer for timestep
}

void loop(void) {
  check_encoders();
  if (do_loop && WRITE) {
    // Apply maximum input for a short time to start motors
    if (step_num < JOLT_STEPS) {
      write_pwm(left_jolt, right_jolt);
      step_num++;
    }
    // If not done running
    else if (step_num < SAMPLE_LEN) {

      // Save positions because _left_position and _right_position
      // can change in the middle of one loop.
      int left_position = left_encoder.pos;
      int right_position = right_encoder.pos;

      /*---------------------------*/
      /*      CODE BLOCK CON0      */
      /*---------------------------*/
      
      /*--------------------------------------*/
      /*     Add the steady-state value of    */
      /*    delta from this calculation to    */
      /*    compensate for initial turning    */
      /*--------------------------------------*/
      float delta = left_position - right_position + delta_ss;

      // Drive straight using feedback
      // Compute the needed pwm values for each wheel using delta and v_star
      int left_cur_pwm = ;
      int right_cur_pwm = ;
      write_pwm(left_cur_pwm, right_cur_pwm);

      /*---------------------------*/
      /*---------------------------*/
      /*---------------------------*/

      lpos[step_num] = left_position;
      rpos[step_num] = right_position;
      deltaArr[step_num] = delta;
      lpwm[step_num] = left_cur_pwm;
      rpwm[step_num] = right_cur_pwm;

      step_num++;
    }

    else { // When step_num has reached SAMPLE_LEN
      // Turn off motors
      write_pwm(0, 0);

      // Print out result
      if (do_write) {
        Serial.println("writing");
        write_to_flash();
        do_write = 0;
      }
    }
    do_loop = 0;
  } else if (!WRITE) {
    //stop wheels
    write_pwm(0,0);
    //read and print to serial monitor
    if (do_read) {
      read_from_flash();
      do_read = 0;
    }
  }
}

/*---------------------------*/
/*     Helper functions      */
/*---------------------------*/

void write_pwm(int pwm_left, int pwm_right) {
  analogWrite(LEFT_MOTOR, (int) min(max(0, pwm_left), 255));
  analogWrite(RIGHT_MOTOR, (int) min(max(0, pwm_right), 255));
}

void reset_blinker(void) {
  digitalWrite(RED_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, LOW);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(GREEN_LED, LOW);
}

/*Runs after data is collected
Stores (pwm, lvel, rvel) as 3 byte chunks
Problems if:
  SAMPLE_LEN is too long (flash mem is small TwT)
  Car goes too fast - Byte cannot store speeds above 255
*/
void write_to_flash() {
  unsigned char ss;
  Flash.erase(flash1);
  Flash.erase(flash2);
  for(int i = 1; i < SAMPLE_LEN; i++) {

    ss = (unsigned char) (lpos[i] - rpos[i]);

    if (i < SAMPLE_LEN / 2) {
      Flash.write(flash1 + (i-1), &ss, 1);

    } else {
      Flash.write(flash2 + (i - (SAMPLE_LEN/2)), &ss, 1);
    }
  }
}

/*Reads information gathered and put into flash
*/
void read_from_flash() {
  unsigned char ss;
  Serial.println("delta");
  for(int i = 1; i < SAMPLE_LEN; i++) {
    if (i< SAMPLE_LEN / 2) {
      Flash.read(flash1 + (i-1), &ss, 1);
    } else {
      Flash.read(flash2 + (i - (SAMPLE_LEN/2)),&ss, 1);
    }

    Serial.println( (int8_t) ss);
  }
}

/*---------------------------*/
/*    Interrupt functions    */
/*---------------------------*/

#define AVG_DECAY_RATE              0.3
#define LOW_THRESH                  ((int) (0.2*4096))
#define HIGH_THRESH                 ((int) (0.4*4096))

void check_encoder(encoder_t* enc) {
  int new_val = analogRead(enc->pin);
  enc->avg = (int) (AVG_DECAY_RATE*enc->avg + (1 - AVG_DECAY_RATE)*new_val);
  if ((enc->level == LOW && HIGH_THRESH < enc->avg) ||
      (enc->level == HIGH && enc->avg < LOW_THRESH)) {
    enc->pos++;
    enc->level = !enc->level;
  }
}

void check_encoders(void) {
  check_encoder(&left_encoder);
  check_encoder(&right_encoder);
}

// Set timer for timestep; use B0 to free up all other PWM ports
void setTimer(void) {
  TB0CCR0 = (unsigned int) (32.768*SAMPLING_INTERVAL); // set the timer based on 32kHz clock
  TB0CCTL0 = CCIE; // enable interrupts for Timer B
  __bis_SR_register(GIE);
  TB0CTL = TASSEL_1 + MC_1 + TACLR + ID_0;
}

// ISR for timestep
#pragma vector=TIMER0_B0_VECTOR    // Timer B ISR
__interrupt void Timer0_B0_ISR(void) {
  do_loop = 1;
}
