/*
 * dynamics_data.ino
 *
 * Collects dynamics data (position) with
 * some varying (bounded) input PWM
 *
 * EE16B Fall 2016
 * John Maidens, Emily Naviasky & Nathaniel Mailoa
 *
 * EE16B Fall 2017
 * Andrew Blatner
 *
 * EE16B Spring 2019
 * Kevin Zhang
 */
#include <MspFlash.h>


#define LEFT_MOTOR                  P2_0
#define LEFT_ENCODER                P6_2
#define RIGHT_MOTOR                 P1_5
#define RIGHT_ENCODER               P6_3
#define WRITE                       1 //change to 1 to collect data, 0 to read collected data
#define PUSH_START                  PUSH1

#define flash1 SEGMENT_D
#define flash2 SEGMENT_B



int do_write = 1;
int do_read = 1;
/*---------------------------*/
/*      CODE BLOCK SID1      */
/*---------------------------*/

// Parameters for sweep of whole PWM range
#define SAMPLING_INTERVAL           500 // in ms
#define SAMPLES_PER_PWM             2
#define LOW_PWM                     100
#define HIGH_PWM                    200
#define PWM_STEP                    5

// Parameters for second sweep
// #define SAMPLING_INTERVAL           500 // in ms
// #define SAMPLES_PER_PWM             4
// #define LOW_PWM                     // your value here!
// #define HIGH_PWM                    // your value here!
// #define PWM_STEP                    5

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/

// First half of samples are from (HIGH-STEP)->LOW_PWM
// Second half are from (LOW+STEP)->HIGH
#define NUM_PWM                     (2*(HIGH_PWM-LOW_PWM)/PWM_STEP)
#define SAMPLE_LEN                  (SAMPLES_PER_PWM*NUM_PWM)

// Arrays for position, velocity, and pwm data storage
uint16_t lpos[SAMPLE_LEN] = {0};
uint16_t rpos[SAMPLE_LEN] = {0};
uint8_t pwm[SAMPLE_LEN] = {0};

int step_num = 0;
volatile boolean do_loop = 0; // timer signal to increment timestep

int cur_pwm = HIGH_PWM;
int dir = -1;

typedef struct encoder {
  int pin;
  int pos;
  bool level;
  int avg;
} encoder_t;

encoder_t left_encoder = {LEFT_ENCODER, 0, LOW, 0};
encoder_t right_encoder = {RIGHT_ENCODER, 0, LOW, 0};

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
  delay(3000); // Wait 3 seconds to put down car
  pwm_init(); //initialize pwm array


  while (digitalRead(PUSH_START) && WRITE) {

  }
  // Start motors
  reset_blinker();
  if (WRITE) write_pwm(255, 255);
  delay(500);
  if (WRITE) write_pwm(cur_pwm, cur_pwm);
  delay(1000); // Wait before starting to record
  // Set timer for timestep
  setTimer();
}

void loop(void) {
  check_encoders();
  if (do_loop && WRITE) {
    // If not done collecting data
    if (step_num < SAMPLE_LEN) {
      lpos[step_num] = left_encoder.pos;
      rpos[step_num] = right_encoder.pos;
      write_pwm(pwm[step_num], pwm[step_num]);
      step_num++;
    }
    else { // If done collecting data
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
  unsigned char p;
  unsigned char lv;
  unsigned char rv;
  Flash.erase(flash1);
  Flash.erase(flash2);
  for(int i = 1; i < SAMPLE_LEN; i++) {
    p = (unsigned char) pwm[i];
    lv = (unsigned char) (lpos[i] - lpos[i-1]);
    rv = (unsigned char) (rpos[i] - rpos[i-1]);

    /*For your debugging pleasure
    Serial.print(pwm[i]);
    Serial.print(',');
    Serial.print(lv);
    Serial.print(',');
    Serial.println(rv);*/

    if (i < SAMPLE_LEN / 2) {
      Flash.write(flash1 + 2*(i-1), &lv, 1);
      Flash.write(flash1 + 2*(i-1) + 1, &rv , 1);

    } else {
      Flash.write(flash2 + 2*(i - (SAMPLE_LEN/2)), &lv, 1);
      Flash.write(flash2 + 2*(i - (SAMPLE_LEN/2)) + 1, &rv, 1);
    }

  }
}
/*Reads information gathered and put into flash
*/
void read_from_flash() {
  unsigned char p;
  unsigned char lv;
  unsigned char rv;
  Serial.println("pwm, lv, rv");
  for(int i = 1; i < SAMPLE_LEN; i++) {
    if (i< SAMPLE_LEN / 2) {
      Flash.read(flash1 + 2*(i-1), &lv, 1);
      Flash.read(flash1 + 2*(i-1) + 1, &rv, 1);
    } else {
      Flash.read(flash2 + 2*(i - (SAMPLE_LEN/2)),&lv, 1);
      Flash.read(flash2 + 2*(i - (SAMPLE_LEN/2)) + 1, &rv, 1);
    }

    Serial.print( (int) pwm[i]);
    Serial.print(',');
    Serial.print((int) lv);
    Serial.print(',');
    Serial.print((int) rv);
    Serial.print('\n');
  }

}

//initialize the PWM sweep t
void pwm_init() {
  for (int i = 0; i < SAMPLE_LEN; i++) {
    pwm[i] = cur_pwm;
    if (i % SAMPLES_PER_PWM) {
      cur_pwm +=dir * PWM_STEP;
      if (cur_pwm <= LOW_PWM || HIGH_PWM <= cur_pwm) {
        dir *= -1;
      }
    }
  }
}
/*---------------------------*/
/*    Interrupt functions    */
/*---------------------------*/
#define AVG_DECAY_RATE              0.3
#define LOW_THRESH                  ((int) (0.1*4096))
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
