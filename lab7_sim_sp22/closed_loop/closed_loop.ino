/*
 * closed_loop.ino
 *
 * EE16B Fall 2016
 * John Maidens, Emily Naviasky & Nathaniel Mailoa
 *
 * EE16B Fall 2017
 * Andrew Blatner
 *
 * EE16B Spring 2021
 * Hossein Najafi
 * EE16B Spring 2022
 * Yi-Hsuan Shih
 */

#define LEFT_MOTOR  10  // Input L
#define RIGHT_MOTOR   9   // Input R
#define encoderPinL   3 // Encoder L
#define encoderPinR   2 // Encoder R


#define RUN_TIME                    (20*1000)
#define SAMPLING_INTERVAL           50
#define SAMPLE_INIT         10

#define SAMPLE_LEN                 30+SAMPLE_INIT //(RUN_TIME/SAMPLING_INTERVAL)


#define JOLT_STEPS                  10

int step_num = 0;
volatile boolean do_loop = 1; // timer signal to increment timestep
volatile long encoderValueL = 0;
volatile long encoderValueR = 0;

int16_t deltaArr[SAMPLE_LEN] = {0};
uint16_t lpos[SAMPLE_LEN] = {0};
uint16_t rpos[SAMPLE_LEN] = {0};
uint8_t lpwm[SAMPLE_LEN] = {0};
uint8_t rpwm[SAMPLE_LEN] = {0};

/*---------------------------*/
/*      CODE BLOCK CON1      */
/*     From open_loop.ino    */
/*       with changes        */
/*---------------------------*/

float theta_left = ;
float theta_right = ;ㄘㄛ
float beta_left =;
float beta_right = ;
float v_star = ;

// maximum PWM inputs to jolt the car (don't change this)
int left_jolt =  255;
int right_jolt = 255;

// Control gains
float f_left = ;
float f_right = ;

/*---------------------------*/
/*      CODE BLOCK CON2      */
/*---------------------------*/

float drive_straight_left(float v_star, float delta) {
  // please write down the formula in this format 
  // (a1~e1 should be chosen from beta_left/theta_left/ f_left/ delta/ v_star)
  return //(a1 + b1 - c1 * d1) / e1;
}

float drive_straight_right(float v_star, float delta) {
  // please write down the formula in this format 
  // (a2~e2 should be chosen from beta_right/theta_right/ f_right/ delta/ v_star)
  return //(a2 + b2 + c2 * d2) / e2;
}


/*---------------------------*/
/*---------------------------*/
/*---------------------------*/

void setup(void) {
  Serial.begin(38400);

  pinMode(encoderPinL, INPUT_PULLUP); 
  pinMode(encoderPinR, INPUT_PULLUP);
  
  pinMode(LEFT_MOTOR, OUTPUT);
  
  pinMode(RIGHT_MOTOR, OUTPUT);
  

  write_pwm(0, 0); // Turn off motors
  attachInterrupt(digitalPinToInterrupt(encoderPinL), updateEncoderL,CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinR), updateEncoderR,CHANGE);

}

void loop(void) {
  if (do_loop) {
    // Apply maximum input for a short time to start motors
    if (step_num < JOLT_STEPS) {
      write_pwm(left_jolt, right_jolt);
      step_num++;
    }
    // If not done running
    else if (step_num < SAMPLE_LEN) {

      // Save positions because _left_position and _right_position
      // can change in the middle of one loop.
      int left_position =  encoderValueL; //left_encoder.pos;
      int right_position =  encoderValueR; //right_encoder.pos;

      /*---------------------------*/
      /*      CODE BLOCK CON0      */
      /*---------------------------*/

      /*--------------------------------------*/
      /*     Add the steady-state value of    */
      /*    delta from this calculation to    */
      /*    compensate for initial turning    */
      /*--------------------------------------*/
      int delta = left_position - right_position;

      // Drive straight using feedback
      // Compute the needed pwm values for each wheel using delta and v_star
      float left_cur_pwm = ;
      float right_cur_pwm = ;
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
      Serial.println("Start");
      Serial.println("delta - left pos - right pos - left pwm - right pwm");
      for (int i = SAMPLE_INIT; i < SAMPLE_LEN; i++) {
        Serial.print(deltaArr[i]);
        Serial.print(',');
        Serial.print(lpos[i]);
        Serial.print(',');
        Serial.print(rpos[i]);
        Serial.print(',');
        Serial.print(lpwm[i]);
        Serial.print(',');
        Serial.print(rpwm[i]);
        Serial.print('\n');
      }
      do_loop = 0;
    }
  }
  delay(100);
  encoderValueL = 0;
  encoderValueR = 0;
  delay(SAMPLING_INTERVAL);       
}

/*---------------------------*/
/*     Helper functions      */
/*---------------------------*/

void write_pwm(int pwm_left, int pwm_right) {
  analogWrite(LEFT_MOTOR, (int) min(max(0, pwm_left), 255));
  analogWrite(RIGHT_MOTOR, (int) min(max(0, pwm_right), 255));
}


/*---------------------------*/
/*        Interruptions      */
/*---------------------------*/

void updateEncoderL()
{
  encoderValueL++;
  
}
void updateEncoderR()
{
  encoderValueR++;
}