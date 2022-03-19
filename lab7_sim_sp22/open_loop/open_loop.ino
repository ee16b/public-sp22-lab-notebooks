/*
 * open_loop.ino
 *
 * EE16B Fall 2016
 * John Maidens, Emily Naviasky & Nathaniel Mailoa
 *
 * EE16B Fall 2017
 * Andrew Blatner
 *
 */


#define RUN_TIME                    (15*1000)
#define   RED_LED       13
#define   GREEN_LED   12
#define LEFT_MOTOR  10  // Input L
#define RIGHT_MOTOR   9   // Input R
#define encoderPinL   3 // Encoder L
#define encoderPinR   2 // Encoder R

unsigned long end_time = 0;

/*---------------------------*/
/*      CODE BLOCK CON1      */
/*---------------------------*/

float theta_left = ;
float theta_right = ;
float beta_left = ;
float beta_right = ;
float v_star = ;

// PWM inputs to jolt the car straight
int left_jolt =  ;
int right_jolt = ;

/*---------------------------*/
/*      CODE BLOCK CON2      */
/*---------------------------*/

float drive_straight_left(float v_star) {
  return ;
}

float drive_straight_right(float v_star) {
  return ;
}

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/


void setup(void) {
  Serial.begin (38400); // initialize serial communication
  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(RIGHT_MOTOR, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  write_pwm(0, 0); // Turn off motors
  delay(2000); // Wait 2 seconds to put down car
  Serial.println("Start Jolt");
  reset_blinker(); // Blink lights to indicate car is running
  write_pwm(left_jolt, right_jolt); // Jolt motors for 200ms
  delay(200);
  Serial.println("Start Open-loop");

  /*---------------------------*/
  /*      CODE BLOCK CON0      */
  /*---------------------------*/

  // Attempt to drive straight using open loop control
  // Compute the PWM input required for each wheel based on v_star
  int left_cur_pwm = ;
  int right_cur_pwm = ;
  write_pwm(left_cur_pwm, right_cur_pwm);
	
  /*---------------------------*/
  /*---------------------------*/
  /*---------------------------*/

  end_time = millis() + RUN_TIME;
}

void loop(void) {
}

/*---------------------------*/
/*     Helper functions      */
/*---------------------------*/

void write_pwm(int pwm_left, int pwm_right) {
  analogWrite(LEFT_MOTOR, (int) min(max(0, pwm_left), 255));
  analogWrite(RIGHT_MOTOR, (int) min(max(0, pwm_right), 255));
  Serial.print("MOTOR PWM Left: ");
  Serial.print(pwm_left);
  Serial.print(",");
  Serial.print("MOTOR PWM Right: ");
  Serial.println(pwm_right);
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