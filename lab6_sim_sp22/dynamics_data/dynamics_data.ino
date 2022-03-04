
/*---------------------------*/
/*          MODE CONTROL      */
/*---------------------------*/

bool MODE = 0;
#define PWM_CONST 			200
#define SAMPLING_INTERVAL	200 	// in ms

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/


/*---------------------------*/
/*      CODE BLOCK SID1      */
/*---------------------------*/

// Parameters for sweep of whole PWM range

#define SAMPLES_PER_PWM             1
#define SAMPLE_INIT					10
#define LOW_PWM                     20
#define HIGH_PWM                    240
#define PWM_STEP                    20

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/

// First half of samples are from (HIGH-STEP)->LOW_PWM
// Second half are from (LOW+STEP)->HIGH
#define NUM_PWM                     (2*(HIGH_PWM-LOW_PWM)/PWM_STEP)
#define SAMPLE_LEN                  (SAMPLES_PER_PWM*NUM_PWM)


// Pin map
#define motorPwmPinL 	10	// Input L
#define motorPwmPinR 	9 	// Input R
#define encoderPinL		3	// Encoder L
#define encoderPinR 	2	// Encoder R

// Pulse count from encoder
volatile long encoderValueL = 0;
volatile long encoderValueR = 0;

// Arrays for position, velocity, and pwm data storage
uint16_t lpos[SAMPLE_LEN+SAMPLE_INIT] = {0};
uint16_t rpos[SAMPLE_LEN+SAMPLE_INIT] = {0};
uint16_t pwm[SAMPLE_LEN+SAMPLE_INIT] = {0};

// Variables
int step_num = 0;
int cur_pwm = HIGH_PWM;
int dir = -1;

void setup () 
{
  
  Serial.begin (38400); // initialize serial communication
  
  // Set encoder as input with internal pullup  
  pinMode(encoderPinL, INPUT_PULLUP); 
  pinMode(encoderPinR, INPUT_PULLUP);
  
  // Set PWM connection as output
  pinMode(motorPwmPinL,OUTPUT); 
  pinMode(motorPwmPinR,OUTPUT); 

  // Initialize pwm
  write_pwm(0, 0); 	// Turn off motors
  // delay(500);	// Somehow this delay would affect a lot
  pwm_init(); 		//initialize pwm array
  
  // Attach interrupt 
  attachInterrupt(digitalPinToInterrupt(encoderPinL), updateEncoderL,CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinR), updateEncoderR,CHANGE);

} 

void loop()
{
  if (!MODE) {		// mode 0, for given PWM
  	write_pwm(PWM_CONST, PWM_CONST);
    Serial.print("Encoder values: ");
    Serial.print(encoderValueL);
    Serial.print(",");
    Serial.println(encoderValueR); 
  } else {			// mode 1, sweep samples
  if (step_num < SAMPLE_LEN+SAMPLE_INIT) {
	lpos[step_num] = encoderValueL;
    rpos[step_num] = encoderValueR;
    step_num++;
    write_pwm(pwm[step_num], pwm[step_num]); 
  } else {			// print all samples
    for(int i=0; i<SAMPLE_LEN; i++){
      Serial.print(pwm[i+SAMPLE_INIT]);
      Serial.print(",");
      Serial.print(lpos[i+SAMPLE_INIT]);
      Serial.print(",");
      Serial.println(rpos[i+SAMPLE_INIT]);
    }
    while(1);
  }}
  encoderValueL = 0;	// reset encoder value to 0
  encoderValueR = 0;
  delay(SAMPLING_INTERVAL);		// add delay

}


/*---------------------------*/
/*     Helper functions      */
/*---------------------------*/

void write_pwm(int pwm_left, int pwm_right) {
  analogWrite(motorPwmPinL, pwm_left);
  analogWrite(motorPwmPinR, pwm_right);
}

void pwm_init() {
  for (int i = 0; i < SAMPLE_LEN+SAMPLE_INIT; i++) {
    if (i<SAMPLE_INIT){
   	  pwm[i] = cur_pwm;
      continue;
    }
    pwm[i] = cur_pwm;
    if (i % SAMPLES_PER_PWM == 0) {
      cur_pwm +=dir * PWM_STEP;
      if (cur_pwm <= LOW_PWM || HIGH_PWM <= cur_pwm) {
        dir *= -1;
      }
    }
  }
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