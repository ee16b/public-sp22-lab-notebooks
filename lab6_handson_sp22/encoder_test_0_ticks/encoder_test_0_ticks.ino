/*
 * encoder_test_0_ticks.ino
 *
 * EE16B Fall 2017
 * Andrew Blatner
 *
 */

#define LEFT_MOTOR                  P2_0
#define LEFT_ENCODER                P6_2
#define RIGHT_MOTOR                 P1_5
#define RIGHT_ENCODER               P6_3

#define PUSH_START                  PUSH1

#define MAX_TICK_ERROR              4
#define SAMPLING_INTERVAL           250
#define SAMPLES_PER_PHASE           16
#define NUM_PHASES                  4

#define SAMPLE_LEN                  (SAMPLES_PER_PHASE*NUM_PHASES)

// Operation modes
#define MODE_LISTEN                 0 // Car waits for button press to start test
#define MODE_DRIVE                  1
#define MODE_STOP                   2

int loop_mode = MODE_LISTEN;

int step_num = 0;
int phase = 0;
volatile boolean do_loop = 0; // timer signal to increment timestep

// Use 240 instead of 255 so PWM signal has edges on the oscilloscope
// Better for checking that there is a PWM signal from the MSP
int phase_pwms[2][NUM_PHASES] = {
  {0, 240, 0, 240},
  {0, 0, 240, 240},
};

int phase_results[NUM_PHASES][2] = {{0}};

int deltaArr[NUM_PHASES][SAMPLES_PER_PHASE] = {{0}};
int lpos[NUM_PHASES][SAMPLES_PER_PHASE] = {{0}};
int rpos[NUM_PHASES][SAMPLES_PER_PHASE] = {{0}};
int lpwm[NUM_PHASES][SAMPLES_PER_PHASE] = {{0}};
int rpwm[NUM_PHASES][SAMPLES_PER_PHASE] = {{0}};

int _left_pwm = 0;
int _right_pwm = 0;

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

  Serial.println("\nPress left button (labeld P2.1) to start test.");

  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(LEFT_ENCODER, INPUT);
  pinMode(RIGHT_MOTOR, OUTPUT);
  pinMode(RIGHT_ENCODER, INPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(PUSH_START, INPUT_PULLUP);

  write_pwm(0, 0);

  setTimer();

  start_listen_mode();
}

void loop(void) {
  check_encoders();
  // Wait for button press to start test
  if (loop_mode == MODE_LISTEN) {
    if (!digitalRead(PUSH_START)) {
      start_drive_mode();
      next_phase();
    }
  }
  else if (loop_mode == MODE_DRIVE) {
    if (do_loop) {
      // blink the LED's while testing
      digitalWrite(RED_LED, step_num % 2);
      digitalWrite(GREEN_LED, step_num % 2);

      write_pwm(phase_pwms[0][phase], phase_pwms[1][phase]);
      collect_data();

      step_num++;
      if (step_num == SAMPLES_PER_PHASE) {
        show_phase_results();
        phase++;
        if (phase == NUM_PHASES) {
          stop_tests();
        }
        else {
          next_phase();
        }
      }
    }

    do_loop = 0;
  }
  else if (loop_mode == MODE_STOP) {
    print_results();
  }
}

/*---------------------------*/
/*     Helper functions      */
/*---------------------------*/

void start_listen_mode(void) {
  loop_mode = MODE_LISTEN;
  write_pwm(0, 0);
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(RED_LED, HIGH);
}

// Switch to testing mode from listen mode
void start_drive_mode(void) {
  loop_mode = MODE_DRIVE;
  write_pwm(0, 0);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(RED_LED, LOW);
  Serial.println("PICK YOUR CAR UP.");
  delay(3000);
}

void collect_data(void) {
  // Save positions because they can change during this function
  int left_pos = left_encoder.pos;
  int right_pos = right_encoder.pos;
  lpos[phase][step_num] = left_pos;
  rpos[phase][step_num] = right_pos;
  deltaArr[phase][step_num] = left_pos - right_pos;
  lpwm[phase][step_num] = _left_pwm;
  rpwm[phase][step_num] = _right_pwm;
}

// Reset counters for next phase
void next_phase(void) {
  Serial.print("Starting phase ");
  Serial.print(phase);
  Serial.print("... ");
  step_num = 0;
  left_encoder.pos = 0;
  right_encoder.pos = 0;
}

void show_phase_results(void) {
  write_pwm(0, 0);
  int left_res = lpos[phase][SAMPLES_PER_PHASE-1] - lpos[phase][0];
  int right_res = rpos[phase][SAMPLES_PER_PHASE-1] - rpos[phase][0];
  phase_results[phase][0] = left_res;
  phase_results[phase][1] = right_res;

  if ((0 <= left_res && left_res <= MAX_TICK_ERROR) &&
      (0 <= right_res && right_res <= MAX_TICK_ERROR)) {
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(RED_LED, LOW);
    if (phase == 0) {
      Serial.println("Success.");
    } else {
      Serial.println("Success if encoder wheels are off. Failure if encoder wheels are on.");
    }
  }
  else {
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(RED_LED, HIGH);
    if (phase == 0) {
      Serial.println("Failure.");
    } else {
      Serial.println("Failure if encoder wheels are off. Success if encoder wheels are on.");
    }
  }
  delay(1000);
}

void stop_tests(void) {
  loop_mode = MODE_STOP;
  write_pwm(0, 0);
  Serial.println("Tests finished.");
}

void print_results(void) {
  Serial.println("Encoder ticks per phase:");
  for (int i = 0; i < NUM_PHASES; i++) {
    Serial.print(phase_results[i][0]);
    Serial.print("\t");
    Serial.println(phase_results[i][1]);
  }
  Serial.println("Raw data:");
  for (int i = 0; i < SAMPLE_LEN; i++) {
    if (i % SAMPLES_PER_PHASE == 0) {
      Serial.println("delta-lpos-rpos-lpwm-rpwm");
    }
    Serial.print((*deltaArr)[i]);
    Serial.print(',');
    Serial.print((*lpos)[i]);
    Serial.print(',');
    Serial.print((*rpos)[i]);
    Serial.print(',');
    Serial.print((*lpwm)[i]);
    Serial.print(',');
    Serial.print((*rpwm)[i]);
    Serial.print('\n');
  }
}

void write_pwm(int pwm_left, int pwm_right) {
  _left_pwm = (int) min(max(0, pwm_left), 255);
  _right_pwm = (int) min(max(0, pwm_right), 255);
  analogWrite(LEFT_MOTOR, _left_pwm);
  analogWrite(RIGHT_MOTOR, _right_pwm);
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

/*---------------------------*/
/*    Interrupt functions    */
/*---------------------------*/

/***
Previous to fall 2017, the encoder counting was done using digital interrupts.
This was extremely prone to noise. Checking with an oscilloscope at the input
pins of the Launchpad, I found the following:
  Pertubation of constant 3.3V signal: ~80ns, min 0.59V->max 5.77V
  Similar for constant 0V and for a square wave produced by actually using
  the encoder wheel
Any hardware filtering would have to be done on the breadboard.
  A large (470uF, 1000uF) capacitor on the breadboard between the encoder
  signal and ground and fails to smooth these pertuabations. This is likely
  because the breadboard ground is not the same as the Launchpad ground.
  A capacitor of this size still changes the square wave to curved sawtooth,
  so the 80ns spikes are likely unavoidable by this type of filtering.

Instead we poll the encoders on analog inputs in the main loop and manually
detect transitions. A first-order IIR filter smooths out this noise.

Motivations for software solution:
It's easily implemented and 100% repeatable. The problem we're solving is out
of scope of 16B lab, so we don't want students spending time debugging this.

Test results:
  Previous interrupt-based counting (rising edges only):
  23  829
  Analog polling (both rising and falling edges):
  0   1323

-- Andrew Blatner
***/

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
