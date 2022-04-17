/*
 * integration.ino
 * Final sketch for SIXT33N Speech version
 *
 * EE16B Fall 2016
 * Emily Naviasky & Nathaniel Mailoa
 *
 * EE16B Fall 2017
 * Andrew Blatner
 *
 * EE16B Spring 2021
 * Zhongkai Wang
 *
 * EECS16B Fall 2021 & EECS16B Spring 2022
 * Bozhi Yin
 *
 */

/********************************************************/
/********************************************************/
/***                                                  ***/
/*** Constants and global variables from turning.ino. ***/
/***                                                  ***/
/********************************************************/
/********************************************************/

#define LEFT_MOTOR    10   // Input L
#define RIGHT_MOTOR   9    // Input R
#define encoderPinL   3    // Encoder L
#define encoderPinR   2    // Encoder R

#define RUN_TIME                    (20*1000)
#define SAMPLING_INTERVAL           200
#define SAMPLE_INIT                 10

#define SAMPLE_LEN                  50+SAMPLE_INIT //(RUN_TIME/SAMPLING_INTERVAL)
#define JOLT_STEPS                  0


// Operation modes
#define MODE_LISTEN                 0
#define MODE_DRIVE                  1

#define NUM_COMMANDS                4    // YOUR CODE HERE
#define DRIVE_FAR                   0
#define DRIVE_LEFT                  1
#define DRIVE_CLOSE                 2
#define DRIVE_RIGHT                 3

int mode = 0; // 0 - plot mode; 1 - data collection mode
boolean loop_mode = MODE_DRIVE;
int drive_mode;
int program_count;

int step_num = 0;
volatile boolean do_loop = 1;       // timer signal to increment timestep
volatile long encoderValueL = 0;
volatile long encoderValueR = 0;

float x_pos = 0.00;
float y_pos = 0.00;
float theta_pos = 0.00;
float old_delta = 0.00;
float delta = 0.00;

/*---------------------------*/
/*      CODE BLOCK CON1      */
/*      From turning.ino     */
/*---------------------------*/
// YOUR CODE HERE

float theta_left = ...;
float theta_right = ...;
float beta_left = ...;
float beta_right =  ...;

// Use the following v_star value, do not need to change this
float v_star = ...;

// PWM inputs to jolt the car straight
float left_jolt = ...;
float right_jolt = ...;

// Control gains
float f_left = ...;
float f_right = ...;

/*---------------------------*/
/*      CODE BLOCK CON2      */
/*      From turning.ino     */
/*---------------------------*/
// YOUR CODE HERE

float driveStraight_left(float delta,float v_star) {
  return ...;
}

float driveStraight_right(float delta, float v_star) {
  return ...;
}

/*---------------------------*/
/*      CODE BLOCK CON3      */
/*      From turning.ino     */
/*---------------------------*/

#define CAR_WIDTH                   60.0 // in cm
#define TURN_RADIUS                 (v_star/5.0 * 80.0)  // in cm - 6 feet diameter

/*---------------------------*/
/*    PREPROGRAMMED PATH     */
/*---------------------------*/

float delta_reference(int i) {
  // YOUR CODE HERE
  // We divide by 5 because our v_star was collected at a 5x slower sampling rate
  float ...;
  if (drive_mode == DRIVE_RIGHT) {
    return ...;
  }
  else if (drive_mode == DRIVE_LEFT) {
    return ...;
  }
  else { // DRIVE_FAR, DRIVE_CLOSE
    return ...;
  }
}

/*********************************************************/
/*********************************************************/
/***                                                   ***/
/*** Constants and glboal variables from classify.ino. ***/
/***                                                   ***/
/*********************************************************/
/*********************************************************/

#define SIZE                        200
#define SIZE_AFTER_FILTER           1

/*---------------------------*/
/*      CODE BLOCK PCA1      */
/*     From classify.ino     */
/*---------------------------*/
// YOUR CODE HERE

// Enveloping and K-means constants
#define SNIPPET_SIZE                40
#define PRELENGTH                   5
#define THRESHOLD                   0.5

#define EUCLIDEAN_THRESHOLD         1000
#define COMMANDS_SIZE				SNIPPET_SIZE * NUM_COMMANDS

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/


/*---------------------------*/
/*      CODE BLOCK PCA2      */
/*     From classify.ino     */
/*---------------------------*/
// YOUR CODE HERE

/*---------------------------*/
/*---- SNIPPET_SIZE = 40 ----*/
float pca_vec1[SNIPPET_SIZE]={-0.0444, -0.0559, -0.0717, -0.0794, -0.1604, -0.2079, -0.3908, -0.4096, -0.3663, -0.2694, -0.1779, -0.1017, -0.0253, -0.0192, -0.0253, -0.0337, -0.0199, -0.0051, -0.0041, 0.0186, 0.0729, 0.1628, 0.1422, 0.1128, 0.1144, 0.1257, 0.1302, 0.1527, 0.1772, 0.1813, 0.1837, 0.1679, 0.1543, 0.1327, 0.1241, 0.1049, 0.0857, 0.0617, 0.0394, 0.0222};
float pca_vec2[SNIPPET_SIZE]={0.002, 0.0245, 0.0598, 0.039, -0.0164, -0.2627, -0.2961, -0.2424, -0.1003, 0.0233, 0.072, 0.1358, 0.1448, 0.2378, 0.2954, 0.3188, 0.305, 0.2857, 0.258, 0.2032, 0.0869, -0.0748, -0.0937, -0.0689, -0.0763, -0.0935, -0.1017, -0.1245, -0.1515, -0.1678, -0.1584, -0.1428, -0.1148, -0.0864, -0.0703, -0.0529, -0.0338, -0.0122, 0.0132, 0.0359};

float projected_mean_vec[2]={-961.2687504840644, -13.282259372108769};

float centroid1[2]={-1200.0910899539192, -563.0522059102683};
float centroid2[2]={341.39762974722584, 409.9548874752801};
float centroid3[2]={-291.7675833294767, 714.9861609621663};
float centroid4[2]={1150.46104353617, -561.8888425271779};
float* centroids[4] = {
  (float *) &centroid1, (float *) &centroid2,
  (float *) &centroid3, (float *) &centroid4
};

int16_t word_array[COMMANDS_SIZE] = {194, 263, 281, 304, 407, 1213, 1664, 1497, 1276, 987, 653, 296, 200, 202, 147, 73, 62, 48, 37, 17, 4, 3, 2, 2, 3, 4, 2, 34, 57, 46, 6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 40, 55, 83, 169, 210, 239, 282, 350, 344, 408, 407, 422, 458, 355, 380, 395, 366, 422, 381, 295, 313, 348, 330, 246, 195, 195, 149, 132, 115, 163, 167, 162, 190, 136, 195, 163, 158, 211, 168, 184, 0, 0, 30, 151, 296, 422, 474, 478, 707, 770, 696, 837, 710, 592, 585, 523, 521, 576, 415, 311, 167, 78, 66, 27, 24, 29, 14, 8, 6, 2, 1, 0, 0, 0, 0, 0, 34, 55, 212, 162, 5, 211, 173, 227, 273, 365, 289, 250, 396, 341, 30, 58, 47, 7, 69, 2, 8, 673, 547, 236, 276, 253, 514, 606, 634, 555, 620, 530, 444, 330, 323, 225, 187, 161, 76, 29, 6, 2, 1, 2};

/*---------------------------*/
/*---------------------------*/

float proj1 = 0;
float proj2 = 0;
// float proj3 = 0; // Uncomment this line if you're using 3 PCA components

char word1_str[] = "Drive Far";
char word2_str[] = "Drive Left";
char word3_str[] = "Drive Close";
char word4_str[] = "Drive Right";
char* word_str[] = {(char *) &word1_str, (char *) &word2_str, (char *) &word3_str, (char *) &word4_str};

float x_pos_arr[NUM_COMMANDS+1] = {0};
float y_pos_arr[NUM_COMMANDS+1] = {0};
int word_idx[NUM_COMMANDS] = {0};

/*---------------------------*/
/*       Norm functions      */
/*---------------------------*/

// Compute the L2 norm of (dim1, dim2) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        centroid: size-2 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm(float dim1, float dim2, float* centroid) {
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2));
}

// Compute the L2 norm of (dim1, dim2, dim3) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        dim3: 3rd dimension coordinate
//        centroid: size-3 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm3(float dim1, float dim2, float dim3, float* centroid) {
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2) + pow(dim3-centroid[2],2));
}

int sample_lens = 12600;
int down_sample_ratio = 1;

void setup(void) {
  Serial.begin(38400);

  pinMode(encoderPinL, INPUT_PULLUP); 
  pinMode(encoderPinR, INPUT_PULLUP);
  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(RIGHT_MOTOR, OUTPUT);

  // for (int i = 0; i < NUM_COMMANDS; i++) {
  //  sample_lens[i] = run_times[i] / SAMPLING_INTERVAL;
  // }

  write_pwm(0, 0);
  attachInterrupt(digitalPinToInterrupt(encoderPinL), updateEncoderL,CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinR), updateEncoderR,CHANGE);
  
  write_pwm(0, 0); // Turn off motors
  start_listen_mode();
  do_loop = 1;
}

void loop(void) {
  if (mode == 1){
    down_sample_ratio = 11;
  }
  if (program_count == NUM_COMMANDS) {
      do_loop = 0;
      start_drive_mode();
      x_pos = 0.00;
      y_pos = 0.00;
      theta_pos = 0.00;
      delta = 0.00;
      old_delta =0.00;
    if (mode == 0){
      Serial.println("Done.");
      Serial.println("Positions after each command.");
      for(int j = 0; j < NUM_COMMANDS+1; j++){
        Serial.print(x_pos_arr[j]);
        Serial.print(",");
        Serial.println(y_pos_arr[j]);
      }
      
      Serial.println("The commands are:");
      for(int j = 0; j < NUM_COMMANDS; j++){
        Serial.println(word_str[word_idx[j]]);
      }
    }
      program_count++;
  }
  else if (loop_mode == MODE_LISTEN) {
    /*---------------------------*/
    /*      CODE BLOCK PCA3      */
    /*---------------------------*/
    proj1 = 0;
    proj2 = 0;
    // proj3 = 0; // Uncomment this line if you're using 3 PCA components
    
    // Project 'word_array' onto the principal components
    for (int i = 0; i < SNIPPET_SIZE; i++) {
      proj1 += pca_vec1[i] * word_array[i+program_count*SNIPPET_SIZE];
      proj2 += pca_vec2[i] * word_array[i+program_count*SNIPPET_SIZE];
      // proj3 += pca_vec3[i] * word_array[i+program_count*SNIPPET_SIZE]; // Uncomment this line if you're using 3 PCA components
    }
    
    // YOUR CODE HERE
    // Demean the projection
    proj1 -= ...;
    proj2 -= ...;
    // proj3 -= ...; // Uncomment this line if you're using 3 PCA components

    // Classification
    // Use the function 'l2_norm' defined above
    // ith centroid: 'centroids[i]'
    float best_dist = 999999;
    int best_index = -1;
    // YOUR CODE HERE: set 'best_dist' and 'best_index' to appropriate values.
    ...

    // Compare 'best_dist' against the 'EUCLIDEAN_THRESHOLD' and print the result
    // If 'best_dist' is less than the 'EUCLIDEAN_THRESHOLD', the recording is a word
    // Otherwise, the recording is noise
    if (mode == 0){
    	if (best_dist < EUCLIDEAN_THRESHOLD) {
      		Serial.print("Classified as word:");
      		Serial.println(word_str[best_index]);
    	}
    	else {
      		Serial.print("Not classified. The closest is word: ");
      		Serial.println(word_str[best_index]);
    	}
    }
        
    /*---------------------------*/
    /*---------------------------*/
    /*---------------------------*/
    drive_mode = best_index;    
    // drive_mode = drive_modes[program_count];
    start_drive_mode();
    // Serial.print(program_count);
  }
  else if (loop_mode == MODE_DRIVE && do_loop) {
    if (step_num < JOLT_STEPS) {
      write_pwm(left_jolt, right_jolt);
    }
    else {
    
      // Save positions because _left_position and _right_position
      // can change in the middle of one loop.
      int left_position = encoderValueL;
      int right_position = encoderValueR;

      /*---------------------------*/
      /*      CODE BLOCK CON0      */
      /*---------------------------*/
      old_delta = delta;
      
      delta = left_position - right_position;
      delta = delta + delta_reference(step_num-JOLT_STEPS);
      theta_pos = theta_pos + (delta-old_delta)/CAR_WIDTH;
    
      x_pos = x_pos + cos(theta_pos)*v_star;
      y_pos = y_pos + sin(theta_pos)*v_star;
      
      float rad = sqrt(x_pos*x_pos + y_pos*y_pos);
      
      if (step_num % down_sample_ratio == 0 or step_num == sample_lens){
      	Serial.print(x_pos/TURN_RADIUS);
      	Serial.print(",");
      	Serial.println(y_pos/TURN_RADIUS);
      }
      // Drive straight using feedback
      // Compute the needed pwm values for each wheel using delta and v_star
      int left_cur_pwm = driveStraight_left(delta, v_star);
      int right_cur_pwm = driveStraight_right(delta, v_star);
      write_pwm(left_cur_pwm, right_cur_pwm);

      /*---------------------------*/
      /*---------------------------*/
      /*---------------------------*/
    }

    // Counter for how many times loop is executed since entering DRIVE MODE
    step_num++;

    if (step_num == sample_lens) {
      // Completely stop and go back to listen MODE after 3 seconds
      delta = 0.00;
      if (mode == 0){
        Serial.print("Executing Next Command. \n\n");
      } else {
        Serial.print("10000");
        Serial.print(",");
        Serial.println("10000");
      }
      
      program_count++;
      start_listen_mode();
    }

  }
}
  
  

/*---------------------------*/
/*     Helper functions      */
/*---------------------------*/

void start_drive_mode(void) {
  loop_mode = MODE_DRIVE;
  step_num = 0;
  if (drive_mode == DRIVE_CLOSE){
    sample_lens = 12600/SAMPLING_INTERVAL;
  }
  else {
    sample_lens = 25200/SAMPLING_INTERVAL;
  }
  word_idx[program_count] = drive_mode;  
}

void start_listen_mode(void) {
  write_pwm(0, 0);
  delay(300);
  loop_mode = MODE_LISTEN;
  x_pos_arr[program_count] = x_pos/TURN_RADIUS;
  y_pos_arr[program_count] = y_pos/TURN_RADIUS;
}

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
  //Serial.print(encoderValueL);
  
}
void updateEncoderR()
{
  encoderValueR++;
  //Serial.print(encoderValueR);
}
