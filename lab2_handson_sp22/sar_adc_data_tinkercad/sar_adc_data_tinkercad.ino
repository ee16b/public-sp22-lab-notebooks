/* Set the serial connection speed */
#define BAUD_RATE       9600

/* Set the simulated clock period in miliseconds */
#define CLOCK_PERIOD    10000000

/* Set which bits correspond to which pins */
#define BIT_3           7
#define BIT_2           8
#define BIT_1           12
#define BIT_0           13

/* Set the feedback pin from which to read the output of the comparator */
#define V_COMP            A1

int result = 0;
int steps = 20;
int v_comp = 0;

/* if mode=0 print the output code; if mode=1 plot the waveform */
int mode =0; 

void setup() {
  // Start a serial connection to send data to the computer
  Serial.begin(BAUD_RATE);

  // Setup the digital pins as outputs
  pinMode(BIT_3, OUTPUT);
  pinMode(BIT_2, OUTPUT);
  pinMode(BIT_1, OUTPUT);
  pinMode(BIT_0, OUTPUT);
  pinMode(V_COMP, INPUT);
  
  
  // Set the initial values of these pins to LOW (0V)
  digitalWrite(BIT_3, LOW);
  digitalWrite(BIT_2, LOW);
  digitalWrite(BIT_1, LOW);
  digitalWrite(BIT_0, LOW);

  Serial.print("Setup done\n");
}

void loop () {

  // cycle 1: clean all
  digitalWrite(BIT_3, LOW);
  digitalWrite(BIT_2, LOW);
  digitalWrite(BIT_1, LOW);
  digitalWrite(BIT_0, LOW);
  result = 0;
  v_comp = 0;
  plot(result, v_comp);
  delayMicroseconds(CLOCK_PERIOD);


  // cycle 2: test BIT_3
  digitalWrite(BIT_3, HIGH);
  result += 8;
  v_comp = digitalRead(V_COMP);
  plot(result, v_comp);
  delayMicroseconds(CLOCK_PERIOD / 2);
  if (v_comp ==0) {
    result -= 8;
    digitalWrite(BIT_3, LOW);
  }
  plot(result, v_comp);
  delayMicroseconds(CLOCK_PERIOD / 2);


  // cycle 3: test BIT_2
  digitalWrite(BIT_2, HIGH);
  result += 4;
  v_comp = digitalRead(V_COMP);
  plot(result, v_comp);
  delayMicroseconds(CLOCK_PERIOD / 2);
  if (v_comp==0) {
    result -= 4;
    digitalWrite(BIT_2, LOW);
  } 
  plot(result, v_comp);
  delayMicroseconds(CLOCK_PERIOD / 2);


  // cycle 4: test BIT_1
  digitalWrite(BIT_1, HIGH);
  result += 2;
  v_comp = digitalRead(V_COMP);
  plot(result, v_comp);
  delayMicroseconds(CLOCK_PERIOD / 2);
  if (v_comp == 0) {
    result -= 2;
    digitalWrite(BIT_1, LOW);
  } 
  plot(result, v_comp);
  delayMicroseconds(CLOCK_PERIOD / 2);


  // cycle 5: test BIT_0
  digitalWrite(BIT_0, HIGH);
  result += 1;
  v_comp = digitalRead(V_COMP);
  plot(result, v_comp);
  delayMicroseconds(CLOCK_PERIOD / 2);
  if (v_comp ==0) {
    result -= 1;
    digitalWrite(BIT_0, LOW);
  } 
  plot(result, v_comp);
  delayMicroseconds(CLOCK_PERIOD / 2);

  // print output code to serial monitor (mode=0)
  if (mode == 0) {
  	int_to_bin(result);
  }
}

void int_to_bin(int var) {
  for (unsigned int test = 0x8; test; test >>= 1) {
    Serial.write(var  & test ? '1' : '0');
  }
  Serial.println();
}

void plot(float result, int v_comp) {
  for (int i = 0; i < steps; i+=1) {
    if (mode == 1) {
      Serial.println(result/16.0,DEC);
      Serial.print(",");
      Serial.println(v_comp);
    }
  }
}

