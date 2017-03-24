////////Code by C. Richards & E. Eberhard, 2016///////
// Credits: 
///////////////////////////////
////////DUE TO AURORA//////////
// 1. Read analog force from AURORA
// 2. Convert and send via rs232 to PC
//(-. in PC) Convert force and pass to integrator
//(-. in PC) step integrator, update position
//(-. in PC) send position via rs232 to DUE 
// 3. Receive position data via rs232 from PC
// 4. Send position to update AURORA

#include <DueTimer.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

//enter your expected maximum and minimum transmission values here
#define MAX_VAL     3.33
#define MIN_VAL     -0.05
#define COMP_RANGE  (MAX_VAL - MIN_VAL)

    /* The fast adc startup time*/
#define ADC_STARTUP_FAST     12

int loop_dt = 1000; //860 loop timer delay in uS
char inData[3]; // Allocate some space for the string
char outData[3];
char inChar= -1; // Where to store the character read
float posOut;
float vref = 3.3;//roughly 3.3, but we need a precise hardware reference at some point
int ai0read_raw;//raw ai0 read value in ui16 - to get volts = 5/68vcc * ai0read/4096
int ai0read;//calibrated ai0 read value in ui16 - to get volts = 5/68vcc * ai0read/4096
int dac0write;//ai0 write value in ui16 - to get volts = 5/6*vcc * ai0read/4096
float forceIn;
float forceIn_raw;
char buffer12bit[4];//buffer to store integers from 0 to 4095
int nsamples = 1;//number of samples to average over
float sum;
float ai0Cal = 1.007;//Calibration from ADC cal
float ai0Offset = 0.00806;//Offset from ADC cal
float ai0Offset_rel;//... relative to 12 bit samples
unsigned char sendbuf[2];
unsigned long start_time;
unsigned long end_time;
int counter = 0;
int ii = 0;
int avail = 0;

void compress12bit(char *send, float value);
float decompress12bit(char *received);

void compress12bit(char *send, float value)
{
  unsigned compressed = (unsigned)round(((value - MIN_VAL) / COMP_RANGE * 4096));
  send[0] = compressed & 0x3F;
  send[1] = (compressed >> 6) & 0x3F;
}


float decompress12bit(char *received)
{
  unsigned compressed = received[0] | (received[1] << 6);
  return (float)(compressed)*COMP_RANGE / 4096 + MIN_VAL;
}

void integrationTimer(){
  start_time = micros();
  //STEP 1. READ in force data from AURORA
  sum = 0;
  for (int j = 0; j< nsamples; j++) {
    ai0read_raw = analogRead(A0);// read from analog input pin 0
    ai0read = (int)(ai0read_raw * ai0Cal + ai0Offset_rel);//apply calibration and offset
    forceIn_raw = ai0read * (vref / 4095.0);
    sum += forceIn_raw;
  }
  forceIn = sum / nsamples; 
  
//  ai0read_raw = analogRead(A0);// read from analog input pin 0
//  ai0read = (int)(ai0read_raw * ai0Cal + ai0Offset_rel);//apply calibration and offset
//  forceIn_raw = ai0read * (vref / 4095.0);
  
 
  
  //STEP2. Convert and SEND force to PC
  compress12bit(outData, forceIn);
  outData[2] = 65;//for stopping
  Serial1.write(outData);// send over due tx to PC

  //STEP3. RECEIVE position information from PC
  avail = 0;
  ii = 0;
  while (Serial1.available() > 0 ) {
    //avail = Serial1.available();
    
    
    inChar = Serial1.read(); 
    if (inChar != 65 && ii < 3) {
      inData[ii] = inChar;
      ii++;
      
  
    }// end if
    else if (inChar != '\0'){
      posOut = decompress12bit(inData);
      dac0write = map(int(posOut * 100), 0, round(330 * 5/6), 0, 4095);//be sure to include DC offset of~0.55V
      analogWrite(DAC0, dac0write);//send to FPGADU for calibration and offset;
      ii = 0;
    }// end else if
  }//end while available

}



void setup() {
  Serial.begin(115200);      // open the serial port at 9600 bps:
  Serial1.begin(115200);
  Serial2.begin(9600);
  analogWriteResolution(12);  // set the analog output resolution to 12 bit (4096 levels)
  analogReadResolution(12);   // set the analog input resolution to 12 bit

  //REG_ADC_MR = (REG_ADC_MR & 0xFFF0FFFF) | 0x00020000;
  REG_ADC_MR = (REG_ADC_MR & 0xFFFFFF0F) | 0x00000080; //enable FREERUN mode
  
  ai0Offset_rel = ai0Offset * 4095.0 / vref;
  //Timer3.attachInterrupt(integrationTimer);
  //Timer3.start(loop_dt); // Calls every specified loop_dt period
  //Timer.getAvailable().attachInterrupt(integrationTimer).start(loop_dt);
  Timer3.attachInterrupt(integrationTimer);
  int trigval = 0;
  // clear any residual data and make sure dac starts at 2.75 volts
  dac0write = map(int(2.75 * 100), 0, round(330 * 5/6), 0, 4095);//be sure to include DC offset of~0.55V
  analogWrite(DAC0, dac0write);//send to FPGADU for calibration and offset;
  
  Serial.println("waiting for trigger...");
  while( trigval < 2000) {
  trigval = analogRead(A1);// read from analog input pin 0
  //Serial.println(trigval);
  }
  //
  analogRead(A0);//clear any data
  Timer3.start(loop_dt);
  Serial.println("triggered");
  delay(3000);
  
}

void loop() {
  while(1) {

    //Serial.println(counter);
    //end_time = micros();
    //Timer3.start(loop_dt);
    //counter ++;
    //delay(10);
    //Serial1.write(outData);// send over due tx to PC
    //Serial.println(forceIn);
    //Timer3.stop();
    counter ++;
    //if (counter == 5000) {
      //counter = 0;
      //trigval = 0;
      //Timer3.stop();
      //delay(2000);
    //}
  }
}
