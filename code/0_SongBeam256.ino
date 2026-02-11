 /* 
  *   REALTIME BEAMFORMING WITH TEENSY 4 AND SONGBEAM HARDWARE
  *       by Frank Dziock DD4WH, 07.02.2026
  *  
  *  - linear microphone array with 4 MEMS IM69D130 (SNR 69dbA) and PDM-to-I2S converter ADAU7002
  *  - separate FIR Bandpass filters for all four mics
  *  - normalization of each mic signal separately
  *  - frequency domain fractional delay (expansion factor 5) FIR filters
  *  - delay-and-sum beamforming
  *  - the prototype uses MQS audio output
  *  - plan is to use a DAC PCM5102 for I2S output
  *  
  *  - included code for correct file timestamps
  *  - warnings eliminated
  *    
  *    HARDWARE
  *    - Teensy 4.1
  *    - SongBeam: linear MEMS mic array with four microphones 
  *    - https://www.cuco.group/songbeam
  *    - Headphone connections (studio headphones with 80 Ohms impedance, attention if using 16 Ohm headphones!!!) soldered to MQS pins 12 & 10 on the Teensy    
  *    - ToDo: connect DAC PCM5102 and use its line output
  */

/*  based on
 *  SongBeam 4 by Robert Lachlan & Lies Zandberg
 *  https://github.com/lzandberg/SongBeam
 *  original LICENSE:  Creative Commons Attribution 4.0 International License CC-BY-4.0
 *         Royal Holloway University of London
 *         Robert Lachlan & Lies Zandberg
 */
  
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

#include <Audio.h>
#include <Wire.h>
#include <TimeLib.h>
#include <TimeAlarms.h>


#include "arm_math.h"
#include "arm_const_structs.h"

extern "C" uint32_t set_arm_clock(uint32_t frequency);

char postfix[5]=".wav";
char devname[5]="R01_";

#define FFT_LEN     256
#define BLOCK_SIZE  128     // 
#define NUM_TAPS    129     // filter length of fractional delay FIR filter
#define FIR_NUM_TAPS 64     // filter length of bandpass FIR filter

// Audio parameters
#define SPEED_SOUND     343.0f      // m/s
#define NUM_MICS        4
#define C_M_TO_MM       1000.0f
#define F_LOW           1200.0f      // 2 kHz
#define F_HIGH          8000.0f     // 9 kHz
#define NORM_WINDOW     1024       // RMS normalization window

// Array geometry: positions in mm (0, 45, 75, 120)
static const float mic_positions_mm[NUM_MICS] = {0.0f, 45.0f, 75.0f, 120.0f};

// Bandpass state (CMSIS-DSP FIR instances)
arm_fir_instance_f32 bp_filters[NUM_MICS];
float32_t bp_state[NUM_MICS][FIR_NUM_TAPS + BLOCK_SIZE - 1]; 
float32_t bp_coeffs[FIR_NUM_TAPS];

// Normalization state (RMS over window)
float32_t norm_buffer[NUM_MICS][NORM_WINDOW];
uint32_t norm_idx[NUM_MICS];
float32_t norm_gain[NUM_MICS];
float32_t norm_sum[NUM_MICS];      // running sum of squared samples

float32_t fir_coeffs[NUM_MICS][NUM_TAPS]; // for fractional delay
static float32_t filter_mask[NUM_MICS][FFT_LEN*2];   // Real + imag (complex)
static float32_t FFT_buffer[FFT_LEN*2];   // Real + imag (complex)
static float32_t iFFT_buffer[FFT_LEN*2];   // Real + imag (complex)
static float32_t last_block[NUM_MICS][BLOCK_SIZE];     // Previous block's tail (saved)
static float32_t float_buffer[NUM_MICS][BLOCK_SIZE];
static float32_t beam_output[BLOCK_SIZE];
uint8_t first_block[NUM_MICS] = {1,1,1,1};

int steering_angle = -45; // steering angle in degrees, range: -90 to 90 degrees

// FFT instance
const static arm_cfft_instance_f32 *FFT_state;
const static arm_cfft_instance_f32 *iFFT_state;
const static arm_cfft_instance_f32 *filter_mask_state;


//write wav
unsigned long ChunkSize = 0L;
unsigned long Subchunk1Size = 16;
unsigned int AudioFormat = 1;
unsigned int numChannels = 4;
unsigned long sampleRate = 44100;
//unsigned long sampleRate = 48000; // does not work properly, DD4WH
unsigned int bitsPerSample = 16;
unsigned long byteRate = sampleRate*numChannels*(bitsPerSample/8);// samplerate x channels x (bitspersample / 8)
unsigned int blockAlign = numChannels*bitsPerSample/8;  
unsigned long Subchunk2Size = 0L;
unsigned long recByteSaved = 0L;
unsigned long NumSamples = 0L;
byte byte1, byte2, byte3, byte4;

AudioInputI2S            audioInput;
AudioRecordQueue         queue1;
AudioRecordQueue         queue2;         //xy=389,145
AudioInputI2S2           audioInput2;
AudioRecordQueue         queue3;
AudioRecordQueue         queue4;         //xy=389,145
AudioPlayQueue           queueOUT2;         
AudioPlayQueue           queueOUT1;         
AudioOutputMQS           mqs1; 

AudioConnection          patchCord1(audioInput, 0, queue1, 0);
AudioConnection          patchCord2(audioInput, 1, queue2, 0);
AudioConnection          patchCord3(audioInput2, 0, queue3, 0);
AudioConnection          patchCord4(audioInput2, 1, queue4, 0);
AudioConnection          patchCord5(queueOUT2, 0, mqs1, 1);
AudioConnection          patchCord6(queueOUT1, 0, mqs1, 0);

int8_t passthru = 1; // for testing, stops recording and starts MQS realtime audio processing


byte bufferL[BLOCK_SIZE * 2]; // Blocksize 128 x 2 bytes
byte bufferR[BLOCK_SIZE * 2];
byte bufferLa[BLOCK_SIZE * 2];
byte bufferRa[BLOCK_SIZE * 2];

q15_t inbuffer1[BLOCK_SIZE]; // Blocksize 128 (16 bit)
q15_t inbuffer2[BLOCK_SIZE];
q15_t inbuffer3[BLOCK_SIZE];
q15_t inbuffer4[BLOCK_SIZE];

int16_t *sp_L;
int16_t *sp_R;

int recDay[7]={1,1,1,1,1,1,1};


int mode = 0;  // 0=stopped, 1=recording, 2=playing
FsFile frec;
File frec2;
elapsedMillis  msecs;

int filecount=0;

int recmins=15;
unsigned int tsamplemillis = 60000*recmins;

int starttimehour=7;
int starttimemin=0;

time_t begintime=0;
time_t endtime=0;
time_t offtime=0;

int ontime=36*60; //number of seconds before power off. 34*60 is with TPL51110 set with real 100Ohm.
int resistChoice=0;

//THIS IS THE DEFAULT ontime for a 100kOhm resistor - this is for the new machines!

int recordPeriodMins=180;

//int hiclock=450000000;
//int hiclock=24000000;
int hiclock=600000000;

int PIN_POWER=32;

#define SDCARD_CS_PIN    BUILTIN_SDCARD

// Compiler needs some help in recognizing these functions
void generate_beamform_fir_coeffs(float32_t theta_degrees, float32_t fir_coeffs[NUM_MICS][NUM_TAPS]);
void generate_bandpass_fir(float32_t coeffs[FIR_NUM_TAPS]);
                         
void setup() {

  set_arm_clock(hiclock);

    Serial.begin(9600);

  for (int i=0; i<10; i++){
    delay(50);
    if (Serial){
      i=10;
    }
    //Serial.println(i);
  }

  setSyncProvider(getTeensy3Time);

  //setTime(7,59,0,1,2,11);

  pinMode(LED_BUILTIN, OUTPUT);

  
  AudioMemory(120);
  
  if (!(SD.sdfs.begin(SdioConfig(FIFO_SDIO)))) {
    // stop here, but print a message repetitively
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(250);               // wait for a second
      digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
      delay(250);
    }
  }
  Serial.println("reading config file");
  readconfig();

  if (numChannels==2){
    PIN_POWER=32;
  }
  
  pinMode(PIN_POWER, OUTPUT);
  digitalWrite(PIN_POWER, LOW);

  Serial.print("devname= ");
  Serial.println(devname);
  Serial.print("starttimemin= ");
  Serial.println(starttimemin);
  Serial.print("starttimehour= ");
  Serial.println(starttimehour);
  Serial.print("recmins= ");
  Serial.println(recmins);\
  Serial.print("recordPeriodMins= ");
  Serial.println(recordPeriodMins);
  Serial.print("numChannels= ");
  Serial.println(numChannels);
  for (int i=0; i<7; i++)
  {
    Serial.print("recDay "); Serial.print(i); Serial.print(" = ");
    Serial.println(recDay[i]);
  }

  if (resistChoice==1){
     ontime=285*6; //number of seconds before power off. 285*6 is with TPL51110 set with 91Ohm
     //Most of the first 10 had these resistors
  }
  else if (resistChoice==2){
     ontime=34*60; //number of seconds before power off. 34*60 is with TPL51110 set with 100Ohm.
     //A couple of the first 10 had these resistors.
  }


  tsamplemillis = 60000*recmins;
  
  tmElements_t tm;
  breakTime(now(), tm);
  tm.Second=0;
  tm.Minute=starttimemin;
  tm.Hour=starttimehour;

  Serial.println("WEEKDAY:");
  Serial.println(tm.Wday);
  int day=tm.Wday-1;

  
  begintime=makeTime(tm);
  endtime=begintime+60*recordPeriodMins;
  
 time_t x=now();
 offtime=x+ontime;
  
  if ((recDay[day]==0)||(x>endtime)||(offtime-60<begintime)){
    digitalWrite(PIN_POWER, HIGH);
    Serial.println("SHUTDOWN");
  }

  offtime=offtime-(recmins+1)*60;

  Serial.print("now() = ");
  Serial.println(x);
  Serial.print("begintime = ");
  Serial.println(begintime);
  Serial.print("endtime = ");
  Serial.println(endtime);

  FsDateTime::callback = dateTime; // DD4WH, for correct file timestamps, taken from microSoundRecorder by WMXZ -> https://github.com/WMXZ-EU/microSoundRecorder/blob/master/audio_logger_if.h

  memset(last_block, 0, sizeof(last_block));

  /********************************************
   * Initialize FFT state variables
   ********************************************/
  FFT_state         = &arm_cfft_sR_f32_len256;
  iFFT_state        = &arm_cfft_sR_f32_len256;
  filter_mask_state = &arm_cfft_sR_f32_len256;

  /********************************************
   * calculate Bandpass coeffs for preprocessing
   ********************************************/
       generate_bandpass_fir(bp_coeffs);

      for (int mic = 0; mic < NUM_MICS; mic++) 
      {
        arm_fir_init_f32(&bp_filters[mic], (uint16_t)FIR_NUM_TAPS, bp_coeffs, bp_state[mic], (uint32_t)BLOCK_SIZE);
      }

   /********************************************
   * Initialize normalization
   ********************************************/
      mic_preprocess_init();

   /********************************************
   * Calculate beamforming fractional delay FIR coeffs and init FFT_mask
   ********************************************/
  // steering angle in degrees is given here
  generate_beamform_fir_coeffs(steering_angle, fir_coeffs);
  
  for(uint8_t i=0; i<4; i++)
  {
      init_filter_mask(i);
  }

  Serial.println("Initialization ready");
  
  delay(1000);
  
  if(passthru == 1)
  {
      queue1.begin();
      queue2.begin();
      queue3.begin();
      queue4.begin();
  }
} // END of SETUP


void loop() {
  int x=now();
  //Serial.print("New loop: ");
  //Serial.println(x);
  if(passthru == 0) 
  {
      if ((x>begintime)&&(x<endtime)){
        Serial.println("Considering recording");
        
        //set_arm_clock(hiclock);
        if (x>offtime){
          int p=tsamplemillis-1000*(x-offtime);
          if (p>10000){
            recordSimple(p);
          }
          else{
            digitalWrite(PIN_POWER, HIGH);
            delay(1000);
          }
        }
        else{
          recordSimple(tsamplemillis);
        }
      }
      else if (x>endtime){
        Serial.println("Decided to turn off");
        digitalWrite(PIN_POWER, HIGH);
        delay(1000);
      }
      else{
        Serial.println("waiting to start");
        //set_arm_clock(loclock);
        delay(1000);
      }
      } // END passthru == 0
      else 
      { // wait until all microphone queues have enough data
        if (queue1.available() >= 2 && queue2.available() >= 2 && queue3.available() >= 2 && queue4.available() >= 2) 
        {
            memcpy(inbuffer1, queue1.readBuffer(), BLOCK_SIZE * 2);
            memcpy(inbuffer2, queue2.readBuffer(), BLOCK_SIZE * 2);
            memcpy(inbuffer3, queue3.readBuffer(), BLOCK_SIZE * 2);
            memcpy(inbuffer4, queue4.readBuffer(), BLOCK_SIZE * 2);
            queue1.freeBuffer();
            queue2.freeBuffer();
            queue3.freeBuffer();
            queue4.freeBuffer();
            // convert 16bit integer input to float and store in float_buffer[4][BLOCK_SIZE]
            arm_q15_to_float(inbuffer1, &float_buffer[0][0], BLOCK_SIZE);            
            arm_q15_to_float(inbuffer2, &float_buffer[1][0], BLOCK_SIZE);
            arm_q15_to_float(inbuffer3, &float_buffer[2][0], BLOCK_SIZE);            
            arm_q15_to_float(inbuffer4, &float_buffer[3][0], BLOCK_SIZE);

/*******************************************************************
 *  PREPROCESS: bandpass + normalize ALL mics
 *******************************************************************/
           for (uint8_t mic = 0; mic < NUM_MICS; mic++) 
          {
            // 2. Bandpass filter
                float32_t bp_out[BLOCK_SIZE];
                arm_fir_f32(&bp_filters[mic], &float_buffer[mic][0], bp_out, BLOCK_SIZE);
        
            // 3. Update sliding RMS window FIRST
            // Update buffer with NEW block samples only
            for (uint32_t i = 0; i < BLOCK_SIZE; i++) 
            {
                int idx = norm_idx[mic];
                
                // Subtract old squared sample leaving window
                float32_t old_sq = norm_buffer[mic][idx];
                norm_sum[mic] -= old_sq;
            
                // Add new squared sample entering window
                float32_t new_sq = bp_out[i] * bp_out[i];
                norm_sum[mic] += new_sq;
            
                // Store new squared sample
                norm_buffer[mic][idx] = new_sq;
                
                // Advance circular index      
                norm_idx[mic] = (norm_idx[mic] + 1) % NORM_WINDOW;
            }      
        
            // 4. Compute RMS from updated buffer
            // RMS normalization (use FULL window average)
            //float32_t rms_sum = 0.0f;
            
            // Sum ALL samples in circular buffer
        /*    for (uint32_t i = 0; i < NORM_WINDOW; i++) 
            {
                rms_sum += norm_buffer[mic][i];
            }
          */  
            // Compute RMS over FULL window (not just block)
//            float32_t rms = sqrtf(rms_sum / NORM_WINDOW);
            float32_t rms = sqrtf(norm_sum[mic] / NORM_WINDOW);
            
            // 5. Compute gain
            // Conservative gain: target RMS = -20dBFS (0.1), max gain = 10
            norm_gain[mic] = fminf(0.1f / (rms + 1e-6f), 10.0f);
            
            // 6. Apply normalization to output buffer (limit peak to prevent clipping)
            for (uint32_t i = 0; i < BLOCK_SIZE; i++) 
            {
                float32_t sample = bp_out[i] * norm_gain[mic];
                float_buffer[mic][i] = fminf(fmaxf(sample, -1.0f), 1.0f);  // Hard limit
            }

          } // end normalization loop
/*******************************************************************
 *  Fractional delay FIR filter (FFT, convolution, iFFT) specific
 *  for each microphone
 *  input: float_buffer[channel], output: float_buffer[channel]
 *******************************************************************/
            for(uint8_t mic = 0; mic < 4; mic++)
            {   
                // this applies the FFT-convolution-iFFT chain 
                // with precalculated FIR fractional delay coefficients
                // specific for each microphone
                Fast_Convolution(mic);
            }             

/***********************************************************************
 *  SUM delayed microphone channels together and 
 *  play the beamformed audio through MQS
 ***********************************************************************/
            for (uint32_t i = 0; i < BLOCK_SIZE; i++) 
            {
                beam_output[i] = (float_buffer[0][i] + float_buffer[1][i] + float_buffer[2][i] + float_buffer[3][i]) * 0.25f;
            }
            // Scaling / Amplification
            arm_scale_f32 (beam_output, 10.0f, beam_output, BLOCK_SIZE);
                         
            // play audio via MQS / I2S
            sp_L = queueOUT2.getBuffer();    
            sp_R = queueOUT1.getBuffer();
            arm_float_to_q15 (&beam_output[0], sp_R, BLOCK_SIZE);            
            arm_float_to_q15 (&beam_output[0], sp_L, BLOCK_SIZE);
            queueOUT2.playBuffer(); // play it !  
            queueOUT1.playBuffer(); // play it !
           
       } // END if queue available
    } // END passthru == 1
  
} // END of loop

// Call back for file timestamps (used by FS).  Only called for file create and sync().
// DD4WH, code taken from microSoundRecorder by WMXZ -> https://github.com/WMXZ-EU/microSoundRecorder/blob/master/audio_logger_if.h
  void dateTime(uint16_t* date, uint16_t* time, uint8_t* ms10) 
  {
    // Return date using FS_DATE macro to format fields.
    *date = FS_DATE(year(), month(), day());
  
    // Return time using FS_TIME macro to format fields.
    *time = FS_TIME(hour(), minute(), second());
  
    // Return low time bits in units of 10 ms.
    *ms10 = second() & 1 ? 100 : 0;
  }

void recordSimple(int ts){
  //set_arm_clock(hiclock);
  delay(500);
  Serial.println("Recording ");
  Serial.println(now());
  elapsedMillis recordingTime = 0;
  if (numChannels==2){
    char* fn=startRecording2Chan();
    while(recordingTime<(long unsigned)ts) continueRecording2Chan();
    stopRecording2Chan(fn);
  }
  else{
    char* fn=startRecording();  
    while(recordingTime<(long unsigned)ts) continueRecording();
    stopRecording(fn);
  }
  delay(1000);
}


char* startRecording() {
  Serial.println("startRecording");
  filecount++;

  char *filename = makeFilename();

  if (SD.exists(filename)) {
    SD.remove(filename);
  }
  frec = SD.sdfs.open(filename, O_WRITE | O_CREAT);
  int FILE_SIZE=4*60*recmins*sampleRate*2*2;

  if (!frec.preAllocate(FILE_SIZE)) {
     Serial.println("preAllocate failed\n");
     
  }
  Serial.print("File opened ");
  //Serial.println(now());
  
  if (frec) {
    queue1.begin();
    queue2.begin();
    queue3.begin();
    queue4.begin();
    mode = 1;
    recByteSaved = 0L;
  }

  return filename;
}

char* startRecording2Chan() {
  Serial.println("startRecording");
  filecount++;

  char *filename = makeFilename();

  if (SD.exists(filename)) {
    SD.remove(filename);
  }
  frec = SD.sdfs.open(filename, O_WRITE | O_CREAT);
  int FILE_SIZE=2*60*recmins*sampleRate*2*2;

  if (!frec.preAllocate(FILE_SIZE)) {
     Serial.println("preAllocate failed\n");
     
  }
  Serial.print("File opened ");
  //Serial.println(now());
  
  if (frec) {
    queue1.begin();
    queue2.begin();
    mode = 1;
    recByteSaved = 0L;
  }

  return filename;
}

void continueRecording() {

  if (queue1.available() >= 2 && queue2.available() >= 2 && queue3.available() >=2 && queue4.available() >=2) {
    byte buffer[1024];

    memcpy(bufferL, queue1.readBuffer(), 256);
    memcpy(bufferR, queue2.readBuffer(), 256);
    memcpy(bufferLa, queue3.readBuffer(), 256);
    memcpy(bufferRa, queue4.readBuffer(), 256);
    queue1.freeBuffer();
    queue2.freeBuffer();
    queue3.freeBuffer();
    queue4.freeBuffer();
    int b = 0;
    for (int i = 0; i < 1024; i += 8) {
      buffer[i] = bufferL[b];
      buffer[i + 1] = bufferL[b + 1];
      buffer[i + 2] = bufferR[b];
      buffer[i + 3] = bufferR[b + 1];
      buffer[i + 4] = bufferLa[b];
      buffer[i + 5] = bufferLa[b + 1];
      buffer[i + 6] = bufferRa[b];
      buffer[i + 7] = bufferRa[b + 1];
      b = b+2;
    }
    //elapsedMicros usec = 0; //
    frec.write(buffer, 1024);  //256 or 512 (dudes code)
    recByteSaved += 1024;
    //Serial.print("SD write, us="); //
    //Serial.println(usec); //
  } 
}

void continueRecording2Chan() {

  if (queue1.available() >= 2 && queue2.available() >= 2) {
    byte buffer[512];

    memcpy(bufferL, queue1.readBuffer(), 256);
    memcpy(bufferR, queue2.readBuffer(), 256);
    
    queue1.freeBuffer();
    queue2.freeBuffer();

    int b = 0;
    for (int i = 0; i < 512; i += 4) {
      buffer[i] = bufferL[b];
      buffer[i + 1] = bufferL[b + 1];
      buffer[i + 2] = bufferR[b];
      buffer[i + 3] = bufferR[b + 1];
      b = b+2;
    }
    //elapsedMicros usec = 0;
    frec.write(buffer, 512);  //256 or 512 (dudes code)
    recByteSaved += 512;
    ////Serial.print("SD write, us=");
    ////Serial.println(usec);
  } 
}

void stopRecording(char* fn) {
  Serial.print("stopRecording ");
  //Serial.println(now());
  queue1.end();
  queue2.end();
  queue3.end();
  queue4.end();
  if (mode == 1) {
    while (queue1.available() > 0 && queue2.available() > 0 && queue3.available() > 0 && queue4.available() > 0) {
      frec.write((byte*)queue1.readBuffer(), 256);
      queue1.freeBuffer();
      frec.write((byte*)queue2.readBuffer(), 256);
      queue2.freeBuffer();
      frec.write((byte*)queue3.readBuffer(), 256);
      queue3.freeBuffer();
      frec.write((byte*)queue4.readBuffer(), 256);
      queue4.freeBuffer();
      recByteSaved += 256;
    }
    frec.truncate();
    frec.close();
    delay(100);


    frec2 = SD.open(fn, FILE_WRITE);
    //frec = SD.sdfs.open(fn, O_WRITE | O_CREAT);
    
    writeOutHeader();
    frec2.close();
  }
  mode = 0;
  Serial.print("finishedRecording ");
  //Serial.println(now());
}

void stopRecording2Chan(char* fn) {
  //Serial.print("stopRecording ");
  //Serial.println(now());
  queue1.end();
  queue2.end();
  if (mode == 1) {
    while (queue1.available() > 0 && queue2.available() > 0) {
      frec.write((byte*)queue1.readBuffer(), 256);
      queue1.freeBuffer();
      frec.write((byte*)queue2.readBuffer(), 256);
      queue2.freeBuffer();
      recByteSaved += 256;
    }
    frec.truncate();
    frec.close();
    delay(100);


    frec2 = SD.open(fn, FILE_WRITE);
    //frec = SD.sdfs.open(fn, O_WRITE | O_CREAT);
    
    writeOutHeader();
    frec2.close();
  }
  mode = 0;
  //Serial.print("finishedRecording ");
  //Serial.println(now());
}


void writeOutHeader() { // update WAV header with final filesize/datasize

//  NumSamples = (recByteSaved*8)/bitsPerSample/numChannels;
//  Subchunk2Size = NumSamples*numChannels*bitsPerSample/8; // number of samples x number of channels x number of bytes per sample
  Subchunk2Size = recByteSaved;
  ChunkSize = Subchunk2Size + 36;
  frec2.seek(0);
  frec2.write("RIFF");
  byte1 = ChunkSize & 0xff;
  byte2 = (ChunkSize >> 8) & 0xff;
  byte3 = (ChunkSize >> 16) & 0xff;
  byte4 = (ChunkSize >> 24) & 0xff;  
  frec2.write(byte1);  frec2.write(byte2);  frec2.write(byte3);  frec2.write(byte4);
  frec2.write("WAVE");
  frec2.write("fmt ");
  byte1 = Subchunk1Size & 0xff;
  byte2 = (Subchunk1Size >> 8) & 0xff;
  byte3 = (Subchunk1Size >> 16) & 0xff;
  byte4 = (Subchunk1Size >> 24) & 0xff;  
  frec2.write(byte1);  frec2.write(byte2);  frec2.write(byte3);  frec2.write(byte4);
  byte1 = AudioFormat & 0xff;
  byte2 = (AudioFormat >> 8) & 0xff;
  frec2.write(byte1);  frec2.write(byte2); 
  byte1 = numChannels & 0xff;
  byte2 = (numChannels >> 8) & 0xff;
  frec2.write(byte1);  frec2.write(byte2); 
  byte1 = sampleRate & 0xff;
  byte2 = (sampleRate >> 8) & 0xff;
  byte3 = (sampleRate >> 16) & 0xff;
  byte4 = (sampleRate >> 24) & 0xff;  
  frec2.write(byte1);  frec2.write(byte2);  frec2.write(byte3);  frec2.write(byte4);
  byte1 = byteRate & 0xff;
  byte2 = (byteRate >> 8) & 0xff;
  byte3 = (byteRate >> 16) & 0xff;
  byte4 = (byteRate >> 24) & 0xff;  
  frec2.write(byte1);  frec2.write(byte2);  frec2.write(byte3);  frec2.write(byte4);
  byte1 = blockAlign & 0xff;
  byte2 = (blockAlign >> 8) & 0xff;
  frec2.write(byte1);  frec2.write(byte2); 
  byte1 = bitsPerSample & 0xff;
  byte2 = (bitsPerSample >> 8) & 0xff;
  frec2.write(byte1);  frec2.write(byte2); 
  frec2.write("data");
  byte1 = Subchunk2Size & 0xff;
  byte2 = (Subchunk2Size >> 8) & 0xff;
  byte3 = (Subchunk2Size >> 16) & 0xff;
  byte4 = (Subchunk2Size >> 24) & 0xff;  
  frec2.write(byte1);  frec2.write(byte2);  frec2.write(byte3);  frec2.write(byte4);
  //frec.close();
  ////Serial.println("header written"); 
  ////Serial.print("Subchunk2: "); 
  ////Serial.println(Subchunk2Size); 
}

char *makeFilename(){ 
  static char filename[40];
  sprintf(filename, "%s_%04d_%02d_%02d_%02d_%02d_%02d%s", devname, year(), month(), day(), hour(), minute(), second(), postfix);
  return filename;  
}

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}


void readconfig(){
  const size_t LINE_DIM = 50;
  char line[LINE_DIM];
  size_t n;
  FsFile file;
  for (int i=0; i<7; i++){recDay[i]=0;}
  if (file.open("config.txt", O_READ)) {
    //int ln = 1;
    int x=0;
    
    while ((n = file.fgets(line, sizeof(line))) > 0) {
      line[strcspn(line, "\n")] = 0;
      if (x>0){
        int a=1;
        String str=String(a);
        if (x==1){
          strcpy(devname, line);
          //devname=line;
        }
        else if (x==2){
          starttimehour=atoi(line);
        }
        else if (x==3){
          starttimemin=atoi(line);
        }
        else if (x==4){
          recordPeriodMins=atoi(line);
        }
        else if (x==5){
          recmins=atoi(line);
        }
        else if (x==6){
          recDay[0]=1;
        }
        else if (x==7){
          recDay[1]=1;
        }
        else if (x==8){
          recDay[2]=1;
        }
        else if (x==9){
          recDay[3]=1;
        }
        else if (x==10){
          recDay[4]=1;
        }
        else if (x==11){
          recDay[5]=1;
        }
        else if (x==12){
          recDay[6]=1;
        }
        else if (x==13){
          numChannels=atoi(line);
        }
        else if (x==14){
          resistChoice=atoi(line);
        }

        x=0;
      }
      
      if (strcmp(line, "DeviceID:")==0){x=1;}
      else if (strcmp(line, "RecordStartHrs:")==0){x=2;}
      else if (strcmp(line, "RecordStartMins:")==0){x=3;}
      else if (strcmp(line, "RecordLengthMins:")==0){x=4;}
      else if (strcmp(line, "FileLengthMins:")==0){x=5;}
      else if (strcmp(line, "RecSun:")==0){x=6;}
      else if (strcmp(line, "RecMon:")==0){x=7;}
      else if (strcmp(line, "RecTue:")==0){x=8;}
      else if (strcmp(line, "RecWed:")==0){x=9;}
      else if (strcmp(line, "RecThu:")==0){x=10;}
      else if (strcmp(line, "RecFri:")==0){x=11;}
      else if (strcmp(line, "RecSat:")==0){x=12;} 
      else if (strcmp(line, "NumChan:")==0){x=13;}
      else if (strcmp(line, "Resist:")==0){x=14;}

  }
  file.close();
  delay(50);
  }
}


void init_filter_mask(uint8_t channel)
{
  /****************************************************************************************
     Calculate the FFT of the FIR filter coefficients once to produce the FIR filter mask
  ****************************************************************************************/
  // the FIR has exactly NUM_TAPS and a maximum of (FFT_length / 2) + 1 taps = coefficients, so we have to add (FFT_length / 2) -1 zeros before the FFT
  // in order to produce a FFT_length point input buffer for the FFT
  // copy coefficients into real values of first part of buffer, rest is zero
  for (unsigned i = 0; i < NUM_TAPS; i++)
  {
    filter_mask[channel][i * 2] = fir_coeffs[channel][i];
    filter_mask[channel][i * 2 + 1] = 0.0f;
  }

  for (unsigned i = FFT_LEN + 1; i < FFT_LEN * 2; i++)
  {
    filter_mask[channel][i] = 0.0;
  }
  // FFT of FIR_filter_mask
  // perform FFT (in-place), needs only to be done once (or every time the filter coeffs change)
  arm_cfft_f32(filter_mask_state, &filter_mask[channel][0], 0, 1);
} // end init_filter_mask


void Fast_Convolution(uint8_t channel)
{
      // 1.) ONLY FOR the VERY FIRST FFT: fill first samples with zeros
      if (first_block[channel]) // fill real & imaginaries with zeros for the first BLOCKSIZE samples
      {
        for (unsigned i = 0; i < BLOCK_SIZE * 2; i++)
        {
          FFT_buffer[i] = 0.0;
        }
        first_block[channel] = 0;
      }
      else

        // HERE IT STARTS for all other instances
        // 2.) fill FFT_buffer with last events audio samples
        for (unsigned i = 0; i < BLOCK_SIZE; i++)
        {
          FFT_buffer[i * 2] = last_block[channel][i]; // real
          FFT_buffer[i * 2 + 1] = 0; // imaginary
        }

      // copy recent samples to last_sample_buffer for next time!
      for (unsigned i = 0; i < BLOCK_SIZE; i++)
      {
        last_block[channel][i] = float_buffer[channel][i];
      }
      // 3.) now fill recent audio samples into FFT_buffer (left channel: re, right channel: im)
      for (unsigned i = 0; i < BLOCK_SIZE; i++)
      {
        FFT_buffer[FFT_LEN + i * 2] = float_buffer[channel][i]; // real
        FFT_buffer[FFT_LEN + i * 2 + 1] = 0.0f; // imaginary
      }
      // perform complex FFT
      // calculation is performed in-place the FFT_buffer [re, im, re, im, re, im . . .]
      arm_cfft_f32(FFT_state, FFT_buffer, 0, 1);
                  
      // complex multiply with filter mask
      arm_cmplx_mult_cmplx_f32 (FFT_buffer, &filter_mask[channel][0], iFFT_buffer, FFT_LEN);

      // perform iFFT (in-place) - CMSIS function arm_cfft_f32: "The inverse transform includes a scale of 1/fftLen as part of the calculation"
      arm_cfft_f32(iFFT_state, iFFT_buffer, 1, 1);

      // overlap & save -> use only 2nd part of iFFT buffer, use only real part
      // recycle float_buffer variable      
      for (unsigned i = 0; i < BLOCK_SIZE; i++)
      {
        float_buffer[channel][i] = iFFT_buffer[FFT_LEN + i * 2] ; // real
      }
}

// Initialize normalization (call once in setup())
void mic_preprocess_init(void)
{
    for (int mic = 0; mic < NUM_MICS; mic++) {
        // Init normalization buffers
        for (int i = 0; i < NORM_WINDOW; i++) {
            norm_buffer[mic][i] = 1e-6f;  // Avoid div-by-zero
        }
        norm_idx[mic] = 0;
        norm_gain[mic] = 1.0f;
    }
}

// without expansion: 
//    Sample rate: 44.1 kHz
//    Band of interest: 2–9 kHz → normalized band is roughly 0.09–0.41 of Nyquist.
//    FIR length: 129 taps → plenty of time support for an accurate fractional‑delay filter 
//    over 0–0.4 of Nyquist using a straightforward windowed‑sinc design.
//    A 129‑tap, well‑windowed sinc fractional‑delay filter at 44.1 kHz already gives 
//    very small group‑delay error and amplitude ripple in the 2–9 kHz band for typical delays used in short audio arrays.

void generate_beamform_fir_coeffs(float theta_deg, float fir_coeffs[NUM_MICS][NUM_TAPS])
{
    // steering angle
    float theta_rad = theta_deg * (float)M_PI / 180.0f;
    float sin_theta = sinf(theta_rad);
    printf("θ=%.1f° → sin(θ)=%.4f\n", theta_deg, sinf(theta_rad));
    // compute geometric delays (in samples) for each mic, relative to mic 0
    float delay_samples[NUM_MICS];
    for (int mic = 0; mic < NUM_MICS; mic++) {
        float x_m = mic_positions_mm[mic] / 1000.0f;              // mm -> m
        float tau_sec = (x_m * sin_theta) / SPEED_SOUND;          // seconds
        float D = tau_sec * sampleRate;                          // samples
        // reference is mic 0
        float x0_m = mic_positions_mm[0] / 1000.0f;
        float tau0_sec = (x0_m * sin_theta) / SPEED_SOUND;
        float D0 = tau0_sec * sampleRate;
        delay_samples[mic] = D - D0;

        printf("Mic %d: pos=%.1f mm, delay=%.4f samples\n",
               mic, mic_positions_mm[mic], delay_samples[mic]);
    }

    // design windowed-sinc fractional-delay FIR for each mic
    float center = (NUM_TAPS - 1) * 0.5f;

    for (int mic = 0; mic < NUM_MICS; mic++) {
        float D = delay_samples[mic];   // desired fractional delay (in samples)
        float sum_coeffs = 0.0f;

        for (int n = 0; n < NUM_TAPS; n++) {
            float t = (float)n - center - D;   // n - center - delay

            float sinc_val;
            if (fabsf(t) < 1e-6f) {
                sinc_val = 1.0f;
            } else {
                float x = (float)M_PI * t;
                sinc_val = sinf(x) / x;
            }

            // Hann window
            //float32_t window = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * (float)n / (float)(NUM_TAPS - 1)));
            // Blackman window
            //float32_t window = 0.42f - 0.5f * cosf(2.0f * M_PI * n / (NUM_TAPS - 1)) + 
                   //0.08f * cosf(4.0f * M_PI * n / (NUM_TAPS - 1));
            // Blackman Harris window 
            float32_t window = 0.4243801f - 0.4973406f * cosf(2.0f * M_PI * n / (NUM_TAPS - 1)) + 
                   0.0782793f * cosf(4.0f * M_PI * n / (NUM_TAPS - 1));

            float h = sinc_val * window;
            fir_coeffs[mic][n] = h;
            sum_coeffs += h;
        }

        // normalize to unity DC gain
        if (fabsf(sum_coeffs) > 1e-6f) {
            float inv = 1.0f / sum_coeffs;
            for (int n = 0; n < NUM_TAPS; n++) {
                fir_coeffs[mic][n] *= inv;
            }
        }
    }
}

#define EXPANSION_FACTOR 5
void generate_beamform_fir_coeffs_old(float theta_degrees, 
                                  float32_t fir_coeffs[NUM_MICS][NUM_TAPS])
// new version, no filtering, expanded delay                                  
{
    float theta_rad = theta_degrees * M_PI / 180.0f;
    printf("θ=%.1f° → sin(θ)=%.4f\n", theta_degrees, sinf(theta_rad));
    // 5x expansion for fractional delay precision
    int num_taps_expanded = NUM_TAPS * EXPANSION_FACTOR;
    float delay_center_expanded = (num_taps_expanded - 1) / 2.0f;
    
    for (int mic = 0; mic < NUM_MICS; mic++) {
        // Geometric delay calculation
        float d_mm = mic_positions_mm[mic];
        float d_m = d_mm / C_M_TO_MM;
        float delay_sec = (d_m * sinf(theta_rad)) / SPEED_SOUND;
        float delay_samples_original = delay_sec * sampleRate;
        float delay_samples_expanded = delay_samples_original * EXPANSION_FACTOR;

        printf("Mic%d: d=%.1fmm → %.3f samples\n", mic, d_mm, delay_samples_original);
        printf("Mic %d: orig delay=%.3f, expanded=%.1f samples\n", 
               mic, delay_samples_original, delay_samples_expanded);
        
        // Generate FIR coeffs via polyphase decimation from expanded domain
        float window_scale = 0.0f;
        
        for (int n_orig = 0; n_orig < NUM_TAPS; n_orig++) {
            // Average EXPANSION_FACTOR expanded taps per original tap
            float sum_delay = 0.0f;
            
            for (int k = 0; k < EXPANSION_FACTOR; k++) {
                int n_exp = n_orig * EXPANSION_FACTOR + k;
                if (n_exp < num_taps_expanded) {
                    float t_exp = n_exp - delay_center_expanded;
                    
                    // Ideal sinc interpolator in expanded domain (5x precision)
                    float delay_sinc_exp;
                    if (fabsf(t_exp - delay_samples_expanded) < 1e-6f) {
                        delay_sinc_exp = 1.0f;
                    } else {
                        delay_sinc_exp = sinf(M_PI * (t_exp - delay_samples_expanded)) / 
                                       (M_PI * (t_exp - delay_samples_expanded));
                    }
                    
                    sum_delay += delay_sinc_exp;
                }
            }
            
            // Map back to original domain tap
            //float delay_center_orig = (NUM_TAPS - 1) / 2.0f;
            //float t_orig = n_orig - delay_center_orig;
            float window_orig = 0.5f * (1.0f - cosf(2.0f * M_PI * n_orig / (NUM_TAPS - 1)));
            
            // Polyphase decimation: average of 5 expanded taps
            fir_coeffs[mic][n_orig] = (sum_delay / EXPANSION_FACTOR) * window_orig;
            window_scale += fir_coeffs[mic][n_orig];
        }
        
        // Final normalization (unity gain)
        for (int n = 0; n < NUM_TAPS; n++) {
            fir_coeffs[mic][n] /= window_scale;
        }
    }
}


// Generate FIR bandpass coefficients
void generate_bandpass_fir(float32_t coeffs[FIR_NUM_TAPS]) {
    float32_t fc_low = F_LOW / sampleRate;  // normalized low cutoff
    float32_t fc_high = F_HIGH / sampleRate; // normalized high cutoff
    
    int32_t M = FIR_NUM_TAPS - 1;  // filter order (63)
    
    for (int32_t n = 0; n < FIR_NUM_TAPS; n++) {
        float32_t t = (float32_t)n - (float32_t)M * 0.5f;  // time index [-31.5, +31.5]
        
        // Ideal bandpass impulse response
        float32_t ideal_h;
        if (fabsf(t) < 1e-6f) {
            ideal_h = 2.0f * (fc_high - fc_low);  // DC gain correction
        } else {
            ideal_h = (sinf(2.0f * PI * fc_high * t) - sinf(2.0f * PI * fc_low * t)) / (PI * t);
        }
        
        // Hamming window
        //float32_t window = 0.54f - 0.46f * cosf(2.0f * PI * (float32_t)n / (float32_t)M);
        // Blackman window
        float32_t window = 0.42f - 0.5f * cosf(2.0f * M_PI * (float32_t)n / (float32_t)M) + 
               0.08f * cosf(4.0f * M_PI * (float32_t)n / (float32_t)M);
        // Hann
        //float window = 0.5f * (1.0f - cosf(2.0f * M_PI * (float32_t)n / (float32_t)M));
        
        // Apply window
        coeffs[n] = ideal_h * window;
    }
    
    // Normalize so max coefficient magnitude = 1.0 (prevents overflow)
    float32_t max_abs = 0.0f;
    for (int32_t n = 0; n < FIR_NUM_TAPS; n++) {
        if (fabsf(coeffs[n]) > max_abs) {
            max_abs = fabsf(coeffs[n]);
        }
    }
    for (int32_t n = 0; n < FIR_NUM_TAPS; n++) {
        coeffs[n] /= max_abs;
    }
}
