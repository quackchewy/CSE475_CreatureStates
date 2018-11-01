
// Solder closed jumper on bottom!
#include <Adafruit_NeoPixel_ZeroDMA.h>
#include <SPI.h>
#include <RH_RF69.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_FeatherOLED.h>
#include <time.h>

#define PIXEL_PIN    19    // Digital IO pin connected to the NeoPixels.
#define VBATPIN A7

#define PIXEL_COUNT 16.

#define LED_PIN 13

#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 1000

// PROTOTYPE FOR OLED FUNCTION
void state1Cycle(uint8_t wait);
void state2Cycle(uint8_t wait);
void state3Cycle(uint8_t wait);
void state4Cycle(uint8_t wait);
void state5Cycle(uint8_t wait);
void state6Cycle(uint8_t wait);
void state7Cycle(uint8_t wait);
//float getBatteryVoltage();
//void tcConfigure(int sampleRate);
//uint32_t Wheel(byte WheelPos);


int sampleRate = 1000; //sample rate of the square wave in Hertz, how many times per second the TC5_Handler() function gets called per second basically

void tcStartCounter(); //starts the timer
void TC5_Handler();

bool isLEDOn = false;
bool playFlag = false;
bool gestFlag = false;
bool noteFlag = false;
uint16_t noteNum = 64;
uint16_t gNum = 255;
uint16_t gCnt = 0;
uint16_t noteCnt = 7;
uint16_t noteLength = 50;
uint16_t duration = 50;
uint16_t tempo = 8;
uint16_t volume = 127;
uint16_t ledcount = 0;

/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0

// Feather M0 w/Radio
  #define RFM69_CS      8
  #define RFM69_INT     3
  #define RFM69_RST     4
  #define LED           13

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

int16_t packetnum = 0;  // packet counter, we increment per xmission

Adafruit_FeatherOLED oled = Adafruit_FeatherOLED();

// See http://www.vlsi.fi/fileadmin/datasheets/vs1053.pdf Pg 31
#define VS1053_BANK_DEFAULT 0x00
#define VS1053_BANK_DRUMS1 0x78
#define VS1053_BANK_DRUMS2 0x7F
#define VS1053_BANK_MELODY 0x79

// See http://www.vlsi.fi/fileadmin/datasheets/vs1053.pdf Pg 32 for more!
#define VS1053_GM1_OCARINA 80

#define MIDI_NOTE_ON  0x90
#define MIDI_NOTE_OFF 0x80
#define MIDI_CHAN_MSG 0xB0
#define MIDI_CHAN_BANK 0x00
#define MIDI_CHAN_VOLUME 0x07
#define MIDI_CHAN_PROGRAM 0xC0

#define VS1053_MIDI Serial1

uint8_t voice[] = {
    7,9,10,12,15,47,105,99,113,114,115,116,118,124 };

uint8_t lightGesture[21] = {36, 36, 36, 72, 72, 72, 108, 108, 108, 144, 144, 144, 180, 180, 180, 216, 216, 216, 255, 255, 255};

uint8_t gesture[21][26] = {
  // ambient 1
  // plays the sound gesture of notes 12, 24, 36, 48, 60, 72, 84, and 96.
   {8,8,
   127,12,80,  127,24,80,  127,36,80,  127,48,80,  
   127,60,80,  127,72,80,  127,84,80,  127,96,80   },  
   {8,8,
   127,12,70,  127,24,70,  127,36,70,  127,48,70,  
   127,60,70,  127,72,70,  127,84,70,  127,96,70   },  
   {8,8,
   127,12,55,  127,24,55,  127,36,55,  127,48,55,  
   127,60,55,  127,72,55,  127,84,55,  127,96,55   },  

   // ambient 2
   {54,8,
   127,12,80,  127,24,80,  127,36,80,  127,48,80,  
   127,60,80,  127,72,80,  127,84,80,  127,96,80   },  
   {54,8,
   127,12,70,  127,24,70,  127,36,70,  127,48,70,  
   127,60,70,  127,72,70,  127,84,70,  127,96,70   },  
   {54,8,
   127,12,55,  127,24,55,  10,36,55,  10,48,55,  
   127,60,55,  127,72,55,  10,84,55,  10,96,55   },  

   // ambient 3
   {53,8,
   127,12,80,  127,24,80,  127,36,80,  127,48,80,  
   127,60,80,  127,72,80,  127,84,80,  127,96,80   },  
   {53,8,
   127,12,70,  127,24,70,  127,36,70,  127,48,70,  
   127,60,70,  127,72,70,  127,84,70,  127,96,70   },  
   {53,8,
   127,12,55,  127,24,55,  127,36,55,  127,48,55,  
   127,60,55,  127,72,55,  127,84,55,  127,96,55   },  


// active 1
   {23,1,
   127,12,35,  127,36,35,  127,36,35,  127,24,35,  
   127,48,35,  127,48,35,  127,36,35,  127,36,35   },  
   {23,8,
   127,12,25,  127,36,25,  127,36,25,  127,24,25,  
   127,48,25,  127,48,25,  127,36,25,  127,36,25   },  
   {23,8,
   127,12,20,  127,36,20,  127,36,20,  127,24,20,  
   127,48,20,  127,48,20,  127,36,20,  127,36,20   },  

   // active 2
   {32,8,
   127,12,35,  127,36,45,  127,24,35,  127,60,45,  
   127,48,35,  127,84,45,  127,72,35,  127,96,45   },  
   {32,8,
   127,12,30,  127,36,35,  127,24,30,  127,60,35,  
   127,48,30,  127,84,35,  127,72,30,  127,96,35   },  
   {32,8,
   127,12,30,  127,36,30,  127,24,30,  127,60,30,  
   127,48,30,  127,84,30,  127,72,30,  127,96,30   },  
   
   // active 3
   {41,8,
   127,12,35,  127,36,45,  127,24,35,  127,60,45,  
   127,48,35,  127,84,45,  127,72,35,  127,96,45   },  
   {41,8,
   127,12,30,  127,36,35,  127,24,30,  127,60,35,  
   127,48,30,  127,84,35,  127,72,30,  127,96,35   },  
   {41,8,
   127,12,30,  127,36,30,  127,24,30,  127,60,30,  
   127,48,30,  127,84,30,  127,72,30,  127,96,30   },  
   
   // startle
   {104,1,
   127,12,25,  127,24,25,  127,36,25,  127,48,25,  
   127,60,25,  127,72,25,  127,84,25,  127,96,25   }, 
   {104,1,
   127,12,20,  127,24,20,  127,36,20,  127,48,20,  
   127,60,20,  127,72,20,  127,84,20,  127,96,20   },  
   {104,1,
   127,12,15,  127,24,15,  127,36,15,  127,48,15,  
   127,60,15,  127,72,15,  127,84,15,  127,96,15   },   
   };

Adafruit_NeoPixel_ZeroDMA strip(PIXEL_COUNT, PIXEL_PIN, NEO_GRBW);

void setup() {
  delay(100);
  pinMode(LED_PIN, OUTPUT);
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  oled.display();
  oled.clearDisplay();
  oled.init();
  oled.setBatteryVisible(true);
  float battery = getBatteryVoltage();

  // update the battery icon
  oled.setBattery(battery);
  oled.renderBattery();
  oled.display();

  strip.begin();
  strip.setBrightness(5);
  strip.show(); // Initialize all pixels to 'off'

  colorWipe(strip.Color(0, 0, 0), 5);    // Black/off
  
  Serial.begin(115200);
  delay(100);
//  while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

  Serial.println("Board test");
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);
  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  
  if (!rf69.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");
  
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(14, true);

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
  
  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");

  VS1053_MIDI.begin(31250); // MIDI uses a 'strange baud rate' 
  midiSetChannelBank(0, VS1053_BANK_MELODY);
  midiSetChannelVolume(0, 127);

  tcConfigure(sampleRate); //configure the timer to run at <sampleRate>Hertz
  tcStartCounter(); //starts the timer
}

void loop() {
  // clear the current count
  oled.clearDisplay();

  // get the current voltage of the battery from
  // one of the platform specific functions below
  float battery = getBatteryVoltage();

  // update the battery icon
  oled.setBattery(battery);
  oled.renderBattery();
  oled.display(); 
 
 // handle the radio 
  if (rf69.waitAvailableTimeout(100)) {
    // Should be a message for us now   
    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    
    if (! rf69.recv(buf, &len)) {
      Serial.println("Receive failed");
       return;
    }
    digitalWrite(LED, HIGH);
    rf69.printBuffer("Received: ", buf, len);
    buf[len] = 0;
    SendPacket();
    
    Serial.print("Got: "); Serial.println((char*)buf);
    Serial.print("RSSI: "); Serial.println(rf69.lastRssi(), DEC);

    oled.println((char*)buf);
    oled.print("RSSI: "); oled.print(rf69.lastRssi());
    oled.display(); 
    digitalWrite(LED, LOW);
  }
  

  //handle sound
  if (!playFlag) {
    //set Gesture 0-15
<<<<<<< HEAD
    if(++gNum > 20) gNum = 0;
=======
    
    // resets the sound gesture index to 0 for repeating.
    // otherwise, out of index.
    if(++gNum >20) gNum = 0;
>>>>>>> cf1b61c1c7ee3519f37be7f7da84f06f40d43842
   // set voice 0-14
   clock_t t;
   t = clock();
   // each gesture is a multiple of 3.
   // 1st gesture is from index 0~2.
   // 2nd gesture is from 3~5.
   // sample code for a function:
   //void playGesture(int gestureNum) {
   // for (int k = 0; k < 3; k++) {
   //   // if you want the second gesture (ambient 2), then
   //   // gNum*3 would allow you to access 3, 4, and 5.
   //   midiSetInstrument(0, gesture[gNum*3][0]);
   // }
   //}

  
   midiSetInstrument(0, gesture[gNum][0]);
<<<<<<< HEAD

   //handle light
   for (int i = 0; i < strip.numPixels(); i++) {
     strip.setPixelColor(i, Wheel(lightGesture[gNum]));
   }
   strip.show();
   
=======
   t = clock()-t;
   double soundGestureDuration = ((double) t)/CLOCKS_PER_SEC;
>>>>>>> cf1b61c1c7ee3519f37be7f7da84f06f40d43842
   Serial.println(gesture[gNum][0], DEC); 
   Serial.print("Duration of a single sound gesture:");
   Serial.println(soundGestureDuration);
   
    noInterrupts();
    // critical, time-sensitive code here
    playFlag = true;

    //set Tempo?? 10-9
    if(--tempo > 0) tempo = 9;
    interrupts();
  }
  
  // other code here
  //rainbowCycle(3);
  //state1Cycle(1);
  //state2Cycle(1);
  //state3Cycle(1);
  //state4Cycle(1);
  //state5Cycle(1);
  //state6Cycle(1);
  //state7Cycle(1);
}
  void SendPacket() {
  {
    char radiopacket[] = "and Hello back to you";

    Serial.print("Sending: "); Serial.println(radiopacket);
    rf69.send((uint8_t *)radiopacket, sizeof(radiopacket));
    rf69.waitPacketSent();
  }
}

void midiSetInstrument(uint8_t chan, uint8_t inst) {
  if (chan > 15) return;
  inst --; // page 32 has instruments starting with 1 not 0 :(
  if (inst > 127) return;
  
  VS1053_MIDI.write(MIDI_CHAN_PROGRAM | chan);  
  delay(10);
  VS1053_MIDI.write(inst);
  delay(10);
}


void midiSetChannelVolume(uint8_t chan, uint8_t vol) {
  if (chan > 15) return;
  if (vol > 127) return;
  
  VS1053_MIDI.write(MIDI_CHAN_MSG | chan);
  VS1053_MIDI.write(MIDI_CHAN_VOLUME);
  VS1053_MIDI.write(vol);
}

void midiSetChannelBank(uint8_t chan, uint8_t bank) {
  if (chan > 15) return;
  if (bank > 127) return;
  
  VS1053_MIDI.write(MIDI_CHAN_MSG | chan);
  VS1053_MIDI.write((uint8_t)MIDI_CHAN_BANK);
  VS1053_MIDI.write(bank);
}

void midiNoteOn(uint8_t chan, uint8_t n, uint8_t vel) {
  if (chan > 15) return;
  if (n > 127) return;
  if (vel > 127) return;
  
  VS1053_MIDI.write(MIDI_NOTE_ON | chan);
  VS1053_MIDI.write(n);
  VS1053_MIDI.write(vel);
}

void midiNoteOff(uint8_t chan, uint8_t n, uint8_t vel) {
  if (chan > 15) return;
  if (n > 127) return;
  if (vel > 127) return;
  
  VS1053_MIDI.write(MIDI_NOTE_OFF | chan);
  VS1053_MIDI.write(n);
  VS1053_MIDI.write(vel);
}


// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}
void rainbow(uint8_t wait) {
  uint16_t i, j;

  for (j = 0; j < 256; j++) {
    for (i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  // We need to first calculate how long this entire cycle lasts, with the given delay.
  // The delay should be obvious, since the device's neo pixel will sit still for the given amount of time (1).
  // And then calculate (or measure) how long each sound gesture is.
  // Find the ratio between the two ( ratioValue = timePerSoundGesture/timePerLightGesture ) and change the value for k accordingly.
  // It is most likely going to be the case such that the value k is going to be a floating point.
  // This may require trial and error.

  // ratioValue = 1 is the default where it goes through this loop once.
  // If we find out that the ratio between the two, as described above, is 2, for example,
  // then change the ratioValue = 2. This will allow each light gesture to perform twice for
  // each sound gesture.

  // this is only a temp solution.
  int ratioValue = 1;
  for (int k = 0; k < ratioValue; k++) {
    // this j = 256*4 is the overall cycle count.
    for (j = 0; j < 256 * 4; j++) { // n cycles of all colors on wheel
      // iterates through the number of pixels available.
      for (i = 0; i < strip.numPixels(); i++) {
        strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
      } 
      strip.show();
      delay(wait);
    }
  }
}

void state1Cycle(uint8_t wait) {
  uint16_t i, j;

  // Handles sound.
  if (!playFlag) {
      //set Gesture 0-15
      if(++gNum > 2) gNum = 0;
     // set voice 0-14
     midiSetInstrument(0, gesture[gNum][0]);
     Serial.println(gesture[gNum][0], DEC); 
      noInterrupts();
      // critical, time-sensitive code here
      playFlag = true;
  
      //set Tempo?? 10-9
      if(--tempo > 0) tempo = 9;
      interrupts();
    }

  // Handles lights.
  for (j = 0; j < 256 * 4; j++) { // n cycles of all colors on wheel
    for (i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(20));
    }
    strip.show();
    delay(wait);
  }
}

void state2Cycle(uint8_t wait) {
  uint16_t i, j;
  
  // Handles sound.
  if (!playFlag) {
      //set Gesture 0-15
      if(++gNum > 5) gNum = 3;
     // set voice 0-14
     midiSetInstrument(0, gesture[gNum][0]);
     Serial.println(gesture[gNum][0], DEC); 
      noInterrupts();
      // critical, time-sensitive code here
      playFlag = true;
  
      //set Tempo?? 10-9
      if(--tempo > 0) tempo = 9;
      interrupts();
    }

  // Handles lights
  for (j = 0; j < 256 * 4; j++) { // n cycles of all colors on wheel
    for (i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(40));
    }
    strip.show();
    delay(wait);
  }
}

void state3Cycle(uint8_t wait) {
  uint16_t i, j;

  // Handles sound.
  if (!playFlag) {
      //set Gesture 0-15
      if(++gNum > 8) gNum = 6;
     // set voice 0-14
     midiSetInstrument(0, gesture[gNum][0]);
     Serial.println(gesture[gNum][0], DEC); 
      noInterrupts();
      // critical, time-sensitive code here
      playFlag = true;
  
      //set Tempo?? 10-9
      if(--tempo > 0) tempo = 9;
      interrupts();
    }

  // Handles lights
  for (j = 0; j < 256 * 4; j++) { // n cycles of all colors on wheel
    for (i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(63));
    }
    strip.show();
    delay(wait);
  }
}

void state4Cycle(uint8_t wait) {
  uint16_t i, j;
  
  // Handles sound.
  if (!playFlag) {
      //set Gesture 0-15
      if(++gNum > 11) gNum = 9;
     // set voice 0-14
     midiSetInstrument(0, gesture[gNum][0]);
     Serial.println(gesture[gNum][0], DEC); 
      noInterrupts();
      // critical, time-sensitive code here
      playFlag = true;
  
      //set Tempo?? 10-9
      if(--tempo > 0) tempo = 9;
      interrupts();
    }

  // Handles lights
  for (j = 0; j < 256 * 4; j++) { // n cycles of all colors on wheel
    for (i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(114));
    }
    strip.show();
    delay(wait);
  }
}

void state5Cycle(uint8_t wait) {
  uint16_t i, j;

  // Handles sound.
  if (!playFlag) {
      //set Gesture 0-15
      if(++gNum >14) gNum = 12;
     // set voice 0-14
     midiSetInstrument(0, gesture[gNum][0]);
     Serial.println(gesture[gNum][0], DEC); 
      noInterrupts();
      // critical, time-sensitive code here
      playFlag = true;
  
      //set Tempo?? 10-9
      if(--tempo > 0) tempo = 9;
      interrupts();
    }

  // Handles lights
  for (j = 0; j < 256 * 4; j++) { // n cycles of all colors on wheel
    for (i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(150));
    }
    strip.show();
    delay(wait);
  }
}

void state6Cycle(uint8_t wait) {
  uint16_t i, j;

  // Handles sound.
  if (!playFlag) {
      //set Gesture 0-15
      if(++gNum > 17) gNum = 15;
     // set voice 0-14
     midiSetInstrument(0, gesture[gNum][0]);
     Serial.println(gesture[gNum][0], DEC); 
      noInterrupts();
      // critical, time-sensitive code here
      playFlag = true;
  
      //set Tempo?? 10-9
      if(--tempo > 0) tempo = 9;
      interrupts();
    }

  // Handles lights
  for (j = 0; j < 256 * 4; j++) { // n cycles of all colors on wheel
    for (i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(200));
    }
    strip.show();
    delay(wait);
  }
}

void state7Cycle(uint8_t wait) {
  uint16_t i, j;

  // Handles sound.
  if (!playFlag) {
      //set Gesture 0-15
      if(++gNum > 20) gNum = 18;
     // set voice 0-14
     midiSetInstrument(0, gesture[gNum][0]);
     Serial.println(gesture[gNum][0], DEC); 
      noInterrupts();
      // critical, time-sensitive code here
      playFlag = true;
  
      //set Tempo?? 10-9
      if(--tempo > 0) tempo = 9;
      interrupts();
    }

  // Handles lights
  for (j = 0; j < 256 * 4; j++) { // n cycles of all colors on wheel
    for (i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(250));
    }
    strip.show();
    delay(wait);
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if (WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

float getBatteryVoltage() {

  float measuredvbat = analogRead(VBATPIN);

  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage

  return measuredvbat;

}

//Configures the TC to generate output events at the sample frequency.
//Configures the TC in Frequency Generation mode, with an event output once
//each time the audio sample frequency period expires.

void tcConfigure(int sampleRate)
{
 // Enable GCLK for TCC2 and TC5 (timer counter input clock)
 GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5)) ;
 while (GCLK->STATUS.bit.SYNCBUSY);

 tcReset(); //reset TC5

 // Set Timer counter Mode to 16 bits
 TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
 // Set TC5 mode as match frequency
 TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
 //set prescaler and enable TC5
 TC5->COUNT16.CTRLA.reg |= TIMER_PRESCALER_DIV | TC_CTRLA_ENABLE;
 //set TC5 timer counter based off of the system clock and the user defined sample rate of waveform
 TC5->COUNT16.CC[0].reg = (uint16_t) (CPU_HZ / sampleRate - 1);
 while (tcIsSyncing());
 
 // Configure interrupt request
 NVIC_DisableIRQ(TC5_IRQn);
 NVIC_ClearPendingIRQ(TC5_IRQn);
 NVIC_SetPriority(TC5_IRQn, 0);
 NVIC_EnableIRQ(TC5_IRQn);

 // Enable the TC5 interrupt request
 TC5->COUNT16.INTENSET.bit.MC0 = 1;
 while (tcIsSyncing()); //wait until TC5 is done syncing 
} 

//Function that is used to check if TC5 is done syncing
//returns true when it is done syncing
bool tcIsSyncing()
{
  return TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY;
}

//This function enables TC5 and waits for it to be ready
void tcStartCounter()
{
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE; //set the CTRLA register
  while (tcIsSyncing()); //wait until snyc'd
}

//Reset TC5 
void tcReset()
{
  TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
  while (tcIsSyncing());
  while (TC5->COUNT16.CTRLA.bit.SWRST);
}

//disable TC5
void tcDisable()
{
  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (tcIsSyncing());
}


//this function gets called by the interrupt at <sampleRate>Hertz
void TC5_Handler (void) {
  //YOUR CODE HERE 
    if(playFlag){
      if(!gestFlag){
        // start Gesture
        gestFlag = true;
        //set Voice
        gCnt = 1;
        //set notecnt
        noteCnt = gesture[gNum][gCnt++];
        
      }
      if(!noteFlag){
        //set volume
        volume = gesture[gNum][gCnt++];
        //set noteNum
        noteNum = gesture[gNum][gCnt++];
        //set duration
        duration = gesture[gNum][gCnt++]-tempo;

       //start note
        midiNoteOn(0, noteNum, volume);
        noteFlag = true;
        digitalWrite(LED_PIN, isLEDOn);
        isLEDOn = !isLEDOn;
      }
      duration -= 1;
      if(duration==0){
        noteCnt -= 1;
        if(noteCnt==0){
          //clear all flags
          playFlag = false;
          gestFlag = false;
        }
        noteFlag = false;
      }
    }  // END OF YOUR CODE
  TC5->COUNT16.INTFLAG.bit.MC0 = 1; //don't change this, it's part of the timer code
}
