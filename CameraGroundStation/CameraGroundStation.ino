#include <SPI.h>   // Comes with Arduino IDE
#include "RF24.h"  // Download and Install (See above)


// --------------------------------
// http://subethasoftware.com/2013/04/09/arduino-compiler-problem-with-ifdefs-solved/
// BOF preprocessor bug prevent - insert me on top of your arduino-code
// From: http://www.a-control.de/arduino-fehler/?lang=en
#if 1
__asm volatile ("nop");
#endif
// --------------------------------

#include <Wire.h>


//#define USE_SOFT_SERIAL
// serial stuff  
static const byte LF = 10; // line feed character
#ifdef USE_SOFT_SERIAL
   #include <SoftwareSerial.h>
   static const unsigned int _BAUDRATE = 19200; // 19200 is the maximum reliable soft serial baud rate for 8MHz processors
   #define txPin A0
   #define rxPin A1
   SoftwareSerial mySerial = SoftwareSerial(rxPin, txPin);  
   SoftwareSerial *serialPtr = &mySerial;
#else
   // 38400 is the maximum supposed reliable UART baud rate for 8MHz processors
   // However, I've had succes at 500000bps with an USB-FTDI cable
   // I've modified the file arduino-1.6.5/hardware/arduino/avr/cores/arduino/HardwareSerial.cpp
   // substituting all the lines with operations like this (for both rx and tx buffers):
   //
   //  _rx_buffer_tail = (rx_buffer_index_t)(_rx_buffer_tail + 1) % SERIAL_RX_BUFFER_SIZE;
   //
   // for two lines like:
   //
   //  _rx_buffer_tail++;
   //  _rx_buffer_tail %= SERIAL_RX_BUFFER_SIZE;
   //
   // see http://mekonik.wordpress.com/2009/03/02/modified-arduino-library-serial/
   //
   static const unsigned long _BAUDRATE = 230400;
   HardwareSerial *serialPtr = &Serial;
#endif    

#define SOFT_SPI_MISO_PIN 12
#define SOFT_SPI_MOSI_PIN 11
#define SOFT_SPI_SCK_PIN 13

#define  CE_PIN  7   // The pins to be used for CE and SN
#define  CSN_PIN 8

//#define QQVGA
#define QQQVGA

#ifdef QQVGA
static const uint8_t fW = 160;
static const uint8_t fH = 120;
#else
#ifdef QQQVGA
static const uint8_t fW = 80;
static const uint8_t fH = 60;
#endif
#endif

static const uint8_t TRACK_BORDER = 4; // always multiple of 2 (YUYV: 2 pixels)
static const uint8_t YUYV_BPP = 2; // bytes per pixel
static const unsigned int MAX_FRAME_LEN = fW * YUYV_BPP;

uint8_t rcvbuf[16], rcvbufpos = 0, c;
byte rowBuf[MAX_FRAME_LEN];
uint8_t volatile thresh = 128;

enum serialRequest_t {
  SEND_NONE = 0,
  SEND_DARK,
  SEND_BRIG,
  SEND_FPS,
  SEND_0PPB = MAX_FRAME_LEN,
  SEND_1PPB = fW,
  SEND_2PPB = fW/2,
  SEND_4PPB = fW/4,
  SEND_8PPB =fW/8 
};

serialRequest_t serialRequest = SEND_NONE;

// fps calculation stuff
static unsigned int oneSecond = 1000;
unsigned int volatile frameCount = 0;
unsigned int fps = 0;
unsigned long volatile lastTime = 0;
unsigned long volatile timeStamp = 0;

boolean volatile bRequestPending = false;


/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus (usually) pins 7 & 8 (Can be changed) */
RF24 radio(CE_PIN, CSN_PIN);

byte addresses[][6] = {"1Node", "2Node"}; // These will be the names of the "Pipes"

unsigned long timeNow;  // Used to grab the current time, calculate delays
unsigned long started_waiting_at;
boolean timeout;       // Timeout? True or False

void setup()   /****** SETUP: RUNS ONCE ******/
{
  serialPtr->begin(_BAUDRATE);
  
  radio.begin();          // Initialize the nRF24L01 Radio
 // radio.setChannel(108);  // Above most WiFi frequencies
  radio.setDataRate(RF24_250KBPS); // Fast enough.. Better range
  // Set the Power Amplifier Level low to prevent power supply related issues since this is a
  // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  // PALevelcan be one of four levels: RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH and RF24_PA_MAX
  radio.setPALevel(RF24_PA_LOW);
  //  radio.setPALevel(RF24_PA_MAX);

  // Open a writing and reading pipe on each radio, with opposite addresses
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1, addresses[1]);

  // Start the radio listening for data
  radio.startListening();

}

void loop()
{

}

// **************************************************************
//                      PROCESS SERIAL REQUEST
// **************************************************************
void processRequest() {
    radio.write(rcvbuf, rcvbufpos);
    switch (serialRequest) {
      case SEND_0PPB: for (int i =0; i< fH; i++) {
                          // fifo_readRow0ppb(rowBuf, rowBuf+serialRequest);
                          while (!radio.available());
                          radio.read(rowBuf, serialRequest);
                          serialPtr->write(rowBuf, serialRequest);
                          serialPtr->write(LF); 
                      } break;
      case SEND_1PPB: for (int i =0; i< fH; i++) {
                          // fifo_readRow1ppb(rowBuf, rowBuf+serialRequest);
                          while (!radio.available());
                          radio.read(rowBuf, serialRequest);
                          serialPtr->write(rowBuf, serialRequest); 
                          serialPtr->write(LF); 
                      } break;
      case SEND_2PPB:  for (int i =0; i< fH; i++) {
                          // fifo_readRow2ppb(rowBuf, rowBuf+serialRequest);
                          while (!radio.available());
                          radio.read(rowBuf, serialRequest);
                          serialPtr->write(rowBuf, serialRequest); 
                          serialPtr->write(LF); 
                      } break;
      case SEND_4PPB: for (int i =0; i< fH; i++) {
                          // fifo_readRow4ppb(rowBuf, rowBuf+serialRequest);
                          while (!radio.available());
                          radio.read(rowBuf, serialRequest);
                          serialPtr->write(rowBuf, serialRequest); 
                          serialPtr->write(LF); 
                      } break;
      case SEND_8PPB: for (int i =0; i< fH; i++) {
                          // fifo_readRow8ppb(rowBuf, rowBuf+serialRequest, thresh);
                          while (!radio.available());
                          radio.read(rowBuf, serialRequest);
                          serialPtr->write(rowBuf, serialRequest); 
                          serialPtr->write(LF); 
                      } break;
      case SEND_BRIG: 
                      //fifo_getBrig(rowBuf, fW, fH, TRACK_BORDER, thresh);
                      while (!radio.available());
                      radio.read(rowBuf, 4);
                      serialPtr->write(rowBuf, 4);
                      serialPtr->write(LF); 
                      break;
      case SEND_DARK: 
                      //fifo_getDark(rowBuf, fW, fH, TRACK_BORDER, thresh);
                      while (!radio.available());
                      radio.read(rowBuf, 4);
                      serialPtr->write(rowBuf, 4);
                      serialPtr->write(LF); 
                      break;
      case SEND_FPS:  
                      // calcFPS(fps);
                      while (!radio.available());
                      radio.read(&fps, DEC);
                      serialPtr->print(fps, DEC);
                      serialPtr->write(LF); 
                      break;

      default : break;
    }
}

// *****************************************************
//               PARSE SERIAL BUFFER
// ****************************************************
void parseSerialBuffer(void) {
       if (strcmp((char *) rcvbuf, "hello") == 0) {
            serialPtr->print("Hello to you too!\n");
        } else if ( strlen((char *) rcvbuf) > 5 && 
                    strncmp((char *) rcvbuf, "send ", 5) == 0) {
            serialRequest = (serialRequest_t)atoi((char *) (rcvbuf + 5)); 
            // serialPtr->print("ACK\n");
            bRequestPending = true;       
        } 
        else if (strlen((char *) rcvbuf) > 5 &&
                strncmp((char *) rcvbuf, "dark ", 5) == 0) {
                  thresh = atoi((char *) (rcvbuf + 5));
                  // serialPtr->print("ACK\n");
                  serialRequest = SEND_DARK;
                  bRequestPending = true;
        }
        else if (strlen((char *) rcvbuf) > 5 &&
                strncmp((char *) rcvbuf, "brig ", 5) == 0) {
                  thresh = atoi((char *) (rcvbuf + 5));
                  // serialPtr->print("ACK\n");
                  serialRequest = SEND_BRIG;
                  bRequestPending = true;
        }
        else if (strlen((char *) rcvbuf) > 7 &&
                strncmp((char *) rcvbuf, "thresh ", 7) == 0) {
                  thresh = atoi((char *) (rcvbuf + 7));
        }

        if(bRequestPending == true) {  
            radio.stopListening();                                    // First, stop listening so we can talk.
            processRequest();
            radio.startListening();
        }
}

void serialEvent() {
  while (serialPtr->available()) {
    // get the new byte:
    c = serialPtr->read();
    if (c != LF) {
            rcvbuf[rcvbufpos++] = c;
    } else if (c == LF) {
        rcvbuf[rcvbufpos++] = 0;
        parseSerialBuffer();
        rcvbufpos = 0;
    }
  }
}
