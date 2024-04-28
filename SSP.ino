/******************
 Schießstandpiepser

 Written by Jeff Katz (github@kraln.com)
 Licensed Mozilla Public License 2.0
 ******************/

#include <SPI.h>
#include <WiFiNINA.h>
#include <ArduinoOTA.h>

#include "webpage.h"

#define HWORDS 128
uint16_t data[HWORDS];

const uint16_t big_sine[256] = {
0x1c2, 0x1cd, 0x1d8, 0x1e3, 0x1ee, 0x1f9, 0x204, 0x20f, 0x21a, 0x225, 0x22f, 0x23a, 0x245, 0x24f, 0x25a, 0x264, 0x26e, 0x278, 0x282, 0x28c, 0x296, 0x2a0, 0x2a9, 0x2b3, 0x2bc, 0x2c5, 0x2ce, 0x2d7, 0x2df, 0x2e8, 0x2f0, 0x2f8,
0x300, 0x308, 0x30f, 0x317, 0x31e, 0x325, 0x32b, 0x332, 0x338, 0x33e, 0x344, 0x34a, 0x34f, 0x354, 0x359, 0x35d, 0x362, 0x366, 0x36a, 0x36d, 0x371, 0x374, 0x377, 0x379, 0x37b, 0x37d, 0x37f, 0x381, 0x382, 0x383, 0x383, 0x384,
0x384, 0x384, 0x383, 0x383, 0x382, 0x381, 0x37f, 0x37d, 0x37b, 0x379, 0x377, 0x374, 0x371, 0x36d, 0x36a, 0x366, 0x362, 0x35d, 0x359, 0x354, 0x34f, 0x34a, 0x344, 0x33e, 0x338, 0x332, 0x32b, 0x325, 0x31e, 0x317, 0x30f, 0x308,
0x300, 0x2f8, 0x2f0, 0x2e8, 0x2df, 0x2d7, 0x2ce, 0x2c5, 0x2bc, 0x2b3, 0x2a9, 0x2a0, 0x296, 0x28c, 0x282, 0x278, 0x26e, 0x264, 0x25a, 0x24f, 0x245, 0x23a, 0x22f, 0x225, 0x21a, 0x20f, 0x204, 0x1f9, 0x1ee, 0x1e3, 0x1d8, 0x1cd,
0x1c2, 0x1b7, 0x1ac, 0x1a1, 0x196, 0x18b, 0x180, 0x175, 0x16a, 0x15f, 0x155, 0x14a, 0x13f, 0x135, 0x12a, 0x120, 0x116, 0x10c, 0x102, 0xf8, 0xee, 0xe4, 0xdb, 0xd1, 0xc8, 0xbf, 0xb6, 0xad, 0xa5, 0x9c, 0x94, 0x8c,
0x84, 0x7c, 0x75, 0x6d, 0x66, 0x5f, 0x59, 0x52, 0x4c, 0x46, 0x40, 0x3a, 0x35, 0x30, 0x2b, 0x27, 0x22, 0x1e, 0x1a, 0x17, 0x13, 0x10, 0x0d, 0x0b, 0x09, 0x07, 0x05, 0x03, 0x02, 0x01, 0x01, 0x00,
0x00, 0x00, 0x01, 0x01, 0x02, 0x03, 0x05, 0x07, 0x09, 0x0b, 0x0d, 0x10, 0x13, 0x17, 0x1a, 0x1e, 0x22, 0x27, 0x2b, 0x30, 0x35, 0x3a, 0x40, 0x46, 0x4c, 0x52, 0x59, 0x5f, 0x66, 0x6d, 0x75, 0x7c,
0x84, 0x8c, 0x94, 0x9c, 0xa5, 0xad, 0xb6, 0xbf, 0xc8, 0xd1, 0xdb, 0xe4, 0xee, 0xf8, 0x102, 0x10c, 0x116, 0x120, 0x12a, 0x135, 0x13f, 0x14a, 0x155, 0x15f, 0x16a, 0x175, 0x180, 0x18b, 0x196, 0x1a1, 0x1ac, 0x1b7};

int status = WL_IDLE_STATUS;
WiFiServer server(80);


///

#define PWMPIN 12
#define TCCx TCC0
#define TCCchannel 3

#define syncTCC while (TCCx->SYNCBUSY.reg & TCC_SYNCBUSY_MASK) {  }

#define PERIOD 96

#define DUTY (PERIOD/2)

void timer_init() {
	uint32_t cc;

  PM->APBCMASK.reg |= PM_APBCMASK_TCC0;

  GCLK->GENDIV.reg = GCLK_GENDIV_DIV(1) | GCLK_GENDIV_ID(4);
  while (GCLK->STATUS.bit.SYNCBUSY);

  GCLK->GENCTRL.reg = GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_ID(4);
  while (GCLK->STATUS.bit.SYNCBUSY);

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |
                    GCLK_CLKCTRL_GEN_GCLK4 |
                    GCLK_CLKCTRL_ID_TCC0_TCC1;

  while (GCLK->STATUS.bit.SYNCBUSY);
 
	TCCx->CTRLA.reg &= ~TCC_CTRLA_ENABLE;   // disable
	syncTCC;
  TCCx->CTRLA.reg |= TCC_CTRLA_PRESCALER(TCC_CTRLA_PRESCALER_DIV1_Val);
	syncTCC;
  TCCx->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;
  syncTCC;
  TCCx->PER.reg = PERIOD;
	syncTCC;
	TCCx->CC[TCCchannel].reg = DUTY;   // initial duty cycle
  syncTCC;
  TCCx->CTRLA.reg |=  TCC_CTRLA_ENABLE ;  // start timer
	syncTCC;
}

typedef struct {
    uint16_t btctrl;
    uint16_t btcnt;
    uint32_t srcaddr;
    uint32_t dstaddr;
    uint32_t descaddr;
} dmacdescriptor ;

volatile dmacdescriptor wrb[12] __attribute__ ((aligned (16)));
dmacdescriptor descriptor_section[12] __attribute__ ((aligned (16)));
dmacdescriptor descriptor __attribute__ ((aligned (16)));

static uint32_t chnl0 = 0;  // DMA channel
#define DMA_TRIGGER TCC0_DMAC_ID_OVF

void dma_init() {
	// trigger on TCC OVF, update TCC duty, circular
    uint32_t temp_CHCTRLB_reg;

    // probably on by default
    PM->AHBMASK.reg |= PM_AHBMASK_DMAC ;
    PM->APBBMASK.reg |= PM_APBBMASK_DMAC ;

    DMAC->BASEADDR.reg = (uint32_t)descriptor_section;
    DMAC->WRBADDR.reg = (uint32_t)wrb;
    DMAC->CTRL.reg = DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN(0xf);

    DMAC->CHID.reg = DMAC_CHID_ID(chnl0);
    DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;
    DMAC->CHCTRLA.reg = DMAC_CHCTRLA_SWRST;
    DMAC->SWTRIGCTRL.reg &= (uint32_t)(~(1 << chnl0));
    temp_CHCTRLB_reg = DMAC_CHCTRLB_LVL(0) |
      DMAC_CHCTRLB_TRIGSRC(DMA_TRIGGER) | DMAC_CHCTRLB_TRIGACT_BEAT;
    DMAC->CHCTRLB.reg = temp_CHCTRLB_reg;
    descriptor.descaddr = (uint32_t) &descriptor_section[chnl0];   // circular
    descriptor.srcaddr = (uint32_t)data + HWORDS*2;
    descriptor.dstaddr = (uint32_t)&DAC->DATA.reg;
    descriptor.btcnt =  HWORDS;
    descriptor.btctrl =  DMAC_BTCTRL_BEATSIZE_HWORD | DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_VALID;
    memcpy(&descriptor_section[chnl0],&descriptor, sizeof(dmacdescriptor));

    // start channel
    DMAC->CHID.reg = DMAC_CHID_ID(chnl0);
    DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;
}


///

void setup() {
  Serial.begin(9600);
  //while(!Serial);
  Serial.println("Initializing...");

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // by default the local IP address will be 192.168.4.1
  // you can override it with the following:
  WiFi.config(IPAddress(10, 10, 10, 10));

  // Create open network. Change this line if you want to create an WEP network:
  status = WiFi.beginAP("Schießstandpiepser");
  
  if (status != WL_AP_LISTENING) {
    Serial.println("Creating access point failed");
    while (true);
  }

  Serial.println("Starting WIFI AP...");

  wl_status_t status = (wl_status_t)WiFi.status();

  while(status != WL_AP_LISTENING)
  {
    delay(150);
  }

  // start the web server on port 80
  server.begin();

  // manage OTA
  ArduinoOTA.begin(WiFi.localIP(), "Schießstandpiepser", "update", InternalStorage);

  Serial.println("Setting up Sinewave");
  analogWriteResolution(10);
	analogWrite(A0,0);   // DAC init setup DAC pin and zero it
  Serial.println("DMA And Timer");

  for(uint16_t i = 0; i < HWORDS; i++)
  {
    data[i] = big_sine[i];
  }
	dma_init();   // do me first
	timer_init();   // startup timer
      DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;

  Serial.println("Done.");
}

bool was_beeping = false;
bool beeping = false;

void loop() 
{  
  if(beeping != was_beeping)
  {
    if (beeping)
    {
      DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;
      Serial.println("Beeping on");
    } else {
      DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;
      Serial.println("Beeping off");
    }
  was_beeping = beeping;
}

  // check for firmware update
  ArduinoOTA.poll();
  
  WiFiClient client = server.available();   // listen for incoming clients

  if (client) {                             // if you get a client,
    Serial.println("new client");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      delayMicroseconds(10);                // This is required for the Arduino Nano RP2040 Connect - otherwise it will loop so fast that SPI will never be served.
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out to the serial monitor
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            client.print(webpageindex);
            client.println();
            client.print(webpageindex2);
            client.println();
            client.print(webpageindex3);

            // the content of the HTTP response follows the header:
            //client.print("Click <a href=\"/H\">here</a> turn the LED on<br>");
            //client.print("Click <a href=\"/L\">here</a> turn the LED off<br>");

            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
          }
          else {      // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        }
        else if (c != '\r') {    // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

        // Check to see if the client request was "GET /H" or "GET /L":
        if (currentLine.endsWith("GET /H")) {
          beeping = true;              // GET /H turns the LED on
        }
        if (currentLine.endsWith("GET /L")) {
          beeping = false;              // GET /L turns the LED off
        }
      }
    }
    // close the connection:
    client.stop();
  }
}