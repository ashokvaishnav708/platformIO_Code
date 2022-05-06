/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 16/03/20

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/


/*******************************************************************************************************
  Program Operation -

  Serial monitor baud rate is set at 9600
*******************************************************************************************************/

#define programversion "V1.0"

#include <Arduino.h>
#include <SPI.h>
#include <SX128XLT.h>
//#include "Settings.h"

#define NSS SS
#define RFBUSY 40
#define NRESET 43
#define LED1 PIN_LED1
#define DIO1 42
//#define DIO2 -1                 //not used 
//#define DIO3 -1                 //not used
//#define RX_EN -1                //pin for RX enable, used on some SX1280 devices, set to -1 if not used
//#define TX_EN -1                //pin for TX enable, used on some SX1280 devices, set to -1 if not used 

        
#define LORA_DEVICE DEVICE_SX1280                //we need to define the device we are using

//*******  Setup LoRa Parameters Here ! ***************

//LoRa Modem Parameters
const uint32_t Frequency = 2445000000;           //frequency of transmissions in hz
const int32_t Offset = 0;                        //offset frequency in hz for calibration purposes
const uint8_t Bandwidth = LORA_BW_0800;          //LoRa bandwidth
const uint8_t SpreadingFactor = LORA_SF8;        //LoRa spreading factor
const uint8_t CodeRate = LORA_CR_4_5;            //LoRa coding rate
const uint16_t Calibration = 11350;              //Manual Ranging calibrarion value

const int8_t TXpower = 10;                       //Transmit power used
const uint32_t RangingAddress = 16;              //must match address in recever

const uint16_t  rangingRXTimeoutmS = 0xFFFF;     //ranging RX timeout in mS

SX128XLT LT;

uint32_t endwaitmS;
uint16_t IrqStatus;
uint32_t response_sent;

void led_Flash(uint16_t flashes, uint16_t delaymS)
{
  uint16_t index;
  for (index = 1; index <= flashes; index++)
  {
    digitalWrite(LED1, HIGH);
    delay(delaymS);
    digitalWrite(LED1, LOW);
    delay(delaymS);
  }
}



void loop()
{

  Serial.println("IN LOOP SECTION");
  LT.receiveRanging(RangingAddress, 0, TXpower, NO_WAIT);

  endwaitmS = millis() + rangingRXTimeoutmS;

  while (!digitalRead(DIO1) && (millis() <= endwaitmS));          //wait for Ranging valid or timeout

  if (millis() >= endwaitmS)
  {
    //Serial.println("Error - Ranging Receive Timeout!!");
    //led_Flash(2, 100);
    //Serial.println("LED FLASHED 2 TIMES");                                             //single flash to indicate timeout
  }
  else
  {
    IrqStatus = LT.readIrqStatus();
    //Serial.println("LED : ON");
    digitalWrite(LED1, HIGH);

    if (IrqStatus & IRQ_RANGING_SLAVE_RESPONSE_DONE)
    {
      response_sent++;
      Serial.print(response_sent);
      Serial.print(" Response sent");
    }
    else
    {
      Serial.print("Slave error,");
      Serial.print(",Irq,");
      Serial.print(IrqStatus, HEX);
      LT.printIrqStatus();
    }
    //Serial.println("LED : OFF");
    digitalWrite(LED1, LOW);
    Serial.println();
  }

}




void setup()
{
  Serial.begin(115200);            //setup Serial console ouput
  Serial.println();
  Serial.println(__FILE__);
  Serial.print(F("Compiled "));
  Serial.print(__TIME__);
  Serial.print(F(" "));
  Serial.println(__DATE__);
  Serial.println(F(programversion));
  Serial.println(F("Stuart Robinson"));
  Serial.println();

  Serial.println("55_Ranging_Slave Starting");

  pinMode(LED1, OUTPUT);
  led_Flash(2, 125);

  SPI.begin();

  if (LT.begin(NSS, NRESET, RFBUSY, DIO1, LORA_DEVICE))
  {
    Serial.println(F("Device found"));
    led_Flash(2, 125);
    delay(1000);
  }
  else
  {
    Serial.println(F("No device responding"));
    while (1)
    {
      led_Flash(50, 50);                                 //long fast speed flash indicates device error
    }
  }

  //The function call list below shows the complete setup for the LoRa device for ranging using the information
  //defined in the Settings.h file.
  //The 'Setup LoRa device for Ranging' list below can be replaced with a single function call, note that
  //the calibration value will be loaded automatically from the table in the library;
  //LT.setupRanging(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, RangingAddress, RangingRole);

  LT.setupRanging(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, RangingAddress, RANGING_SLAVE);
  Serial.println("AFTER SETUP RANGING");
  //***************************************************************************************************
  //Setup LoRa device for Ranging Slave
  //***************************************************************************************************
  LT.setMode(MODE_STDBY_RC);
  LT.setPacketType(PACKET_TYPE_RANGING);
  LT.setModulationParams(SpreadingFactor, Bandwidth, CodeRate);
  LT.setPacketParams(12, LORA_PACKET_VARIABLE_LENGTH, 0, LORA_CRC_ON, LORA_IQ_NORMAL, 0, 0);
  LT.setRfFrequency(Frequency, Offset);
  LT.setTxParams(TXpower, RADIO_RAMP_02_US);
  LT.setRangingMasterAddress(RangingAddress);
  LT.setRangingSlaveAddress(RangingAddress);
  LT.setRangingCalibration(LT.lookupCalibrationValue(SpreadingFactor, Bandwidth));
  LT.setRangingRole(RANGING_SLAVE);
  LT.writeRegister(REG_RANGING_FILTER_WINDOW_SIZE, 8); //set up window size for ranging averaging
  LT.setHighSensitivity();
  //***************************************************************************************************

  Serial.println("AFTER SETUP LORA");
  LT.setRangingCalibration(11300);               //override automatic lookup of calibration value from library table

  Serial.print(F("Calibration,"));
  Serial.println(LT.getSetCalibrationValue());           //reads the calibratuion value currently set
  delay(2000);
}
