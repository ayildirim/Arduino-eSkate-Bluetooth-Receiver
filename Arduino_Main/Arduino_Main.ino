#include <Servo.h>
#include <VirtualWire.h>
#include "buffer.h"
#include "crc.h"
#include "datatypes.h"
#include "local_datatypes.h" 

#define Bluetooth Serial2
#define UartVESC Serial1
int BluetoothData; // the data given from Computer
Servo ESC;
int CTS=23;
int RTS=22;
int conPin = 21;
struct bldcMeasure measuredValues;

// RF Variables
int maxempty = 0;
int emptycounter = 0;
bool disconnected = false;
int lastNumber = 0;
bool rfStarted = false;
int readValue = 0;

// Failsafe
bool failSafeActivated_BT = false;

uint8_t buf[VW_MAX_MESSAGE_LEN];
uint8_t buflen = VW_MAX_MESSAGE_LEN;

// Timer
unsigned long previousMillis = 0;
const long interval = 10;

void setup() {
  // Start PPM signal
  ESC.attach(3);

  // Set connection check pin
  pinMode(conPin,INPUT);
  Serial.begin(115200);
  UartVESC.begin(115200);
  Bluetooth.begin(115200);
  Bluetooth.attachCts(CTS);
  Bluetooth.attachRts(RTS);
}

void setupRF()
{
  // Start RF receiver
  vw_set_ptt_inverted(true);    // Required for RX Link Module
  vw_setup(1000);                   // Bits per sec
  vw_set_rx_pin(4);           
  vw_rx_start();   
  rfStarted = true;
}

void poll_Bluetooth()
{
  
}
void countdownLostSignal_RF()
{
    emptycounter++;
    if(maxempty < emptycounter)
    {
      maxempty = emptycounter;
    }
    if(maxempty > 21)
    {
      disconnected = true;
      maxempty = 0;
    } 
}

void loop() {
    unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
  
   if (digitalRead(conPin)){
    if(Bluetooth.available())
    {
      BluetoothData=Bluetooth.read();  
      sendThrottle(BluetoothData);
    }
    failSafeActivated_BT = true;
   }
   else if(vw_get_message(buf, &buflen))
   {
      readValue = word(buf[1], buf[0]);
      emptycounter = 0;
      disconnected = false;
      failSafeActivated_BT = false;
   }
   else
   {
      if(!failSafeActivated_BT)
      {
         countdownLostSignal_RF();
         if(disconnected)
         {
          sendThrottle(85);
         }
      }
      else
      {
        sendThrottle(85);
      }
   }
  }
}

void SerialYaz(int d)
{
    Serial.println(d);
}
void sendThrottle(int gaz)
{
    ESC.write(gaz);
    SerialYaz(gaz);
}

void yazdir()
{
  if(VescUartGetValue(measuredValues))
      {
        
        String msg = "";
        msg += "$";

        /*
        msg += measuredValues.avgMotorCurrent;
        msg += "$";
        msg += measuredValues.avgInputCurrent;
        msg += "$";
        msg += measuredValues.dutyCycleNow;
        msg += "$"; 
        */
        msg += measuredValues.rpm;
        msg += "$";
        msg += measuredValues.inpVoltage;

        /*
        msg += "$";
        msg += measuredValues.ampHours;
        msg += "$";
        msg += measuredValues.ampHoursCharged;
        msg += "$";
        msg += measuredValues.tachometer;
        msg += "$";
        msg += measuredValues.tachometerAbs;
        */
        msg += ">";
        
        Bluetooth.println(msg);
      }
      else
      {
        //Bluetooth.println("$0$1$2$3$4$5$6$7>");
      }
}

bool UnpackPayload(uint8_t* message, int lenMes, uint8_t* payload, int lenPa);
bool ProcessReadPacket(uint8_t* message, bldcMeasure& values, int len);

int ReceiveUartMessage(uint8_t* payloadReceived) {

  //Messages <= 255 start with 2. 2nd byte is length
  //Messages >255 start with 3. 2nd and 3rd byte is length combined with 1st >>8 and then &0xFF
   
  int counter = 0;
  int endMessage = 256;
  bool messageRead = false;
  uint8_t messageReceived[256];
  int lenPayload = 0;

  while (UartVESC.available()) {

    messageReceived[counter++] = UartVESC.read();

    if (counter == 2) {//case if state of 'counter' with last read 1

      switch (messageReceived[0])
      {
      case 2:
        endMessage = messageReceived[1] + 5; //Payload size + 2 for sice + 3 for SRC and End.
        lenPayload = messageReceived[1];
        break;
      case 3:
        //ToDo: Add Message Handling > 255 (starting with 3)
        break;
      default:
        break;
      }

    }
    if (counter >= sizeof(messageReceived))
    {
      break;
    }

    if (counter == endMessage && messageReceived[endMessage - 1] == 3) {//+1: Because of counter++ state of 'counter' with last read = "endMessage"
      messageReceived[endMessage] = 0;
#ifdef DEBUG
      DEBUGSERIAL.println("End of message reached!");
#endif      
      messageRead = true;
      break; //Exit if end of message is reached, even if there is still more data in buffer. 
    }
  }
  bool unpacked = false;
  if (messageRead) {
    unpacked = UnpackPayload(messageReceived, endMessage, payloadReceived, messageReceived[1]);
  }
  if (unpacked)
  {
    return lenPayload; //Message was read

  }
  else {
    return 0; //No Message Read
  }
}

bool UnpackPayload(uint8_t* message, int lenMes, uint8_t* payload, int lenPay) {
  uint16_t crcMessage = 0;
  uint16_t crcPayload = 0;
  //Rebuild src:
  crcMessage = message[lenMes - 3] << 8;
  crcMessage &= 0xFF00;
  crcMessage += message[lenMes - 2];
#ifdef DEBUG
  DEBUGSERIAL.print("SRC received: "); DEBUGSERIAL.println(crcMessage);
#endif // DEBUG

  //Extract payload:
  memcpy(payload, &message[2], message[1]);

  crcPayload = crc16(payload, message[1]);
#ifdef DEBUG
  DEBUGSERIAL.print("SRC calc: "); DEBUGSERIAL.println(crcPayload);
#endif
  if (crcPayload == crcMessage)
  {
#ifdef DEBUG
    DEBUGSERIAL.print("Received: "); SerialPrint(message, lenMes); DEBUGSERIAL.println();
    DEBUGSERIAL.print("Payload :      "); SerialPrint(payload, message[1] - 1); DEBUGSERIAL.println();
#endif // DEBUG

    return true;
  }
  else
  {
    return false;
  }
}

int PackSendPayload(uint8_t* payload, int lenPay) {
  uint16_t crcPayload = crc16(payload, lenPay);
  int count = 0;
  uint8_t messageSend[256];

  if (lenPay <= 256)
  {
    messageSend[count++] = 2;
    messageSend[count++] = lenPay;
  }
  else
  {
    messageSend[count++] = 3;
    messageSend[count++] = (uint8_t)(lenPay >> 8);
    messageSend[count++] = (uint8_t)(lenPay & 0xFF);
  }
  memcpy(&messageSend[count], payload, lenPay);

  count += lenPay;
  messageSend[count++] = (uint8_t)(crcPayload >> 8);
  messageSend[count++] = (uint8_t)(crcPayload & 0xFF);
  messageSend[count++] = 3;
  messageSend[count] = NULL;

#ifdef DEBUG
  DEBUGSERIAL.print("UART package send: "); SerialPrint(messageSend, count);

#endif // DEBUG

  //Sending package
  UartVESC.write(messageSend, count);


  //Returns number of send bytes
  return count;
}


bool ProcessReadPacket(uint8_t* message, bldcMeasure& values, int len) {
  COMM_PACKET_ID packetId;
  int32_t ind = 0;

  packetId = (COMM_PACKET_ID)message[0];
  message++;//Eliminates the message id
  len--;

  switch (packetId)
  {
  case COMM_GET_VALUES:
    ind = 14; //Skipped the first 14 bit.
    values.avgMotorCurrent = buffer_get_float32(message, 100.0, &ind);
    values.avgInputCurrent = buffer_get_float32(message, 100.0, &ind);
    values.dutyCycleNow = buffer_get_float16(message, 1000.0, &ind);
    values.rpm = buffer_get_int32(message, &ind);
    values.inpVoltage = buffer_get_float16(message, 10.0, &ind);
    values.ampHours = buffer_get_float32(message, 10000.0, &ind);
    values.ampHoursCharged = buffer_get_float32(message, 10000.0, &ind);
    ind += 8; //Skip 9 bit
    values.tachometer = buffer_get_int32(message, &ind);
    values.tachometerAbs = buffer_get_int32(message, &ind);
    return true;
    break;

  default:
    return false;
    break;
  }

}

bool VescUartGetValue(bldcMeasure& values) {
  uint8_t command[1] = { COMM_GET_VALUES };
  uint8_t payload[256];
  PackSendPayload(command, 1);
  delay(100); //needed, otherwise data is not read
  int lenPayload = ReceiveUartMessage(payload);
  if (lenPayload > 55) {
    bool read = ProcessReadPacket(payload, values, lenPayload); //returns true if sucessful
    return read;
  }
  else
  {
    return false;
  }
}

void VescUartSetCurrent(float current) {
  int32_t index = 0;
  uint8_t payload[5];
    
  payload[index++] = COMM_SET_CURRENT ;
  buffer_append_int32(payload, (int32_t)(current * 1000), &index);
  PackSendPayload(payload, 5);
}

void VescUartSetCurrentBrake(float brakeCurrent) {
  int32_t index = 0;
  uint8_t payload[5];

  payload[index++] = COMM_SET_CURRENT_BRAKE;
  buffer_append_int32(payload, (int32_t)(brakeCurrent * 1000), &index);
  PackSendPayload(payload, 5);

}
