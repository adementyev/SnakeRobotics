

#include <Wire.h>
//#include <SoftwareSerial.h>

//Define pins
#define BT_ENABLE      6
#define TAPE_POWER     5
#define LED_BLUE       8 
#define LED_RED        9
#define PIN_NEOPIXEL   13
#define P2P_SERIAL_RX  11
#define P2P_SERIAL_TX  10

#define NUMPIXELS      1

boolean lookUpPresence[128];
int lookUpNumberOfSensors[128]; 
unsigned int output = 0 ;
unsigned int output2 = 0; 
unsigned int  output3 = 0;
unsigned int deviceID = 200;
unsigned char char1 = 0; 
unsigned char char2 =  0; 
unsigned char char3 = 0; 
unsigned char char4 = 0;
unsigned char char5 = 0; 
unsigned char char6 = 0; 
unsigned char char7 = 0; 
unsigned char char8 = 0; 
int x = 0;
int incomingByte = 0; 
char c = 'a';

//SoftwareSerial p2pSerial(P2P_SERIAL_RX,P2P_SERIAL_TX); //RX, TX
// the setup routine runs once when you press reset:
//unsigned char serialBuffer[500];
uint8_t computerReceiveBuffer[29] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                             0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                             0x00,0x00,0x00,0x00,0x00};
uint8_t buf[29] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                             0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                             0x00,0x00,0x00,0x00,0x00};

int RECEIVE_BUFFER_SIZE = 29;
int count = 0;
void setup() {                
  // initialize the digital pin as an output.
  pinMode(LED_BLUE, OUTPUT);   
  pinMode(BT_ENABLE, OUTPUT);
  pinMode(TAPE_POWER,OUTPUT);
  digitalWrite(BT_ENABLE,HIGH); //LOW - turn off
  digitalWrite(TAPE_POWER, HIGH);// HIGH - tape on 
  
  //Serial.begin(500000); //115200
 // Serial.println("Connected");
  Serial1.begin(9600);
  SerialUSB.begin(115200);
  Serial2.begin(1000000);
  
  Wire.begin();  // Start I2C Bus as Master
 // sercom3.disableWIRE();                         // Disable the I2C bus
 // SERCOM3->I2CM.BAUD.bit.BAUD = SystemCoreClock / ( 2 * 400000) - 1 ;   // // Set the I2C SCL frequency to 400kHz
 // sercom3.enableWIRE();   
 Wire.setClock(400000);

  
  delay(10);
 // p2pSerial.begin(9600);
  //digitalWrite(TAPE_POWER, HIGH);// HIGH - tape on 
  
  
  delay(90); //100 ms originally
  int foo = 200;
  Serial1.write(foo);
  foo=1; 
  Serial1.write(foo);
  //Serial.write('C');   
  delay(2000);//master should transmit only after slaves are ready
  Serial1.end(); 
  Serial1.begin(500000);
 // delay(150);//slaves need some time to determine their addresses
  //Serial.write('D');   
  //TWBR = 12;  //This sets the clock to 400kHz for I2C ((16*10^6) / (16+2*(12)*1))/1000

  findWhoIsAround();
  
   
}

// the loop routine runs over and over again forever:
void loop() {

  
  digitalWrite(LED_BLUE,HIGH); 
  int numBytes =0;
  Serial1.write(Serial.available());
  while(Serial.available()) { 
    buf[numBytes] = SerialUSB.read(); ;
    Serial2.write(buf[numBytes]); 
    numBytes++;  
  }
  if (buf[0] == '@') { 
      int whereToSend = buf[1]*256; 
      whereToSend = whereToSend + buf[2];
      byte opCode = buf[3]; 
      byte sizeofData = buf[4]; 
      for (int i=0; i<sizeofData; i++) {
        computerReceiveBuffer[i] = buf[i+5]; 
      }//end for 
      sendBytesToSlave(whereToSend, opCode, sizeofData, computerReceiveBuffer); 
  }
  
  digitalWrite(LED_BLUE,LOW);
  /*
  //Serial receive code
  int count=0;
  char buf[RECEIVE_BUFFER_SIZE];
  while (count<RECEIVE_BUFFER_SIZE) {
    if (Serial.available()) {  // receive all 11 bytes into "buf"
      buf[count++] = Serial.read();
    }//end if 
  }//end while

  if (buf[0] == '@') { 
      int whereToSend = buf[1]*256; 
      whereToSend = whereToSend + buf[2];
      byte opCode = buf[3]; 
      byte sizeofData = buf[4]; 
      for (int i=0; i<sizeofData; i++) {
        computerReceiveBuffer[i] = buf[i+1]; 
      }//end for 
      delay(1); //The communications might crash without this delay 
      //Check just in case if the commands are below 128. This is the maximum that can be sent by java Char (-127 to 127) 
      sendBytesToSlave(whereToSend, opCode, sizeofData, computerReceiveBuffer); 
      //void sendBytesToSlave(int address, uint8_t opCode, uint8_t dataSize, uint8_t toSend []) { 
      //delayMicroseconds(500);
      SerialUSB.flush();
      digitalWrite(LED_BLUE, LOW);
  }
  */
  /*
   if (SerialUSB.available()) { 
    char buf[RECEIVE_BUFFER_SIZE];
    digitalWrite(LED_BLUE,HIGH);
    //delay(1);  //The communications sometimes crashes without this delay.

    if (SerialUSB.read() == '@') { 
      count = 0; 
      Serial1.write(255);
      
    
       while (count < RECEIVE_BUFFER_SIZE) {
          if (SerialUSB.available()) {  // receive all 11 bytes into "buf"
            computerReceiveBuffer[count] = SerialUSB.read();
            Serial2.write(computerReceiveBuffer[count]);
            count++;       
           }//end if 
        }//end while    
      }
  }//end if
  */
  
  /*
   Serial1.write(SerialUSB.available());
  if (SerialUSB.available() >= 3) { 
  //  Serial1.write(Serial.available());
    digitalWrite(LED_BLUE,HIGH);
    delay(1);  //The communications sometimes crashes without this delay. 
    int whereToSend = SerialUSB.read()*256; 
    whereToSend = whereToSend + SerialUSB.read(); 
    byte opCode = SerialUSB.read();
    byte sizeofData = SerialUSB.read(); 
    //Serial2.write(sizeofData);
    //wait for the rest of the bytes to arrive, otherwise they get corrupted
    int i = 0; 
    if (SerialUSB.available()) { 
      computerReceiveBuffer[i] = SerialUSB.read();
      Serial2.write(computerReceiveBuffer[i]);
    }

    delay(1); //The communications might crash without this delay 
    //Check just in case if the commands are below 128. This is the maximum that can be sent by java Char (-127 to 127) 
    sendBytesToSlave(whereToSend, opCode, sizeofData, computerReceiveBuffer); 
    //void sendBytesToSlave(int address, uint8_t opCode, uint8_t dataSize, uint8_t toSend []) { 
    //delayMicroseconds(500);
    SerialUSB.flush();
    digitalWrite(LED_BLUE, LOW);
  }//end if Serial is available 
  */
  /*
    boolean getNumbersFromSerial() {
      int count = 0;
      time_t pctime = 0;
      while (count < 10) {
        if (Serial.available()) {
        char c = Serial.read();
          if (c == '@') {
            pctime = 0;  // always begin anew when '@' received
            count = 0;
          } else {
            if (c >= '0' && c <= '9') {
              pctime = (10 * pctime) + (c - '0') ; // convert digits to a number
              count++;
            }
          }
        }
      }
      DateTime.sync(pctime);   // Sync clock to the time received
    }
    */
    /*
    while(SerialUSB.available()<sizeofData) { 
      Serial1.write(SerialUSB.available());
    }
    for(int i=0; i<sizeofData; i++) { 
        computerReceiveBuffer[i] = SerialUSB.read();  
    }
    */
    /*
    delay(1); //The communications might crash without this delay 
    //Check just in case if the commands are below 128. This is the maximum that can be sent by java Char (-127 to 127) 
    sendBytesToSlave(whereToSend, opCode, sizeofData, computerReceiveBuffer); 
    //void sendBytesToSlave(int address, uint8_t opCode, uint8_t dataSize, uint8_t toSend []) { 
    //delayMicroseconds(500);
    SerialUSB.flush();
    digitalWrite(LED_BLUE, LOW);
  }//end if Serial is available 
  */
  
 
  for(int i=0; i<128; i++) { 
    if(lookUpPresence[i]) { 
     // uint8_t data[11] = {50,50,50,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B};
    //  sendBytesToSlave(i,data,11); 
     // delay(1);
      requestFromDevice(i,16);
    }//end if
  }//end for
}//end loop


void setToZeros() { 
  unsigned int output = 0 ;
  unsigned int output2 = 0; 
  unsigned int  output3 = 0;
  unsigned int deviceID = 200 ;
  unsigned char char1 = 0; 
  unsigned char char2 =  0; 
  unsigned char char3 = 0; 
  unsigned char char4 = 0;
  unsigned char char5 = 0; 
  unsigned char char6 = 0; 
  unsigned char char7 = 0; 
  unsigned char char8 = 0; 
  
}


void findWhoIsAround() { 
    for (int i=0; i<128; i++) { 
      requestFromDeviceDuringStart(i,14); 
      if(deviceID==i){ 
        SerialUSB.print(i);
        SerialUSB.println(" is present");
        lookUpPresence[i] = true; 
      } 
      else {
        //Serial.print(", ");
       // Serial.print(i);
       // Serial.println(" is not present"); 
        lookUpPresence[i] = false;
      } 
      //delay(1000);
      digitalWrite(LED_RED, !digitalRead(LED_RED)); 
  }//end for
}//end whoIsAround



void requestFromDevice(int device, int bytes) { 
    unsigned char inputArray[bytes] ;
    setToZeros(); 
    Wire.requestFrom(device, bytes);   
    while(Wire.available())    // slave may send less than requested
    { 
      unsigned char  c= Wire.read(); 
      inputArray[x] = c; 
      x++;   
    }
    x = 0; 
  
    SerialUSB.print("S");
    SerialUSB.print(",");
    for(int i=0; i<bytes; i = i+2) { 
      SerialUSB.print((inputArray[i+1]<<8) + inputArray[i]);
      if (i==bytes-2) { 
      //DO nothing
        }
      else
        SerialUSB.print(",");
    }
    SerialUSB.println(" ");
    deviceID = (inputArray[1]<<8) + inputArray[0];
   // delay(1);//8 is ok for 115200
    
    
}

void requestFromDeviceDuringStart(int device, int bytes) { 
    unsigned char inputArray[bytes] ;
    setToZeros(); 
    if (Wire.requestFrom(device, bytes)>0) {   
      while(Wire.available())    // slave may send less than requested
      { 
        unsigned char  c= Wire.read(); 
        inputArray[x] = c; 
        x++;   
      }
      x = 0; 
    
      SerialUSB.print("A");
      SerialUSB.print(",");
      for(int i=0; i<bytes; i = i+2) { 
        SerialUSB.print((inputArray[i+1]<<8) + inputArray[i]);
        if(i==bytes-2) { 
        //DO nothing
          }
        else
          SerialUSB.print(",");
      }
      SerialUSB.println(" ");
      deviceID = (inputArray[1]<<8) + inputArray[0];
      delay(5); //was 5 before
    }
}

void sendByteToSlave(int address, uint8_t command1, uint8_t command2, uint8_t command3) { 
  Wire.beginTransmission(address); 
  Wire.write(command1); 
  Wire.write(command2); 
  Wire.write(command3); 
  Wire.endTransmission(); 
} //end sendByteToSlave

void sendBytesToSlave(int address, byte opCode, byte dataSize, byte toSend []) { 
  Wire.beginTransmission(address); 
  Wire.write(opCode); 
  Wire.write(dataSize);
  for(byte i=0; i<dataSize; i++) { 
    Wire.write(toSend[i]); 
  }
  Wire.endTransmission();
}
