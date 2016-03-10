

#include <Wire.h>
#include <SoftwareSerial.h>

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

SoftwareSerial p2pSerial(P2P_SERIAL_RX,P2P_SERIAL_TX); //RX, TX
// the setup routine runs once when you press reset:
//unsigned char serialBuffer[500];
uint8_t computerReceiveBuffer[24] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                             0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,};


void setup() {                
  // initialize the digital pin as an output.
  pinMode(LED_BLUE, OUTPUT);   
  pinMode(BT_ENABLE, OUTPUT);
  pinMode(TAPE_POWER,OUTPUT);
  digitalWrite(BT_ENABLE,HIGH); //LOW - turn off
  digitalWrite(TAPE_POWER, HIGH);// HIGH - tape on 
  
  Serial.begin(500000); //115200
  Serial.println("Connected");
  Serial1.begin(500000);
  Wire.begin();  // Start I2C Bus as Master
 
  
  delay(10);
  p2pSerial.begin(9600);
  //digitalWrite(TAPE_POWER, HIGH);// HIGH - tape on 
  
  
  delay(90); //100 ms originally
  int foo = 200;
  p2pSerial.write(foo);
  foo=1; 
  p2pSerial.write(foo);
  //Serial.write('C');   
  delay(2000);//master should transmit only after slaves are ready

 // delay(150);//slaves need some time to determine their addresses
  //Serial.write('D');   
  TWBR = 12;  //This sets the clock to 400kHz for I2C ((16*10^6) / (16+2*(12)*1))/1000

  findWhoIsAround();
  
   
}

// the loop routine runs over and over again forever:
void loop() {

   Serial1.write(Serial.available());
  if (Serial.available() >= 1) { 
    Serial1.write(Serial.available());
    digitalWrite(LED_BLUE,HIGH);
    delay(1);  //The communications sometimes crashes without this delay. 
    int whereToSend = Serial.read()*256; 
    whereToSend = whereToSend + Serial.read(); 
    byte opCode = Serial.read();
    byte sizeofData = Serial.read(); 
    //wait for the rest of the bytes to arrive, otherwise they get corrupted
    while(Serial.available()<sizeofData) { 
      Serial1.write(Serial.available());
    }
    for(int i=0; i<sizeofData; i++) { 
        computerReceiveBuffer[i] = Serial.read();  
    }
    delay(1); //The communications might crash without this delay 
    //Check just in case if the commands are below 128. This is the maximum that can be sent by java Char (-127 to 127) 
    sendBytesToSlave(whereToSend, opCode, sizeofData, computerReceiveBuffer); 
    //void sendBytesToSlave(int address, uint8_t opCode, uint8_t dataSize, uint8_t toSend []) { 
    //delayMicroseconds(500);
    Serial.flush();
    digitalWrite(LED_BLUE, LOW);
  }//end if Serial is available 
 
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
        Serial.print(i);
        Serial.println(" is present");
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
  
    Serial.print("S");
    Serial.print(",");
    for(int i=0; i<bytes; i = i+2) { 
      Serial.print((inputArray[i+1]<<8) + inputArray[i]);
      if (i==bytes-2) { 
      //DO nothing
        }
      else
        Serial.print(",");
    }
    Serial.println(" ");
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
    
      Serial.print("A");
      Serial.print(",");
      for(int i=0; i<bytes; i = i+2) { 
        Serial.print((inputArray[i+1]<<8) + inputArray[i]);
        if (i==bytes-2) { 
        //DO nothing
          }
        else
          Serial.print(",");
      }
      Serial.println(" ");
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
