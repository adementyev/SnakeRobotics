  /*
This is for the rigid flex board
Added a servo angle commands in this code
 */
 //TODO
 //1) fast commands from the master
//#include <Servo.h>
#include <SPI.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include "I2Cdev.h"
#include <avr/io.h>
#include <Adafruit_NeoPixel.h>


 
// Pin 13 has an LED connected on most Arduino boards.
// give it a name:

const int RELAY_CONTROL = 7;
const int ANALOG_SERVO_READ = A6; 
const int LED_BLUE  = A0; 
const int LED_RED = A1; 
const int LED_GREEN = A2;
const int P2P_TX = 8; 
const int P2P_RX = 2; 
const int SERVO_PWM = 10;
const int NEOPIXEL_PIN = 6;
const int NUMBER_NEOPIXEL = 8; 
int val = 0; 

int8_t length = 14;

uint8_t i2caddr = 0x25;
unsigned char sensorValues [15] = {'a','a','a','a','a','a','a','a','a','a','a','a','a','a','a'}; 


int numberOfSensors = 3;
int deviceType = 1;
int deviceID = 0; 

boolean startUp = true; 
int incomingByte = 0;

byte val1; 
byte val2;  

int lightAnalogRead = 2000; 
int thermistorAnalogRead = 3000; 
int positionAnalogRead = 4000;
String valueString = "a"; 
byte valAll[1]; 

int receiveCommand = 1000;

uint8_t masterReceive[3] = {0x00,0x00,0x00};
uint8_t capData[6] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
uint8_t data[14] = {'a','a','a','a','a','a','a','a','a','a','a','a','a','a'}; ;

uint8_t opCode = 0x00;
uint8_t sizeofData = 0x00;
//Servo servoMotor; 
SoftwareSerial p2pSerial(P2P_RX,P2P_TX); //RX, TX
 
 int temp = 0;
boolean requested = false;

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMBER_NEOPIXEL,
                          NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

uint8_t masterReceiveBuffer[24] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                             0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,};
void setup() {      
    // initialize the digital pin as an output.
    pinMode(LED_BLUE, OUTPUT);  
    pinMode(LED_RED, OUTPUT);  
    pinMode(LED_GREEN, OUTPUT);  
    pinMode(RELAY_CONTROL, OUTPUT);  
    pinMode(SERVO_PWM,OUTPUT);
   
    digitalWrite(RELAY_CONTROL, LOW); //HIGH ON
    digitalWrite(LED_RED, HIGH); //HIGH ON
    p2pSerial.begin(9600);

    delay(50); 
    
    while(true){ 
      if(p2pSerial.available()>1) { 
        if(p2pSerial.read()==200) { 
          incomingByte = p2pSerial.read(); 
          deviceID=incomingByte; 
              
          p2pSerial.write(200);
          p2pSerial.write(incomingByte+1);
          digitalWrite(LED_RED, LOW);  
          break; 
        }
      }
    }
    p2pSerial.end(); 
    
    I2Cdev::i2c_init();
  
    Wire.begin(deviceID); // Start I2C Bus as a Slave (Device Number 2)
    Wire.onRequest(requestEvent); // register event to send data to master (respond to requests)
    Wire.onReceive(receiveEvent); //register even to receice data from master
    
    setBaseline();
    
    
    pixels.begin(); 
    //pixels.setPixelColor(0, pixels.Color(200,0,0)); // Moderately bright green color.    
    for (int i=0; i<NUMBER_NEOPIXEL; i++) {
      pixels.setPixelColor(i, pixels.Color(0,20,0)); // Moderately bright green color.
    }
    pixels.show();
    
    
}

// the loop routine runs over and over again forever:
void loop() {
    
    /*
    digitalWrite(LED_BLUE, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(100);               // wait for a second
    digitalWrite(LED_BLUE, LOW);   // turn the LED on (HIGH is the voltage level)
    delay(100);               // wait for a second
    */
    
    noInterrupts();
    digitalWrite(SERVO_PWM,HIGH); 
    delayMicroseconds(receiveCommand);
    digitalWrite(SERVO_PWM,LOW);
    interrupts();

    delay(10);
    readSensorValues();  
    val = analogRead(ANALOG_SERVO_READ);   // read the input pin
    
    
}//end loop



void requestEvent()
{
    if (startUp) { 
//   delay(10);
        byte a = deviceID & 0xFF;
        byte b = (deviceID >>8 ) & 0xFF;
        
        byte c = numberOfSensors & 0xFF; ; 
        byte d = (numberOfSensors >>8 ) & 0xFF;
        
        byte e = deviceType & 0xFF; ; 
        byte f = (deviceType >>8 ) & 0xFF;
        
        int tempread = digitalRead(9); 
        byte g = tempread & 0xFF; ; 
        byte h = (tempread >>8 ) & 0xFF;     
        
        tempread = digitalRead(9); 
        byte i = tempread & 0xFF; ; 
        byte j = (tempread >>8 ) & 0xFF;   
        
        tempread = digitalRead(9); 
        byte k = tempread & 0xFF; ; 
        byte l = (tempread >>8 ) & 0xFF; 
        
        tempread = digitalRead(9); 
        byte m = tempread & 0xFF; ; 
        byte n = (tempread >>8 ) & 0xFF; 
        
        byte All [] = {a,b,c,d,e,f,g,h,i,j,k,l,m,n};  
           
        Wire.write(All, 14); // respond with message 
        startUp = false;
    } 
    
    else { 
        requested = true;
        digitalWrite(LED_GREEN, HIGH);
        
      //  val = analogRead(ANALOG_SERVO_READ);   // read the input pin
        
      //  readSensorValues();
        
        byte a = deviceID & 0xFF;
        byte b = (deviceID >>8 ) & 0xFF;
        
        byte All [] = {a,b,
                       data[0] & 0xFF, 0x00, 
                       data[1] & 0xFF, 0x00, 
                       data[2] & 0xFF, 0x00, 
                       data[3] & 0xFF, 0x00, 
                       data[4] & 0xFF, 0x00, 
                       data[5] & 0xFF, 0x00, 
                       val & 0xFF, (val >>8 ) & 0xFF };  
                    
        Wire.write(All, 16); // respond with message of 6 bytes
        digitalWrite(LED_GREEN, LOW);   
       // digitalWrite(LED_BLUE,LOW); 
    }//end else        
}//end request event

void receiveEvent(int howMany)
{
 // digitalWrite(LED_BLUE, !digitalRead(LED_BLUE));
  //int i = 0; 
  if (Wire.available()) {    
    opCode = Wire.read(); 
    sizeofData = Wire.read();
   // uint8_t data[sizeofData]; 
    for(int i=0; i<sizeofData; i++) { 
      masterReceiveBuffer[i] = Wire.read();
    }
  }//end if
  
  switch (opCode) { 
    case 0: 
      break; 
    case 1: 
      digitalWrite(RELAY_CONTROL, LOW); 
      break;  
    case 2: 
      digitalWrite(RELAY_CONTROL, HIGH); 
      break; 
    case 3: 
      temp=0;
      temp = masterReceiveBuffer[0]*256; 
      temp = temp + masterReceiveBuffer[1];
      receiveCommand = (temp*9)+900;
      break;
    case 4: 
      digitalWrite(LED_BLUE, HIGH);
      break; 
    case 5: 
      digitalWrite(LED_BLUE, LOW);
      break;
    case 6: 
      for (int i=0; i<NUMBER_NEOPIXEL; i++) {
        pixels.setPixelColor(i, pixels.Color(masterReceiveBuffer[3*i],masterReceiveBuffer[3*i+1],masterReceiveBuffer[3*i+2])); // Moderately bright green color.
      }
      pixels.show(); 
      break;
    case 7: 
      pixels.setPixelColor(masterReceiveBuffer[0], pixels.Color(masterReceiveBuffer[1],0,0)); // Moderately bright green color.
      pixels.show(); 
      break;
    case 8: 
      pixels.setPixelColor(6, pixels.Color(0,0,20)); // Moderately bright green color.
      pixels.show(); 
      
    default:
    break;
  } 
}//end receiveEvent

void readSensorValues() { 
        int8_t count  = 0;
   
        I2Cdev::i2c_start(0x4A); //default address is 0x25 or b100101 
  	I2Cdev::i2c_write(0x80);  
  	I2Cdev::i2c_stop();
       // delayMicroseconds(6);
  	I2Cdev::i2c_start(0x4B);
  	//delayMicroseconds(6);
  	for (; count < length;) { 
    		data[count] = I2Cdev::i2c_read(false);
    		count++;
    	}
    	I2Cdev::i2c_read(true);
    	I2Cdev::i2c_stop();    
}//end readSensorValues

void setBaseline() { 
  I2Cdev::i2c_start(0x4A); //default address is 0x25 or b100101 
  I2Cdev::i2c_write(0x04);  
  I2Cdev::i2c_write(0x01);  
  I2Cdev::i2c_stop();
}


