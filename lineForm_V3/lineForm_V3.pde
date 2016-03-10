/*
LineForm Modular Nodes code V1
For use with rigid flex pcb 

Created by Artem Dementyev
Modified on September 14, 2015
*/

import processing.serial.*;

Serial myPort;        // The serial port 
boolean start = true; 
int deviceID = 0; 
int deviceCounter = 1;
sensorNode[] tempArraySensors = new sensorNode[128];
ArrayList <sensorNode>listSensors = new ArrayList<sensorNode>(); 
visualize drawStuff = new visualize(this);
boolean orientationFlag = true; 
int visualizeSensorType = 1;
int posToSend = 1000;
 int controlServoID = 2;
int someInt = 900; 
int numberOfNodes = 16;
int currentServo = 1;

final int SEND_BUFFER_SIZE_BYTES = 500;
final int SEND_PACKET_SIZE_BYTES = 27;
byte [] sendBuffer; 

boolean sendingDone = false;

int servoSpeed = 10;
void setup()
{
   size(1850, 1000, P3D); // animation is much smoother in P2D; text looks better with JAVA2D
   background(255);
   
   //Find serial port
    for (int i=0; i<Serial.list().length; i++) { 
      System.out.println(i + " : " + Serial.list()[i]); 
    }//end for
   // if (
    myPort = new Serial(this, Serial.list()[13], 115200);
    
    //Use this code to declare the nodes at the start. Avoid needing a restart every time rerunning the program, a lot faster for development

    int [] cuts = {0,0,0,0};  
    for (int i=1; i<numberOfNodes+1; i++) { 
      sensorNode tempSensorNode = new sensorNode(i, 3, 1, cuts );   
      listSensors.add(tempSensorNode);  
    }//end for
    
    //[opcode][motor][motor][R1][G1][B1]....[R8][G8][B8]
    sendBuffer = new byte[SEND_BUFFER_SIZE_BYTES];
    for(int i=0; i<SEND_BUFFER_SIZE_BYTES; i++) { 
      sendBuffer[i] = 0x00;
    }
    
    frameRate(60);
    //turnServoOn(1); 
    turnOnBlueLED(4);
   turnOnBlueLED(5);
   
   turnOnBlueLED(3);
   turnOnBlueLED(2);
   turnOnBlueLED(1);
   //testCommand2(5);
   turnServoOn(3); 
    
}//end setup

void draw() { 
      smooth();
      background(255);
       
      if(!start) {  
          drawStuff.drawSensors(listSensors,visualizeSensorType);
        
      }//end if
   
      if (!start && orientationFlag) {
        determineOrientation(); 
        orientationFlag = false; 
      }//end if
      
      
      
      /*
      // send every 30 frames: frameCount % 30 == 0
      if (frameCount % 30 == 0) {
        if(sendingDone == true) { 
          thread("sendData");
          sendingDone = false;
        }
      } 
      */
        
        
        
        //SERVO TEST PROGRAM
        if (someInt % 1 == 0){
        sendServoPosConverted(currentServo,someInt);//someInt);
       // turnOnBlueLED(currentServo);
        //sendServoPosConverted(currentServo+1,someInt);//someInt);
       // sendServoPosConverted(currentServo+2,someInt);//someInt);
        }
        someInt = someInt+servoSpeed;
        if (someInt >1700) { 
          someInt =1300; 
          currentServo++;     
          turnServoOn(currentServo); 
          if (currentServo==6) currentServo=0;
        }
        
        
     
      
}//end draw

 public void serialEvent (Serial myPort) {
     try {     //Use try, catch to avoid crashing
       // get the ASCII string:
       String inString = myPort.readStringUntil('\n');   
      
       //Check if the string is good
       //I use the 'S' character at the beggining
       //Otherwise, I get a null point, and computer may crash (MAC 10.6)
       if (inString != null && inString.charAt(0) == 'A' && start) {
         System.out.println(inString);
         inString = trim(inString);
         String[] list = split(inString, ',');
         if(list.length == 8 ) {
           deviceID = Integer.parseInt(list[1]);
           if (Integer.parseInt(list[1]) == deviceCounter) {
             System.out.println("Found: " + Integer.toString(deviceCounter));
             
            // int [] cuts = {Integer.parseInt(list[4]),Integer.parseInt(list[5]),Integer.parseInt(list[6]),Integer.parseInt(list[7])};                    
      
             //TODO NOW Declaring a fake sensor node, has to fix the restart problem 
             int [] _cuts = {0,0,0,0};  
             sensorNode tempSensorNode = new sensorNode(Integer.parseInt(list[1]), 
                 3, 1, _cuts );        
             listSensors.add(tempSensorNode);      
             }         
         }
         deviceCounter++;
       }//end if start
       
       //This is the normal data 
       else if (inString != null && inString.charAt(0) == 'S' && listSensors.size()>0) {
         start = false;
         // trim off any whitespace:
         System.out.println(inString);
         inString = trim(inString);
         String[] list = split(inString, ',');
                 
         if (list.length==9) { 
           sensorNode tempSensorNode = listSensors.get(Integer.parseInt(list[1])-1);
           int tempInput[]; 
           tempInput = new int [list.length-1];
           for (int i =1; i<list.length; i++) { 
             tempInput[i-1] = Integer.parseInt(list[i]); 
           }//end if        
     
           tempSensorNode.addData(tempInput);  
           listSensors.remove(Integer.parseInt(list[1])-1);
           listSensors.add(Integer.parseInt(list[1])-1,tempSensorNode );           
         }//end if list length is correct
       }//end if String is correct     
      } catch (Exception e){ 
        System.out.println("Error: " + e);
      }  
}//end SerialEvent

//Press keys to change visualization 
public void keyPressed() {
  
  
    //visualize light sensors
    if (key == '1') {
      servoSpeed=servoSpeed-5;
    }
    //visualize proximity sensors
    else if (key == '2')
     servoSpeed=servoSpeed+5;
    //visualize orientation sensors
    
  else if (key =='3') { 
    sendServoPosConverted(3,posToSend);
    posToSend = posToSend -10;
    }
  else if (key =='4') { 
    sendServoPosConverted(3,posToSend);
    posToSend = posToSend +10;
    
    }
  else if (key =='5') { 
    //turnServoOn(controlServoID);
    sendServoPosConverted(3,1000);
  }
  else if (key =='6') { 
    //turnServoOff(controlServoID);
    //sendServoPosConverted(3,1500);
    testCommand4(5) ;
  }
    else if (key =='7') { 
  //  sendData();
   // testCommand2(5);
    testCommand2(5);
  }
  
   else if (key =='9') { 
      turnOnBlueLED(3);
   }
   else if (key =='0') { 
      turnOffBlueLED(3);

  }
}//end keyPressed

public void sendCommandsToNode( int node,byte command1, byte command2, byte command3) {
    //nice page explaining: 
    //http://blog.danielkerris.com/?p=349     
    myPort.write((char)(node/256));
    myPort.write(node & 0xff);
    myPort.write(command1);
    myPort.write(command2);
    myPort.write(command3); 
    //char gg = 130; 
}//end changeLEDCOlor

public void sendServoPos( int node,int pos) {
    //nice page explaining: 
    //http://blog.danielkerris.com/?p=349     
    myPort.write((char)(node/256));
    myPort.write(node & 0xff);
    myPort.write((char)(pos/256));
    myPort.write(pos & 0xff);
    myPort.write((byte)4);
    
    
}//end changeLEDCOlor

public void sendServoPosConverted( int node,int pos) {
    //nice page explaining: 
    //http://blog.danielkerris.com/?p=349     
    
    if (pos>900 && pos<2100) { 
      float tempPos = (pos-900)/9; 
      int convertedPos = round(tempPos);
      
      byte [] data = {(byte)(convertedPos/256),(byte)(convertedPos & 0xff)};  
      sendData(node, (byte)3, (byte)2, data );
      
      /*
      myPort.write((char)(node/256));
      myPort.write(node & 0xff);
      
      myPort.write((char)(convertedPos/256));
      myPort.write(convertedPos & 0xff);
      myPort.write((byte)4);
      */
      
    }
    
    
}//end changeLEDCOlor


public void turnServoOn(int node) {
    byte [] data = {1}; 
    sendData(node,(byte)2,(byte)1,data); 
   /*     
    myPort.write((char)(node/256));
    myPort.write(node & 0xff);
    myPort.write(0x00);
    myPort.write(0x00);
    myPort.write((byte)50); //turn on command
    */
    
}//end changeLEDCOlor

public void turnServoOff(int node) {
    byte [] data = {1}; 
    sendData(node,(byte)1,(byte)1,data);
    /*
    myPort.write((char)(node/256));
    myPort.write(node & 0xff);
    myPort.write(0x00);
    myPort.write(0x00);
    myPort.write((byte)100); //turn off command
    */
}//end changeLEDCOlor
   
 public void determineOrientation() {  
   //Initial x and y positions
    int xpos = 100; 
    int ypos = 300;
    //Assign positions to each node
    for (int i=0; i<listSensors.size(); i++ ) {                      
       xpos = xpos + 70;
       sensorNode tempSensorNode = listSensors.get(i);
       tempSensorNode.setPosition(xpos,ypos);  
       listSensors.remove(i);
       listSensors.add(i,tempSensorNode);    
    }//end for  
  }//end determineOrientation
  
void sendData() {
      for(int i=0; i<SEND_BUFFER_SIZE_BYTES  ; i++) { 
        //myPort.write(sendBuffer[i]);
        myPort.write(i);
      }
      sendingDone = true;
}

void turnOnBlueLED(int node) { 
    byte [] data = {1}; 
    sendData(node,(byte)4,(byte)1,data);
}

void turnOffBlueLED(int node) { 
    byte [] data = {1}; 
    sendData(node,(byte)5,(byte)1,data);
}

void testCommand1(int node) { 
  byte [] data = {2,30}; 
  sendData(node, byte(7), byte(2),data );
  
}

void testCommand3(int node) { 
  byte [] data = {2,2}; 
  sendData(node, byte(8), byte(2),data );
  
}

void testCommand2(int node) { 
  byte [] data = {20,0,0, 
                  20,0,0,
                  20,0,0,
                  20,0,0,
                  0,20,0,
                  0,20,0,
                  0,20,0,
                  0,20,0};
  sendData(node, byte (6), byte (24),data );
}

void testCommand4(int node) { 
  byte [] data = {0,20,0, 
                  0,20,0,
                  0,20,0,
                  0,20,0,
                  0,0,20,
                  0,0,20,
                  0,0,20,
                  0,0,20};
  sendData(node, byte (6), byte (24),data );
}

void sendData(int node, byte opCode, byte sizeofData, byte []data ) { 
    //nice page explaining: 
    //http://blog.danielkerris.com/?p=349   
    
    //Send address
    myPort.write((char)(node/256));
    myPort.write(node & 0xff);
    //Send opcode
    myPort.write(opCode); 
    //send sizeofData 
    myPort.write(sizeofData); 
    //send data
    for(int i=0; i<sizeofData; i++) { 
      myPort.write(data[i]);
    }   
}

void sendDataLEDS(byte[] data) { 
}