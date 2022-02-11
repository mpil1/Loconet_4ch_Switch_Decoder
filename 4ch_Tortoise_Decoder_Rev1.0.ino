/***********************************************
*4Ch Tortoise Motor Loconet Decoder
*Rev 1.0, 
*Feb, 10, 2022
*Michael Pilyih
*(c) 2022
*************************************************
*************************************************
*Loconet.h Header:
*Copyright (C) 2009 to 2013 Alex Shepherd
*Portions Copyright (C) Digitrax Inc.
*Portions Copyright (C) Uhlenbrock Elektronik Gmb
*************************************************

Description:
4 Output Tortoise (Stall) Motor Driver.  
-Motor Leads: Connect one lead to outAx, the other to an OutBx
-DCC Switch address for each output stored in EEPROM
  - if no address is stored in outputs (address=0), decoder will default that output's address to its output number 1-4
-Serial Debug mode thru Arduino USB port = 115200 baud

Normal Run Mode: 
 -Red Loconet Status LED on, Green Program Mode LED off.  Red LED blinks when traffic on Loconet
 -Input Buttons will locally toggle switch outputs, can operate without Loconet
 - Input Buttons also send Loconet Switch Request based on direction and address.  "Output" parameter is always 0
 -Listens for Loconet SwitchSensor message.  Will toggle outputs and echo that request back to the Loconet once

Program Mode:
  1) Press and hold Program Button for approx 5 Seconds.  When Red Loconet Status LED goes out and Green Program
     Mode LED lights, immediately let go of button.  Exit Program Mode at any time by pressing the Program button again. 
  2.)After approx 3 seconds the Green Program LED will flash slowly. Use a DCC throttle to send a THROWN command
     to the desired switch address for Output 1. When the command is recieved, the Loconet Status and Program Mode LED's
     will alternate very quickly indicating the decoder has saved the selected address to the EEPROM.
  3.)The Red Loconet Status LED will go out and the Green Program Mode LED will now begin blinking faster, indicating the 
     decoder is ready to accpept the address for output 2.   Once again, from DCC throttle, send a THROWN command to the
     address desired for Output 2.   
  4.)Repeat Step 2 for Output 3 and 4.  Each time the Green Program Mode LED will blink faster.
  5.)After the address for Output 4 is selected, the decoder will return to Normal Run Mode and function normally.  The 
     selected addresses are stored in EEPROM and will be held when powered off.
     
     
Various Notes: 

//Loconet Turnout Direction parameter Transmitted: 1= Closed, 0 = Thrown
//--Command station sends a value of "32" for close, 0 for thrown
//Outputs: When Closed outAx=1, outBx=0.    When Thrown outAx=0, outBx=1
*/


#include <LocoNet.h>
#include <EEPROM.h>

#define PROGButton  A4  //Connects input to GND when closed
#define ledLocStat 12   //Annode to ouput pin, cathode to GND
#define ledProg 13      //Annode to ouput pin, cathode to GND


const int BUTTON[] = {A0, A1, A2, A3}; //declare and initialize Button #'s, buttons connect inputs to GND when closed
const int outA[]={2, 4, 7, 10};        //declare and initialize OUTA's (flat side of LED), one lead of motors
const int outB[]={3, 5, 9, 11};        //declare and initialize OUTB's (roud side of LED), other lead of motors

float rev = 1.0;

//Turnout Addresses
int address[4];
int addressArrayLen = (sizeof(address)/sizeof(address[0]));
//int address1 = 17;
//int address2 = 18;
//int address3 = 19;
//int address4 = 20;
//int addr1, addr2, addr3, addr4;
bool stillPressed[]= {0, 0, 0, 0};
int turnoutDirection[]={1, 1, 1, 1};
int currentDirection[]={1, 1, 1, 1};
int buttonArrayLen = (sizeof(BUTTON)/sizeof(BUTTON[0]));
int outALen = (sizeof(outA)/sizeof(outA[0]));
int outBLen = (sizeof(outB)/sizeof(outB[0]));

int progMode = 0;         //0= Run Mode, 1=Address Program Mode

unsigned long buttonPressMillis = 0;
unsigned long progDelay = 5000;
unsigned long flashRate[4]={750, 300, 100, 50};
unsigned long flashMillis;
bool flashState = 1;
int x;    //generic counter
int y;    //use to count eeprom address
int z;    //counter to quick flash led's in prog mode
bool z1;   //led toggle state for quick flash in prog mode
int dccPacketNum; //use to count and ignore second packet sent by cmd station
//****Digitrax Command Station sends two switch packets at once

lnMsg *LnPacket;      //need this before setup() to be used by Loconet.h

void setup() {
  //Initialize Debug Serial
  Serial.begin(115200);
  Serial.println("Serial Debug Started");
  Serial.println("4 Challen Tortoise Motor Decoder");
  Serial.print("Firmware Rev ");
  Serial.println(rev);
  Serial.println("(c) 2022 Mike Pilyih");
  //Serial.print("Number of Turnouts= ");
  //Serial.println(buttonArrayLen);
  
  
  
  //INPUT Pin Setup
  pinMode(PROGButton, INPUT_PULLUP);  
  for(int i = 0; i < buttonArrayLen; i++){
    pinMode(BUTTON[i], INPUT_PULLUP);
  }

  //OUTPUT Pin Setup
  pinMode(ledLocStat, OUTPUT);
  pinMode(ledProg, OUTPUT);
  
  for(int i = 0; i < outALen; i++){
    pinMode(outA[i], OUTPUT);
  }
  for(int i = 0; i < outBLen; i++){
    pinMode(outB[i], OUTPUT);
  }
  
  
  //initialize Outputs to Closed / Green
  for(int i = 0; i < outALen; i++){
    digitalWrite(outA[i], 1);
  }
  for(int i = 0; i < outBLen; i++){
    digitalWrite(outB[i], 0);
  }
  Serial.println("Outputs Initialized to Closed");

  //Read Address Value from EEPROM, if value is zero default address to output number 1-4
  y = 0;
  for(int count = 0; count < addressArrayLen; count++){
    EEPROM.get(y, address[count]);
    if (address[count]== 0) address[count] = count + 1;
    Serial.print("Turnout 1 Address= ");
    Serial.println(address[count]);
    y += 2;
  }

  //initialize Loconet, default TX pin = 6, RX pin = 8
  LocoNet.init();
  Serial.println("Loconet.h Initialized");
  digitalWrite(ledLocStat, 1);

  Serial.println("Decoder Ready!");
  
}

void loop() {
 
      
   addressProgramMode();
   buttonRead();
   updateTurnout(); 
 
  
 //Check for and process Loconet incoming message
   LnPacket = LocoNet.receive();
   if(LnPacket) {
    digitalWrite(ledLocStat, 0);
    delay(30);
    digitalWrite(ledLocStat, 1);
    LocoNet.processSwitchSensorMessage(LnPacket);
   }
   
}


//******Loconet Switch Request Handler*************

void notifySwitchRequest( uint16_t Address, uint8_t Output, uint8_t Direction ) {
  if(progMode == 0){             //If we're in Normal Run Mode
    
     for (int turnoutNum = 0; turnoutNum < addressArrayLen; turnoutNum ++){
        if(Address == address[turnoutNum]){
           if(Direction == 0){
              turnoutDirection[turnoutNum] = 0;   
           }
           else turnoutDirection[turnoutNum] = 1;
        }
     }
  } 
  
   if(progMode == 1){           //If we're in Program Mode
      if(Output != 0){
        return;
      }
      address[x] = Address;
      EEPROM.put(y, address[x]);
      Serial.print("Turnout ");
      Serial.print(x + 1);
      Serial.print(" address saved as: ");
      Serial.println(address[x]);
      ++x;
      y += 2;
      z1 = 0;
      for (z = 0; z <= 10; z++){
         digitalWrite(ledProg, z1);
         digitalWrite(ledLocStat, !z1);
         z1 = !z1;
         delay(100);
      }
   }   
}



//********* Other Functions*********************
void buttonRead(){
  for (int turnoutNum = 0; turnoutNum < addressArrayLen; turnoutNum ++){
    if(!digitalRead(BUTTON[turnoutNum]) == 1 && !stillPressed[turnoutNum]){   //Turnout Button is pressed
      if(turnoutDirection[turnoutNum] == 0) turnoutDirection[turnoutNum] = 1; //toggle turnout direction
      else turnoutDirection[turnoutNum] = 0;
      stillPressed[turnoutNum] = 1;
    }
    else if(!digitalRead(BUTTON[turnoutNum]) == 0) stillPressed[turnoutNum] = 0;
  }
}




void updateTurnout(){
   for (int turnoutNum = 0; turnoutNum < addressArrayLen; turnoutNum ++){
      if(turnoutDirection[turnoutNum] != currentDirection[turnoutNum]){
         if(turnoutDirection[turnoutNum] == 0){          //Turnout turnoutNum Thrown
            LocoNet.requestSwitch(address[turnoutNum], 0, turnoutDirection[turnoutNum]);
            Serial.print("Turnout ");
            Serial.print(address[turnoutNum]);
            Serial.println(" Thrown"); 
            currentDirection[turnoutNum] = 0;
            digitalWrite(outA[turnoutNum], 0);
            digitalWrite(outB[turnoutNum], 1);   
         }
         else{                                           //Turnout turnoutNum Closed
            LocoNet.requestSwitch(address[turnoutNum], 0, turnoutDirection[turnoutNum]);
            Serial.print("Turnout ");
            Serial.print(address[turnoutNum]);
            Serial.println(" Closed"); 
            currentDirection[turnoutNum] = 1;
            digitalWrite(outA[turnoutNum], 1);
            digitalWrite(outB[turnoutNum], 0);   
         }
      }  
   }
}

void addressProgramMode(){
  if(!digitalRead(PROGButton) == 1){
     buttonPressMillis = millis();
     while(!digitalRead(PROGButton) == 1){
       if ((millis()-buttonPressMillis) >= progDelay){
         Serial.println("Entering Address Program Mode");
         //Do Address Programming Routine within this IF statement
         digitalWrite(ledLocStat, 0);
         digitalWrite(ledProg, 1);
         delay(3000); 
         progMode = 1;
         x = 0;     //use this for both flash rate and address counter
         y = 0;
         while(!digitalRead(PROGButton) == 0){
            if (x > addressArrayLen -1){
              break;
            }
            //LED Flash Sequence- rate starts slow, will increment each time a switch request is received
            if((millis()-flashMillis) > flashRate[x]){
            flashState = !flashState; 
            digitalWrite(ledProg, flashState);   
            Serial.print("Waiting for Switch ");
            Serial.print(x + 1);
            Serial.println(" Address from LocoNet.");
            flashMillis = millis();
            }
            
            
            LnPacket = LocoNet.receive();
            if(LnPacket) {
               LocoNet.processSwitchSensorMessage(LnPacket);       //this line will call notifySwitchRequest()
            } 
         }   
         digitalWrite(ledProg, 0);
         digitalWrite(ledLocStat, 1);
         progMode = 0;
         Serial.println("Leaving Address Program Mode");
         delay(1000);
       }
     }
   }
}
