//All Axis controller v0.1

//Parameters to set the run speeds
const short utRunSpeed = 1000;
const short utRunAccel = 1000;
const short ltRunSpeed = 1000;
const short ltRunAccel = 4000;
const short caRunSpeed = 50;
const short caRunAccel = 50;
//parameters to set the jog speeds
const short utJogSpeed = 1000;
const short utJogAccel = 1000;
const short ltJogSpeed = 1000;
const short ltJogAccel = 4000;
const short caJogSpeed = 50;
const short caJogAccel = 50;

//Camera arm homing direction 0 = up, 1 = down
const short caHomeDir = 0;

float caPosDegs[7] = {10, //Angular distance between position 1 & 2
                      50, //Angular distance between position 2 & 3
                      10, //Angular distance between position 3 & 4
                      30, //Angular distance between position 4 & 5
                      10, //Angular distance between position 5 & 6
                      50, //Angular distance between position 6 & 7
                      10};//Angular distance between position 7 & 8


#include <AccelStepper.h>
#include <Arduino.h>

#define utstep 2
#define utdir 3
#define ltstep 14
#define ltdir 15
#define castep 16
#define cadir 17
#define camfoc 66
#define camtrig 64
#define LED 7
#define jogsel 56
#define utjsled 6
#define ltjsled 5
#define cajsled 4
#define btnback 18 //back/stop
#define btnfor 19
#define btnstart 20 //start/pause
#define calim 60

// /define the accelstepper object here
AccelStepper utstepper(1,utstep, utdir);
AccelStepper ltstepper(1,ltstep, ltdir);
AccelStepper castepper(1,castep, cadir);

//the delay between each segment in which the photo will be taken
const uint16_t segmentInterval = 1500;

//The variable to store the timer value at the start of the segment timer
unsigned long segmentTimer = 0;

//the steps per segment. We're using 1600 steps per rev motor on a 5:1 gear ratio, so 8000 steps per rev.
//For 36 segments 8000/36 = 222.2222222
const float utstepsPerSegment = 222.2222;
const float castepsPerSegment = 1000;
const float ltstepsPerSegment = 222.2222;

// segment counter
uint8_t utsegment = 0;
uint8_t casegment = 0;
uint8_t ltsegment = 0;

//total number of segments to complete
const uint8_t utnumSegments = 36;
const uint8_t canumSegments = 18;
const uint8_t ltnumSegments = 36;

//Arrival flag
bool utjustArrived = true;
bool ltjustArrived = true;

//flag to continue turning
bool utSegmentsLeft = true;
bool ltSegmentsLeft = true;

//flags to check the button situation
bool jogselDown = false;
bool jogselFirst = true;
bool btnbackDown = false;
bool btnbackFirst = true;
bool btnforDown = false;
bool backJogRapid = false;
bool btnforFirst = true;
bool forJogRapid = false;
bool btnstartDown = false;
bool btnstartFirst = true;
bool upperSequenceRunning = false;
bool lowerSequenceRunning = false;
bool upperSequenceFinished = false;
bool lowerSequenceFinished = false;

bool pictureToTake = false;
bool picFirst[2] = {true, true};

//timers for button debounce and hold
unsigned long jogselDTimer = 0;
unsigned long btnbackDTimer = 0;
unsigned long btnforDTimer = 0;
unsigned long btnstartDTimer = 0;
unsigned long btnbackDownTimer = 0;
unsigned long btnforDownTimer = 0;

unsigned long pictureTimer = 0;

int8_t curJogAxis = 0;

uint8_t curState = 2;

void startupFlash(void);
void homeSteppers(void);
void checkButtons(void);
void runState(void);
void changeJogAxis(void);
void takePicture(void);


void setup() {
  pinMode(utstep, OUTPUT);
  pinMode(utdir, OUTPUT);
  pinMode(ltstep, OUTPUT);
  pinMode(ltdir, OUTPUT);
  pinMode(castep, OUTPUT);
  pinMode(cadir, OUTPUT);
  pinMode(camtrig, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(jogsel, INPUT_PULLUP);
  pinMode(utjsled, OUTPUT);
  pinMode(ltjsled, OUTPUT);
  pinMode(cajsled, OUTPUT);
  pinMode(btnback, INPUT_PULLUP);
  pinMode(btnfor, INPUT_PULLUP);
  pinMode(btnstart, INPUT_PULLUP);
  // pinMode(utlim, INPUT_PULLUP);
  // pinMode(ltlim, INPUT_PULLUP);
  pinMode(calim, INPUT_PULLUP);

  Serial.begin(115200);
  
  utstepper.setMaxSpeed(1000);
  utstepper.setAcceleration(1000);
  ltstepper.setMaxSpeed(1000);
  ltstepper.setAcceleration(4000);
  castepper.setMaxSpeed(1000);
  castepper.setAcceleration(4000);

  
  
  startupFlash();
  homeSteppers();
  
  curJogAxis--;
  changeJogAxis();

}

void loop() {
  utstepper.run();
  ltstepper.run();
  castepper.run();
  checkButtons();
  runState();
  takePicture();
}

void startupFlash(){
  int flashTimer = 50;
  for( int i = 0; i < 10; i++ ){
    digitalWrite(LED,!digitalRead(LED));
    delay(flashTimer);
  }
}

void homeSteppers(){
  uint8_t homeStep = 1;
  // while(utlim){
  //   utstepper.moveTo( utstepper.currentPosition() + homeStep );
  // }
  // utstepper.setCurrentPosition(0);
  // while(ltlim){
  //   ltstepper.moveTo( ltstepper.currentPosition() + homeStep );
  // }
  // ltstepper.setCurrentPosition(0);
  Serial.println("Camera arm homing started");
  while(digitalRead(calim)){
    castepper.moveTo( castepper.currentPosition() + homeStep );
    // castepper.move(homeStep);
    castepper.run();
  }
  Serial.println("Camera arm homed");
  castepper.setCurrentPosition(0);
  // curState = 1;
}

void checkButtons(){
  uint8_t debounce = 50;
  uint16_t jogRapid = 1000;
  if( millis() - jogselDTimer > debounce ){
    if( !digitalRead(jogsel) && !jogselDown ){
      jogselDown = true;
      jogselDTimer = millis();
    }else if( digitalRead(jogsel) && jogselDown ){
      jogselDown = false;
      jogselDTimer = millis();
      jogselFirst = true;
    }
  }
  if( millis() - btnbackDTimer > debounce ){
    if( !digitalRead(btnback) && !btnbackDown ){
      btnbackDown = true;
      btnbackDTimer = millis();
      btnbackDownTimer = millis();
    }else if( digitalRead(btnback) && btnbackDown ){
      btnbackDown = false;
      btnbackDTimer = millis();
      btnbackFirst = true;
      backJogRapid = false;
    }
  }
  if( millis() - btnbackDownTimer > jogRapid && btnbackDown){//} && btnbackFirst ){
    backJogRapid = true;
  }
  if( millis() - btnforDTimer > debounce ){
    if( !digitalRead(btnfor) && !btnforDown ){
      btnforDown = true;
      btnforDTimer = millis();
      btnforDownTimer = millis();
    }else if( digitalRead(btnfor) && btnforDown ){
      btnforDown = false;
      btnforDTimer = millis();
      btnforFirst = true;
      forJogRapid = false;
    }
  }
  if( millis() - btnforDownTimer > jogRapid && btnforDown ){
    forJogRapid = true;
  }
  if( millis() - btnstartDTimer > debounce ){
    if( !digitalRead(btnstart) && !btnstartDown ){
      btnstartDown = true;
      btnstartDTimer = millis();
      // btnforDownTimer = millis();
    }else if( digitalRead(btnstart) && btnstartDown ){
      btnstartDown = false;
      btnstartDTimer = millis();
      btnstartFirst = true;
      // forJogRapid = false;
    }
  }
}

void changeJogAxis(){
    curJogAxis == 2 ? curJogAxis = 0 : curJogAxis++;
    String jogAxes[3] = {"Upper turntable",
                        "Lower turntable",
                        "Camera arm"};
    Serial.println(jogAxes[curJogAxis]);
    int jsled[3] = {utjsled, ltjsled, cajsled};
    for(int i = 0; i < 3; i++){
      i == curJogAxis ? digitalWrite(jsled[i], HIGH) : digitalWrite(jsled[i], LOW);
    }
}

void runState(){
  switch(curState){
    case 0: //homing
      homeSteppers();
    break;
    case 1:

    break;
    case 2: //jog state
      if( jogselFirst && jogselDown ){
        jogselFirst = false;
        changeJogAxis();
      }
      if( btnbackFirst && btnbackDown ){
      Serial.println("Jog back");
        switch( curJogAxis ){
          case 0:
            utstepper.move(1);
          break;
          case 1:
            ltstepper.move(1);
          break;
          case 2:
            castepper.move(1);
          break;
        }
        btnbackFirst = false;
      }else if( btnforFirst && btnforDown ){
      Serial.println("Jog forward");
        switch( curJogAxis ){
          case 0:
            utstepper.move(-1);
          break;
          case 1:
            ltstepper.move(-1);
          break;
          case 2:
            castepper.move(-1);
          break;
        }
        btnforFirst = false;
      }
      if( backJogRapid ){
        switch( curJogAxis ){
          case 0:
            if( utstepper.distanceToGo() == 0 ) {
              utstepper.move(1);
              Serial.println("upper turntable back rapid mode");
            }
          break;
          case 1:
            if( ltstepper.distanceToGo() == 0 ) {
              ltstepper.move(1);
              Serial.println("lower turntable back rapid mode");
            }
          break;
          case 2:
            if( castepper.distanceToGo() == 0 ) {
              castepper.move(1);
              Serial.println("camera arm back rapid mode");
            }
          break;
        }
      }else if( forJogRapid ){
        switch( curJogAxis ){
          case 0:
            if( utstepper.distanceToGo() == 0 ) {
              utstepper.move(-1);
              Serial.println("upper turntable forward rapid mode");
            }
          break;
          case 1:
            if( ltstepper.distanceToGo() == 0 ) {
              ltstepper.move(-1);
              Serial.println("lower turntable forward rapid mode");
            }
          break;
          case 2:
            if( castepper.distanceToGo() == 0 ) {
              castepper.move(-1);
              Serial.println("camera arm forward rapid mode");
            }
          break;
        }
      }
      if( btnstartFirst && btnstartDown ){ //starts the sequence coming from jog mode
        btnstartFirst = false;
        curState = 3;
        upperSequenceRunning = true;
        Serial.println("Start upper turntable sequence");
        casegment = 0;
        castepper.moveTo( int( float( casegment ) * castepsPerSegment ) );
        //turn off jog leds
        int jsled[3] = {utjsled, ltjsled, cajsled};
        for(int i = 0; i < 3; i++){
          digitalWrite(jsled[i], LOW);
        }
      }
    break;
    case 3: //run upper turntable sequence
      if( btnstartFirst && btnstartDown && !upperSequenceFinished ){ //pause or continue the sequence whilst in run mode
        btnstartFirst = false;
        upperSequenceRunning = !upperSequenceRunning;
        upperSequenceRunning ? Serial.println("resumed") : Serial.println("paused");
      }
      if( btnbackFirst && btnbackDown ){ //stop has been pressed while running sequence
        btnbackFirst = false;
        upperSequenceRunning = false;
        curState = 2;
        Serial.println("Sequence cancelled");
        //decrement jog axis before increment function to switch existing led back on
        curJogAxis--;
        changeJogAxis();
        // reset the segment counters
        utsegment = 0;
        utSegmentsLeft = true;
        ltSegmentsLeft = true;
      }
      //when upper turntable has finished and start button is pressed, move to lower sequence
      if( btnstartFirst && btnstartDown && upperSequenceFinished ){ 
        btnstartFirst = false;
        curState = 4;
        Serial.println("Commencing lower turntable");
      }
      if( upperSequenceRunning ){
            //update our time variable
        unsigned long currentTimer = millis();

        //Most of this code runs when we have reached the segment position and are waiting to move ot the next position.
        //The accelstepper library handles all the movemnets.
        //Once the final segment is reached, the flag gets tripped.
        if (utstepper.distanceToGo() == 0 && utSegmentsLeft ){
              //justArrive is flag on arrival for every segment
              if( utjustArrived && castepper.distanceToGo() == 0 ){
                  //If we reached the final position already, turn the flag off.
                  if ( utsegment == utnumSegments ){
                        utSegmentsLeft = false;
                        Serial.println("upper turntable cycle finished");
                        // reset the current position to zero
                        utstepper.setCurrentPosition( 0 );
                        if( casegment < ( canumSegments / 2 ) ){
                          casegment++;
                          castepper.moveTo( int( float( casegment ) * castepsPerSegment ) );
                          Serial.print( "Moving camera arm to: ");
                          Serial.println(casegment);
                          utsegment = 0;
                          utSegmentsLeft = true;
                        }else{
                          Serial.println("upper turntable use finished, move to lower turntable");
                          upperSequenceFinished = true;
                        }
                  }else{
                        Serial.print("upper turntable segment: ");
                        Serial.println(utsegment);
                        //increment the segment counter once we have reached the position
                        utsegment++;
                        //reset the timer value to be current
                        segmentTimer = millis();
                        //turn the flag off so we only increment once.
                        utjustArrived = false;
                        //raise the flag to take the picture and start the timer
                        pictureToTake = true;
                        pictureTimer = millis();
                  }
              }

              //once we are in position, this conditional is checked until the timer exceeds the interval
              if( millis() - segmentTimer > segmentInterval && utSegmentsLeft && castepper.distanceToGo() == 0 ){
                  // The step position to move to. It is converted to float to maintain highest precision
                  // This will make the distanceToGo call not return zero
                  utstepper.moveTo( int( float( utsegment ) * utstepsPerSegment ) );
                  //Switch back on the flag for when we arrive
                  utjustArrived = true;
                  Serial.print("Moving to: ");
                  Serial.println(utsegment);
              }
        }

      }
    break;
    case 4: //run lower turntable
      if( btnstartFirst && btnstartDown && !lowerSequenceFinished ){ //pause or continue the sequence whilst in run mode
        btnstartFirst = false;
        lowerSequenceRunning = !lowerSequenceRunning;
        lowerSequenceRunning ? Serial.println("lower resumed") : Serial.println("lower paused");
      }
      if( btnbackFirst && btnbackDown ){ //stop has been pressed while running sequence
        btnbackFirst = false;
        lowerSequenceRunning = false;
        curState = 2;
        Serial.println("Sequence cancelled");
        //decrement jog axis before increment function to switch existing led back on
        curJogAxis--;
        changeJogAxis();
        // reset the segment counters
        utsegment = 0;
        ltsegment = 0;
        utSegmentsLeft = true;
        ltSegmentsLeft = true;
      }
      if( lowerSequenceRunning ){
        if (ltstepper.distanceToGo() == 0 && ltSegmentsLeft ){
          //justArrive is flag on arrival for every segment
          if( ltjustArrived && castepper.distanceToGo() == 0 ){
            //If we reached the final position already, turn the flag off.
            if ( ltsegment == ltnumSegments ){
              ltSegmentsLeft = false;
              Serial.println("lower turntable cycle finished");
              // reset the current position to zero
              ltstepper.setCurrentPosition( 0 );
              if( casegment < canumSegments ){
                casegment++;
                castepper.moveTo( int( float( casegment ) * castepsPerSegment ) );
                Serial.print( "Moving camera arm to: ");
                Serial.println(casegment);
                ltsegment = 0;
                ltSegmentsLeft = true;
              }else{
                Serial.println("upper and lower turntable use finished");
                lowerSequenceFinished = true;
              }
            }else{
              Serial.print("lower turntable segment: ");
              Serial.println(ltsegment);
              //increment the segment counter once we have reached the position
              ltsegment++;
              //reset the timer value to be current
              segmentTimer = millis();
              //turn the flag off so we only increment once.
              ltjustArrived = false;
              //raise the flag to take the picture and start the timer
              pictureToTake = true;
              pictureTimer = millis();
            }
          }

          //once we are in position, this conditional is checked until the timer exceeds the interval
          if( millis() - segmentTimer > segmentInterval && ltSegmentsLeft && castepper.distanceToGo() == 0 ){
            // The step position to move to. It is converted to float to maintain highest precision
            // This will make the distanceToGo call not return zero
            ltstepper.moveTo( int( float( ltsegment ) * ltstepsPerSegment ) );
            //Switch back on the flag for when we arrive
            ltjustArrived = true;
            Serial.print("Moving to: ");
            Serial.println(ltsegment);
          }
        }
      }
      
    break;
  }
}

void takePicture(){
  
  uint8_t focusOnTime = 100;
  uint8_t triggerOnTime = 100;
  
  if( pictureToTake ){
    
    if( millis() - pictureTimer < focusOnTime && picFirst[0] ){
      picFirst[0] = false;
      digitalWrite( camfoc, LOW );
      // Serial.println("focus on");
    }else if( millis() - pictureTimer > focusOnTime && millis() - pictureTimer < ( focusOnTime + triggerOnTime ) && picFirst[1] ){
      picFirst[1] = false;
      digitalWrite( camtrig, LOW );
      // Serial.println("trigger on");
    }else if( millis() - pictureTimer > ( focusOnTime + triggerOnTime ) ){
      digitalWrite( camfoc, HIGH );
      digitalWrite( camtrig, HIGH );
      // Serial.println("focus off, trigger off");
      pictureToTake = false;
      picFirst[0]=true;picFirst[1]=true;
    }
  }
}
