//All Axis controller v0.1

//Parameters to set the run speeds
const long utRunSpeed = 1000;
const long utRunAccel = 1000;
const long ltRunSpeed = 24000;
const long ltRunAccel = 800;
const long caRunSpeed = 5000;
const long caRunAccel = 1000;
//parameters to set the jog speeds
const long utJogSpeed = 100000;
const long utJogAccel = 100000;
const long ltJogSpeed = 1000;
const long ltJogAccel = 4000;
const long caJogSpeed = 100000;
const long caJogAccel = 500000;

//the delay between each segment in which the photo will be taken
const int segmentInterval = 1500;

long cam1Del = 500;//delay after camera fires
long cam2Del = 500;
long camTrigDel = 100; //delay for how long the trigger stays on

int utWobbleDelay = 100;
int ltWobbleDelay = 100;
int caWobbleDelay = 5000;

long wobbleTimer;

//Camera arm homing direction 1 = up, -1 = down
const short caHomeDir = -1;
//Camera arm homing step distance. (larger number will increase homing speed)
const short caHomeStep = 6;

//Number of positions the camera arm is to stop at
const short utNumCAPos = 4;
const short ltNumCAPos = 4;
//angular position of the 1st camera. This is based on camera 1 being at 0degs, camera 2 20 degs & camera 3 40 degs
float utcaPosDegs[utNumCAPos] = {0, //Angular distance of position 1 
                                10, //Angular distance of position 2
                                60, //Angular distance of position 3
                                70};//Angular distance of position 4
                            
float ltcaPosDegs[ltNumCAPos] = {70, //Angular distance of position 1 (5)
                                80, //Angular distance of position 2 (6)
                                130, //Angular distance of position 3 (7)
                                140};//Angular distance of position 4 (8)

//The variable to store the timer value at the start of the segment timer
unsigned long segmentTimer = 0;

//the steps per segment. We're using 1600 steps per rev motor on a 5:1 gear ratio, so 8000 steps per rev.
//For 36 segments 8000/36 = 222.2222222
const float utstepsPerSegment = 222.2222;
const float castepsPerSegment = 634.9206;
const float ltstepsPerSegment = 1777.7777;

//total number of segments to complete
const short utnumSegments = 36;
const short canumSegments = 18;
const short ltnumSegments = 36;

// segment counter
short utsegment = 0;
short casegment = 0;
short ltsegment = 0;

#include <AccelStepper.h>
#include <Arduino.h>

#define utstep 2
#define utdir 3
#define ltstep 14
#define ltdir 15
#define castep 16
#define cadir 17
#define camfoc 66
#define camtrig1 64
#define camtrig2 54
#define camtrig3 55
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

bool cam1Taken = false;//flags for firing the 3 cameras
bool cam2Taken = false;
bool cam3Taken = false;
bool cam2Off = true;
bool cam3Off = true;

bool utStartWobble = false;
bool ltStartWobble = false;
bool caStartWobble = false;

bool pictureToTake = false;
bool picFirst[2] = {true, true};

//timers for button debounce and hold
unsigned long jogselDTimer = 0;
unsigned long btnbackDTimer = 0;
unsigned long btnforDTimer = 0;
unsigned long btnstartDTimer = 0;
unsigned long btnbackDownTimer = 0;
unsigned long btnforDownTimer = 0;
long cam1Timer, cam2Timer, cam3Timer;

unsigned long pictureTimer = 0;

int8_t curJogAxis = 0;

short curState = 2;

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
  pinMode(camtrig1, OUTPUT);
  pinMode(camtrig2, OUTPUT);
  pinMode(camtrig3, OUTPUT);
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
  
  utstepper.setMaxSpeed(utRunSpeed);
  utstepper.setAcceleration(utRunAccel);
  ltstepper.setMaxSpeed(ltRunSpeed);
  ltstepper.setAcceleration(ltRunAccel);
  castepper.setMaxSpeed(caRunSpeed);
  castepper.setAcceleration(caRunAccel);

  
  
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
  castepper.setMaxSpeed(caJogSpeed);
  castepper.setAcceleration(caJogAccel);
  Serial.println("Camera arm homing started");
  while(digitalRead(calim)){
    castepper.moveTo( castepper.currentPosition() + ( caHomeStep * caHomeDir ) );
    // castepper.move(homeStep);
    castepper.run();
  }
  Serial.println("Camera arm homed");
  castepper.setCurrentPosition(0);
  castepper.setMaxSpeed(caRunSpeed);
  castepper.setAcceleration(caRunAccel);
  // curState = 1;
}

void checkButtons(){
  short debounce = 50;
  int jogRapid = 1000;
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
  //Set jog speeds
  utstepper.setMaxSpeed(utJogSpeed);
  utstepper.setAcceleration(utJogAccel);
  ltstepper.setMaxSpeed(ltJogSpeed);
  ltstepper.setAcceleration(ltJogAccel);
  castepper.setMaxSpeed(caJogSpeed);
  castepper.setAcceleration(caJogAccel);

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
            castepper.move(2);
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
            castepper.move(-2);
          break;
        }
        btnforFirst = false;
      }
      if( backJogRapid ){
        switch( curJogAxis ){
          case 0:
            if( utstepper.distanceToGo() == 0 ) {
              utstepper.move(3);
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
              utstepper.move(-3);
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
        //Set run speeds
        utstepper.setMaxSpeed(utRunSpeed);
        utstepper.setAcceleration(utRunAccel);
        ltstepper.setMaxSpeed(ltRunSpeed);
        ltstepper.setAcceleration(ltRunAccel);
        castepper.setMaxSpeed(caRunSpeed);
        castepper.setAcceleration(caRunAccel);
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
        lowerSequenceRunning = true;
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
                        if( casegment < ( utNumCAPos - 1 ) ){
                          casegment++;
                          long newPos = long( utcaPosDegs[casegment] ) * long( castepsPerSegment );
                          castepper.moveTo( newPos );
                          Serial.print( "Moving camera arm to: ");
                          Serial.print(casegment);
                          Serial.print( ", travelling to step position: ");
                          Serial.print(newPos);
                          Serial.print( ", travelling degrees: ");
                          Serial.println(utcaPosDegs[casegment]);
                          utsegment = 0;
                          utSegmentsLeft = true;
                        }else{
                          Serial.println("upper turntable use finished, move to lower turntable");
                          upperSequenceFinished = true;
                        }
                  }else if( !utStartWobble && utstepper.currentPosition() > 0 ){
                        //Here we have just finished rotating the UT to the correct position and now start the wobble delay timer
                        //as well as the segment timer. The picture won't take until the wobble delay is up and the segment wont
                        //move on to the next until the segment timer is up
                        // Serial.print("ut upper turntable segment: ");
                        // Serial.println(utsegment);
                        utStartWobble = true;
                        wobbleTimer = millis();
                        //reset the timer value to be current
                        segmentTimer = millis();
                  }else if( millis() - wobbleTimer > utWobbleDelay && utStartWobble ){

                      //increment the segment counter once we have reached the position
                      utsegment++;
                      //turn the flag off so we only increment once.
                      utjustArrived = false;
                      //raise the flag to take the picture and start the timer
                      pictureToTake = true;
                      pictureTimer = micros();
                      utStartWobble = false;
                      // Serial.println("UT wobble delay finished");

                  }else if( !caStartWobble && utstepper.currentPosition() == 0 ){
                        //Here we have just finished rotating the CA to the correct position and now start the CA wobble delay timer
                        //as well as the segment timer. The picture won't take until the wobble delay is up and the segment wont
                        //move on to the next until the segment timer is up. This is separate from the UT delays
                        Serial.print("ca upper turntable segment: ");
                        Serial.println(utsegment);
                        caStartWobble = true;
                        wobbleTimer = millis();
                        //reset the timer value to be current
                        segmentTimer = millis();
                  }else if( millis() - wobbleTimer > caWobbleDelay && caStartWobble ){

                      //increment the segment counter once we have reached the position
                      utsegment++;
                      //turn the flag off so we only increment once.
                      utjustArrived = false;
                      //raise the flag to take the picture and start the timer
                      pictureToTake = true;
                      pictureTimer = micros();
                      caStartWobble = false;
                      Serial.println("CA wobble delay finished");

                  }
              }

              //once we are in position, this conditional is checked until the timer exceeds the interval
              if( millis() - segmentTimer > segmentInterval && utSegmentsLeft && castepper.distanceToGo() == 0 && !utjustArrived ){
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
              if( casegment < ( ( utNumCAPos - 1 ) + ( ltNumCAPos - 1 ) ) ){
                casegment++;
                float totalAngle = 0;
                //add the separate angle distances together
                // for(int i = 0; i < casegment; i++){
                //   totalAngle = totalAngle + int( ltcaPosDegs[i] );
                // }
                // long newPos = long( totalAngle ) * long( castepsPerSegment );
                long newPos = long( ltcaPosDegs[ casegment - ( utNumCAPos - 1 ) ] ) * long( castepsPerSegment );
                castepper.moveTo( newPos );
                Serial.print( "Moving camera arm to: ");
                Serial.print(casegment);
                Serial.print( ", travelling to step position: ");
                Serial.print(newPos);
                Serial.print( ", travelling degrees: ");
                Serial.println( ltcaPosDegs[ casegment - ( utNumCAPos - 1 ) ] );
                ltsegment = 0;
                ltSegmentsLeft = true;
              }else{
                Serial.println("upper and lower turntable use finished");
                lowerSequenceFinished = true;
              }
            }else if( !ltStartWobble && ltstepper.currentPosition() > 0 ){
                        //Here we have just finished rotating the LT to the correct position and now start the wobble delay timer
                        //as well as the segment timer. The picture won't take until the wobble delay is up and the segment wont
                        //move on to the next until the segment timer is up
                        // Serial.print("lt lower turntable segment: ");
                        // Serial.println(ltsegment);
                        ltStartWobble = true;
                        wobbleTimer = millis();
                        //reset the timer value to be current
                        segmentTimer = millis();
                  }else if( millis() - wobbleTimer > ltWobbleDelay && ltStartWobble ){

                      //increment the segment counter once we have reached the position
                      ltsegment++;
                      //turn the flag off so we only increment once.
                      ltjustArrived = false;
                      //raise the flag to take the picture and start the timer
                      pictureToTake = true;
                      pictureTimer = micros();
                      ltStartWobble = false;
                      // Serial.println("LT wobble delay finished");

                  }else if( !caStartWobble && ltstepper.currentPosition() == 0 ){
                        //Here we have just finished rotating the CA to the correct position and now start the CA wobble delay timer
                        //as well as the segment timer. The picture won't take until the wobble delay is up and the segment wont
                        //move on to the next until the segment timer is up. This is separate from the LT delays
                        Serial.print("ca lower turntable segment: ");
                        Serial.println(ltsegment);
                        caStartWobble = true;
                        wobbleTimer = millis();
                        //reset the timer value to be current
                        segmentTimer = millis();
                  }else if( millis() - wobbleTimer > caWobbleDelay && caStartWobble ){

                      //increment the segment counter once we have reached the position
                      ltsegment++;
                      //turn the flag off so we only increment once.
                      ltjustArrived = false;
                      //raise the flag to take the picture and start the timer
                      pictureToTake = true;
                      pictureTimer = micros();
                      caStartWobble = false;
                      Serial.println("CA wobble delay finished");

                  }
          }

          //once we are in position, this conditional is checked until the timer exceeds the interval
          if( millis() - segmentTimer > segmentInterval && ltSegmentsLeft && castepper.distanceToGo() == 0 && !ltjustArrived ){
            // The step position to move to. It is converted to float to maintain highest precision
            // This will make the distanceToGo call not return zero
            ltstepper.moveTo( long( float( ltsegment ) * ltstepsPerSegment ) );
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
  
  unsigned long flashOnTime = segmentInterval;// (microseconds) camfoc is now used for triggering the flash as of 24-11-22
  // unsigned long triggerOnTime = 1000; //(milliseconds)
  
  if( pictureToTake ){
    unsigned long timeElapsed = micros() - pictureTimer;

    if( timeElapsed < flashOnTime && picFirst[0] ){
      picFirst[0] = false;
      digitalWrite( camtrig1, HIGH );
      cam1Taken = true;
      cam1Timer = millis();
      // Serial.print(timeElapsed);
      // Serial.println(", cam on");
    }else if( timeElapsed > flashOnTime && timeElapsed < ( flashOnTime + ( 1000 * camTrigDel ) ) && picFirst[1] ){
      picFirst[1] = false;
      digitalWrite( camfoc, HIGH );//camfoc pin is connected to flash
      // Serial.print(timeElapsed);
      // Serial.println(", flash on");
    }else if( timeElapsed > ( flashOnTime + ( 1000 * camTrigDel ) ) && !picFirst[1] ){
      digitalWrite( camfoc, LOW );
      digitalWrite( camtrig1, LOW );
      // Serial.print(timeElapsed);
      // Serial.println(", flash off, camera off");
      picFirst[0]=true;picFirst[1]=true;
    }else if( millis() - cam1Timer > cam1Del && cam1Taken && !cam2Taken ){
      digitalWrite( camtrig2, HIGH );
      cam2Taken = true;
      cam2Off = false;
      cam2Timer = millis();
      // Serial.println("cam2 trig on");
    }else if( millis() - cam2Timer > camTrigDel && cam2Taken && !cam2Off ){
      digitalWrite( camtrig2, LOW );
      // Serial.println("cam2 trig off");
      cam2Off = true;
    }else if( millis() - cam2Timer > cam1Del && cam2Taken && !cam3Taken ){
      digitalWrite( camtrig3, HIGH );
      cam3Taken = true;
      cam3Timer = millis();
      // Serial.println("cam3 trig on");
      cam3Off = false;
    }else if( millis() - cam3Timer > camTrigDel && cam3Taken && !cam3Off ){
      digitalWrite( camtrig3, LOW);
      pictureToTake = false;
      cam1Taken = false;
      cam2Taken = false;
      cam3Taken = false;
      // Serial.println("cam3 trig off");
      cam3Off = true;
    }
  }
}
