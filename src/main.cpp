//#include <FlashAsEEPROM.h>
#include "Arduino.h"
#include <Servo.h>
#include <SPI.h>
#include "SD.h"
//#include <Stepper.h>

#define LINE_BUFFER_LENGTH 512

/*
   PARAMETRES DE CONFIGURATION
*/

const int penZUp = 30;                 // angle when pen is up
const int penZDown = 80;                 // angle when pen is down
const int penServoPin = 4;              // Pin sur lequel est relié le servomoteur
const int stepsPerRevolution = 20;              // Valeur par défaut
const int vitesseDeplacement = 250;             // Vitesse de déplacement des axes X et Y

Servo penServo;                     // Objet pour actionner le servomoteur

// Initialisation des moteurs pas à pas pilotés à l'aide d'un pont H L293D
//Stepper myStepperX(stepsPerRevolution, 8,9,10,11);
//Stepper myStepperY(stepsPerRevolution, 2,3,4,5);

// Calibration, nombre de pas par millimètre
float StepsPerMillimeterX = 200; // Here you need to chage steps per mm
float StepsPerMillimeterY = 400;

/*
    FIN DE LA CONFIGURATION
*/

/* Structures, global variables    */
struct point {
  float x;
  float y;
  float z;
};

// Current position of plothead
struct point actuatorPos;

//  Drawing settings, should be OK
float StepInc = 1;
int StepDelay = 0;
int LineDelay = 50;
int penDelay = 50;

// Drawing robot limits, in mm
// OK to start with. Could go up to 50 mm if calibrated well.
float Xmin = 0;
float Xmax = 40;
float Ymin = 0;
float Ymax = 40;
float Zmin = 0;
float Zmax = 1;

float Xpos = Xmin;
float Ypos = Ymin;
float Zpos = Zmax;

// Set to true to get debug output.
boolean verbose = false;

//  Needs to interpret
//  G1 for moving
//  G4 P300 (wait 150ms)
//  M300 S30 (pen down)
//  M300 S50 (pen up)
//  Discard anything with a (
//  Discard any other command!


void moveStepperX(int steps) {
  //stepper motor 1

  if(steps>0)
    digitalWrite(6, HIGH);
  else {
    digitalWrite(6, LOW);
    steps = steps*(-1);
  }
  delay(5);
  for (int i = 0; i < steps; i++) {
    digitalWrite(5, HIGH);
    delayMicroseconds(400);
    digitalWrite(5, LOW);
    delayMicroseconds(400);
  }

}

void moveStepperY(int steps) {
  //stepper motor 2

  if(steps>0)
    digitalWrite(8, HIGH);
  else {
    digitalWrite(8, LOW);
    steps = steps*(-1);
  }
  delay(5);
  for (int i = 0; i < steps; i++) {
    digitalWrite(7, HIGH);
    delayMicroseconds(400);
    digitalWrite(7, LOW);
    delayMicroseconds(400);
  }

}

//  Raises pen
void penUp() {
  penServo.write(penZUp);
  delay(LineDelay);
  Zpos = Zmax;
  if (verbose) {
    SerialUSB.println("Pen up!");
  }
}
//  Lowers pen
void penDown() {
  penServo.write(penZDown);
  delay(LineDelay);
  Zpos = Zmin;
  if (verbose) {
    SerialUSB.println("Pen down.");
  }
}


/*********************************
   Draw a line from (x0;y0) to (x1;y1).
   Bresenham algo from https://www.marginallyclever.com/blog/2013/08/how-to-build-an-2-axis-arduino-cnc-gcode-interpreter/
   int (x1;y1) : Starting coordinates
   int (x2;y2) : Ending coordinates
 **********************************/
void drawLine(float x1, float y1) {

  if (verbose)
  {
    SerialUSB.print("fx1, fy1: ");
    SerialUSB.print(x1);
    SerialUSB.print(",");
    SerialUSB.print(y1);
    SerialUSB.println("");
  }

  //  Bring instructions within limits
  if (x1 >= Xmax) {
    x1 = Xmax;
  }
  if (x1 <= Xmin) {
    x1 = Xmin;
  }
  if (y1 >= Ymax) {
    y1 = Ymax;
  }
  if (y1 <= Ymin) {
    y1 = Ymin;
  }

  if (verbose)
  {
    SerialUSB.print("Xpos, Ypos: ");
    SerialUSB.print(Xpos);
    SerialUSB.print(",");
    SerialUSB.print(Ypos);
    SerialUSB.println("");
  }

  if (verbose)
  {
    SerialUSB.print("x1, y1: ");
    SerialUSB.print(x1);
    SerialUSB.print(",");
    SerialUSB.print(y1);
    SerialUSB.println("");
  }

  //  Convert coordinates to steps
  x1 = (int)(x1 * StepsPerMillimeterX);
  y1 = (int)(y1 * StepsPerMillimeterY);
  float x0 = Xpos;
  float y0 = Ypos;

  //  Let's find out the change for the coordinates
  long dx = abs(x1 - x0);
  long dy = abs(y1 - y0);
  int sx = x0 < x1 ? StepInc : -StepInc;
  int sy = y0 < y1 ? StepInc : -StepInc;

  long i;
  long over = 0;

  if (dx > dy) {
    for (i = 0; i < dx; ++i) {
      //myStepperX.step(sx);
      moveStepperX(sx);
      over += dy;
      if (over >= dx) {
        over -= dx;
        //myStepperY.step(sy);
        moveStepperY(sy);
      }
      delay(StepDelay);
    }
  }
  else {
    for (i = 0; i < dy; ++i) {
      //myStepperY.step(sy);
      moveStepperY(sy);
      over += dx;
      if (over >= dy) {
        over -= dy;
        //myStepperX.step(sx);
        moveStepperX(sx);
      }
      delay(StepDelay);
    }
  }

  if (verbose)
  {
    SerialUSB.print("dx, dy:");
    SerialUSB.print(dx);
    SerialUSB.print(",");
    SerialUSB.print(dy);
    SerialUSB.println("");
  }

  if (verbose)
  {
    SerialUSB.print("Going to (");
    SerialUSB.print(x0);
    SerialUSB.print(",");
    SerialUSB.print(y0);
    SerialUSB.println(")");
  }

  //  Delay before any next lines are submitted
  delay(LineDelay);
  //  Update the positions
  Xpos = x1;
  Ypos = y1;
}

void processIncomingLine(const char* line, int charNB ) {
  int currentIndex = 0;
  char buffer[ 64 ];                                 // Hope that 64 is enough for 1 parameter
  struct point newPos;

  newPos.x = 0.0;
  newPos.y = 0.0;

  //  Needs to interpret
  //  G1 for moving
  //  G4 P300 (wait 150ms)
  //  G1 X60 Y30
  //  G1 X30 Y50
  //  M300 S30 (pen down)
  //  M300 S50 (pen up)
  //  Discard anything with a (
  //  Discard any other command!

  while ( currentIndex < charNB ) {
    switch ( line[ currentIndex++ ] ) {              // Select command, if any
      case 'U':
        penUp();
        break;
      case 'D':
        penDown();
        break;
      case 'G':
        buffer[0] = line[ currentIndex++ ];          // /!\ Dirty - Only works with 2 digit commands
        //      buffer[1] = line[ currentIndex++ ];
        //      buffer[2] = '\0';
        buffer[1] = '\0';

        switch ( atoi( buffer ) ) {                  // Select G command
          case 0:                                   // G00 & G01 - Movement or fast movement. Same here
          case 1:
            // /!\ Dirty - Suppose that X is before Y
            char* indexX = strchr( line + currentIndex, 'X' ); // Get X/Y position in the string (if any)
            char* indexY = strchr( line + currentIndex, 'Y' );
            if ( indexY <= 0 ) {
              newPos.x = atof( indexX + 1);
              newPos.y = actuatorPos.y;
            }
            else if ( indexX <= 0 ) {
              newPos.y = atof( indexY + 1);
              newPos.x = actuatorPos.x;
            }
            else {
              newPos.y = atof( indexY + 1);
              *indexY = '\0';
              newPos.x = atof( indexX + 1);
            }
            drawLine(newPos.x, newPos.y );
            //        Serial.println("ok");
            actuatorPos.x = newPos.x;
            actuatorPos.y = newPos.y;
            break;
        }
        break;
      case 'M':
        buffer[0] = line[ currentIndex++ ];        // /!\ Dirty - Only works with 3 digit commands
        buffer[1] = line[ currentIndex++ ];
        buffer[2] = line[ currentIndex++ ];
        buffer[3] = '\0';
        switch ( atoi( buffer ) ) {
          case 300:
            {
              char* indexS = strchr( line + currentIndex, 'S' );
              float Spos = atof( indexS + 1);
              //          Serial.println("ok");
              if (Spos == 30) {
                penDown();
              }
              if (Spos == 50) {
                penUp();
              }
              break;
            }
          case 114:                                // M114 - Repport position
            Serial.print( "Absolute position : X = " );
            Serial.print( actuatorPos.x );
            Serial.print( "  -  Y = " );
            Serial.println( actuatorPos.y );
            break;
          default:
            Serial.print( "Command not recognized : M");
            Serial.println( buffer );
        }
    }
  }



}

void SDcardReadlinebyline(File *fp){
    while(fp->available()){
        String line = fp->readStringUntil('\n');
        processIncomingLine(line.c_str(),line.length());
    }
}

/**********************
   void setup() - Initialisations
 ***********************/
void setup() {
  //  Setup
  SerialUSB.begin( 9600 );
  Serial.begin(9600);

  penServo.attach(penServoPin);
  penServo.write(penZUp);
  delay(200);

  // Decrease if necessary
  //myStepperX.setSpeed(vitesseDeplacement);
  //myStepperY.setSpeed(vitesseDeplacement);

  //  Set & move to initial default position
  // TBD

  //  Notifications!!!
  Serial.println("Mini CNC Plotter alive and kicking!");
  Serial.print("X range is from ");
  Serial.print(Xmin);
  Serial.print(" to ");
  Serial.print(Xmax);
  Serial.println(" mm.");
  Serial.print("Y range is from ");
  Serial.print(Ymin);
  Serial.print(" to ");
  Serial.print(Ymax);
  Serial.println(" mm.");

  //setup stepper motor pins
 pinMode(11, OUTPUT); // ENABLE MOTORS   ATSAMD21:PA16
  pinMode(5, OUTPUT); // STEP MOTOR 1 ATSAMD21:PA15
  pinMode(6, OUTPUT); // DIR MOTOR 1  ATSAMD21:PA20
  pinMode(7, OUTPUT); // STEP MOTOR 2 ATSAMD21:PA21
  pinMode(8, OUTPUT); // DIR MOTOR 2  ATSAMD21:PA06
  pinMode(9, OUTPUT); // STEP MOTOR 3 ATSAMD21:PA07
  pinMode(10, OUTPUT); // DIR MOTOR 3  ATSAMD21:PA18

  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);

  pinMode(A4, OUTPUT);    // Microstepping output
  digitalWrite(A4, LOW); // 1/16 (default config)
  digitalWrite(6, HIGH);
  digitalWrite(8, HIGH);
  digitalWrite(11, LOW);

  SerialUSB.print("Initializing SD card...");

  if (!SD.begin(9)) {
    SerialUSB.println("initialization failed!");
    while (1);
  }
  SerialUSB.println("initialization done.");
}

/**********************
   void loop() - Main loop
 ***********************/
void loop()
{
  delay(200);
  char line[ LINE_BUFFER_LENGTH ];
  char c;
  int lineIndex;
  bool lineIsComment, lineSemiColon;

  lineIndex = 0;
  lineSemiColon = false;
  lineIsComment = false;

  bool fileStart = false;
  File dataFile;

  while (1) {

    // Serial reception - Mostly from Grbl, added semicolon support
    while ( Serial.available() > 0 ) {
      c = Serial.read();
      if (( c == '\n') || (c == '\r') || (c == '@') ) {             // End of line reached
        if ( lineIndex > 0 ) {                        // Line is complete. Then execute!
          line[ lineIndex ] = '\0';                   // Terminate string
          if (verbose) {
            SerialUSB.print( "Received : ");
            SerialUSB.println( line );
          }
          if((line == "OK+CONN") || (line == "OK+LOST")){
              //dom nothing just clear buffer
          }
          else if((fileStart == false) && (line == "##START")) {
              if(verbose){
                  SerialUSB.println("SD card opened file");
              }
              dataFile = SD.open("datalog.txt", FILE_WRITE);
              fileStart = true;
          }
          else if(fileStart == true) {
              if(line == "##OK"){
                  if(verbose){
                      SerialUSB.println("SD card closed file");
                  }
                  Serial.println("ack"); // let BLE know file is completely received
                  dataFile.close();
                  delay(500);
                  dataFile = SD.open("datalog.txt", FILE_READ);
                  //reopen file in reopen and iterate line by lines
                  SDcardReadlinebyline(&dataFile);
              }
              else{
                  dataFile.println(line);
              }
          }

          lineIndex = 0;
        }
        else {
          // Empty or comment line. Skip block.
        }
        lineIsComment = false;
        lineSemiColon = false;
        Serial.println("ok");
      }
      else {
        if ( (lineIsComment) || (lineSemiColon) ) {   // Throw away all comment characters
          if ( c == ')' )  lineIsComment = false;     // End of comment. Resume line.
        }
        else {
          if ( c <= ' ' ) {                           // Throw away whitepace and control characters
          }
          else if ( c == '/' ) {                    // Block delete not supported. Ignore character.
          }
          else if ( c == '(' ) {                    // Enable comments flag and ignore all characters until ')' or EOL.
            lineIsComment = true;
          }
          else if ( c == ';' ) {
            lineSemiColon = true;
          }
          else if ( lineIndex >= LINE_BUFFER_LENGTH - 1 ) {
            Serial.println( "ERROR - lineBuffer overflow" );
            lineIsComment = false;
            lineSemiColon = false;
          }
          else if ( c >= 'a' && c <= 'z' ) {        // Upcase lowercase
            line[ lineIndex++ ] = c - 'a' + 'A';
          }
          else {
            line[ lineIndex++ ] = c;
            SerialUSB.println(line);
          }
        }
      }
    }
  }
}
