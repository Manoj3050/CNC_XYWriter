//#include <FlashAsEEPROM.h>
#include "Arduino.h"
#include <Servo.h>
#include <SPI.h>
#include "SD.h"
//#include <Stepper.h>

// ****************** COMMANDS
#define GCODE_RECEIVED_SUCCESS "ok"
#define GCODE_RECEIVED_UNSUCCESS "bad"
#define PRINT_FINISHED "Printed successful."
#define SD_INIT_FAILED "SD card initialization failed"
//////////////////////////////

#define LINE_BUFFER_LENGTH 512

#define CS_PIN 9
#define LIMIT_SWITCH_PIN 10
#define DELAY 5
/*
PARMITERS OF CONFIGURATION
*/

const int penZUp =32;                 // angle when pen is up
const int penZDown = 80;                 // angle when pen is down
const int penServoPin = 4;              //Servo pin
const int stepsPerRevolution = 20;              // steps per revolution
const int vitesseDeplacement = 250;             // Vitesse of displacment for axes X and Y
const int EJECT_PAPER_STEPS = 800;   //change here to adjust steps to eject paper
Servo penServo;                     // Object servo motor

// Initialisation des moteurs pas à pas pilotés à l'aide d'un pont H L293D
//Stepper myStepperX(stepsPerRevolution, 8,9,10,11);
//Stepper myStepperY(stepsPerRevolution, 2,3,4,5);

// Scaling Calibration, in millimeters
float StepsPerMillimeterX = 50; // Here you need to chage steps per mm
float StepsPerMillimeterY = 30; //this is scaling.
/*
FINAL CONFIGURATION
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

// Drawing limits, in mm
// OK to start with. Could go up to 50 mm if calibrated well.
float Xmin = 0; // home X
float Xmax = 485; // nneed to set these as in website
float Ymin = 0; //home Y
float Ymax = 442.7; // need to set these as in website
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
    digitalWrite(6, LOW);
    else {
        digitalWrite(6, HIGH);
        steps = steps*(-1);
    }
    delayMicroseconds(250); //adjusts Speed
    for (int i = 0; i < steps; i++) {
        digitalWrite(5, HIGH);
        delayMicroseconds(DELAY);
        digitalWrite(5, LOW);
        delayMicroseconds(DELAY);
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
    delayMicroseconds(250); //adjusts speed
    for (int i = 0; i < steps; i++) {
        digitalWrite(7, HIGH);
        delayMicroseconds(DELAY);
        digitalWrite(7, LOW);
        delayMicroseconds(DELAY);
    }

}
//move stepper X until hit the limit switch
void moveStepperXUntil(){
    while(digitalRead(LIMIT_SWITCH_PIN) == LOW){
        moveStepperX(10);
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

void ejectMotors(){
    //turn X axis motor to en
    penUp();
    drawLine(Xmin, Ymin);
    moveStepperY(EJECT_PAPER_STEPS); // increae this 332 until it ejects
    Ypos = EJECT_PAPER_STEPS;
    drawLine(Xmin,0);
    moveStepperXUntil();
    digitalWrite(11, HIGH); //disable motors
    penServo.detach();

}

void SDcardReadlinebyline(File *fp){
    penServo.attach(penServoPin);
    digitalWrite(11, LOW); //enable motors
    while(fp->available()){
        String line = fp->readStringUntil('\n');
        if(verbose)
        SerialUSB.println(line);
        processIncomingLine(line.c_str(),line.length());
    }
    ejectMotors();
    Serial.println(PRINT_FINISHED); // send this line after a successful print
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
    //pinMode(9, OUTPUT); // STEP MOTOR 3 ATSAMD21:PA07
    //pinMode(10, OUTPUT); // DIR MOTOR 3  ATSAMD21:PA18

    pinMode(4, OUTPUT);
    digitalWrite(4, HIGH);

    pinMode(A4, OUTPUT);    // Microstepping output
    digitalWrite(A4, HIGH); // 1/16 (default config)
    digitalWrite(6, HIGH);
    digitalWrite(8, HIGH);
    digitalWrite(11, HIGH); //disable motors for sttart

    pinMode(LIMIT_SWITCH_PIN,INPUT); // set input for limit switch

    SerialUSB.print("Initializing SD card...");

    if (!SD.begin(CS_PIN)) {
        SerialUSB.println("initialization failed!");
        Serial.println(SD_INIT_FAILED);
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
                    if((String(line) == "OK+CONN") || (String(line) == "OK+LOST")) {
                        //dom nothing just clear buffer
                    }
                    else if((fileStart == false) && (String(line) == "##START")) {
                        if(verbose) {
                            SerialUSB.println("SD card opened file");
                        }
                        //removes file if already exists
                        if(SD.exists("datalog.txt")){
                            SD.remove("datalog.txt");
                        }
                        dataFile = SD.open("datalog.txt", FILE_WRITE);
                        fileStart = true;
                    }
                    else if(fileStart == true) {
                        if(String(line) == "##OK") {
                            if(verbose){
                                SerialUSB.println("SD card closed file");
                            }
                            Serial.println(GCODE_RECEIVED_SUCCESS); // let BLE know file is completely received
                            dataFile.close();
                            delay(500);
                            dataFile = SD.open("datalog.txt", FILE_READ);
                            //reopen file in reopen and iterate line by lines
                            SDcardReadlinebyline(&dataFile);
                            fileStart = false;
                        }
                        else if(String(line).indexOf("CONVERTERFAILEDWITHCODE") > 0){
                            if(verbose){
                                SerialUSB.println("SD card closed file.Bad Gcode");
                            }
                            Serial.println(GCODE_RECEIVED_UNSUCCESS); // let BLE know file is completely received
                            dataFile.close();
                            fileStart = false;
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
                }
            }
        }
    }
}
}
