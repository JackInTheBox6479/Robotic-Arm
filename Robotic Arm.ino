#include <AccelStepper.h>
#include <VarSpeedServo.h>

AccelStepper baseStepper(4, 2, 4, 3, 5);
AccelStepper shoulderStepper(4, 9, 7, 8, 6);

int baseGearRatio = 3;
int baseMaxSpeed = 750;
int shoulderGearRatio = 3;
int shoulderMaxSpeed = 400;

VarSpeedServo elbow;
#define ELBOW_PIN A0
VarSpeedServo wrist;
#define WRIST_PIN A1

String inputBuffer = "";
long base_rot = 0;      
long shoulder_rot = 0;
#define PI 3.141592653589793
const double L1 = 4.0; 
const double L2 = 7.25; 
const double STEPS_PER_REV = 2048.0;

void setup() {
    Serial.begin(115200);

    baseStepper.setMaxSpeed(baseMaxSpeed);
    baseStepper.setAcceleration(800);
  
    shoulderStepper.setMaxSpeed(shoulderMaxSpeed);
    shoulderStepper.setAcceleration(800);

    elbow.attach(ELBOW_PIN);
    elbow.write(90, 30, true);
    wrist.attach(WRIST_PIN);
    wrist.write(90, 30, true);
}

void loop() {
    runTestSequence(2);
}

void runTestSequence(int sqnc) {
    if(sqnc == 1) {
        Serial.println("Center - Straight up");
        moveMotorsToPoint(0, 0, 8);
        delay(5000);

        Serial.println("Full Extension - Right");
        moveMotorsToPoint(8, 0, 0);
        delay(5000);

        Serial.println("Full Extension - Back");
        moveMotorsToPoint(0, 8, 0);
        delay(5000);

        Serial.println("Mid-Air Symmetric");
        moveMotorsToPoint(4, 0, 4);
        delay(5000);
    
        Serial.println("High Reach, Bent");
        moveMotorsToPoint(6, 0, 5);
        delay(5000);

        Serial.println("Return Home");
        moveMotorsToPoint(0, 0, 8);
        delay(5000);
    }
    else if (sqnc == 2) {
        Serial.println("(0, 0, 11.25)");
        moveMotorsToPoint(0, 0, 11.25);
        delay(1000);

        Serial.println("(7.95, 7.95, 0)");
        moveMotorsToPoint(7.95, 7.95, 0);
        delay(1000);

        Serial.println("(-7.95, -7.95, 0)");
        moveMotorsToPoint(-7.95, -7.95, 0);
        delay(1000);

        Serial.println("(0, -7.95, 7.95)");
        moveMotorsToPoint(0, -7.95, 7.95);
        delay(1000);
    
        Serial.println("(3, -2, 4)");
        moveMotorsToPoint(3, -2, 4);
        delay(1000);

        Serial.println("(-2, 6, 3)");
        moveMotorsToPoint(-2, 6, -3);
        delay(1000);

        Serial.println("(-3, 0, -3)");
        moveMotorsToPoint(-3, 0, -3);
        delay(1000);
    }
}

void moveMotorsToPoint(double x, double y, double z) {
    double baseAngle = atan2(y, x) * 180.0 / PI;

    double l = sqrt(x * x + y * y);
    double h_sq = l * l + z * z; 
    double h = sqrt(h_sq);

    if (h > (L1 + L2) || h < fabs(L1 - L2)) {
        Serial.println("Target point is out of reach.");
        return;
    }

    double cos_beta = (L1*L1 + L2*L2 - h_sq) / (2.0 * L1 * L2);
    cos_beta = max(-1.0, min(1.0, cos_beta));
    double elbow_internal_angle = acos(cos_beta); 
    
    double cos_gamma = (L1*L1 + h_sq - L2*L2) / (2.0 * L1 * h);
    cos_gamma = max(-1.0, min(1.0, cos_gamma));

    double gamma = acos(cos_gamma);
    double phi = atan2(z, l); 

    double shoulderAngleRad = phi + gamma;
    double elbowAngleRad = PI - elbow_internal_angle;
    double shoulderAngle = shoulderAngleRad * 180.0 / PI;
    double elbowAngle = elbowAngleRad * 180.0 / PI;

    moveToAngle(baseAngle, shoulderAngle, elbowAngle);
}

void moveToAngle(double baseAngle, double shoulderAngle, double elbowAngle) {
    long baseTarget = round((baseAngle / 360.0) * STEPS_PER_REV * baseGearRatio);
    long shoulderTarget = round(((shoulderAngle - 90) / 360.0) * STEPS_PER_REV * shoulderGearRatio);

    int elbowTarget = round(90 + elbowAngle); 
    
    int wristTarget = 90;

    moveMotorsToPosition(baseTarget, shoulderTarget, elbowTarget, wristTarget);
}

void moveMotorsToPosition(long baseTarget, long shoulderTarget, int elbowTarget, int wristTarget) {
    baseStepper.setMaxSpeed(baseMaxSpeed);
    shoulderStepper.setMaxSpeed(shoulderMaxSpeed);

    baseStepper.moveTo(baseTarget);
    shoulderStepper.moveTo(shoulderTarget);
    
    while (shoulderStepper.distanceToGo() != 0 || baseStepper.distanceToGo() != 0) {
        wrist.write(wristTarget, 30, false);
        elbow.write(elbowTarget, 30, false);
        shoulderStepper.run();
        baseStepper.run();
    }
    
    elbow.write(elbowTarget, 75, false);
    
    printCurrentPosition();
}

void getPosition(double &x, double &y, double &z) {
    double baseAngle = (baseStepper.currentPosition() * 360.0) / (STEPS_PER_REV * baseGearRatio);
    double shoulderAngle = ((double)shoulderStepper.currentPosition() * 360.0 / (STEPS_PER_REV * shoulderGearRatio)) + 90.0;
    double elbowAngle = elbow.read() - 90.0;

    double baseRad = baseAngle * PI / 180.0;
    double shoulderRad = shoulderAngle * PI / 180.0;
    double elbowRad = elbowAngle * PI / 180.0;

    double l = L1 * cos(shoulderRad) + L2 * cos(shoulderRad - elbowRad);
    x = l * cos(baseRad);
    y = l * sin(baseRad);
    z = L1 * sin(shoulderRad) + L2 * sin(shoulderRad - elbowRad);
}

void printCurrentPosition() {
    double currX, currY, currZ;
    getPosition(currX, currY, currZ);

    Serial.print("Current Position -> X: ");
    Serial.print(currX);
    Serial.print(" Y: ");
    Serial.print(currY);
    Serial.print(" Z: ");
    Serial.println(currZ);
    Serial.println();
}

void readSerial() {
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n') {
            processCommand(inputBuffer);
            inputBuffer = "";
        } else if (c != '\r') { 
            inputBuffer += c;
        }
    }
}

void processCommand(String cmd) {
    cmd.trim();
    if (cmd == "Right") {
        baseStepper.setSpeed(400);
    } 
    else if (cmd == "Left") {
        baseStepper.setSpeed(-400); 
    } 
    else if (cmd == "Horizontal_Stop") {
        baseStepper.setSpeed(0);
    }

  
    else if (cmd == "Up") {
        shoulderStepper.setSpeed(400);
    } 
    else if (cmd == "Down") {
        shoulderStepper.setSpeed(-400);
    } 
    else if (cmd == "Vertical_Stop") {
        shoulderStepper.setSpeed(0);
    }

 
    else if (cmd == "Zero") {
        moveMotorsToPosition(500, 0, 90, 90);
    }
    else if (cmd == "Set_Zero") {
        baseStepper.setSpeed(0);
        shoulderStepper.setSpeed(0);
    
        baseStepper.setCurrentPosition(0);
        shoulderStepper.setCurrentPosition(0);
    }
}