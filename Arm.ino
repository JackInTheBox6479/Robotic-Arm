#include <AccelStepper.h>
#include <VarSpeedServo.h>

AccelStepper baseStepper(4, 10, 12, 11, 13);
AccelStepper shoulderStepper(4, 9, 7, 8, 6);

VarSpeedServo elbow;
#define ELBOW_PIN 5

String inputBuffer = "";
long base_rot = 0;      
long shoulder_rot = 0;

void setup() {
    Serial.begin(115200);

    baseStepper.setMaxSpeed(400);
    baseStepper.setAcceleration(500);
  
    shoulderStepper.setMaxSpeed(200);
    shoulderStepper.setAcceleration(500);

    elbow.attach(ELBOW_PIN);
    elbow.write(90, 30, true);
}

void loop() {
    readSerial();

    baseStepper.runSpeed();
    shoulderStepper.runSpeed();
    
    elbow.write(90, 30, false);
    moveMotorsTo(1000, 1100, 0);
    delay(1000);
    moveMotorsTo(-1000, -1100, 180);
    delay(1000);
    moveMotorsTo(1, 1, 45);
    delay(100);
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
    if (cmd == "Base_CW") {
        Serial.println("Base_CW");
        baseStepper.setSpeed(400);
    } 
    else if (cmd == "Base_CCW") {
        baseStepper.setSpeed(-400); 
    } 
    else if (cmd == "Base_Stop") {
        baseStepper.setSpeed(0);
    }

  
    else if (cmd == "Shoulder_CW") {
        shoulderStepper.setSpeed(400);
    } 
    else if (cmd == "Shoulder_CCW") {
        shoulderStepper.setSpeed(-400);
    } 
    else if (cmd == "Shoulder_Stop") {
        shoulderStepper.setSpeed(0);
    }

 
    else if (cmd == "Zero") {
        moveMotorsTo(500, 0, 90);
    }
    else if (cmd == "Set_Zero") {
        baseStepper.setSpeed(0);
        shoulderStepper.setSpeed(0);
    
        baseStepper.setCurrentPosition(0);
        shoulderStepper.setCurrentPosition(0);
    }
}

void moveMotorsTo(long baseTarget, long shoulderTarget, int elbowTarget) {
    if (!elbow.attached()) {
        elbow.attach(ELBOW_PIN);
    }

    baseStepper.setSpeed(0);
    shoulderStepper.setSpeed(0);

    int distanceToGoBase = baseTarget - baseStepper.currentPosition();

    long distBase = abs(baseTarget - baseStepper.currentPosition());
    long distShoulder = abs(shoulderTarget - shoulderStepper.currentPosition());

    long maxDistance = max(distBase, distShoulder);

    if (maxDistance == 0) {
      return;
    }


    float maxSpeedBase = 400.0 * ((float)distBase / maxDistance);
    float maxSpeedShoulder = 400.0 * ((float)distShoulder / maxDistance);

    baseStepper.setMaxSpeed(maxSpeedBase);
    shoulderStepper.setMaxSpeed(maxSpeedShoulder);

    baseStepper.moveTo(baseTarget);
    shoulderStepper.moveTo(shoulderTarget);
    
    elbow.write(elbowTarget, 75, false);

    while (baseStepper.distanceToGo() != 0 || shoulderStepper.distanceToGo() != 0) {
        baseStepper.run();
        shoulderStepper.run();
    }
    while(elbow.isMoving()) {
        delay(1); 
    }
    elbow.detach();
}