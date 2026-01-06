#include <AccelStepper.h>
#include <VarSpeedServo.h>

AccelStepper baseStepper(4, 2, 4, 3, 5);
AccelStepper baseCounterStepper(4, 10, 12, 11, 13);
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

void setup() {
    Serial.begin(115200);

    baseStepper.setMaxSpeed(50);
    baseStepper.setAcceleration(400);

    baseCounterStepper.setMaxSpeed(100);
    baseCounterStepper.setAcceleration(400);
  
    shoulderStepper.setMaxSpeed(200);
    shoulderStepper.setAcceleration(400);

    elbow.attach(ELBOW_PIN);
    elbow.write(90, 30, true);
    wrist.attach(WRIST_PIN);
    wrist.write(90, 30, true);
}

void loop() {
    //  readSerial();

    baseStepper.runSpeed();
    baseCounterStepper.runSpeed();
    shoulderStepper.runSpeed();
    
    moveMotorsTo(1000, 400, 0, 0);
    delay(2000);
    moveMotorsTo(-1000, -400, 180, 180);
    delay(2000);
    moveMotorsTo(0, 0, 90, 90);
    delay(2000);
}

void moveMotorsTo(long baseTarget, long shoulderTarget, int elbowTarget, int wristTarget) {
    long distBase = abs((baseTarget - baseStepper.currentPosition()) * baseGearRatio);
    long distShoulder = abs((shoulderTarget - shoulderStepper.currentPosition()) * shoulderGearRatio);

    long maxDistance = max(distBase, distShoulder);

    if (maxDistance == 0) {
      return;
    }

    float maxSpeedBase = baseMaxSpeed * ((float)distBase / maxDistance);
    float maxSpeedShoulder = shoulderMaxSpeed * ((float)distShoulder / maxDistance);

    if(maxSpeedBase > baseMaxSpeed) {
        maxSpeedBase = baseMaxSpeed;
    }
    if(maxSpeedShoulder > shoulderMaxSpeed) {
        maxSpeedShoulder = shoulderMaxSpeed;
    }

    baseStepper.setMaxSpeed(baseMaxSpeed);
    baseCounterStepper.setMaxSpeed(baseMaxSpeed);
    shoulderStepper.setMaxSpeed(shoulderMaxSpeed);

    baseStepper.moveTo(baseTarget * baseGearRatio);
    baseCounterStepper.moveTo(baseTarget * baseGearRatio);
    shoulderStepper.moveTo(shoulderTarget * shoulderGearRatio);
        
    
    while (shoulderStepper.distanceToGo() != 0 || baseStepper.distanceToGo() != 0) {
        reduceShoulderTorque(elbowTarget, shoulderTarget);
        wrist.write(wristTarget, 30, false);
        shoulderStepper.run();baseStepper.run();
        baseCounterStepper.run();
    }
    
    elbow.write(elbowTarget, 75, true);
}

void reduceShoulderTorque(int elbowTarget, int shoulderTarget) {
    if(shoulderStepper.currentPosition() >= 300 && shoulderTarget < 300) {
        elbow.write(180, 100, true);
        Serial.println("1");
    }
    else if (shoulderStepper.currentPosition() <= -300 && shoulderTarget > -300) {
        elbow.write(0, 100, true);
        Serial.println("2");
    }
    else {
        elbow.write(elbowTarget, 30, false);
        Serial.println("3");
    }
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
        baseCounterStepper.setSpeed(-400);
    } 
    else if (cmd == "Base_CCW") {
        baseStepper.setSpeed(-400); 
        baseCounterStepper.setSpeed(400);
    } 
    else if (cmd == "Base_Stop") {
        baseStepper.setSpeed(0);
        baseCounterStepper.setSpeed(0);
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
        moveMotorsTo(500, 0, 90, 90);
    }
    else if (cmd == "Set_Zero") {
        baseStepper.setSpeed(0);
        baseCounterStepper.setSpeed(0);
        shoulderStepper.setSpeed(0);
    
        baseStepper.setCurrentPosition(0);
        baseCounterStepper.setCurrentPosition(0);
        shoulderStepper.setCurrentPosition(0);
    }
}