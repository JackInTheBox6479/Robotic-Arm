#include <AccelStepper.h>
#include <VarSpeedServo.h>

AccelStepper baseStepper(4, 2, 4, 3, 5);
AccelStepper shoulderStepper(4, 9, 7, 8, 6);

int baseGearRatio = 3;
int baseMaxSpeed = 600;
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

const int MAX_PATH_POINTS = 100;
struct PathPoint {
    long baseTarget;
    long shoulderTarget;
    int elbowTarget;
    int wristTarget;
};
PathPoint pathBuffer[MAX_PATH_POINTS];
int pathIndex = 0;
int pathLength = 0;

long lastBaseTarget = 0;
long lastShoulderTarget = 0;
const float SPEED_SCALE = 0.7;

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

// 8, 5
void loop() {
    runTestSequence(4);
}

void moveMotorsToPoint(double x, double y, double z) {
    double baseAngle = atan2(y, x) * 180.0 / PI;
    double r = sqrt(x * x + y * y);
    double h_sq = r * r + z * z;
    double h = sqrt(h_sq);

    if (h > (L1 + L2) || h < fabs(L1 - L2)) {
        Serial.print("Target out of reach! h=");
        Serial.println(h);
        return;
    }

    double cos_beta = (L1 * L1 + L2 * L2 - h_sq) / (2.0 * L1 * L2);
    cos_beta = constrain(cos_beta, -1.0, 1.0);

    double cos_gamma = (L1 * L1 + h_sq - L2 * L2) / (2.0 * L1 * h);
    cos_gamma = constrain(cos_gamma, -1.0, 1.0);
    double gamma = acos(cos_gamma);
    double phi = atan2(z, r);

    double shoulderAngleDown = (phi + gamma) * 180.0 / PI;
    double shoulderAngleUp = (phi - gamma) * 180.0 / PI;
    
    double shoulderRadDown = shoulderAngleDown * PI / 180.0;
    double L2_angle_down = atan2(z - L1*sin(shoulderRadDown), r - L1*cos(shoulderRadDown));
    double elbowExteriorDown = (shoulderAngleDown - (L2_angle_down * 180.0 / PI));
    
    double shoulderRadUp = shoulderAngleUp * PI / 180.0;
    double L2_angle_up = atan2(z - L1*sin(shoulderRadUp), r - L1*cos(shoulderRadUp));
    double elbowExteriorUp = (shoulderAngleUp - (L2_angle_up * 180.0 / PI));
    
    double shoulderAngle, elbowExteriorAngle;
    bool downValid = (elbowExteriorDown >= -90 && elbowExteriorDown <= 90);
    bool upValid = (elbowExteriorUp >= -90 && elbowExteriorUp <= 90);
    
    if (downValid) {
        shoulderAngle = shoulderAngleDown;
        elbowExteriorAngle = elbowExteriorDown;
    } else {
        shoulderAngle = shoulderAngleUp;
        elbowExteriorAngle = constrain(elbowExteriorUp, -90, 90);
    }

    moveToAngle(baseAngle, shoulderAngle, elbowExteriorAngle);
}

void moveToAngle(double baseAngle, double shoulderAngle, double elbowExteriorAngle) {
    long baseTarget = round((baseAngle / 360.0) * STEPS_PER_REV * baseGearRatio);
    long shoulderTarget = round(((90.0 - shoulderAngle) / 360.0) * STEPS_PER_REV * shoulderGearRatio);
    int elbowTarget = constrain(round(90.0 - elbowExteriorAngle), 0, 180);
    
    moveMotorsToPositionSmooth(baseTarget, shoulderTarget, elbowTarget, 90);
}

void getPosition(double &x, double &y, double &z) {
    double baseAngle = (baseStepper.currentPosition() * 360.0) / (STEPS_PER_REV * baseGearRatio);
    double shoulderAngle = 90.0 - ((double)shoulderStepper.currentPosition() * 360.0 / (STEPS_PER_REV * shoulderGearRatio));
    double elbowExteriorAngle = 90.0 - elbow.read();

    double baseRad = baseAngle * PI / 180.0;
    double shoulderRad = shoulderAngle * PI / 180.0;
    double elbowExteriorRad = elbowExteriorAngle * PI / 180.0;

    double r = L1 * cos(shoulderRad) + L2 * cos(shoulderRad - elbowExteriorRad);
    x = r * cos(baseRad);
    y = r * sin(baseRad);
    z = L1 * sin(shoulderRad) + L2 * sin(shoulderRad - elbowExteriorRad);
}

void moveMotorsToPointSmooth(double x, double y, double z, int steps = 15) {
    if(steps <= 1) {
        moveMotorsToPoint(x, y, z);
        return;
    }
    
    double currX, currY, currZ;
    getPosition(currX, currY, currZ);
    
    pathLength = 0;
    
    for(int i = 1; i <= steps; i++) {
        double t = (double)i / (double)steps;
        double interpX = currX + t * (x - currX);
        double interpY = currY + t * (y - currY);
        double interpZ = currZ + t * (z - currZ);
        
        addPointToPath(interpX, interpY, interpZ);
    }
    
    executePath();
}

void addPointToPath(double x, double y, double z) {
    if (pathLength >= MAX_PATH_POINTS) return;
    
    double baseAngle = atan2(y, x) * 180.0 / PI;
    double r = sqrt(x * x + y * y);
    double h_sq = r * r + z * z;
    double h = sqrt(h_sq);

    if (h > (L1 + L2) || h < fabs(L1 - L2)) return;

    double cos_beta = (L1 * L1 + L2 * L2 - h_sq) / (2.0 * L1 * L2);
    cos_beta = constrain(cos_beta, -1.0, 1.0);
    
    double cos_gamma = (L1 * L1 + h_sq - L2 * L2) / (2.0 * L1 * h);
    cos_gamma = constrain(cos_gamma, -1.0, 1.0);
    double gamma = acos(cos_gamma);
    double phi = atan2(z, r);

    double shoulderAngleDown = (phi + gamma) * 180.0 / PI;
    double shoulderAngleUp = (phi - gamma) * 180.0 / PI;
    
    double shoulderRadDown = shoulderAngleDown * PI / 180.0;
    double L2_angle_down = atan2(z - L1*sin(shoulderRadDown), r - L1*cos(shoulderRadDown));
    double elbowExteriorDown = (shoulderAngleDown - (L2_angle_down * 180.0 / PI));
    
    double shoulderRadUp = shoulderAngleUp * PI / 180.0;
    double L2_angle_up = atan2(z - L1*sin(shoulderRadUp), r - L1*cos(shoulderRadUp));
    double elbowExteriorUp = (shoulderAngleUp - (L2_angle_up * 180.0 / PI));
    
    double shoulderAngle, elbowExteriorAngle;
    bool downValid = (elbowExteriorDown >= -90 && elbowExteriorDown <= 90);
    
    if (downValid) {
        shoulderAngle = shoulderAngleDown;
        elbowExteriorAngle = elbowExteriorDown;
    } else {
        shoulderAngle = shoulderAngleUp;
        elbowExteriorAngle = constrain(elbowExteriorUp, -90, 90);
    }

    pathBuffer[pathLength].baseTarget = round((baseAngle / 360.0) * STEPS_PER_REV * baseGearRatio);
    pathBuffer[pathLength].shoulderTarget = round(((90.0 - shoulderAngle) / 360.0) * STEPS_PER_REV * shoulderGearRatio);
    pathBuffer[pathLength].elbowTarget = constrain(round(90.0 - elbowExteriorAngle), 0, 180);
    pathBuffer[pathLength].wristTarget = 90;
    
    pathLength++;
}

void executePath() {
    if (pathLength == 0) return;
    
    pathIndex = 0;
    
    baseStepper.moveTo(pathBuffer[0].baseTarget);
    shoulderStepper.moveTo(pathBuffer[0].shoulderTarget);
    elbow.write(pathBuffer[0].elbowTarget, 60, false);
    wrist.write(pathBuffer[0].wristTarget, 60, false);
    
    while (pathIndex < pathLength) {
        long baseDist = abs(baseStepper.distanceToGo());
        long shoulderDist = abs(shoulderStepper.distanceToGo());
        long maxDist = max(baseDist, shoulderDist);
        
        if (maxDist < 150 && pathIndex < pathLength - 1) {
            pathIndex++;
            baseStepper.moveTo(pathBuffer[pathIndex].baseTarget);
            shoulderStepper.moveTo(pathBuffer[pathIndex].shoulderTarget);
            elbow.write(pathBuffer[pathIndex].elbowTarget, 60, false);
            wrist.write(pathBuffer[pathIndex].wristTarget, 60, false);
        }
        
        if (pathIndex == pathLength - 1 && maxDist < 5) {
            break;
        }
        
        baseStepper.run();
        shoulderStepper.run();
    }
    
    while (abs(baseStepper.distanceToGo()) > 0 || abs(shoulderStepper.distanceToGo()) > 0) {
        baseStepper.run();
        shoulderStepper.run();
    }
}

void moveMotorsToPositionSmooth(long baseTarget, long shoulderTarget, int elbowTarget, int wristTarget) {
    long baseDist = abs(baseTarget - baseStepper.currentPosition());
    long shoulderDist = abs(shoulderTarget - shoulderStepper.currentPosition());
    
    baseStepper.moveTo(baseTarget);
    shoulderStepper.moveTo(shoulderTarget);
    
    elbow.write(elbowTarget, 60, false);
    wrist.write(wristTarget, 60, false);
    
    while (baseStepper.distanceToGo() != 0 || shoulderStepper.distanceToGo() != 0) {
        baseStepper.run();
        shoulderStepper.run();
    }
}

void moveMotorsToPosition(long baseTarget, long shoulderTarget, int elbowTarget, int wristTarget) {
    moveMotorsToPositionSmooth(baseTarget, shoulderTarget, elbowTarget, wristTarget);
    printCurrentPosition();
}

void printCurrentPosition() {
    double currX, currY, currZ;
    getPosition(currX, currY, currZ);

    Serial.print("Position -> X: ");
    Serial.print(currX);
    Serial.print(" Y: ");
    Serial.print(currY);
    Serial.print(" Z: ");
    Serial.println(currZ);
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

bool isPointReachable(double x, double y, double z) {
    double r = sqrt(x * x + y * y);
    double h = sqrt(r * r + z * z);
    
    if (h > (L1 + L2) || h < fabs(L1 - L2)) {
        return false;
    }
    
    double h_sq = h * h;
    double cos_beta = (L1 * L1 + L2 * L2 - h_sq) / (2.0 * L1 * L2);
    cos_beta = constrain(cos_beta, -1.0, 1.0);
    
    double cos_gamma = (L1 * L1 + h_sq - L2 * L2) / (2.0 * L1 * h);
    cos_gamma = constrain(cos_gamma, -1.0, 1.0);
    double gamma = acos(cos_gamma);
    double phi = atan2(z, r);
    
    double shoulderAngleDown = (phi + gamma) * 180.0 / PI;
    double shoulderRadDown = shoulderAngleDown * PI / 180.0;
    double L2_angle_down = atan2(z - L1*sin(shoulderRadDown), r - L1*cos(shoulderRadDown));
    double elbowExteriorDown = shoulderAngleDown - (L2_angle_down * 180.0 / PI);
    
    double shoulderAngleUp = (phi - gamma) * 180.0 / PI;
    double shoulderRadUp = shoulderAngleUp * PI / 180.0;
    double L2_angle_up = atan2(z - L1*sin(shoulderRadUp), r - L1*cos(shoulderRadUp));
    double elbowExteriorUp = shoulderAngleUp - (L2_angle_up * 180.0 / PI);
    
    bool downValid = (elbowExteriorDown >= -90 && elbowExteriorDown <= 90) && 
                     (shoulderAngleDown >= 0 && shoulderAngleDown <= 180);
    bool upValid = (elbowExteriorUp >= -90 && elbowExteriorUp <= 90) && 
                   (shoulderAngleUp >= 0 && shoulderAngleUp <= 180);
    
    return (downValid || upValid);
}

void mapWorkspace() {
    Serial.println("Mapping reachable workspace...");
    
    int reachable = 0;
    int total = 0;
    
    for (double x = -11; x <= 11; x += 1.0) {
        for (double y = -11; y <= 11; y += 1.0) {
            for (double z = -5; z <= 11; z += 1.0) {
                total++;
                if (isPointReachable(x, y, z)) {
                    reachable++;
                }
            }
        }
    }
    
    Serial.print("Reachable points: ");
    Serial.print(reachable);
    Serial.print(" / ");
    Serial.print(total);
    Serial.print(" (");
    Serial.print((reachable * 100.0) / total);
    Serial.println("%)");
}

void runTestSequence(int sqnc) {
    if(sqnc == 1) {
        Serial.println("=== STAR PATTERN ===");
        moveMotorsToPointSmooth(0, 0, 9, 10);
        moveMotorsToPointSmooth(0, 0, 11, 10);
        moveMotorsToPointSmooth(7, 5, 4, 15);
        moveMotorsToPointSmooth(-5, -7, 7, 15);
        moveMotorsToPointSmooth(-7, 5, 4, 15);
        moveMotorsToPointSmooth(5, -7, 7, 15);
        moveMotorsToPointSmooth(0, 0, 11, 15);
        moveMotorsToPointSmooth(0, 0, 9, 10);
        Serial.println("=== STAR COMPLETE ===");
    }
    else if (sqnc == 2) {
        Serial.println("=== SPIRAL SEQUENCE ===");
        moveMotorsToPointSmooth(0, 0, 11, 10);
        
        for(int i = 0; i < 24; i++) {
            double angle = i * 15.0 * PI / 180.0;
            double radius = 2 + i * 0.375;
            double height = 11 - i * 0.3;
            
            double x = radius * cos(angle);
            double y = radius * sin(angle);
            
            moveMotorsToPoint(x, y, height);
        }
        Serial.println("=== SPIRAL COMPLETE ===");
    }
    else if (sqnc == 3) {
        Serial.println("=== DOME PATTERN ===");
        moveMotorsToPointSmooth(0, 0, 11, 10);
        
        for(int lat = 0; lat <= 6; lat++) {
            double heightFactor = 1.0 - (lat * 0.15);
            double height = 11 * heightFactor;
            double radius = 9 * sqrt(1.0 - heightFactor * heightFactor);
            
            if(height < 2) height = 2;
            
            for(int angle = 0; angle <= 330; angle += 30) {
                double rad = angle * PI / 180.0;
                double x = radius * cos(rad);
                double y = radius * sin(rad);
                
                moveMotorsToPoint(x, y, height);
            }
        }
        Serial.println("=== DOME COMPLETE ===");
    }
    else if (sqnc == 4) {
        Serial.println("=== PERIMETER SWEEP ===");
        
        pathLength = 0;
        for(int angle = 0; angle <= 360; angle += 10) {
            double rad = angle * PI / 180.0;
            addPointToPath(5 * cos(rad), 5 * sin(rad), 9);
        }
        executePath();
        
        pathLength = 0;
        for(int step = 0; step <= 20; step++) {
            double angle = step * 18.0;
            double rad = angle * PI / 180.0;
            double radius = 5 + step * 0.25;
            double height = 9 - step * 0.35;
            addPointToPath(radius * cos(rad), radius * sin(rad), height);
        }
        executePath();
        
        pathLength = 0;
        for(int angle = 0; angle <= 360; angle += 10) {
            double rad = angle * PI / 180.0;
            addPointToPath(10 * cos(rad), 10 * sin(rad), 2);
        }
        executePath();
        
        Serial.println("=== PERIMETER COMPLETE ===");
    }
    else if (sqnc == 5) {
        Serial.println("=== WAVE MOTION ===");
        
        pathLength = 0;
        for(int i = 0; i <= 50; i++) {
            double x = -9 + i * 0.36;
            double y = 0;
            double z = 6 + 3 * sin(i * PI / 10.0);
            addPointToPath(x, y, z);
        }
        executePath();
        
        pathLength = 0;
        for(int i = 0; i <= 50; i++) {
            double x = 0;
            double y = -9 + i * 0.36;
            double z = 6 + 2.5 * cos(i * PI / 8.0);
            addPointToPath(x, y, z);
        }
        executePath();
        
        Serial.println("=== WAVE COMPLETE ===");
    }
    else if (sqnc == 6) {
        Serial.println("=== PYRAMID PATTERN ===");
        
        double baseSize = 8.0;
        double apex = 11.0;
        double baseHeight = 2.0;
        
        moveMotorsToPointSmooth(0, 0, apex, 10);
        
        double corners[4][2] = {
            {baseSize, 0}, {0, baseSize}, {-baseSize, 0}, {0, -baseSize}
        };
        
        for(int i = 0; i < 4; i++) {
            moveMotorsToPointSmooth(corners[i][0], corners[i][1], baseHeight, 20);
            moveMotorsToPointSmooth(0, 0, apex, 20);
        }
        
        pathLength = 0;
        for(int angle = 0; angle <= 360; angle += 30) {
            double rad = angle * PI / 180.0;
            addPointToPath(baseSize * cos(rad), baseSize * sin(rad), baseHeight);
        }
        executePath();
        
        Serial.println("=== PYRAMID COMPLETE ===");
    }
    else if (sqnc == 7) {
        Serial.println("=== ULTIMATE SHOWCASE ===");
        moveMotorsToPointSmooth(0, 0, 11.25, 10);
        
        pathLength = 0;
        for(int angle = 0; angle < 360; angle += 10) {
            double rad = angle * PI / 180.0;
            addPointToPath(3 * cos(rad), 3 * sin(rad), 11);
        }
        executePath();
        
        pathLength = 0;
        for(int i = 0; i <= 40; i++) {
            double angle = i * 12.0 * PI / 180.0;
            double radius = 3 + i * 0.2;
            double height = 11 - i * 0.2;
            addPointToPath(radius * cos(angle), radius * sin(angle), height);
        }
        executePath();
        
        pathLength = 0;
        for(int angle = 0; angle <= 360; angle += 8) {
            double rad = angle * PI / 180.0;
            addPointToPath(9.5 * cos(rad), 9.5 * sin(rad), 4);
        }
        executePath();
        
        pathLength = 0;
        for(int i = 0; i <= 50; i++) {
            double t = i * PI / 25.0;
            double x = 7 * sin(t);
            double y = 7 * sin(2 * t) / 2;
            double z = 4 + i * 0.14;
            addPointToPath(x, y, z);
        }
        executePath();
        
        moveMotorsToPointSmooth(0, 0, 11.2, 20);
        
        Serial.println("=== SHOWCASE COMPLETE ===");
    }
    else if (sqnc == 8) {
        Serial.println("=== INFINITY PATTERN ===");
        
        pathLength = 0;
        for(int i = 0; i <= 60; i++) {
            double t = i * 2.0 * PI / 60.0;
            
            double scale = 7.0;
            double x = scale * sin(t) / (1 + cos(t) * cos(t));
            double y = scale * sin(t) * cos(t) / (1 + cos(t) * cos(t));
            double z = 7 + 2 * sin(2 * t);
            
            addPointToPath(x, y, z);
        }
        executePath();
        
        Serial.println("=== INFINITY COMPLETE ===");
    }
    else if (sqnc == 9) {
        Serial.println("=== HELIX PATTERN ===");
        
        pathLength = 0;
        for(int i = 0; i <= 80; i++) {
            double angle = i * 18.0 * PI / 180.0;
            double radius = 8 - i * 0.05;
            double height = 2 + i * 0.115;
            
            double x = radius * cos(angle);
            double y = radius * sin(angle);
            
            addPointToPath(x, y, height);
        }
        executePath();
        
        Serial.println("=== HELIX COMPLETE ===");
    }
    else if (sqnc == 10) {
        Serial.println("=== DANCE SEQUENCE ===");
        
        for(int circle = 0; circle < 4; circle++) {
            double height = 4 + circle * 2;
            double radius = 9 - circle * 1.5;
            
            pathLength = 0;
            for(int angle = 0; angle <= 360; angle += 10) {
                double rad = angle * PI / 180.0;
                addPointToPath(radius * cos(rad), radius * sin(rad), height);
            }
            executePath();
        }
        
        pathLength = 0;
        for(int i = 0; i <= 60; i++) {
            double t = i * PI / 30.0;
            double x = 8 * cos(t * 2);
            double y = 8 * sin(t);
            double z = 7 + 2 * sin(t * 3);
            
            addPointToPath(x, y, z);
        }
        executePath();
        
        moveMotorsToPointSmooth(0, 0, 10, 20);
        
        Serial.println("=== DANCE COMPLETE ===");
    }
}