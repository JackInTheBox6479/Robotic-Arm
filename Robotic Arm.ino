#include <Servo.h> 

#define STEPPER_PIN1 9
#define STEPPER_PIN2 10
#define STEPPER_PIN3 11
#define STEPPER_PIN4 12
int step_number = 0;
int rot = 0;

char serialCommand = 'S';
int serialValue = 0;

Servo shoulder;

void setup() {
    pinMode(STEPPER_PIN1, OUTPUT);
    pinMode(STEPPER_PIN2, OUTPUT);
    pinMode(STEPPER_PIN3, OUTPUT);
    pinMode(STEPPER_PIN4, OUTPUT);

    shoulder.attach(3);
    shoulder.write(97);

    Serial.begin(115200); 
}

void loop() {
    readSerial();
    moveStepper();
    delay(calculateDelay());
}

void readSerial() {
    while (Serial.available() > 0) {
        char incoming = Serial.read();
        if (incoming >= '0' && incoming <= '9') {
            serialValue = (serialValue * 10) + (incoming - '0');
        } 
        else if (incoming == 'V') {
            serialValue = 0; 
        } 
        else if (incoming == '\n' || incoming == '\r') {
            shoulder.write(serialValue);
        }
        else if (incoming == 'Z') {
            zero();
            serialCommand = 'S';
        } 
        else if (incoming == 'H') {
            rot = 0;
        }
        else if (incoming == 'F' || incoming == 'B' || incoming == 'S') {
            serialCommand = incoming;
        }
    }
}

void moveStepper() {
    if (serialCommand == 'F') {
        OneStep(false);
    } 
    else if (serialCommand == 'B') {
        OneStep(true);
    }
}

double calculateDelay() {
    return 2;
}

void OneStep(bool dir) {
    if (dir) {
        if(rot >= 2048) {
            return;
        }
        step_number++;
        rot++;
        if (step_number > 3) {
            step_number = 0;
        }
    } else {
        if(rot <= -2048) {
            return;
        }
        step_number--;
        rot--;
        if (step_number < 0) {
            step_number = 3;
        }
    }
    switch (step_number) {
        case 0:
            digitalWrite(STEPPER_PIN1, HIGH);
            digitalWrite(STEPPER_PIN2, LOW);
                digitalWrite(STEPPER_PIN3, LOW);
                digitalWrite(STEPPER_PIN4, LOW);
                break;
            case 1:
                digitalWrite(STEPPER_PIN1, LOW);
                digitalWrite(STEPPER_PIN2, HIGH);
                digitalWrite(STEPPER_PIN3, LOW);
                digitalWrite(STEPPER_PIN4, LOW);
                break;
            case 2:
                digitalWrite(STEPPER_PIN1, LOW);
                digitalWrite(STEPPER_PIN2, LOW);
                digitalWrite(STEPPER_PIN3, HIGH);
                digitalWrite(STEPPER_PIN4, LOW);
                break;
            case 3:
                digitalWrite(STEPPER_PIN1, LOW);
                digitalWrite(STEPPER_PIN2, LOW);
                digitalWrite(STEPPER_PIN3, LOW);
                digitalWrite(STEPPER_PIN4, HIGH);
                break;
    }
}

void zero() {
    shoulder.write(90);
    while (rot != 0) {
        if (rot > 0) {
            OneStep(false);
        } else {
            OneStep(true);
        }
        delay(2);
    }
}