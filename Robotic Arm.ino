#include <Servo.h> 

#define STEPPER_PIN1 9
#define STEPPER_PIN2 10
#define STEPPER_PIN3 11
#define STEPPER_PIN4 12
int step_number = 0;
int rot = 0;

char serialCommand = 'S';
int serialValue = 0;

#define ANALOG_X_PIN A2
#define ANALOG_Y_PIN A1 
#define ANALOG_BUTTON_PIN A5
#define ANALOG_X_CORRECTION 128
#define ANALOG_Y_CORRECTION 128 

#define POT_PIN A3
int val; 

Servo shoulder;

struct button { 
	byte pressed = 0; 
}; 

struct analog { 
	double x, y; 

    button button;
}; 


void setup() {
    pinMode(STEPPER_PIN1, OUTPUT);
    pinMode(STEPPER_PIN2, OUTPUT);
    pinMode(STEPPER_PIN3, OUTPUT);
    pinMode(STEPPER_PIN4, OUTPUT);

    pinMode(ANALOG_X_PIN, INPUT);
    pinMode(ANALOG_Y_PIN, INPUT);
    pinMode(ANALOG_BUTTON_PIN, INPUT_PULLUP); 

    shoulder.attach(3);
    shoulder.write(90);

    Serial.begin(115200); 
}

void loop() {
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
        else if (incoming == 'F' || incoming == 'B' || incoming == 'S') {
            serialCommand = incoming;
        }
    }
    
    analog log; 
	 
    log.x = map(analogRead(ANALOG_X_PIN), 0, 1023, 0, 255) - ANALOG_X_CORRECTION; 
    log.y = map(analogRead(ANALOG_Y_PIN), 0, 1023, 0, 255) - ANALOG_Y_CORRECTION; 
    log.button.pressed = isAnalogButtonPressed(ANALOG_BUTTON_PIN);

    if(log.button.pressed) {
        Serial.println("Zeroing");
        zero();
    }

    if (serialCommand == 'F' || log.x > 20) {
        OneStep(false);
    } 
    else if (serialCommand == 'B' || log.x < -20) {
        OneStep(true);
    }
    
    int speedValue = abs(log.x); 
    
    if (speedValue < 20 && serialCommand != 'S') {
        speedValue = 128;
    }

    double delayTime = 2.0 / (speedValue / 128.0 + 0.1);
    if (delayTime < 3) delayTime = 3;
    delay(delayTime);
}

bool isAnalogButtonPressed(int pin) 
{ 
	 return digitalRead(pin) == 0; 
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
        delay(3);
    }
}