#include <SoftwareSerial.h>
#include<ESP8266Manager.h>
#define DEBUG true


// ----- ----- ----- ----- -----  CONSTANTS FOR THE MOTORS/SENDORS MANAGEMENT
const long MAX_MOVEMENT_DELAY_IN_MILLIS = 60000;

// MOTOR BLOCKER PINS
const int MOTOR_BLOCKER_CONTROL_PIN_1 = 2;
const int MOTOR_BLOCKER_CONTROL_PIN_2 = 3;
const int MOTOR_BLOCKER_ENNABLE_PIN = 4;

// MOTOR LOCK PINS
const int MOTOR_LOCK_CONTROL_PIN_1 = 5;
const int MOTOR_LOCK_CONTROL_PIN_2 = 6;
const int MOTOR_LOCK_ENNABLE_PIN = 7;

// SESORS LOCK
const int SENSOR_LOCK_OPEN = A2;
const int SENSOR_LOCK_CLOSED = A3;

// SENSORS PIN
const int SENSOR_PIN_OPEN = A4;
const int SENSOR_PIN_CLOSED = A5;

// DEFINE COMPONENT STATE
const int COMPONENT_STATE_OPEN = 0;
const int COMPONENT_STATE_CLOSE = 1;
const int COMPONENT_STATE_INVALID = -1;

// DEFINE STATES
const int DOOR_LOCK_STATE_IDLE = 0;
const int DOOR_LOCK_STATE_OPEN = 1;
const int DOOR_LOCK_STATE_UNLOCKED = 2;
const int DOOR_LOCK_STATE_LOCKED = 3;
const int DOOR_LOCK_STATE_MOVING = 4;
const int DOOR_LOCK_STATE_INVALID = -1;

const int MAGNET_DETCTED_MAX_VALUE = 150;


// ----- ----- ----- ----- -----  CONSTANTS FOR THE WIFI MANAGEMENT
const int CONNECTION_ATTEMPTS_LIMIT = 5;  // In movement

const String SSID = "LEXRULES";
const String PASSWORD = "lexrules";
const String HOST_NAME = "192.168.1.70";

// ----- ----- ----- ----- -----  VARIABLE MANAGEMENT
// DEFINE COMPONENTS
int lock = COMPONENT_STATE_CLOSE;
int pin = COMPONENT_STATE_CLOSE;

int doorLock = DOOR_LOCK_STATE_IDLE;

const int baudRate = 9600;
ESP8266Manager espManager(baudRate, 12, 13);

// ----- ----- ----- ----- -----  LIFECYCLE METHODS
void setup() {
    Serial.begin(baudRate);
    Serial.println(F("SETUP BEGIN"));

    // ENNABLE MOTOR BLOCKER PINS
    pinMode(MOTOR_BLOCKER_CONTROL_PIN_1, OUTPUT);
    pinMode(MOTOR_BLOCKER_CONTROL_PIN_2, OUTPUT);
    pinMode(MOTOR_BLOCKER_ENNABLE_PIN, OUTPUT);

    // ENNABLE MOTOR LOCK PINS
    pinMode(MOTOR_LOCK_CONTROL_PIN_1, OUTPUT);
    pinMode(MOTOR_LOCK_CONTROL_PIN_2, OUTPUT);
    pinMode(MOTOR_LOCK_ENNABLE_PIN, OUTPUT);

    pinMode(SENSOR_LOCK_OPEN, INPUT); //set sensor pin as input
    pinMode(SENSOR_LOCK_CLOSED, INPUT); //set sensor pin as input
  
    pinMode(SENSOR_PIN_OPEN, INPUT); //set sensor pin as input
    pinMode(SENSOR_PIN_CLOSED, INPUT); //set sensor pin as input

    digitalWrite(MOTOR_BLOCKER_ENNABLE_PIN, LOW);

    // Validating if wifi module is alive
    String response;
    do {
        println("PINGING WIFI MODULE");
        response = espManager.writeESP8266("AT\r\n", 6000);
    } while (!espManager.findText("OK", response));
    println("WIFI MODULE OK");

    unBlockDoorLock();
}

void loop() {
    // Update the lock state based on the sensors read
    updateDoorLockState();

    // Update the lock state based on the server state
    int state = getServerLockState();
    if (state != doorLock) {
        printState(state);
        switchState(state);
    } else {
        print(F("NO UPDATE REQUIRED"));
    }

    delay(5000);
}

// ----- ----- ----- ----- -----  STATE
void printState(int state) {
    switch(state) {
        case DOOR_LOCK_STATE_OPEN:
            println(F("DOOR_LOCK_STATE_OPEN"));
        break;
        case DOOR_LOCK_STATE_UNLOCKED:
            println(F("DOOR_LOCK_STATE_UNLOCKED"));
        break;
        case DOOR_LOCK_STATE_LOCKED:
            println(F("DOOR_LOCK_STATE_LOCKED"));
        break;
        case DOOR_LOCK_STATE_MOVING:
            println(F("DOOR_LOCK_STATE_MOVING"));
        break;
        case DOOR_LOCK_STATE_INVALID:
            println(F("DOOR_LOCK_STATE_INVALID"));
        break;
        default:
            println(F("DOOR_LOCK_STATE_INVALID"));
        break;
    }
}


// ----- ----- ----- ----- -----  CLOUD MANAEMENT
int getServerLockState() {
    int attempts = 0;
    boolean connected;

    // Validate the connection status
    do {
        if (attempts > CONNECTION_ATTEMPTS_LIMIT) {
            println("ABORT WS CALL");
            return DOOR_LOCK_STATE_INVALID;
        }

        println(F("VALIDATING CONNECTION"));
        attempts++;
        connected = isConnected();

        if (!connected) {
            println(F("NO AP, TRYIN TO CONNECT"));
            connected = connect();
        }

    } while (!connected);

    println(F("CONNECTION OK"));

    return getServerState();
}

int getServerState() {
    Serial.println(F("UPDATING STATE"));
    
    String response = espManager.writeESP8266("AT+CIPSTART=\"TCP\",\"" + HOST_NAME + "\",8080\r\n", 15000); 

    if (espManager.findText(F("OK"), response)) {
        Serial.println(F("CONNECTED TO THE SERVER"));
    } else if (espManager.findText(F("ALREADY CONNECTED"), response)) {
        espManager.writeESP8266(F("AT+CIPCLOSE\r\n"), 5000);
        return DOOR_LOCK_STATE_INVALID;
    } else {
        Serial.println(F("CONNECTION IMPOSSIBLE"));
        return DOOR_LOCK_STATE_INVALID;
    }

    Serial.println(F("SENDING DATA LENGTH"));
    response = espManager.writeESP8266(F("AT+CIPSEND=56\r\n"), 10000); 

    if (!espManager.findText(F(">"), response)) {
        Serial.println(F("ERROR IN CONNECTION"));
        espManager.writeESP8266(F("AT+CIPCLOSE\r\n\r\n"), 500); 
        return DOOR_LOCK_STATE_INVALID;
    }

    Serial.println(F("CONNECTED TO THE SERVER, SENDING THE PATH"));
    espManager.writeESP8266(F("GET /RaspberryHouse/rest/LockService/state\r\n"), 10); 
    response = espManager.writeESP8266(F("HTTP/1.1\r\n\r\n"), 10000);
    if (response.length() > 0) {
        if (espManager.findText(F("-1CLOSED"), response)) {
            Serial.println(F("-- INVALID POSITION --"));
            return DOOR_LOCK_STATE_INVALID;
        } else if (espManager.findText(F("1CLOSED"), response)) {
            Serial.println(F("-- OPEN --"));
            return DOOR_LOCK_STATE_OPEN;
        } else if (espManager.findText(F("2CLOSED"), response)) {
            Serial.println(F("-- UNLOCKED --"));
            return DOOR_LOCK_STATE_UNLOCKED;
        } else if (espManager.findText(F("3CLOSED"), response)) {
            Serial.println(F("-- LOCKED --"));
            return DOOR_LOCK_STATE_LOCKED;
        } else if(espManager.findText(F("ERROR"), response)) {
            Serial.println(F("-- INVALID RESPONSE --"));
            return DOOR_LOCK_STATE_INVALID;
        }
    }

    Serial.println(F("-- NO ANSWER --"));
    return DOOR_LOCK_STATE_INVALID;
}

bool isConnected() {
    println(F("IS CONNECTED TO ACCESS POINT ?"));
    String response = espManager.writeESP8266(F("AT+CWJAP?\r\n"), 5000); 

    if (espManager.findText(F("OK"), response)) {
        if (espManager.findText(F("NO AP"), response)) {
            println(F("NO, IT'S NOT CONNECTED"));
            return false;
        } else {
            println(F("YES, IT'S CONNECTED"));
            return true;
        }
    } else {
        println(F("DON'T KNOW IF IT'S CONNECTED"));
        return false;
    }
}

bool connect() {
    print(F("CONNECTING TO ACCESS POINT: "));
    println("AT+CWJAP=\"" + SSID + "\",\"" + PASSWORD + "\"\r\n");

    String response = espManager.writeESP8266("AT+CWJAP=\"" + SSID + "\",\"" + PASSWORD + "\"\r\n", 500);
    
    delay(5000);
    return isConnected();
}

bool disConnect(){
    println(F("DISCONNECTING FROM ACCESS POINT"));
    String response = espManager.writeESP8266(F("AT+CWQAP\r\n"), 500);   
    delay(1000);
    if (espManager.findText(F("OK"), response)) {
        return true;
    } else {
        return false;
    }
}

// ----- ----- ----- ----- -----  DOOR LOCK STATE WRITE
boolean switchState(int targetState) {
    if (targetState < DOOR_LOCK_STATE_OPEN || targetState > DOOR_LOCK_STATE_LOCKED) {
        return false;
    } else if (targetState == doorLock) {
        return true;
    } else {
        switch(targetState) {
            case DOOR_LOCK_STATE_LOCKED:
                println(F("SET DOOR LOCK STATE LOCKED"));
                break;
            case DOOR_LOCK_STATE_UNLOCKED:
                println(F("SET DOOR LOCK STATE UNLOCKED"));
                break;
            case DOOR_LOCK_STATE_OPEN:
                println(F("SET DOOR LOCK STATE OPEN"));
                break;
            case DOOR_LOCK_STATE_INVALID:
                println(F("SET DOOR LOCK STATE INVALID"));
                break;
            default:
                println(F("SET DOOR LOCK STATE INVALID"));
                break;
        }
        return moveTo(targetState);
    }
}

boolean moveTo(int targetState) {
    long elapsedTime = 0;
    int readDelay = 100;
    int doorLockState = getDoorLockState();

    if (!blockDoorLock()) {
        return false;
    }

    println(F("BLOCKER RELEASED"));

    // Moving the state of the doorlock
    // Activate de motor
    digitalWrite(MOTOR_LOCK_CONTROL_PIN_1, ((targetState > doorLock)? HIGH : LOW));
    digitalWrite(MOTOR_LOCK_CONTROL_PIN_2, ((targetState > doorLock)? LOW : HIGH));
    digitalWrite(MOTOR_LOCK_ENNABLE_PIN, HIGH);

    println(F("--- MOVING ---"));

    // Wait for the motor to reach the target state
    while ( doorLockState != targetState && elapsedTime < MAX_MOVEMENT_DELAY_IN_MILLIS) {
        // Update sensor read
        doorLockState = getDoorLockState();

        // Delay for the next read
        delay(readDelay);
        elapsedTime = elapsedTime + readDelay;
    }
    println(F("--- DONE ---"));

    // Stop the motor movement
    digitalWrite(MOTOR_LOCK_ENNABLE_PIN, LOW);

    if (!unBlockDoorLock()) {
        return false;
    }

    Serial.println("BLOCKER SETTED");

    return (doorLockState == targetState);
}

boolean unBlockDoorLock() {
    return moveBlocker(true);
}

boolean blockDoorLock() {
    return moveBlocker(false);
}

boolean moveBlocker(boolean toRelease) {
    //int sensorOpen1 = analogRead(motorSensorTargetPin);
    //int elapsedTime = 0;
    int readDelay = 1000;

    if (toRelease) {
        println(F("RELEASING BLOCKER"));
    } else {
        println(F("SETTING BLOCKER"));
    }

    // Activate de motor
    digitalWrite(MOTOR_BLOCKER_CONTROL_PIN_1, ((toRelease)? HIGH : LOW));
    digitalWrite(MOTOR_BLOCKER_CONTROL_PIN_2, ((toRelease)? LOW : HIGH));
    digitalWrite(MOTOR_BLOCKER_ENNABLE_PIN, HIGH);

    // Wait for the motor to reach the target state
    delay(readDelay);

    // Deactivate de motor
    digitalWrite(MOTOR_BLOCKER_ENNABLE_PIN, LOW);

    return true;
}


// ----- ----- ----- ----- -----  DOOR LOCK STATE READ
void updateLockState() {
    int sensorOpenLock = analogRead(SENSOR_LOCK_OPEN);
    int sensorClosedLock = analogRead(SENSOR_LOCK_CLOSED);
    if (sensorOpenLock < MAGNET_DETCTED_MAX_VALUE) {
        if (lock != COMPONENT_STATE_OPEN) {
            println(F("OPPENED LOCK GOTTEN"));
            lock = COMPONENT_STATE_OPEN;
        }
    } else if (sensorClosedLock < MAGNET_DETCTED_MAX_VALUE) {
        if (lock != COMPONENT_STATE_CLOSE) {
            println(F("CLOSED LOCK GOTTEN"));
            lock = COMPONENT_STATE_CLOSE;
        }
    } else {
        if (lock != COMPONENT_STATE_INVALID) {
            println(F("INVALID LOCK GOTTEN"));
            lock = COMPONENT_STATE_INVALID;
        }
    }
}

void updatePinState() {
    int sensorOpenPin = analogRead(SENSOR_PIN_OPEN);
    int sensorClosedPin = analogRead(SENSOR_PIN_CLOSED);
    if (sensorOpenPin < MAGNET_DETCTED_MAX_VALUE) {
        if (pin != COMPONENT_STATE_OPEN) {
            println(F("OPPENED PIN GOTTEN"));
            pin = COMPONENT_STATE_OPEN;
        }
    } else if (sensorClosedPin < MAGNET_DETCTED_MAX_VALUE) {
        if (pin != COMPONENT_STATE_CLOSE) {
            println(F("CLOSED PIN GOTTEN"));
            pin = COMPONENT_STATE_CLOSE;
        }
    } else {
        if (pin != COMPONENT_STATE_INVALID) {
            println(F("INVALID PIN GOTTEN"));
            pin = COMPONENT_STATE_INVALID;
        }
    }
}

void updateDoorLockState() {

    // Get the new door lock state
    int newState = getDoorLockState();

    // Validate if the state has changes
    if (doorLock != newState) {
        doorLock = newState;

        switch(doorLock) {
            case DOOR_LOCK_STATE_LOCKED:
                println(F("DOOR LOCK STATE LOCKED"));
                break;
            case DOOR_LOCK_STATE_UNLOCKED:
                println(F("DOOR LOCK STATE UNLOCKED"));
                break;
            case DOOR_LOCK_STATE_OPEN:
                println(F("DOOR LOCK STATE OPEN"));
                break;
            case DOOR_LOCK_STATE_INVALID:
                println(F("DOOR LOCK STATE INVALID"));
                break;
            default:
                println(F("DOOR LOCK STATE INVALID"));
                break;
        }
    }
}

int getDoorLockState() {
    // Update the sensor values
    //updateBlockerState();
    updateLockState();
    updatePinState();

    // Initiate the new state of the door
    int newState = DOOR_LOCK_STATE_INVALID;

    // Calculate the new value based on the sensor's read
    if (lock == COMPONENT_STATE_CLOSE) {
        newState = DOOR_LOCK_STATE_LOCKED;
    } else if (lock == COMPONENT_STATE_OPEN) {
        if (pin == COMPONENT_STATE_CLOSE) {
            newState = DOOR_LOCK_STATE_UNLOCKED;
        } else if (pin == COMPONENT_STATE_OPEN) {
            newState = DOOR_LOCK_STATE_OPEN;
        } else {
            newState = DOOR_LOCK_STATE_INVALID;
        }
    } else {
        newState = DOOR_LOCK_STATE_INVALID;
    }

    return newState;
}


// ----- ----- ----- ----- ----- LOG METHODS
void println(long log) {
    if (DEBUG) {
        Serial.println(log);
    }
}

void println(double log) {
    if (DEBUG) {
        Serial.println(log);
    }
}

void println(int log) {
    if (DEBUG) {
        Serial.println(log);
    }
}

void println(String log) {
    if (DEBUG) {
        Serial.println(log);
    }
}

void print(long log) {
    if (DEBUG) {
        Serial.print(log);
    }
}

void print(double log) {
    if (DEBUG) {
        Serial.print(log);
    }
}

void print(int log) {
    if (DEBUG) {
        Serial.print(log);
    }
}

void print(String log) {
    if (DEBUG) {
        Serial.print(log);
    }
}
