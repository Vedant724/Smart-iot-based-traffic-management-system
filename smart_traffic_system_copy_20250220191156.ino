#define ARDUINO_ARCH_ESP32 
 
// Define sensor and LED pins for each direction 
#define SENSOR_NORTH 32 
#define SENSOR_SOUTH 34 
#define SENSOR_EAST 35 
#define SENSOR_WEST 15 
#define RED_NORTH 5 
#define YELLOW_NORTH 4 
#define GREEN_NORTH 2  
#define RED_SOUTH 33 
#define YELLOW_SOUTH 25 
#define GREEN_SOUTH 26 
#define RED_EAST 27 
#define YELLOW_EAST 14 
#define GREEN_EAST 12 
#define RED_WEST 22 
#define YELLOW_WEST 21 
#define GREEN_WEST 19 
 
// Traffic Signal States 
#define RED 0 
#define YELLOW 1 
#define GREEN 2 
 
// Traffic Signal Timings (in milliseconds) 
#define GREEN_TIME 10000  // 10 seconds 
#define YELLOW_TIME 2000  // 2 seconds 
#define MIN_SENSOR_THRESHOLD 50 
#define MAX_SENSOR_THRESHOLD 200 
 
int northState = GREEN; 
int southState = RED; 
int eastState = RED; 
16 
 
int westState = RED; 
 
unsigned long lastTime; 
unsigned long currentTime; 
unsigned long greenDuration = GREEN_TIME; 
unsigned long yellowDuration = YELLOW_TIME; 
 
void setup() { 
    pinMode(SENSOR_NORTH, INPUT); 
    pinMode(SENSOR_SOUTH, INPUT); 
    pinMode(SENSOR_EAST, INPUT); 
    pinMode(SENSOR_WEST, INPUT); 
    pinMode(RED_NORTH, OUTPUT); 
    pinMode(YELLOW_NORTH, OUTPUT); 
    pinMode(GREEN_NORTH, OUTPUT); 
    pinMode(RED_SOUTH, OUTPUT); 
    pinMode(YELLOW_SOUTH, OUTPUT); 
    pinMode(GREEN_SOUTH, OUTPUT); 
    pinMode(RED_EAST, OUTPUT); 
    pinMode(YELLOW_EAST, OUTPUT); 
    pinMode(GREEN_EAST, OUTPUT); 
    pinMode(RED_WEST, OUTPUT); 
    pinMode(YELLOW_WEST, OUTPUT); 
    pinMode(GREEN_WEST, OUTPUT); 
 
    // Set pin 35 as OUTPUT (for ground) 
    pinMode(35, OUTPUT); 
    digitalWrite(35, LOW); 
 
    Serial.begin(9600); 
    lastTime = millis(); 
} 
 
void loop() { 
    currentTime = millis(); 
    unsigned long elapsedTime = currentTime - lastTime; 
 
    // Read sensor inputs 
    int sensorNorth = readSensor(SENSOR_NORTH); 
    int sensorSouth = readSensor(SENSOR_SOUTH); 
    int sensorEast = readSensor(SENSOR_EAST); 
    int sensorWest = readSensor(SENSOR_WEST); 
 
    // Calculate traffic density based on sensor readings 
    int densityNorth = calculateDensity(sensorNorth); 
17 
 
    int densitySouth = calculateDensity(sensorSouth); 
    int densityEast = calculateDensity(sensorEast); 
    int densityWest = calculateDensity(sensorWest); 
 
    // Update traffic signal states based on traffic density and timing 
    updateTrafficSignal(RED_NORTH, YELLOW_NORTH, GREEN_NORTH, &northState, 
densityNorth, southState, eastState, westState); 
    updateTrafficSignal(RED_SOUTH, YELLOW_SOUTH, GREEN_SOUTH, &southState, 
densitySouth, northState, eastState, westState); 
    updateTrafficSignal(RED_EAST, YELLOW_EAST, GREEN_EAST, &eastState, 
densityEast, northState, southState, westState); 
    updateTrafficSignal(RED_WEST, YELLOW_WEST, GREEN_WEST, &westState, 
densityWest, northState, southState, eastState); 
 
    // Print current traffic light states and time delays 
    Serial.println("---- Traffic Light Status ----"); 
    printTrafficLightStatus("North", northState, greenDuration, yellowDuration); 
    printTrafficLightStatus("South", southState, greenDuration, yellowDuration); 
    printTrafficLightStatus("East", eastState, greenDuration, yellowDuration); 
    printTrafficLightStatus("West", westState, greenDuration, yellowDuration); 
    Serial.println("-----------------------------"); 
 
    // Debugging information 
    Serial.print("North Sensor Value: "); 
    Serial.println(sensorNorth); 
    Serial.print("South Sensor Value: "); 
    Serial.println(sensorSouth); 
    Serial.print("East Sensor Value: "); 
    Serial.println(sensorEast); 
    Serial.print("West Sensor Value: "); 
    Serial.println(sensorWest); 
 
    delay(1000); 
} 
 
int readSensor(int sensorPin) { 
    int sensorValue = analogRead(sensorPin); 
    if (sensorValue < 0 || sensorValue > 4095) { 
        return 0; // If sensor reading is out of range, return default value 
    } 
 
    // Check if sensor pin is not connected (low sensor value) 
    if (sensorValue < 100) { 
        return 0; 
    } 
18 
 
 
    return sensorValue; 
} 
 
int calculateDensity(int sensorValue) { 
    // Map sensor value to a percentage of maximum reading 
    return map(sensorValue, 0, 4095, 0, 100); 
} 
 
void updateTrafficSignal(int redPin, int yellowPin, int greenPin, int* state, int 
density, int otherState1, int otherState2, int otherState3) { 
    switch (*state) { 
        case RED: 
            digitalWrite(redPin, HIGH); 
            digitalWrite(yellowPin, LOW); 
            digitalWrite(greenPin, LOW); 
            if (density < MIN_SENSOR_THRESHOLD && otherState1 == RED && 
otherState2 == RED && otherState3 == RED) { 
                *state = GREEN; 
            } 
            break; 
        case YELLOW: 
            digitalWrite(redPin, LOW); 
            digitalWrite(yellowPin, HIGH); 
            digitalWrite(greenPin, LOW); 
            if (millis() - lastTime >= yellowDuration) { 
                *state = RED; 
                lastTime = millis(); 
            } 
            break; 
        case GREEN: 
            digitalWrite(redPin, LOW); 
            digitalWrite(yellowPin, LOW); 
            digitalWrite(greenPin, HIGH); 
            if (density > MAX_SENSOR_THRESHOLD || millis() - lastTime >= 
greenDuration) { 
                *state = YELLOW; 
                lastTime = millis(); 
            } 
            break; 
        default: 
            break; 
    } 
} 
 
19 
 
void printTrafficLightStatus(String direction, int state, unsigned long 
greenTime, unsigned long yellowTime) { 
    String stateStr; 
    switch (state) { 
        case RED: 
            stateStr = "RED"; 
            break; 
        case YELLOW: 
            stateStr = "YELLOW"; 
            break; 
        case GREEN: 
            stateStr = "GREEN"; 
            break; 
        default: 
            stateStr = "UNKNOWN"; 
            break; 
    } 
    Serial.print(direction); 
    Serial.print(" Light State: "); 
    Serial.print(stateStr); 
    Serial.print(" - "); 
    if (state == GREEN) { 
        Serial.print("Remaining Time: "); 
        Serial.print(greenTime - (millis() - lastTime)); 
    } else if (state == YELLOW) { 
        Serial.print("Remaining Time: "); 
        Serial.print(yellowTime - (millis() - lastTime)); 
    } 
    Serial.println(" ms");