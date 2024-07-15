
#define sensorSerial Serial1
#define ANALOG_OUT_PIN A0

// Measured distance:
int16_t distance;

// Flag to indicate new data.
bool newData = false; 

// Serial data buffer:
uint8_t buffer[4];
uint8_t idx = 0;

void setup() {

    // Setup serial output.
    Serial.begin(115200);
    while (!Serial) {
        delay(10); 
    }

    Serial.println("Serial monitor ready");

    // Setup sensor serial port.
    sensorSerial.begin(9600);
}

void loop() { 

    // If we have a byte from the sensor...
    if (sensorSerial.available()) {

        // Read byte.
        uint8_t c = sensorSerial.read();
        Serial.println(c, HEX);

        // Header byte:
        if (idx == 0 && c == 0xFF) {
            buffer[idx++] = c;
        }

        // Data bytes:
        else if ((idx == 1) || (idx == 2)) {
            buffer[idx++] = c;
        }

        // Checksum byte:
        else if (idx == 3) {
            uint8_t sum = 0;
            sum = buffer[0] + buffer[1] + buffer[2];
            if (sum == c) {
                distance = ((uint16_t)buffer[1] << 8) | buffer[2];
                if (distance != 0) {
                    newData = true;
                }
            }
            idx = 0;
        }
    }
  
    // If a new data point was received...
    if (newData) {

        Serial.print("Distance: ");
        Serial.print(distance);
        Serial.println(" mm");

        if (distance > 1000) {
            distance = 1000;
        }

        // Normalize data point and output to analog pin.
        int32_t  tmp = (distance * 1023)  / 1000;
        analogWrite(ANALOG_OUT_PIN, tmp);

        Serial.print("AOut: ");
        Serial.println(tmp);

        newData = false;
    }
}
