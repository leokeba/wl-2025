#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <filters.h>

#include <BMP180I2C.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

#define I2C_ADDRESS 0x77

//create an BMP180 object using the I2C interface
BMP180I2C bmp180(I2C_ADDRESS);

const uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
// C8:C9:A3:C9:BA:74
const uint8_t receiverAddress[] = {0xC8, 0xC9, 0xA3, 0xC9, 0xBA, 0x74};

// 0C:B8:15:F6:52:CC
// const uint8_t receiverAddress[] = {0x0C, 0xB8, 0x15, 0xF6, 0x52, 0xCC};

// #define receiverAddress broadcastAddress

// Structure to hold message data
typedef struct {
    int messageId;
    uint8_t dmxData[128];
    float pressure;
    float accX;
    float accY;
    float accZ;
} MessageData;

// Callback function when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("Last Packet Send Status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Callback function when data is received
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
    MessageData receivedData;
    memcpy(&receivedData, data, sizeof(receivedData));

    digitalWrite(LED_BUILTIN, receivedData.messageId % 2);  // Turn on LED to indicate data received
    
    Serial.print("Message ID: ");
    Serial.println(receivedData.messageId);
    Serial.print("Pressure: ");
    Serial.println(receivedData.pressure);
    Serial.print("Acceleration: x=");
	Serial.print(receivedData.accX);
	Serial.print(", y=");
	Serial.print(receivedData.accY);
	Serial.print(", z=");
	Serial.println(receivedData.accZ);

    static float lastPressure = 0.0;
    float currentPressure = receivedData.pressure;
    float pressureChange = currentPressure - lastPressure;
    lastPressure = currentPressure;

	Serial.print("> Pressure: "); 
	Serial.print(pressureChange);
	Serial.println(" Pa");
}

void setup() {
    Serial.begin(115200);
    Wire.begin(4, 15);
    
    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Configure for Long Range
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);

    // Register callbacks
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);

    // Register peer (broadcast address)
    esp_now_peer_info_t peerInfo = {};
    // memset(&peerInfo, 0, sizeof(peerInfo));
    // for (int i = 0; i < 6; i++) {
    //     peerInfo.peer_addr[i] = 0xFF;  // Broadcast address
    // }
    memcpy(peerInfo.peer_addr, receiverAddress, sizeof(receiverAddress));
    peerInfo.ifidx = WIFI_IF_STA;  // Use the station interface
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    // Add peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }

    if(!accel.begin())
    {
        /* There was a problem detecting the ADXL345 ... check your connections */
        Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
        while(1);
    }
    
    //begin() initializes the interface, checks the sensor ID and reads the calibration parameters.  
	if (!bmp180.begin())
	{
		Serial.println("begin() failed. check your BMP180 Interface and I2C Address.");
		while (1);
	}

	//reset sensor to default parameters.
	bmp180.resetToDefaults();

	//enable ultra high resolution mode for pressure measurements
	bmp180.setSamplingMode(BMP180MI::MODE_UHR);

    pinMode(LED_BUILTIN, OUTPUT);
}

MultiChannelRandomOscillator oscillators = MultiChannelRandomOscillator(54, 0.01, 0.1, 0., 0.5);
const int chanRepeat = 1;

RGBChaser rgbChaser(18);  // Create an RGB chaser with 18 fixtures

void loop() {
    static int messageCount = 0;
    MessageData data;

    /* Get a new sensor event */ 
    sensors_event_t event; 
    accel.getEvent(&event);
    	if (!bmp180.measurePressure())
	{
		Serial.println("could not start perssure measurement, is a measurement already running?");
		return;
	}

	//wait for the measurement to finish. proceed as soon as hasValue() returned true. 
	do
	{
		delay(100);
	} while (!bmp180.hasValue());

    float pressure = bmp180.getPressure();

    static SimpleExponentialFilter pressureFilter = SimpleExponentialFilter(0.1);  // Create a filter with alpha = 0.1
    double filteredPressure = pressureFilter.filter(pressure);

    static NumericalDerivativeFilter pressureDerivativeFilter = NumericalDerivativeFilter(1.0);  // Create a derivative filter with dt = 1 second
    double pressureDerivative = pressureDerivativeFilter.filter(pressure);

    static SimpleExponentialFilter derivativeFilter = SimpleExponentialFilter(0.1);  // Create a filter for the derivative with alpha = 0.1
    double filteredDerivative = derivativeFilter.filter(pressureDerivative);

    static NumericalDerivativeFilter secondDerivativeFilter = NumericalDerivativeFilter(1.0);  // Create a second derivative filter with dt = 1 second
    double secondDerivative = secondDerivativeFilter.filter(filteredDerivative);

    Serial.println("> FilteredPressure: " + String(filteredPressure));
    Serial.println("> PressureDerivative: " + String(pressureDerivative));
    Serial.println("> FilteredDerivative: " + String(filteredDerivative));
    Serial.println("> SecondDerivative: " + String(secondDerivative));
    Serial.println("> Pressure: " + String(pressure));

    float totalAcceleration = sqrt(event.acceleration.x * event.acceleration.x +
                              event.acceleration.y * event.acceleration.y +
                              event.acceleration.z * event.acceleration.z);

    float pitch = atan2(event.acceleration.y, event.acceleration.z) * 180.0 / PI;  // Convert to degrees
    float roll = atan2(event.acceleration.x, event.acceleration.z) * 180.0 / PI;   // Convert to degrees
    roll = fmod(roll - 90.0, 360.0) - 180.;  // Ensure roll is in the range [0, 360)
    // pitch = fmod(pitch + 90.0, 360.0) - 180.;  // Ensure pitch is in the range [0, 360)

    // detect free fall
    if (totalAcceleration < 5.) {
        Serial.println("> Free fall detected!");
        oscillators.randomizePhases();
        oscillators.randomizeFrequencies();  // Randomize frequencies of oscillators
        oscillators.randomizeAmplitudes();  // Randomize amplitudes of oscillators
    }
    
    static SimpleExponentialFilter accelerationFilter = SimpleExponentialFilter(0.1);  // Create a filter with alpha = 0.1
    double filteredAcceleration = accelerationFilter.filter(totalAcceleration);

    // Serial.println("> Filtered Acceleration: " + String(filteredAcceleration));
    // Serial.println("> Total Acceleration: " + String(totalAcceleration));
    // Serial.println("> Pitch: " + String(pitch));
    // Serial.println("> Roll: " + String(roll));

    uint8_t dmxValue = constrain(abs(filteredDerivative)*2., 0, 255);  // Constrain the value to fit in DMX range (0-255)

    oscillators.setFrequencyFactor(abs(roll / 180.0)*10.);  // Scale frequency factor based on roll angle  
    oscillators.setAmplitudeFactor(abs(pitch / 180.0));  // Scale amplitude factor based on pitch angle

    auto oscValues = oscillators.nextSample();  // Update oscillators for the next sample

    rgbChaser.setFrequency(dmxValue / 127. + 0.2);
    rgbChaser.setSineWidth(abs(roll / 180.0) * 0.5);
    rgbChaser.setSinePeriod(abs(pitch / 180.0) * 1. + 1.);

    rgbChaser.chase();  // Update the RGB chaser for the next frame
    auto rgbValues = rgbChaser.getDMXValues();  // Get the next RGB values from the chaser

    // Fill the DMX data with RGB values
    for (size_t i = 0; i < rgbValues.size(); i++) {
        data.dmxData[i * 3] = rgbValues[i][0];     // Red
        data.dmxData[i * 3 + 1] = rgbValues[i][1]; // Green
        data.dmxData[i * 3 + 2] = rgbValues[i][2]; // Blue
        // teleplot
        // Serial.println("> R" + String(i) + ": " + String(rgbValues[i][0]));
        // Serial.println("> G" + String(i) + ": " + String(rgbValues[i][1]));
        // Serial.println("> B" + String(i) + ": " + String(rgbValues[i][2]));
    }

    for (size_t i = 0; i < oscValues.size() * chanRepeat; i++) {
        // uint8_t oscValue = oscValues[i/chanRepeat] * 255.0;  // Scale oscillator values to fit in DMX range
        // data.dmxData[i] = constrain(oscValue, 0, 255);  // Scale oscillator values to fit in DMX range
        // Serial.print("> Oscillator");
        // Serial.print(i);
        // Serial.print(": ");
        // Serial.println(data.dmxData[i]);
    }

    // copy the dmxValue to the dmxData array
    for (int i = oscValues.size() - 1; i < 128; i++) {
        // data.dmxData[i] = dmxValue;  // Fill the DMX data with the same value for simplicity
    }

    // Prepare data to send
    data.messageId = messageCount++;
    data.pressure = pressure;
    data.accX = event.acceleration.x;
    data.accY = event.acceleration.y;
    data.accZ = event.acceleration.z;
    // Serial.print("Message ID: ");
    // Serial.println(data.messageId);
    // Serial.print("Acceleration: x=");
    // Serial.print(data.accX);
    // Serial.print(", y=");
    // Serial.print(data.accY);
    // Serial.print(", z=");
    // Serial.println(data.accZ);
    // Serial.print("Pressure: ");
    // Serial.println(data.pressure);

    // static float lastPressure = 0.0;
    // float currentPressure = bmp180.getPressure();
    // float pressureChange = currentPressure - lastPressure;
    // lastPressure = currentPressure;

	// Serial.print("> Pressure: "); 
	// Serial.print(pressureChange);
	// Serial.println(" Pa");

    // // Send message
    esp_err_t result = esp_now_send(receiverAddress, (uint8_t *)&data, sizeof(MessageData));
    
    // if (result == ESP_OK) {
    //     Serial.println("Message sent successfully");
    // } else {
    //     Serial.println("Error sending message");
    // }

    // digitalWrite(LED_BUILTIN, messageCount % 2);  // Turn on LED to indicate sending

    // delay(250);  // Wait 5 seconds before next transmission
}