#include <HTTPClient.h>
#include <Arduino.h>
#if defined ARDUINO_ARCH_ESP8266
#include <ESP8266WiFi.h>
#elif defined ARDUINO_ARCH_ESP32
#include <WiFi.h>
#else
#error Wrong platform
#endif 

#include <WifiLocation.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// #include <IRremote.h>
// #define IR_RECEIVE_PIN 2

#include <LiquidCrystal_I2C.h>
int lcdColumns = 16;
int lcdRows = 2;
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);  

Adafruit_MPU6050 mpu;

const char* googleApiKey = "AIzaSyByy9mROHxlLmc2v_oforADV6nvOpq5X5w";

const char* ssid = "DomainNorthgate_Resident_SS";               // Your Wi-Fi SSID
const char* password = "Shaan123!";       // Your Wi-Fi Password
const char* ntfyTopic = "JM4j2e0yT6akacaQ";    // ntfy.sh topic name

WifiLocation location (googleApiKey);

bool on = false;

void setClock () {
    configTime (0, 0, "pool.ntp.org", "time.nist.gov");

    Serial.print ("Waiting for NTP time sync: ");
    time_t now = time (nullptr);
    while (now < 8 * 3600 * 2) {
        delay (500);
        Serial.print (".");
        now = time (nullptr);
    }
    struct tm timeinfo;
    gmtime_r (&now, &timeinfo);
    Serial.print ("\n");
    Serial.print ("Current time: ");
    Serial.print (asctime (&timeinfo));
}



void setup() {
    // Serial.begin(115200);

    // IrReceiver.begin(IR_RECEIVE_PIN);

    // Connect to Wi-Fi
    // initialize LCD
    lcd.init();
    // turn on LCD backlight                      
    lcd.backlight();
    lcd.setCursor(0, 0);

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        lcd.setCursor(0, 0);
        lcd.print("Connecting to Wifi");
        delay(1000);
        Serial.println("Connecting to WiFi...");
        //LCD output
        lcd.setCursor(0, 0);
        lcd.print("Connecting to WiFi.");

    }

    Serial.println("Connected to WiFi");
    
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Connected to WiFi");

    setClock ();
    location_t loc = location.getGeoFromWiFi();
    Serial.println ("Location: " + String (loc.lat, 7) + "," + String (loc.lon, 7));
    Serial.println ("Accuracy: " + String (loc.accuracy));

    while (!Serial)
        delay(10); // will pause Zero, Leonardo, etc until serial console opens

    Serial.println("Adafruit MPU6050 test!");

    // Try to initialize!
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) {
            delay(10);
        }
    }
    Serial.println("MPU6050 Found!");

    mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
    mpu.setMotionDetectionThreshold(1);
    mpu.setMotionDetectionDuration(20);
    mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
    mpu.setInterruptPinPolarity(true);
    mpu.setMotionInterrupt(true);

    Serial.println("");
    delay(100);
  

}

void sendTestNotification() {
    Serial.begin(115200);

    if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;

        // URL for your ntfy topic
        String serverPath = "https://ntfy.sh/" + String(ntfyTopic);

        // Send notification request
        http.begin(serverPath);
        http.addHeader("Content-Type", "text/plain");

        setClock ();
        location_t loc = location.getGeoFromWiFi();
        // Notification message
        String message = ("Location: " + String (loc.lat, 7) + "," + String (loc.lon, 7));

        int httpResponseCode = http.POST(message);

        if (httpResponseCode > 0) {
            Serial.print("Notification sent, response code: ");
            Serial.println(httpResponseCode);
            String response = http.getString();
            Serial.println(response);  // Print server response
        } else {
            Serial.print("Error sending notification, code: ");
            Serial.println(httpResponseCode);
        }

        http.end();
    } else {
        Serial.println("WiFi not connected");
    }

    
}

void loop() {

    
    // if (IrReceiver.decode()) {
    //     IrReceiver.resume();
    //     Serial.println(IrReceiver.decodedIRData.command);
    // }

   
    if(mpu.getMotionInterruptStatus()) {
        /* Get new sensor events with the readings */
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        /* Print out the values */
        Serial.print("AccelX:");
        Serial.print(a.acceleration.x);
        Serial.print(",");
        Serial.print("AccelY:");
        Serial.print(a.acceleration.y);
        Serial.print(",");
        Serial.print("AccelZ:");
        Serial.print(a.acceleration.z);
        Serial.print(", ");
        Serial.print("GyroX:");
        Serial.print(g.gyro.x);
        Serial.print(",");
        Serial.print("GyroY:");
        Serial.print(g.gyro.y);
        Serial.print(",");
        Serial.print("GyroZ:");
        Serial.print(g.gyro.z);
        Serial.println("");

        sendTestNotification();
    }
  

  // sendTestNotification();
    delay(10);  // Wait 10 seconds between notifications
}