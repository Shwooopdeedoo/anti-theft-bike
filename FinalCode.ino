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
    Serial.begin(115200);

    // IrReceiver.begin(IR_RECEIVE_PIN);

    // Connect to Wi-Fi
    // initialize LCD
    lcd.init();
    // turn on LCD backlight                      
    lcd.backlight();

    lcd.clear();
    

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
       //LCD output
        
        lcd.setCursor(0, 0);
        lcd.print("WiFi: Connecting");


        lcd.setCursor(0,1);
        lcd.print("[>>>           ]");
        delay(300);
      
        lcd.setCursor(0, 1);
        lcd.print("[ >>>          ]");
        delay(300);
        
        lcd.setCursor(0, 1);
        lcd.print("[  >>>         ]");
        delay(300);

        lcd.setCursor(0, 1);
        lcd.print("[   >>>        ]");
        delay(300);

        lcd.setCursor(0, 1);
        lcd.print("[    >>>       ]");
        delay(300);

        lcd.setCursor(0, 1);
        lcd.print("[     >>>      ]");
        delay(300);

        lcd.setCursor(0, 1);
        lcd.print("[      >>>     ]");
        delay(300);

        lcd.setCursor(0, 1);
        lcd.print("[       >>>    ]");
        delay(300);

        lcd.setCursor(0, 1);
        lcd.print("[        >>>   ]");
        delay(300);

        lcd.setCursor(0, 1);
        lcd.print("[         >>>  ]");
        delay(300);

        lcd.setCursor(0, 1);
        lcd.print("[          >>> ]");
        delay(300);

        lcd.setCursor(0, 1);
        lcd.print("[           >>>]");
        delay(300);

        lcd.setCursor(0, 1);
        lcd.print("[>           >>]");
        delay(300);

        lcd.setCursor(0, 1);
        lcd.print("[>>           >]");
        delay(300);

        Serial.println("Connecting to WiFi...");

    }

    Serial.println("Connected to WiFi");
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WiFi: Connecting");
    lcd.setCursor(0, 1);
    lcd.print("Connected!");
    delay(1500);

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("WiFi: Connected");

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
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);  // Adjust based on your needs
    mpu.setGyroRange(MPU6050_RANGE_250_DEG); // Set gyroscope range (default is ±250 degrees per second)
    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);   // Set the data rate

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

bool detectMovement(float movementThreshold) {
  // Get accelerometer and gyroscope events
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  // Print accelerometer data (in m/s²)
  Serial.print("Accel X: "); Serial.print(accel.acceleration.x); Serial.print(" ");
  Serial.print("Accel Y: "); Serial.print(accel.acceleration.y); Serial.print(" ");
  Serial.print("Accel Z: "); Serial.println(accel.acceleration.z);

  // Convert m/s² to Gs (1G = 9.81 m/s²)
  float accelX = accel.acceleration.x / 9.81;
  float accelY = accel.acceleration.y / 9.81;
  float accelZ = accel.acceleration.z / 9.81;

  // Check if movement exceeds the threshold on any axis
  if (abs(accelX) > movementThreshold || 
      abs(accelY) > movementThreshold || 
      abs(accelZ) > movementThreshold) {
    return true;  // Movement detected
  } else {
    return false;  // No movement detected
  }
    delay(100);
}

void loop() {

    
    // if (IrReceiver.decode()) {
    //     IrReceiver.resume();
    //     Serial.println(IrReceiver.decodedIRData.command);
    // }

   
    if(detectMovement(1.0)) {
        /* Get new sensor events with the readings */

        delay(2000);

        if(detectMovement(1.0)) {
            
            Serial.print("Movement Detected!");

            lcd.clear();
            lcd.setCursor(0,0);
            lcd.print("WiFi: Connected");
            lcd.setCursor(0,1);
            lcd.print("Motion Detected!");
            delay(3500);
            lcd.clear();
            lcd.setCursor(0,0);
            lcd.print("WiFi: Connected");
            sendTestNotification();
        }
        else {
            Serial.println("False Alarm!");
            Serial.println("");

            lcd.setCursor(0,1);
            lcd.print("False Alarm!");
            delay(3500);
            lcd.clear();
            lcd.setCursor(0,0);
            lcd.print("WiFi: Connected");
        }
        
        
    }
  

  // sendTestNotification();
    delay(10);  // Wait 10 seconds between notifications
}
