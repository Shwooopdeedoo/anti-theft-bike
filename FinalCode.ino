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


const char* googleApiKey = "AIzaSyByy9mROHxlLmc2v_oforADV6nvOpq5X5w";

const char* ssid = "DomainNorthgate_Resident_SS";               // Your Wi-Fi SSID
const char* password = "Shaan123!";       // Your Wi-Fi Password
const char* ntfyTopic = "JM4j2e0yT6akacaQ";    // ntfy.sh topic name

WifiLocation location (googleApiKey);

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

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  setClock ();
  location_t loc = location.getGeoFromWiFi();
  Serial.println ("Location: " + String (loc.lat, 7) + "," + String (loc.lon, 7));
  Serial.println ("Accuracy: " + String (loc.accuracy));
}

void sendTestNotification() {
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
  // Send a test notification every 10 seconds
  sendTestNotification();
  delay(10000);  // Wait 10 seconds between notifications
}