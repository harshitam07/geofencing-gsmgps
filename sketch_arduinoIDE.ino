#include <SoftwareSerial.h>
#include <AltSoftSerial.h>
#include <TinyGPS++.h>

//--------------------------------------------------------------
// Enter your personal phone number to receive SMS alerts.
// Phone number must start with country code.
const String PHONE = "ENTER_PHONE_NUMBER";
//--------------------------------------------------------------

// GSM Module RX pin to Arduino 3
// GSM Module TX pin to Arduino 2
#define rxPin 2
#define txPin 3
SoftwareSerial sim800(rxPin, txPin);

// GPS Module RX pin to Arduino 9
// GPS Module TX pin to Arduino 8
AltSoftSerial neogps;
TinyGPSPlus gps;

//--------------------------------------------------------------
#define BUZZER 4

// Alarm
int buzzer_timer = 0;
bool alarm = false;
boolean send_alert_once = true;

//--------------------------------------------------------------
// Size of the geo fence (in meters)
const float maxDistance = 30;

//--------------------------------------------------------------
float initialLatitude = 0;
float initialLongitude = 0;
float latitude, longitude;

//--------------------------------------------------------------
void getGps(float& latitude, float& longitude);

/*****************************************************************************************
 * setup() function
 *****************************************************************************************/
void setup() {
    Serial.begin(9600);
    sim800.begin(9600);
    neogps.begin(9600);
    pinMode(BUZZER, OUTPUT);

    sim800.println("AT"); // Check GSM Module
    delay(1000);
    sim800.println("ATE1"); // Echo ON
    delay(1000);
    sim800.println("AT+CPIN?"); // Check SIM ready
    delay(1000);
    sim800.println("AT+CMGF=1"); // SMS text mode
    delay(1000);
    sim800.println("AT+CNMI=1,1,0,0,0"); // Decide how newly arrived SMS should be handled
    delay(1000);

    delay(20000);
    buzzer_timer = millis();
}

/*****************************************************************************************
 * loop() function
 *****************************************************************************************/
void loop() {
    getGps(latitude, longitude);

    float distance = getDistance(latitude, longitude, initialLatitude, initialLongitude);

    Serial.print("Latitude= "); Serial.println(latitude, 6);
    Serial.print("Longitude= "); Serial.println(longitude, 6);
    Serial.print("Initial Latitude= "); Serial.println(initialLatitude, 6);
    Serial.print("Initial Longitude= "); Serial.println(initialLongitude, 6);
    Serial.print("Current Distance= "); Serial.println(distance);

    // Set alarm on?
    if (distance > maxDistance) {
        if (send_alert_once) {
            digitalWrite(BUZZER, HIGH);
            sendAlert();
            alarm = true;
            send_alert_once = false;
            buzzer_timer = millis();
        }
    } else {
        send_alert_once = true;
    }

    // Handle alarm
    if (alarm) {
        if (millis() - buzzer_timer > 5000) {
            digitalWrite(BUZZER, LOW);
            alarm = false;
            buzzer_timer = 0;
        }
    }

    // Handle GSM and serial communication
    while (sim800.available()) {
        Serial.println(sim800.readString());
    }

    while (Serial.available()) {
        sim800.println(Serial.readString());
    }
}

/*****************************************************************************************
 * getDistance() function
 *****************************************************************************************/
// Calculate distance between two points
float getDistance(float flat1, float flon1, float flat2, float flon2) {
    float dist_calc = 0;
    float dist_calc2 = 0;
    float diflat = 0;
    float diflon = 0;

    diflat = radians(flat2 - flat1);
    flat1 = radians(flat1);
    flat2 = radians(flat2);
    diflon = radians(flon2 - flon1);

    dist_calc = pow(sin(diflat / 2.0), 2);
    dist_calc2 = cos(flat1) * cos(flat2) * pow(sin(diflon / 2.0), 2);
    dist_calc += dist_calc2;

    dist_calc = 2 * atan2(sqrt(dist_calc), sqrt(1.0 - dist_calc));
    dist_calc *= 6371000.0; // Converting to meters

    return dist_calc;
}

/*****************************************************************************************
 * getGps() function
 *****************************************************************************************/
void getGps(float& latitude, float& longitude) {
    boolean newData = false;
    for (unsigned long start = millis(); millis() - start < 2000;) {
        while (neogps.available()) {
            if (gps.encode(neogps.read())) {
                newData = true;
                break;
            }
        }
    }

    if (newData) { // If newData is true
        latitude = gps.location.lat();
        longitude = gps.location.lng();
    } else {
        Serial.println("No GPS data is available");
        latitude = 0;
        longitude = 0;
    }
}

/*****************************************************************************************
 * sendAlert() function
 *****************************************************************************************/
void sendAlert() {
    String sms_data;
    sms_data = "Alert! The object is outside the fence.\r";
    sms_data += "http://maps.google.com/maps?q=loc:";
    sms_data += String(latitude) + "," + String(longitude);

    sim800.print("AT+CMGF=1\r");
    delay(1000);
    sim800.print("AT+CMGS=\"" + PHONE + "\"\r");
    delay(1000);
    sim800.print(sms_data);
    delay(100);
    sim800.write(0x1A); // ASCII code for Ctrl-Z
    delay(1000);

    Serial.println("SMS Sent Successfully.");
}
