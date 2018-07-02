// equation for YF-B7 thermistor: R = 167493 e^(-0.0469T);
//                            or: T = 263 - 22 ln(R), where R is res and T is temp in C.
// to find thermistor res from read value for series res of Rs (float): R = Rs/( Vcc*1023.0/ADC - 1.0);
// series res for temp sensor should be connected between Vcc and A0, thermistor between A0 and gnd.
//
// Author: dc

#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// Update these with values suitable for your network.

// cnk vars begin:
const char* ssid = "2.4g";
const char* password = "dauyndauyn";
const char* mqtt_server = "lubuntuN7";
const char* mqtt_topic = "outTopic";
// cnk vars end;

// flow vars begin:
volatile int flow_frequency; // Measures flow sensor pulses
double l_min; // Calculated litres/hour
unsigned char flowsensor = 13; // Sensor Input
unsigned long currentTime;
unsigned long cloopTime;
// flow vars end;


// temp vars begin:
unsigned char tempsensor = A0;
int sensorValue = 0;
double temp;
double R;
double Rs = 470000.0;
double Vcc = 3.3;
char* tempC;
// temp vars end;

void flow () // Interrupt function
{
   flow_frequency++;
}

WiFiClient espClient;
PubSubClient client(espClient);
char msg[50];

void setup_wifi() 
{
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) 
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    //digitalWrite(BUILTIN_LED, LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is acive low on the ESP-01)
  } else {
    //digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
  }

}

void reconnect() 
{
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      //char* buf;
      //(clientId + " reconnected.").toCharArray(buf, 20);
      //clientId.toCharArray(buf, 20);
      client.publish("outTopic", "esp8266 client reconnected");
      // ... and resubscribe
      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() 
{
  //pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  pinMode(flowsensor, INPUT);
  digitalWrite(flowsensor, HIGH); // Optional Internal Pull-Up
  attachInterrupt(flowsensor, flow, RISING); // Setup Interrupt
  sei(); // Enable interrupts
  currentTime = millis();
  cloopTime = currentTime;
}

void loop() 
{
  //digitalWrite(BUILTIN_LED, LOW);
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  currentTime = millis();
  // Every second, calculate and print litres/min and temperature in C
  if(currentTime >= (cloopTime + 1000))
  {
     // temp loop blurb begin:
     sensorValue = analogRead(tempsensor);
     R = Rs / ( ( ( Vcc * 1023.0 ) / sensorValue ) - 1.0 );
     temp = 263.0 - 22.0*log(R);
     sprintf(msg, "temperature C: %d.%d", int(temp), int(temp * 100) %100);
     Serial.println(msg);
     client.publish(mqtt_topic, msg); 
     // temp loop blurb end;

     // flow loop blurb begin:
     cloopTime = currentTime; // Updates cloopTime
     l_min = (flow_frequency / 11.0); // (Pulse frequency (Hz) / 11) = flowrate in L/min for YF-B7 flow sensor.
     flow_frequency = 0; // Reset Counter
     sprintf(msg, "flow rate L/min: %d.%d", int(l_min), int(l_min*100) %100);
     Serial.println(msg);
     client.publish(mqtt_topic, msg);  
     // flow loop blurb end;
  }
}
