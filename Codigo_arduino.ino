#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include <Servo.h>  
#include <ThingSpeak.h> 

// Parámetros de la red Wi-Fi
const char* ssid = "Red_prototipo";
const char* password = "passwd-arduino";

// Parámetros del servidor MQTT
const char* mqtt_server = "broker.emqx.io";

// Credenciales de ThingSpeak
const char* thingSpeakAPIKey = "UCKTB8JZWUR470WJ"; // Clave API TS
unsigned long myChannelNumber = 2766295;      // ID CANAL TS
WiFiClient  clientThingSpeak; 

//  Wi-Fi y MQTT
WiFiClient espClient;
PubSubClient client(espClient);

// Configuracion Modulo BT
SoftwareSerial BTSerial(D1, D2); // RX, TX

// Pines conectados a ESP8266
const int yellowLED = D5;  // AA (D5 en la placa)
const int greenLED = D6;    // GPIO12 (D6 en la placa)
const int redLED = D7;     // GPIO13 (D7 en la placa)

// Pin para el servoMotor
const int servoPin = D3;  // Pin donde está conectado el servo motor

Servo myServo;

// Variables para el apagado de los leds
unsigned long yellowLEDTimer = 0;
unsigned long greenLEDTimer = 0;
unsigned long redLEDTimer = 0;
const unsigned long LED_DURATION = 20000; // 20 segundos para que se apague el LED

// Configuración Wi-Fi
void setup_wifi() {
  Serial.begin(115200);
  delay(10);

  // Conexion a la red Wi-Fi
  Serial.println();
  Serial.print("Conectando a WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("Conectado a Wi-Fi!");
  Serial.print("Dirección IP: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  // Intentar reconectar al servidor MQTT
  while (!client.connected()) {
    Serial.print("Intentando conexión MQTT...");
    
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);

    if (client.connect(clientId.c_str())) {
      Serial.println("Conectado al servidor MQTT");

      // Suscribirse al tópico "led/control"
      client.subscribe("led/control");
      client.subscribe("servo/control");  // Suscribirse al topico "servo/control"
    } else {
      Serial.print("Fallo, rc=");
      Serial.print(client.state());
      Serial.println(" Intentando nuevamente en 5 segundos...");
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Mensaje recibido en el tópico: ");
  Serial.print(topic);
  Serial.print(" -> ");
  String message = "";


  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  Serial.println(message); 

  // Comprobar el mensaje y controlar los LEDs y el servo en consecuencia
  if (String(topic) == "led/control") {
    controlLED(message);
  } else if (String(topic) == "servo/control") {
    controlServo(message);
  }
}

// Función para manejar el apagado automático de los LEDs
void checkLEDTimers() {
  unsigned long currentTime = millis();

  // Verificar y apagar LED amarillo
  if (digitalRead(yellowLED) == HIGH && (currentTime - yellowLEDTimer >= LED_DURATION)) {
    digitalWrite(yellowLED, LOW);
  }

  // Verificar y apagar LED verde
  if (digitalRead(greenLED) == HIGH && (currentTime - greenLEDTimer >= LED_DURATION)) {
    digitalWrite(greenLED, LOW);
  }

  // Verificar y apagar LED rojo
  if (digitalRead(redLED) == HIGH && (currentTime - redLEDTimer >= LED_DURATION)) {
    digitalWrite(redLED, LOW);
  }
}

void controlLED(String message) {
  // Comando para encender la luz amarilla
  if (message == "mitad") {
    digitalWrite(yellowLED, HIGH); // Enciende el LED amarillo
    yellowLEDTimer = millis(); // Iniciar temporizador
    client.publish("led/status", "Cortinas abiertas hasta la mitad");
  }
  
  // Comando para encender la luz verde
  else if (message == "Encender luz verde") {
    digitalWrite(greenLED, HIGH);
    greenLEDTimer = millis(); // Iniciar temporizador
    client.publish("led/status", "Cortinas completamente abiertas");
  }
  
  // Comando para encender la luz roja
  else if (message == "cerrar") {
    digitalWrite(redLED, HIGH);
    redLEDTimer = millis(); // Iniciar temporizador
    client.publish("led/status", "Cortinas completamente cerradas");
  }
}

void controlServo(String message) {
  // Comando para mover el servo a la posición completa
  if (message == "abrir") {
    myServo.write(180);  // Posición completa (180 grados)
    digitalWrite(greenLED, HIGH);  // Enciende la luz verde
    greenLEDTimer = millis(); // Iniciar temporizador
    digitalWrite(redLED, LOW);     // Apaga la luz roja si está encendida
    digitalWrite(yellowLED, LOW);  // Apaga la luz amarilla si está encendida
    client.publish("servo/status", "Cortinas abiertas por completo 180°");
    client.publish("led/status", "Cortinas completamente abiertas");
  }
  // Comando para mover el servo a la posición media
  else if (message == "mitad") {
    myServo.write(90);  // Posición a la mitad (90 grados)
    digitalWrite(yellowLED, HIGH);  
    yellowLEDTimer = millis(); 
    digitalWrite(greenLED, LOW);    // Apaga la luz verde si está encendida
    digitalWrite(redLED, LOW);      // Apaga la luz roja si está encendida
    client.publish("servo/status", "Cortinas abiertas hasta la mitad 90°");
    client.publish("led/status", "Cortinas abiertas hasta la mitad");
  }
  // Comando para mover el servo a la posición cerrada
  else if (message == "cerrar") {
    myServo.write(0);  // Posición cerrada (0 grados)
    digitalWrite(redLED, HIGH);  
    redLEDTimer = millis(); 
    digitalWrite(greenLED, LOW); // Apaga la luz verde si está encendida
    digitalWrite(yellowLED, LOW); // Apaga la luz amarilla si está encendida
    client.publish("servo/status", "Cortinas cerradas 0°");
    client.publish("led/status", "Cortinas cerradas por completo");
  }
}

void sendDataToThingSpeak() {
  // Envía el estado de los LEDs y el servo a ThingSpeak
  ThingSpeak.setField(1, digitalRead(yellowLED));  // Estado del LED amarillo
  ThingSpeak.setField(2, digitalRead(greenLED));   // Estado del LED verde
  ThingSpeak.setField(3, digitalRead(redLED));     // Estado del LED rojo
  ThingSpeak.setField(4, myServo.read());          // Posición del servo

  // Enviar los datos al canal de ThingSpeak
  int x = ThingSpeak.writeFields(myChannelNumber, thingSpeakAPIKey);
  if (x == 200) {
    Serial.println("Datos enviados a ThingSpeak correctamente.");
  } else {
    Serial.println("Error al enviar datos a ThingSpeak. Código de error: " + String(x));
  }
}

void setup() {

  pinMode(yellowLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(redLED, OUTPUT);

  // Inicializar los LEDs en estado apagado
  digitalWrite(yellowLED, LOW);
  digitalWrite(greenLED, LOW);
  digitalWrite(redLED, LOW);

  // Configurar el servo motor
  myServo.attach(servoPin); 

  // Iniciar comunicación serial con el módulo Bluetooth
  BTSerial.begin(9600);

  // Configurar la conexión Wi-Fi
  setup_wifi();

  // MQTT Configuracion Cliente MQTT
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  // Iniciar ThingSpeak
  ThingSpeak.begin(clientThingSpeak);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }

  client.loop();

  checkLEDTimers();  // Controla el temporizador de los LEDs

  // Enviar datos a ThingSpeak cada 10 segundos
  static unsigned long lastSendTime = 0;
  if (millis() - lastSendTime > 10000) {
    lastSendTime = millis();
    sendDataToThingSpeak();  // Envía datos a ThingSpeak
  }

  // Controlar LEDs y servo desde Bluetooth
  if (BTSerial.available()) {
    String command = BTSerial.readStringUntil('\n');
    command.trim(); 

    if (command == "abrir") {
      controlServo("abrir");
    } else if (command == "mitad") {
      controlServo("mitad");
    } else if (command == "cerrar") {
      controlServo("cerrar");
    } else if (command == "Encender luz amarilla") {
      controlLED("mitad");
    } else if (command == "Encender luz verde") {
      controlLED("Encender luz verde");
    } else if (command == "Encender luz roja") {
      controlLED("cerrar");
    } else {
      Serial.println("Comando no válido recibido desde Bluetooth: " + command);
    }
  }


  if (Serial.available()) {
    String message = Serial.readStringUntil('\n');
    message.trim();  

    if (message.startsWith("MQTT:")) {
      String payload = message.substring(5);
      client.publish("arduino/command", payload.c_str());
      Serial.println("Mensaje MQTT enviado: " + payload);
    } else {
      Serial.println("Comando no válido recibido desde Serial: " + message);
    }
  }
}
