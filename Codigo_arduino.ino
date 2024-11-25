#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include <Servo.h>  // Incluir la librería para el servo motor

// Definir los parámetros de la red Wi-Fi
const char* ssid = "Red_prototipo";
const char* password = "passwd-arduino";

// Definir los parámetros del servidor MQTT
const char* mqtt_server = "broker.emqx.io";

// Crear un objeto Wi-Fi y MQTT
WiFiClient espClient;
PubSubClient client(espClient);

// Configurar SoftwareSerial para el módulo Bluetooth
SoftwareSerial BTSerial(D1, D2); // RX, TX

// Definir los pines para los LEDs
const int yellowLED = D5;  // GPIO14 (D5 en la placa)
const int greenLED = D6;    // GPIO12 (D6 en la placa)
const int redLED = D7;     // GPIO13 (D7 en la placa)

// Definir el pin para el servo motor
const int servoPin = D3;  // Pin donde está conectado el servo motor

// Crear un objeto para el servo
Servo myServo;

// Estados iniciales de los LEDs
bool state_yellow = LOW;
bool state_green = LOW;
bool state_red = LOW;

// Configuración Wi-Fi
void setup_wifi() {
  Serial.begin(115200);
  delay(10);

  // Conectar a la red Wi-Fi
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
    
    // Crear un ID de cliente aleatorio
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);

    if (client.connect(clientId.c_str())) {
      Serial.println("Conectado al servidor MQTT");

      // Suscribirse al tópico "led/control"
      client.subscribe("led/control");
      client.subscribe("servo/control");  // Suscribirse también al tópico para el servo
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

  // Convertir el payload a un String
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

void controlLED(String message) {
  // Comando para encender la luz amarilla
  if (message == "Encender luz amarilla") {
    state_yellow = true;
    digitalWrite(yellowLED, HIGH); // Enciende el LED amarillo
    client.publish("led/status", "LED Amarillo encendido");
  }
  // Comando para apagar la luz amarilla
  else if (message == "Apagar luz amarilla") {
    state_yellow = false;
    digitalWrite(yellowLED, LOW); // Apaga el LED amarillo
    client.publish("led/status", "LED Amarillo apagado");
  }
  
  // Comando para encender la luz verde
  else if (message == "Encender luz verde") {
    state_green = true;
    digitalWrite(greenLED, HIGH);
    client.publish("led/status", "LED Verde encendido");
  }
  // Comando para apagar la luz verde
  else if (message == "Apagar luz verde") {
    state_green = false;
    digitalWrite(greenLED, LOW);
    client.publish("led/status", "LED Verde apagado");
  }
  
  // Comando para encender la luz roja
  else if (message == "Encender luz roja") {
    state_red = true;
    digitalWrite(redLED, HIGH);
    client.publish("led/status", "LED Rojo encendido");
  }
  // Comando para apagar la luz roja
  else if (message == "Apagar luz roja") {
    state_red = false;
    digitalWrite(redLED, LOW);
    client.publish("led/status", "LED Rojo apagado");
  }
}

void controlServo(String message) {
  // Comando para mover el servo a la posición completa
  if (message == "Abrir servo") {
    myServo.write(180);  // Posición completa (180 grados)
    client.publish("servo/status", "Servo abierto");
  }
  // Comando para mover el servo a la posición media
  else if (message == "Posición media servo") {
    myServo.write(90);  // Posición media (90 grados)
    client.publish("servo/status", "Servo en posición media");
  }
  // Comando para mover el servo a la posición cerrada
  else if (message == "Cerrar servo") {
    myServo.write(0);  // Posición cerrada (0 grados)
    client.publish("servo/status", "Servo cerrado");
  }
}

void setup() {
  // Configurar los pines de los LEDs como salidas
  pinMode(yellowLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(redLED, OUTPUT);

  // Inicializar los LEDs en estado apagado
  digitalWrite(yellowLED, LOW);
  digitalWrite(greenLED, LOW);
  digitalWrite(redLED, LOW);

  // Configurar el servo motor
  myServo.attach(servoPin);  // Conectar el servo al pin definido

  // Iniciar comunicación serial con el módulo Bluetooth
  BTSerial.begin(9600);

  // Configurar la conexión Wi-Fi
  setup_wifi();

  // Configurar el cliente MQTT
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void loop() {
  // Si no estamos conectados al servidor MQTT, intentar reconectar
  if (!client.connected()) {
    reconnect();
  }

  // Mantener la conexión con el servidor MQTT
  client.loop();

  // Verificar si hay datos disponibles desde Bluetooth
  if (BTSerial.available()) {
    String command = BTSerial.readStringUntil('\n');
    command.trim();
    controlLED(command);
  }

  // Verificar si hay datos disponibles desde el puerto serial
  if (Serial.available()) {
    String message = Serial.readStringUntil('\n');
    if (message.startsWith("MQTT:")) {
      String payload = message.substring(5);
      client.publish("arduino/command", payload.c_str());
      Serial.println("Mensaje MQTT enviado: " + payload);
    }
  }
}

