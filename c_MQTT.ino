/*
Routine pour MQTT

# Déclenche les actions à la réception d'un message
void callback(char* topic, byte* payload, unsigned int length)

# tente une reconnection au seveur mqtt 
void reconnect()

Variable et objet nécessaire a déclarer :

const char* mqtt_server = "192.168.1.81";
const char* mqttUser = "mod";
const char* mqttPassword = "Plaqpsmdp";
const char* svrtopic = "domoticz/in";
const char* topic_Domoticz_OUT = "domoticz/out"; 
 
// Création objet, on peu choisir le nom de l'objet
WiFiClient modlum;
PubSubClient client(mqtt_server,1883,callback,modlum);

bool debug = true;     // Affiche sur la console si True
bool mess = false;      // true si message reçu
String sujet = "";      // contient le topic
String messageReceived="";  // contient le message reçu
char message_buff[100]; // Buffer qui permet de décoder les messages MQTT reçus
String mesg = "";       // contient le message reçu


*/

//========================================
// Déclenche les actions à la réception d'un message
// D'après http://m2mio.tumblr.com/post/30048662088/a-simple-example-arduino-mqtt-m2mio
void callback(char* topic, byte* payload, unsigned int length) {
  String receivedMessage = "";
  sujet = String(topic);
  // create character buffer with ending null terminator (string)
  for(int i=0; i<length; i++) {
    receivedMessage.concat((char)payload[i]);
  }
  receivedMessage.concat("\0");
 
  String msgString = String(message_buff);
  mesg = receivedMessage;
  if ( debug ) {
    Serial.println("Message recu =>  topic: " + String(topic));
    Serial.print(" | longueur: " + String(length,DEC));
    Serial.println(" Payload: " + receivedMessage);
  }
  mess = true;
 
}

//========================================
/* tente une reconnection au seveur mqtt */
void reconnect() {
  // Loop until we're reconnected
  int compt = 0;
  boolean noconnect = true;
  // tant qu'il ne trouve pas un serveur affiche rond
  while ((!client.connected()) && (noconnect == true)) {
    //delay(500);
    if (debug ) {
      Serial.print("Attempting MQTT connection...");
    }
    // Attempt to connect
    if (client.connect("modlum", mqttUser, mqttPassword )) {
      if (debug ) {
        Serial.println("connected");
      }
      // Once connected, publish an announcement...
      client.publish("mod_lum", "mod_lum vous salut !");
      // ... and resubscribe
      client.subscribe("#");
      // On affiche que c'est OK
      noconnect=true;
      digitalWrite(lbp,HIGH);
      
    } else {
      if ( debug ) {
        Serial.print("failed, rc=");
        Serial.print(client.state());
        Serial.println(" try again in 3 seconds");
      }  
      // Wait 3 seconds before retrying
      // Si 3 essais manqués on arrête  !
      // ceci afin de ne pas être bloqué dans la fonction du module par le wifi.
      if (compt <= 3) {
        delay(3000);
        compt++;
      } else {
        noconnect=false;
      }
    }
 
  }
  delay(1500);
  digitalWrite(lbp,LOW);
  //delay(1500);
}
 
