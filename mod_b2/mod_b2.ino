 /*
 Projet de sonde luminosité.
 Ajout de commande des volets roulants pour une gestion de la fermeture la nuit du logement.
 
 Basé sur l'exemple Basic ESP8266 MQTT
 
 Il cherche une connexion sur un serveur MQTT puis :
  - Lit la luminusité et réagit en fonction
    * Au dessus du seuil - rien
    * au dessous - ferme les volets, 
  - Il envoie tout cela en MQTT
  
Programmation via OTA possible
 
 FUTUR :
 On pourra définir les paramètres via une instruction MQTT
 
 Exemples :
 MQTT:
 https://github.com/aderusha/IoTWM-ESP8266/blob/master/04_MQTT/MQTTdemo/MQTTdemo.ino
 Witty:
 https://blog.the-jedi.co.uk/2016/01/02/wifi-witty-esp12f-board/
 Module tricapteur:
 http://arduinolearning.com/code/htu21d-bmp180-bh1750fvi-sensor-example.php
 
 Commande et conf MQTT
 - mod_lum/conf = Défini le seuil de luminosité bas
   mosquitto_pub -d -t mod_lum/conf -m "175"
 - mod_lum/cmd  = 
    si mesg == "ON" 	On allume la led
    si mesg == "OFF" 	On éteint la led
 - mod_lum/haut = si mesg == "ON" on monte les volets
 - mod_lum/bas = si mesg == "ON" on baisse les volets
 - mod_lum/cmd = 
    si mesg == "aff" On renvoi la valeur de conf luminosité topic MQTT "mod_lum/conflum"  
    mosquitto_pub -d -t mod_lum/cmd -m "aff"
    si mesg == "tmp" On renvoi le valeur de conf du temps topic MQTT mod_lum/conflum"  
  - mod_lum/conftemps Défini la valeur de temps entre 2 mesures.
 
 
*/
#include <ESP8266WiFi.h>
// includes necessaires au fonctionnement de l'OTA :
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <EEPROM.h>
//#include <PString.h>
#include <ArduinoJson.h> 
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
// Include pour les fonctions dans le même répertoires.
#include "def.h"

 
// Utilisation d’une photo-résistance 
// Et ports pour cmd volet
const int port = A0;    // LDR
// port de commande des volets
#define haut 13		
//#define arret 13
#define bas 15
//#define lbp 14
#define lbp 12
#define radar 4     // port du radar // au BP
#define relais 5    // port pour le relais
 
// Update these with values suitable for your network.

// Buffer pour convertir en chaine de l'adresse IP de l'appareil
byte mac[] = {0xDE,0xED,0xBA,0xFE,0xFE,0xED};
IPAddress ip(192,168,1,100);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);

/* mis dans c_wifi
const char* ssid1 = "Freebox.._EXT";
const char* ssid2 = "Freebox-..";
const char* password = "Pass";
*/
const char* mqtt_server = "192.168.1.81";
const char* mqttUser = "mod";
const char* mqttPassword = "Plaqpsmdp";
const char* svrtopic = "domoticz/in";
const char* topic_Domoticz_OUT = "domoticz/out"; 
 
// Création objet
WiFiClient modbalcon;
PubSubClient client(mqtt_server,1883,callback,modbalcon);
 // DHT sensor


//StaticJsonDocument<300> root;
//JsonObject& JSONencoder = jsonBuffer.createObject();
//StaticJsonDocument < 300 > jsonBuffer;
//DeserializationError error = deserializeJson (jsonBuffer, createObject());
//DynamicJsonBuffer jsonBuffer( MQTT_MAX_PACKET_SIZE );

// Variables
int idx = 3404;         // idx pour domoticz
int valeur = 0;         // temperature
float vin = 0;          // voltage capteur
char msg[100];          // Buffer pour envoyer message mqtt
int value = 0;
unsigned long readTime;
char floatmsg[10];      // Buffer pour les nb avec virgules
char message_buff[100]; // Buffer qui permet de décoder les messages MQTT reçus
long lastMsg = 0;       // Horodatage du dernier message publié sur MQTT
long timeLast = 0;      // pour le comptage des 24H
long timeNow = 0;       // pour le comptage des 24H
long lastbas = 0;       // compteur entre 2 lecture de capteur
long lasttpre = 0;      // temps de présence
long lastdetect = 0;    // temps d'envoi de notification de détection.
long lasttfetes = 0;    // temps de fetes

bool debug = true;     // Affiche sur la console si True
bool mess = false;      // true si message reçu
String sujet = "";      // contient le topic
String mesg = "";       // contient le message reçu
int lum = 120;           // Valeur de luminosité par défaut
int lfetes = 140;        // Valeur de luminosité par défaut pour le mode fetes
long tfetes = 21600000; // temps d'allumagpour les fêtes 6 heures, redéfini dans le setup
long tpre = 20000;      // tempo pour une présence 1min, redéfini dans le setup
long lsmg = 60000;      // Valeur de temps entre 2 lectures, redéfini dans le setup

bool enbas = false;     // Flag de test volet bas
bool pres = false;      // flag de présence
bool detect = false;    // flag si detection
bool relactif = false;  // Flag état du relais.
bool fetes = false;     // flag de fetes.
bool tempo = false;     // Flag on est dans l'allumage des fetes.
bool calcul = false;    // flag indiquant qu'on est en calcul des 14H
unsigned int addr = 0;  // addresse de stockage de lum
//String idx = "1545";    // Idx du module dans domoticz
 
//========================================
void setup() {
  pinMode(haut, OUTPUT);     // Initialize le mvmt haut
//  pinMode(arret, OUTPUT);     // Initialize le mvmt arret
  pinMode(bas, OUTPUT);     // Initialize le mvmt bas
  pinMode(lbp, OUTPUT);     // Initialize la LED verte 
//  digitalWrite(bas,HIGH);
  
  Serial.begin(115200);
  setupwifi(debug);
  ArduinoOTA.setHostname("mod_lum"); // on donne une petit nom a notre module
  ArduinoOTA.begin(); // initialisation de l'OTA
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  client.subscribe(topic_Domoticz_OUT);

  EEPROM.begin(512);
 // On regarde ce qui est dans l'eeprom, sinon on met la valeur par défaut.
  lum=eeGetInt(100);
  if ((lum == 0)|| (lum == -1))  {
    lum=90;
  }
  lsmg=eeGetInt(50);
  if ((lsmg == 0) || (lsmg == -1)) {
    lsmg=10000;
  }
  lfetes=eeGetInt(110);
  if ((lfetes == 0)|| (lfetes == -1))  {
    tfetes=90;
  }
  tfetes=eeGetInt(120);
  if ((tfetes == 0)|| (tfetes == -1))  {
    tfetes=21600000;
  }
  tpre=eeGetInt(60);
  if ((tpre == 0) || (tpre == -1)) {
    tpre=20000;
  }

  if (debug) {
    Serial.print("luminosité enregistrée ");
    Serial.println(lum);
    Serial.print("temps enregistré ");
    Serial.println(lsmg);
  }
}
// FIN du setup


//========================================
void loop() {
   // a chaque iteration, on verifie si une mise a jour nous est envoyee
   // si tel est cas, la lib ArduinoOTA se charge de gerer la suite :)
  ArduinoOTA.handle(); 
  
//  String idx;
  //char volt[8];
  String volt;
  
  // test de connection, sinon reconnecte
  //int counter = 0;
  if (!client.connected()) {
    reconnect();
  }
  // doit être appelée régulièrement pour permettre au client de traiter les messages entrants pour envoyer des données de publication et de rafraîchir la connexion
  client.loop();
 
  // affiche message reçu en MQTT
  // if domoticz message
  if ( mess ) {    
    traiteMQTT();
  }
  litcapteur();
   // valeur = analogRead(port);
//  if (!detect) {
//    lastdetect=millis();
//  }
//  if (digitalRead(radar) == LOW) { 
//    if (!detect)  { //si pas en mode presence 
//      // Envoi notification de présence
//      String name = "Présence"; 
//      String svalue = "Détection";
//      Emetmessage(idx, name, svalue);
//      client.publish(svrtopic,msg,false);
//      delay(500);
//      detect=true;
//      Serial.println("detection!");
//    }
//    //Serial.println(millis() - lastdetect);
//    if (millis() - lastdetect > 20000) { //si temps > 2min
//      detect=false;
//      //Serial.println(millis() - lastdetect);
//    }
//    if ( valeur < lum )  {
//       pres=true;
//       lasttpre=millis();
//    }
  //}
  
//  traitepre();
  traitelum();
 
} 
// fin loop

//========================================
void litcapteur() {
// test de présence et active les prises si la luminosité est basse.
//  valeur = analogRead(port);
  if (!detect) {
    lastdetect=millis();
  }
  if (digitalRead(radar) == LOW) { 
    if (!detect)  { //si pas en mode presence 
      // Envoi notification de présence
      String name = "Présence"; 
      String svalue = "Détection";
      Emetmessage(idx, name, svalue);
      client.publish(svrtopic,msg,false);
      //delay(500);
      detect=true;
      Serial.println("detection!");
    }
    //Serial.println(millis() - lastdetect);
    if (millis() - lastdetect > 20000) { //si temps > 2min
      detect=false;
      //Serial.println(millis() - lastdetect);
    }
    if ( valeur < lum )  {
       pres=true;
       lasttpre=millis();
    }
  }  
}

//========================================
void eeWriteInt(int pos, int val) {
    byte* p = (byte*) &val;
    EEPROM.write(pos, *p);
    EEPROM.write(pos + 1, *(p + 1));
    EEPROM.write(pos + 2, *(p + 2));
    EEPROM.write(pos + 3, *(p + 3));
    EEPROM.commit();
}

//========================================
int eeGetInt(int pos) {
  int val;
  byte* p = (byte*) &val;
  *p        = EEPROM.read(pos);
  *(p + 1)  = EEPROM.read(pos + 1);
  *(p + 2)  = EEPROM.read(pos + 2);
  *(p + 3)  = EEPROM.read(pos + 3);
  return val;
}

//========================================
void attend(long duree) {
  long tp=millis(); 
  //while (millis() - tp >= duree);
  do {} while (millis() - tp >= duree);
}

//========================================
void traitelum() {
  // renvoie le niveau de la ldr tous les lsmg sec
  if (millis() - lastMsg > lsmg) {
    lastMsg = millis();
    int idx = 3404;
    String name = "Luminosité"; 
    String svalue = String(analogRead(port),DEC);
    Emetmessage(idx, name, svalue);
    client.publish(svrtopic,msg,false);

    valeur = analogRead(port);
    String val = String(valeur);
    // convertit l’entrée en volt 
    vin = (valeur * 3.3) / 1024.0;
//    volt=String(vin);

    if (( valeur < lum ) && (!enbas)) {
        baisse_volet();
        enbas = true;
    }
    if (( valeur > lum ) && (enbas)) {
      // renvoie les niveaux des capteurs tous les 180 sec (3min)
      // et traite les infos des capteurs pour lever/abaisser les volets
      long now2 = millis();
      if (lastbas == 0) {
        lastbas=now2;
      }
      if (now2 - lastbas > 8000) {
        lastbas = 0;
        enbas = false;
      }
    }
    if (debug) {
      Serial.print("valeur = ");
      Serial.println(valeur);   
      Serial.print("volt = ");
      Serial.println(vin); 
      Serial.print("JSON: ");
      Serial.println(msg);
//      if (enbas) {
//        Serial.println("état volet = true");
//      } else {
//        Serial.println("état volet = false");
//      }
    }
    
  }    
}

//========================================
void traiteMQTT() {
    if ( sujet == topic_Domoticz_OUT) {
      String recept; 
      const char* cmd;
      const char* command;
      Receptionmessage(debug, recept, cmd, command);
      if ( recept == "1545" ) {      
        if ( strcmp(command, "bas") == 0) {
          if ( strcmp(cmd, "ON") == 0 ) {  // On baisse
            digitalWrite(bas,HIGH);
            delay(5000);
            digitalWrite(bas,LOW);
          }
        }
        if ( strcmp(command, "haut") == 0) {
          if ( strcmp(cmd, "ON") == 0 ) {  // On baisse
            digitalWrite(haut,HIGH);
            delay(5000);
            digitalWrite(haut,LOW);
          }
        }   
      }  // if ( idx == 1 ) {
      recept="";               
    } // if domoticz message
  
// avec mod_lum
      
   // pinMode(lbp,OUTPUT);
    if ( sujet == "mod_lum/conf" ) {
      lum = mesg.toInt();
 //     sauverInt(100, lum);
      eeWriteInt(100, lum);
    }
    if ( sujet == "mod_lum/cmd" ) {
      if ( mesg == "ON" ) {
        digitalWrite(lbp,HIGH);  
      } 
      if (mesg == "OFF") {
        digitalWrite(lbp,LOW);  
      }
    }
    if ( sujet == "mod_lum/haut" ) {
      if ( mesg == "ON" ) {
        digitalWrite(haut,HIGH);
        delay(5000);
        digitalWrite(haut,LOW);
      }
    } 
    if ( sujet == "mod_lum/bas" ) {
      if ( mesg == "ON" ) {
        digitalWrite(bas,HIGH);
        delay(5000);
        digitalWrite(bas,LOW);
      }
    } 
    if ( sujet == "mod_lum/cmd" ) {
      if ( mesg == "aff" ) {
        digitalWrite(lbp,HIGH);
        delay(5000);
        digitalWrite(lbp,LOW);
        snprintf (msg, 80, "Valeur de conf luminosité", String(lum).c_str());
        client.publish("mod_lum/conflum", String(lum).c_str());  
        if (debug) {
          Serial.print("conf lum = ");
          Serial.println(lum);   
        }
      }
      if ( mesg == "tmp" ) {
        digitalWrite(lbp,HIGH);
        delay(5000);
        digitalWrite(lbp,LOW);
        snprintf (msg, 80, "Valeur de conf du temps", String(lsmg).c_str());
        client.publish("mod_lum/conflum", String(lsmg).c_str());  
        if (debug) {
          Serial.print("conf lsmg = ");
          Serial.println(lsmg);   
        }
      }
    }
    if ( sujet == "mod_lum/conftemps" ) {
      lsmg = mesg.toInt();
      eeWriteInt(50, lsmg);
      if (debug) {
        Serial.print("temps récupéré ");
        Serial.println(lsmg);
      }
    }
     
   // pinMode(lbp,INPUT);
    mess = false;
    delay(800);
}

//========================================
/* Baiise le volet
 * essai en 2 fois séparé par 120sec
 */
 void baisse_volet() {
  long tmp = millis();
  
  int cmpt = 1;
  while ( cmpt < 3 ) {
    digitalWrite(bas,HIGH);
    delay(1000);
    digitalWrite(bas,LOW);
    delay(8000);
    client.publish("mod_lum", "On baisse les volets");
    if (debug) { 
      Serial.println("cmpt ");
      Serial.println(cmpt);
    }
    //cmpt = cmpt + 1;
    ++cmpt;
    if (debug) { 
      Serial.println("cmpt++ ");
      Serial.print(cmpt); 
    }
  }
 }
 
