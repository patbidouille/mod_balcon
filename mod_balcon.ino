 /*
 Projet de gestion du balcon.
  - Commande des volets roulants pour la fermeture la nuit du logement.
  - Test de luminosité pour fermer les volets.
  - Test de présence par Radar Doppler.
  - Relais d'allumage/extinction des prises secteurs.
 
 Basé sur l'exemple Basic ESP8266 MQTT
 
 Il cherche une connexion sur un serveur MQTT puis3 modes :
 Mode : gestion volets
    - Lit la luminosité et réagit en fonction
    * Au dessus du seuil - rien
    * au dessous - ferme les volets, 
  - Il envoie tout cela en MQTT
  
Mode présence :
    - lit le radar 
    * Si présence et si luminosité < seuil -> allume les prises durant une tempo.
    * Envoi une notification de présence par mqtt

Mode fêtes :
    - Si ce mode est activé.
    - Suivant un seuil de luminosité, on allume les prise durant un temps    
  
Programmation via OTA.
 
Commande et conf MQTT
 - mod_balcon/confvolet = Défini le seuil de luminosité bas pour la fermeture des volets
   mosquitto_pub -d -t mod_balcon/conf -m "175"
 - mod_balcon/cmd  = 
    si mesg == "ON" 	On allume la led
    si mesg == "OFF" 	On éteint la led
    si mesg == "aff" On renvoi la valeur de conf luminosité topic MQTT "mod_balcon/conflum"  
    mosquitto_pub -d -t mod_balcon/cmd -m "aff"    
    si mesg == "tmp" On renvoi le valeur de conf du temps topic MQTT mod_balcon/conflum"  
 - mod_balcon/haut = si mesg == "ON" on monte les volets
 - mod_balcon/bas = si mesg == "ON" on baisse les volets
 - mod_balcon/conftemps Défini la valeur de temps entre 2 mesures de luminosité.
 - mod_balcon/conflum = Défini le seuil de luminosité pour les volets.
 
 - mod_balcon/confLfetes = défini le seuil de muminosité pour les fêtes
 - mod_balcon/confTfetes = defini le temps d'allumage des prises.
 - mod_balcon/confTpre = defini la tempo d'allumage pour une présence
 
 
*/
#include <ESP8266WiFi.h>
// includes necessaires au fonctionnement de l'OTA :
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <EEPROM.h>
//#include <PString.h>
#include <ArduinoJson.h> 
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
IPAddress ip(192,168,1,103);
IPAddress gateway(192,168,1,254);
IPAddress subnet(255,255,255,0);

/* mis dans c_wifi
const char* ssid1 = "Freebox-587BA2_EXT";
const char* ssid2 = "Freebox-587BA2";
const char* password = "Pl_aqpsmdp11";
*/
const char* mqtt_server = "192.168.1.81";
const char* mqttUser = "mod";
const char* mqttPassword = "Plaqpsmdp";
const char* svrtopic = "domoticz/in";
const char* topic_Domoticz_OUT = "domoticz/out"; 
 
// Création objet
WiFiClient espClient;
PubSubClient client(mqtt_server,1883,callback,espClient); //,1883,callback,modbalcon);

// Variables
String clientId = "modbalcon"; // Id du client
int idx = 3404;         // idx pour domoticz
int valeur = 0;         // Luminosité
float vin = 0;          // voltage capteur
char msg[100];          // Buffer pour envoyer message mqtt
int value = 0;
unsigned long readTime;
char floatmsg[10];      // Buffer pour les nb avec virgules
char message_buff[100]; // Buffer qui permet de décoder les messages MQTT reçus
long now = 0;           // variable du temps actuel
long lastMsg = 0;       // Horodatage du dernier message publié sur MQTT
long timeLast = 0;      // pour le comptage des 24H
long timeNow = 0;       // pour le comptage des 24H
long lastbas = 0;       // compteur entre 2 lecture de capteur
long lasttpre = 0;      // temps de présence
long lastdetect = 0;    // temps d'envoi de notification de détection.
long lasttfetes = 0;    // temps de fetes
bool debug = true;      // Affiche sur la console si True
bool mess = false;      // true si message reçu
String sujet = "";      // contient le topic
String mesg = "";       // contient le message reçu
int lum = 90;           // Valeur de luminosité par défaut des volets
int lfetes = 90;        // Valeur de luminosité par défaut pour le mode fetes
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
//Time start Settings:
int startingHour = 12;
// set your starting hour here, not below at int hour. This ensures accurate daily correction of time
int seconds = 0;
int minutes = 33;
int hours = startingHour;
int days = 0;
//Accuracy settings
//int dailyErrorFast = 0; // set the average number of milliseconds your microcontroller's time is fast on a daily basis
//int dailyErrorBehind = 0; // set the average number of milliseconds your microcontroller's time is behind on a daily basis
//int correctedToday = 1; // do not change this variable, one means that the time has already been corrected today for the error in your boards crystal. This is true for the first day because you just set the time when you uploaded the sketch. 
 
//========================================
void setup() {
  pinMode(haut, OUTPUT);     // Initialize le mvmt haut
//  pinMode(arret, OUTPUT);     // Initialize le mvmt arret
  pinMode(bas, OUTPUT);     // Initialize le mvmt bas
  pinMode(lbp, OUTPUT);     // Initialize la LED verte 
  pinMode(relais, OUTPUT);     // Initialize le realis
  pinMode(radar, INPUT);     // Initialize le radar
//  digitalWrite(bas,HIGH);
  
  Serial.begin(115200);
  setupwifi(debug);
  ArduinoOTA.setHostname("mod_balcon"); // on donne une petit nom a notre module
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
    Serial.print("temps présence ");
    Serial.println(tpre);
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
 
  if ( mess ) { 
    traiteMQTT(); 
    mess=false;
  }
  evenement();
  traitepre();
  traitelum();
  traitefetes();

} 
// fin loop

//========================================
void attend(int duree) {
  long tp=millis(); 
  Serial.print("td=");
  Serial.println(duree);
  while (millis()-tp < duree);
  
  Serial.print("-tfff=");
  Serial.println(millis()-tp);
}

//========================================
//Il faut intialiser les variables second, minutes, hours avant de recommencer les 24h
bool cpt14() { //compteur de 14H
  return false;
  timeNow = millis()/1000; // the number of milliseconds that have passed since boot
  seconds = timeNow - timeLast;
  //the number of seconds that have passed since the last time 60 seconds was reached.
  if (seconds == 60) {
    timeLast = timeNow;
    minutes = minutes + 1; }
  //if one minute has passed, start counting milliseconds from zero again and add one minute to the clock.
  if (minutes == 60){
    minutes = 0;
    hours = hours + 1; }
  // if one hour has passed, start counting minutes from zero and add one hour to the clock
  if (hours == 14){
    hours = 0;
    return true;
  }
}

//========================================
void evenement() {
// test de présence et active les prises si la luminosité est basse.
  //valeur = analogRead(port);
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
      detect=true;
      //Serial.println("detection!");
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
void traitepre() {
  if ((pres) &  (millis() - lasttpre < tpre)) {
    if (!relactif) {
      digitalWrite(relais, HIGH);
      Serial.println("présence ALLUMAGE");
      relactif=!relactif;
    }
  } else {
    if (relactif) {
      digitalWrite(relais, LOW);
      Serial.println("présence ETEINT");
      pres=false;
      relactif=!relactif;
    }
  }
  if (debug) {
    //Serial.println(pres);
  }
}

//========================================
void traitefetes() {
  if (fetes) {
    if ((!tempo) & (!calcul)) { 
      lasttfetes=millis(); 
      seconds =0;
      minutes=0;
      hours=0;
    }
    if ((calcul) & (cpt14())) { calcul=false; }
    if (!calcul) {
      if  ((valeur < lfetes) & (millis() - lasttfetes > tfetes)) {
        tempo=true;
        digitalWrite(relais, HIGH);
      } else {
        tempo=false;
        digitalWrite(relais, LOW);
        calcul=true;
      }
    }
  }
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
void traiteMQTT () {
    // affiche message reçu en MQTT
  // if domoticz message
      
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
  
// avec mod_balcon
      
   // pinMode(lbp,OUTPUT);
    if ( sujet == "mod_balcon/conflum" ) {
      lum = mesg.toInt();
 //     sauverInt(100, lum);
      eeWriteInt(100, lum);
    }
    if ( sujet == "mod_balcon/conftemps" ) {
      lsmg = mesg.toInt();
      eeWriteInt(50, lsmg);
      if (debug) {
        Serial.print("temps récupéré ");
        Serial.println(lsmg);
      }
    }
    if ( sujet == "mod_balcon/confLfetes" ) {
      lfetes = mesg.toInt();
 //     sauverInt(100, lum);
      eeWriteInt(110, lfetes);
    }
    if ( sujet == "mod_balcon/confTfetes" ) {
      tfetes = mesg.toInt();
 //     sauverInt(100, lum);
      eeWriteInt(120, tfetes);
    }
    if ( sujet == "mod_balcon/confTpre" ) {
      tpre = mesg.toInt();
 //     sauverInt(100, lum);
      Serial.print("init tpre !!!");
      eeWriteInt(60, tpre);
    }
    if ( sujet == "mod_balcon/cmd" ) {
      if ( mesg == "ON" ) {
        digitalWrite(lbp,HIGH);  
      } 
      if (mesg == "OFF") {
        digitalWrite(lbp,LOW);  
      }
    }
    if ( sujet == "mod_balcon/haut" ) {
      if ( mesg == "ON" ) {
        digitalWrite(haut,HIGH);
        delay(5000);
        digitalWrite(haut,LOW);
      }
    } 
    if ( sujet == "mod_balcon/bas" ) {
      if ( mesg == "ON" ) {
        digitalWrite(bas,HIGH);
        delay(5000);
        digitalWrite(bas,LOW);
      }
    } 
    if ( sujet == "mod_balcon/cmd" ) {
      if ( mesg == "aff" ) {
        digitalWrite(lbp,HIGH);
        delay(5000);
        digitalWrite(lbp,LOW);
        snprintf (msg, 80, "Valeur de conf luminosité", String(lum).c_str());
        client.publish("mod_balcon/conflum", String(lum).c_str());  
        snprintf (msg, 80, "Valeur de conf tempo entre 2 mesures de lum", String(lsmg).c_str());
        client.publish("mod_balcon/conftemps", String(lsmg).c_str());
        snprintf (msg, 80, "Valeur de conf luminosité fêtes", String(lfetes).c_str());
        client.publish("mod_balcon/confLfetes", String(lfetes).c_str());
        snprintf (msg, 80, "Valeur de conf Tempo fêtes", String(tfetes).c_str());
        client.publish("mod_balcon/conftempsTfetes", String(tfetes).c_str());
        snprintf (msg, 80, "Valeur de conf temps présence", String(tpre).c_str());
        client.publish("mod_balcon/confTpre", String(tpre).c_str());
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
        client.publish("mod_balcon/conflum", String(lsmg).c_str());  
        if (debug) {
          Serial.print("conf lsmg = ");
          Serial.println(lsmg);   
        }
      }
      if ( mesg == "fetesON" ) {    // Activation du mode fêtes
        fetes=true;
      }
      if ( mesg == "fetesOFF" ) {    // Désactivation du mode fêtes
        fetes=false;
      }
    }
  mess = false;
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
//on veut sauvegarder par exemple le nombre décimal 55084, en binaire : 1101 0111 0010 1100

//fonction d'écriture d'un type int en mémoire EEPROM
void sauverInt(int adresse, int val) 
{   
    //découpage de la variable val qui contient la valeur à sauvegarder en mémoire
    unsigned char faible = val & 0x00FF; //récupère les 8 bits de droite (poids faible) -> 0010 1100 
    //calcul : 1101 0111 0010 1100 & 0000 0000 1111 1111 = 0010 1100

    unsigned char fort = (val >> 8) & 0x00FF;  //décale puis récupère les 8 bits de gauche (poids fort) -> 1101 0111
    //calcul : 1101 0111 0010 1100 >> 8 = 0000 0000 1101 0111 puis le même & qu’avant

    //puis on enregistre les deux variables obtenues en mémoire
    EEPROM.write(adresse, fort) ; //on écrit les bits de poids fort en premier
    EEPROM.write(adresse+1, faible) ; //puis on écrit les bits de poids faible à la case suivante
    EEPROM.commit();  

  
}
 
//========================================
//lecture de la variable de type int enregistrée précédemment par la fonction que l'on a créée

int lireInt(int adresse)
{
    int val = 0 ; //variable de type int, vide, qui va contenir le résultat de la lecture

    unsigned char fort = EEPROM.read(adresse);     //récupère les 8 bits de gauche (poids fort) -> 1101 0111
    unsigned char faible = EEPROM.read(adresse+1); //récupère les 8 bits de droite (poids faible) -> 0010 1100

    //assemblage des deux variable précédentes
    val = fort ;         // val vaut alors 0000 0000 1101 0111
    val = val << 8 ;     // val vaut maintenant 1101 0111 0000 0000 (décalage)
    val = val | faible ; // utilisation du masque
    // calcul : 1101 0111 0000 0000 | 0010 1100 = 1101 0111 0010 1100

    return val ; //on n’oublie pas de retourner la valeur lue !
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
    client.publish("mod_balcon", "On baisse les volets");
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
 
