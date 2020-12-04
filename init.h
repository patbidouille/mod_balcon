/* Fichier d'initiation des variables "sensibles"
 * passord, IP, ...
 */

// Buffer pour convertir en chaine de l'adresse IP de l'appareil
byte mac[] = {0xDE,0xED,0xBA,0xFE,0xFE,0xED};
IPAddress ip(192,168,1,100);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);

const char* ssid1 = "Freebox-587BA2_EXT";
const char* ssid2 = "Freebox-587BA2";
const char* password = "Pl_aqpsmdp11";
const char* mqtt_server = "192.168.1.81";
const char* mqttUser = "mod";
const char* mqttPassword = "Plaqpsmdp";
const char* svrtopic = "domoticz/in";
const char* topic_Domoticz_OUT = "domoticz/out"; 
