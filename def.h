// c_wifi initialise le wifi et cherche à se connecter
void setupwifi(boolean debug);

// c_mqtt retour et reconnecte pour mqtt
void callback(char* topic, byte* payload, unsigned int length);
void reconnect();

//re-mqtt reception/emission JSON
void Receptionmessage(boolean debug, String recept, const char* cmd, const char* command);
void Emetmessage (int idx,String name, String svalue);
