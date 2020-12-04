//========================================================================================
/* Connection au wifi
- test le premier ssid
- sinon passe au seconds
*/

const char* ssid1 = "Freebox-587BA2";
const char* ssid2 = "Freebox-587BA2_EXT";
const char* password = "Pl_aqpsmdp11";

void setupwifi(boolean debug) {
 
  int cpt = 0;
  boolean ssid = true;
  delay(10);
//  boolean debug = true;
 
  // We start by connecting to a WiFi network
  while (WiFi.status() != WL_CONNECTED) {
    if (ssid) {
      WiFi.begin(ssid1, password);
    }
    else {
      WiFi.begin(ssid2, password);
    }
    if (debug) {
        Serial.println();
        Serial.print("Connecting to ");
        if (ssid) {
          Serial.println(ssid1);
        } else {
          Serial.println(ssid2);
        }
    }
 
    while ((WiFi.status() != WL_CONNECTED) && (cpt <= 30)) {
      delay(500);
      Serial.print(".");
      cpt=cpt+1;
    }
 
    if (cpt >= 30) {
      Serial.println(":");
      if (ssid) {
        ssid=false;
      } else {
        ssid=true;
      }
      cpt = 0;
    }
  }

  // on définit l'ip
  // WiFi.config(ip, gateway, subnet);
   
  // SINON On récupère et on prépare le buffer contenant l'adresse IP attibué à l'ESP-01
  IPAddress ip = WiFi.localIP();   

  if ( debug ) {
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  }
 
}
