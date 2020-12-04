/* Routine pour JSON
 *  Configure le buffer 
 *  Déserialise/serialize le message
 */



void Receptionmessage(boolean debug, String recept, const char* cmd, const char* command) {
  StaticJsonDocument<300> root;
  auto error = deserializeJson(root,mesg);
  if (error) {
    digitalWrite(lbp,HIGH);
    if (debug) {
      Serial.println("parsing Domoticz/out JSON Received Message failed");
    }
  } else {
    const char* idxChar = root["idx"];
    String recept = String( idxChar);
    const char* cmd = root["nvalue"];
    const char* command = root["command"];
    if (debug) {
      Serial.println("parsing Domoticz/out JSON Received Message OK !!!");
    }
  }
  if (debug) {
    Serial.print("recept=" );Serial.println(recept);
    Serial.print("command=" );Serial.println(command);
    Serial.print("cmd=" );Serial.println(cmd);
  }
} // if domoticz message

//========================
void Emetmessage (int idx,String name, String svalue) {
  StaticJsonDocument<300> root;
  root["idx"] = idx;
  root["name"] = name;
  root['nvalue'] = "0";
  root["svalue"] = svalue;
  serializeJson(root, msg);
}
