
#include <ESP8266WiFi.h>

//Variablen zum Starten des Webservers
const char* ssid = "KontiBat-Greifer_Web_Server";
const char* password = "";
unsigned long ulReqcount = 0;
WiFiServer server(80);
String header;

//Variablen für Greifersteuerung
String output5State = "off";
String output4State = "off";
int Program;

//Variablen für zeitgesteuerte Prozesse
unsigned long currentTime = millis();
unsigned long previousTime = 0; 
const long timeoutTime = 5000;


//Vorbereitungsprogramm
void setup()
{
  //Ausgabevariablen
  pinMode(2, OUTPUT);
  Serial.begin(57600);
  
  //Aufbau des WLAN-Zugriffpunktes/Debug-Fenster
  Serial.print("Konfiguriere soft-AP ... ");
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  server.begin();
  Serial.println("Server listening");
  Serial.println();
  Serial.print("Server IP address: ");
  Serial.println(WiFi.softAPIP());
  Serial.print("Server MAC address: ");
  Serial.println(WiFi.softAPmacAddress());
}


//Hauptprogramm
void loop() {

  //Überprüfung, ob der Server zugegriffen wurde
  WiFiClient client = server.available();
  if (!client) 
  {
    return;
  }
  if (client) {                             // If a new client connects,
    //Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    currentTime = millis();
    previousTime = currentTime;
    while (client.connected() && currentTime - previousTime <= timeoutTime) { // loop while the client's connected
      currentTime = millis();         
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        //Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();

            
            
            //Schalten der Greiferfunktionen
            if (header.indexOf("GET /5/on") >= 0) {

              //Programm "Öffnen"
              output5State = "Offen";
              Program = 1;
              
            } else if (header.indexOf("GET /5/off") >= 0) {

              //Programm "Schließen"
              output5State = "Geschlossen";
              Program = 2;
              
            } else if (header.indexOf("GET /4/on") >= 0) {

              //Programm zur Kraftregelung der WS-Position
              output4State = "WS-Kraftregelung";
              Program = 3;
              
            } else if (header.indexOf("GET /4/off") >= 0) {

              //Programm zur WS-Positionierung nach Bahnkurve
              output4State = "WS-Bahnkurve";
              Program = 4;
                    
            }

            //Übergabe der Variable
            Serial.write(Program);
            
            
            //HTML-Gestaltung der Seite
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #195B6A; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #77878A;}</style></head>");
            
            //Banner
            client.println("<body><h1>KontiBat-Greifer Steuerung</h1>");
            
            //Darstellung der Schaltflächen zur ÖS-Steuerung
            client.println("<p>Greiferzustand: " + output5State + "</p>");      
            if (output5State=="Geschlossen") {
              client.println("<p><a href=\"/5/on\"><button class=\"button\">OEFFNEN</button></a></p>");
            } else {
              client.println("<p><a href=\"/5/off\"><button class=\"button button2\">SCHLIESSEN</button></a></p>");
            } 
               
            //Darstellung der Schaltflächen zur WS-Positionierung
            client.println("<p>WS-Positionierung: " + output4State + "</p>");    
            if (output4State=="WS-Bahnkurve") {
              client.println("<p><a href=\"/4/on\"><button class=\"button\">KRAFTREGELUNG</button></a></p>");
            } else {
              client.println("<p><a href=\"/4/off\"><button class=\"button button2\">BAHNKURVE</button></a></p>");
            }
            client.println("</body></html>");
            
            
            client.println();
            
            break;
          } else { 
            currentLine = "";
          }
        } else if (c != '\r') {  
          currentLine += c;   
        }
      }
    }
    
    header = "";
    client.stop();

  }
}
