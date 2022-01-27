/*
  Blink

  Turns an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/Blink
*/

#include <WiFi.h>
#include <ArduinoJson.h>
 #include <BluetoothSerial.h>
#include <time.h>
#include <AsyncTelegram2.h>
#include <WiFiClient.h>
#include <SSLClient.h>  
#include "tg_certificate.h"
#include <EEPROM.h>

//************CONFIGURACIÓN DE TELEGRAM*****************

#define USE_CLIENTSSL true  
// Timezone definition
#define MYTZ "CET-1CEST,M3.5.0,M10.5.0/3"

WiFiClient base_client;
SSLClient client(base_client, TAs, (size_t)TAs_NUM, A0, 1, SSLClient::SSL_ERROR);

AsyncTelegram2 myBot(client);
//const char* ssid  =  "milton";     // SSID WiFi network
//const char* pass  =  "paternal";     // Password  WiFi network
//String ssid = "milton";
//String password = "paternal";
String ssid = "wifi01-ei";
String password = "Ax32MnF1975-ReB";

const char* token =  "5021419842:AAGWdRxVtpBNxiWtD3oEDN_CIkK1LnuqefE";  // Telegram token

bool ledState = LOW;

//TBMessage msg;

//************CONFIGURACIÓN DE TELEGRAM*****************

#define LED_BUILTIN 2
#define LED_ONBOARD 2
#define SEMAFORO_INPUT 27

BluetoothSerial SerialBT;

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// define the number of bytes you want to access
#define EEPROM_SIZE 300

//#define EEPROM_ADDRESS_OFFSET 0//double (8 bytes)
//#define EEPROM_ADDRESS_NUMSENSOR 8//uint8_t (1 byte)
//#define EEPROM_ADDRESS_FLAG_MODO_AP 16//uint8_t (1 byte)
//#define EEPROM_ADDRESS_FLAG_OTA 18//uint8_t (1 byte)
#define EEPROM_ADDRESS_WIFI_SSID 20//String
#define EEPROM_ADDRESS_WIFI_PASS 60//String
#define EEPROM_ADDRESS_MQTT_SERVER 100//String

// **************** EEPROM *************************
String brokerEEPROM = "";
String wifiSSIDEEPROM = {};
String wifiPASSEEPROM = {};
// **************** EEPROM *************************

char flagEstadoSemaforoIN1 = 0;


void handleNewMessages(int);
void chequearTelegram(void);
void setup_telegram(void);
void setup_wifi(void);
void cargarDesdeEEPROM(void);


// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SEMAFORO_INPUT, INPUT);
  SerialBT.begin("INTI-LearningFactory"); //Bluetooth device name

    WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
    
    Serial.begin(115200);
	cargarDesdeEEPROM();//levanta las variables guardadas previamente en EEPROM
	setup_wifi();
	setup_telegram();// Set the Telegram bot properies

}

// the loop function runs over and over again forever
void loop() {

	//myBot.sendMessage(msg, "Hola"); 

  	cambioDeParametros();
	//Chequea cada cierto tiempo mensajes enviados por Telegram
	chequearTelegram();
	
	//si la entrada del semáforo está en alto y su estado anterior estaba en bajo
	if(digitalRead(SEMAFORO_INPUT) == 1 && flagEstadoSemaforoIN1 == 0){
		digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
		//myBot.sendMessage(msg, "Semáforo de estación Nº1 encendido");       // notify the sender
		flagEstadoSemaforoIN1 = !flagEstadoSemaforoIN1;//invierte el estado (lo pone en 1)
		
		myBot.sendTo(1461403941, "Semáforo de estación Nº1 encendido");
	}else if(digitalRead(SEMAFORO_INPUT) == 0 && flagEstadoSemaforoIN1 == 1){
		digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (LOW is the voltage level)
		//myBot.sendMessage(msg, "Semáforo de estación Nº1 apagado");       // notify the sender
		flagEstadoSemaforoIN1 = !flagEstadoSemaforoIN1;//invierte el estado (lo pone en 0)
		myBot.sendTo(1461403941, "Semáforo de estación Nº1 apagado");
	}
	
	delay(1000);
/*
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
  */
}


//*************************************************************************
// FUNCIONES
//*************************************************************************


//*****************************
//***    CONEXION WIFI      ***
//*****************************


void setup_wifi(){

  //reset settings - wipe credentials for testing
  //wm.resetSettings();

  //reset settings - wipe credentials for testing
  //wm.resetSettings();

    //WiFiManager wm;
   
    //bool res;
   


   

      //Serial.println("Modo reconexión");//si no viene de un reset

      int cuenta = 0;
      //delay(10);
      // Nos conectamos a nuestra red Wifi
      Serial.println();
      Serial.print("Conectando a ssid: ");
      //Serial.println(savedSSID);
      Serial.println(ssid);

      //const char* ssidConverted = savedSSID.c_str();
      //const char* passwordConverted = savedPASS.c_str();
      const char* ssidConverted = ssid.c_str();
      const char* passwordConverted = password.c_str();
        
      //WiFi.disconnect(1);
        
      //WiFi.persistent(false);
      WiFi.begin(ssidConverted, passwordConverted);
	  


      while ((WiFi.status() != WL_CONNECTED) && cuenta < 20) {//límite de 20 intentos de 500 ms
        delay(500);
        Serial.print(".");
        cuenta++;
      
      }
      if(WiFi.status() != WL_CONNECTED){//si no logró conectarse
      
        Serial.println("No es posible conectar a WiFi");
        Serial.println("Se cambia a MODO LOCAL");
        SerialBT.println("No es posible conectar a WiFi");
        SerialBT.println("Se cambia a MODO LOCAL");

      }else{//si logró conectarse

        Serial.println("");
        Serial.println("Conectado a red WiFi!");
        Serial.println("Dirección IP: ");
        SerialBT.println("Conectado a red WiFi!");
        SerialBT.println("Dirección IP: ");
        Serial.println(WiFi.localIP());
        SerialBT.println(WiFi.localIP());
        delay(5000);
		#ifdef ESP32
			//client.setCACert(TELEGRAM_CERTIFICATE_ROOT); // Add root certificate for api.telegram.org
		#endif	 

      }


}

void setup_telegram(void){

    myBot.setUpdateTime(1000);
    myBot.setTelegramToken(token);

    // Check if all things are ok
    Serial.print("\nTest Telegram connection... ");
    myBot.begin() ? Serial.println("OK") : Serial.println("NOK");

    Serial.print("Bot name: @");
    Serial.println(myBot.getBotName());

}

void chequearTelegram(void){

  // local variable to store telegram message data
  TBMessage msg;
  //myBot.sendMessage(msg.chatId, "Hola"); 
  
	//if(myBot.noNewMessage() == 0){//chequea que no haya mensajes en el buffer
	//	myBot.sendMessage(msg, "Hola"); 
	//}
  
  /*
  if(digitalRead(SEMAFORO_INPUT) == 1){//si la entrada del semáforo está en alto
		digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
		myBot.sendMessage(msg, "Semáforo de estación Nº1 encendido");       // notify the sender
	}else{
		digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (LOW is the voltage level)
		myBot.sendMessage(msg, "Semáforo de estación Nº1 apagado");       // notify the sender
	}
  */
  // if there is an incoming message...
  if (myBot.getNewMessage(msg)) {
	  //myBot.sendMessage(msg, "Hola"); 
    String msgText = msg.text;

    if (msgText.equals("/light_on")) {                 // if the received message is "LIGHT ON"...
      digitalWrite(LED_ONBOARD, HIGH);                          // turn on the LED (inverted logic!)
      myBot.sendMessage(msg, "Light is now ON");       // notify the sender
    }
    else if (msgText.equals("/light_off")) {           // if the received message is "LIGHT OFF"...
      digitalWrite(LED_ONBOARD, LOW);                          // turn off the led (inverted logic!)
      myBot.sendMessage(msg, "Light is now OFF");       // notify the sender
    }else if (msgText.equals("/parpadear")) {                 // if the received message is "LIGHT ON"...
      digitalWrite(LED_ONBOARD, HIGH);
	  delay(1000);                          // turn on the LED (inverted logic!)
	  digitalWrite(LED_ONBOARD, LOW);
	  delay(1000);
	  digitalWrite(LED_ONBOARD, HIGH);
	  delay(1000);                          // turn on the LED (inverted logic!)
	  digitalWrite(LED_ONBOARD, LOW);
	  delay(1000);
      myBot.sendMessage(msg, "LED parpadeando");       // notify the sender
    }
    else {                                              // otherwise...
      // generate the message for the sender
      String reply;
      reply = "Bienvenido" ;
      reply += msg.sender.username;
      reply += ".\nLos comandos aceptados son: \n";
	  reply += "/light_on\n/light_off\n/parpadear";
      myBot.sendMessage(msg, reply);                    // and send it
    }
  }

}

//puede cambiar parámetros a través del puerto serie o por bluetooth
//Se debe enviar un caracter de identificación del parámetro a cambiar y
//luego el valor.
//Por ejemplo: cambiar el tiempo entre lecturas de temperatura
//enviar T100  siendo T: tiempoEntreLecturas; 100: 100 ms
//los parámetros que se pueden modificar son:
//  distanciaConfigurada--> D;
//  distanciaTolerancia--> t;
//  tempFiebre--> F;
//  tempMin--> m;
//  tempMax--> M;
//  tempOffset--> O;
//  tiempoEntreLecturas--> T;
//  cantLecturas--> C;
//  emisividad--> E;
//  Wifi--> W;  [Ejemplo: Wmyssid mypassword](El espacio se usa como delimitador)
//  debug--> d  [1 para activarlo; 0 para desactivarlo]
//  cantSensoresIR-->S
//  consultarLecturas-->P
//  escannearDispositivosI2C-->s  [1 para activarlo; 0 para desactivarlo]
//  cambiarDireccionI2C-->A       [A90 91]
//  analizarLecturasCantidad-->U
//  intercambiarSensores-->I;
//  consultarContadorReconexion--X; 
void cambioDeParametros(void){

  char charParamID = ' ';
  String valorParam = "";
  int inChar = 0;
  String inString = "";
    
  
  //**** Chequeo por Serie o Bluetooth ***************
  while (Serial.available() > 0 || SerialBT.available() > 0) {

    if(Serial.available() > 0){
      inChar = Serial.read();
    }else if(SerialBT.available() > 0){
      inChar = SerialBT.read();
    }
    

    if(inChar != '\n'){
      Serial.print((char)inChar);

      inString += (char)inChar;//encola los caracteres recibidos

    }else{//si llegó el caracter de terminación
      
      Serial.print("Input string: ");
      Serial.println(inString);
      Serial.print("string length: ");
      Serial.println(inString.length());


      //obtiene el identificador
      charParamID = inString.charAt(0);
      
      Serial.println(charParamID);
      
      //obtiene el valor
      for(int i = 1; i < inString.length(); i++){
        valorParam += inString.charAt(i);
      }

      Serial.println(valorParam);

      //evalua el identificador y los parámetros enviados
      switchCaseParametros(charParamID, valorParam);
      
      //borra el contenido y lo prepara para recibir uno nuevo
      inString = "";
    
    }
  }

}

void switchCaseParametros(char charParamID, String valorParam){

  int inChar = 0;
  int index = 0;
  int valorParamLength = 0;
  int endIndex = 0;
  int modoDebug = 0;
  int consultarLecturas = 0;
  int correccionActivada = 0;
  uint8_t numSensor = 0;
  uint16_t direccion = 0;
  int scanActivado = 0;
  byte oldAddress = 0;
  byte newAddress = 0;
  int analizarLecturasCantidad = 0;
  int intercambioSensores = 0;
  int color = 0;
  String nombreSensor = "";
  
  //valorParam = 
  valorParam.replace(0x0A,'\0');//Se filtra el caracter LF
  valorParam.replace(0x0D,'\0');//Se filtra el caracter CR

  switch(charParamID){
 
    case 'W':

      Serial.println("Wifi: ");
      valorParamLength = strlen(valorParam.c_str());
      endIndex = valorParamLength;
      index = valorParam.indexOf(' ');
      ssid = valorParam.substring(0, index);
      Serial.println(ssid);
      //password = valorParam.substring(index + 1, endIndex - 1);
      password = valorParam.substring(index + 1, endIndex);
      Serial.println(password);
      //guarda config wifi en EEPROM
      EEPROM.writeString(EEPROM_ADDRESS_WIFI_SSID, ssid);
      EEPROM.commit();
      EEPROM.writeString(EEPROM_ADDRESS_WIFI_PASS, password);
      EEPROM.commit();
      setup_wifi();

    break;
 
    default:
      Serial.println("Parámetro incorrecto");
    break;

  }  
}

     
void cargarDesdeEEPROM(void){

  // initialize EEPROM with predefined size
  EEPROM.begin(EEPROM_SIZE);
  
  /*
  brokerEEPROM = EEPROM.readString(EEPROM_ADDRESS_MQTT_SERVER);//lee el broker de la eeprom
  Serial.print("broker EEPROM: ");
  Serial.println(brokerEEPROM);
  broker = brokerEEPROM;
*/
  wifiSSIDEEPROM = EEPROM.readString(EEPROM_ADDRESS_WIFI_SSID);//lee el SSID de la eeprom
  Serial.print("SSID EEPROM: ");
  Serial.println(wifiSSIDEEPROM);
  ssid = wifiSSIDEEPROM;

  wifiPASSEEPROM = EEPROM.readString(EEPROM_ADDRESS_WIFI_PASS);//lee el PASS de la eeprom
  Serial.print("PASS EEPROM: ");
  Serial.println(wifiPASSEEPROM);
  password = wifiPASSEEPROM;

  //cambiarConfigMQTT(numeroSensor);

}