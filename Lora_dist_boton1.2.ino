//Prueba de codigo BLE, Vib, dist e interrupcion para LoRa-----Codigo del 21/08/2022. Andrew Gonzalez. 
//Programa de funcionamiento para Tesis versión sonar.
//El modelo de ESP32 es el Lora 915 encontrado en banggood.
//El pin ground a lado de 5v no funca. No usarlo, crea corto en el microporcesador.

//Librearias para el uso de BLE.
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

//Librerias para Lora
#include <Wire.h>
#include <SPI.h>             
#include <LoRa.h>
///////////////////////////////////////////////////////////
//Variables a declarar
int dist, vib,i=0,imprimir,j=0;          //dist = variable usada para determinar la distancia;vib= valor de intervalo de vibración; i= contador para flujo de LoRa.
float duration_us;          // Variable que retorna el tiempo que demoró en rebotar la señal en un objeto.
int a= 25;                  //Pin del vibrador
const int TRIG_PIN = 13;    // ESP32 pin GIOP13 connected to Ultrasonic Sensor's TRIG pin
const int ECHO_PIN = 12;  
const int BUTTON_PIN = 17;  // Pin 17 (creo) conecta con el boton para cambiar de función.
volatile int cont = 0;      // para interrumpir el funcionamiento nominal y pasar al modo LoRa
char cDist[15];             // char para almacenar distancia
char Nodo[50];              //char para mandar por BLE.
///Variables de ubicacion dentro de la Universidad
int u[3];
int Ant[4];
String ubicacion ="";
String cambio ="";
char sms[20];
char MFinal[100];
int b=500,c=3000;

///////////////////////////////////////////////////////////
//Definicion de variables BLE
BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
float txValue = 0;

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
  void onRead(BLECharacteristic *pCharacteristic) {
    char txString[8]; // make sure this is big enuffz
    dtostrf(txValue, 1, 2, txString); // float_val, min_width, digits_after_decimal, char_buffer
    
      pCharacteristic->setValue(txString);
  }
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        Serial.println("*********");
        Serial.print("Received Value: ");

        for (int i = 0; i < rxValue.length(); i++) {
          Serial.print(rxValue[i]);
        }

        Serial.println();

        // Do stuff based on the command received from the app
        if (rxValue.find("A") != -1) { 
          Serial.println("Turning ON!");
          //digitalWrite(a, HIGH);
        }
        else if (rxValue.find("B") != -1) {
          Serial.println("Turning OFF!");
          //digitalWrite(a, LOW);
        }

        Serial.println();
        Serial.println("*********");
      }
    }
};
//LoRa variables
const int csPin = 18;          // LoRa radio chip select
const int resetPin = 14;       // LoRa radio reset
const int irqPin = 26;         // change for your board; must be a hardware interrupt pin
const int led = 25;

byte MasterNode = 0xFF;       // dirección del "
byte Node1 = 0xBB;            // nodo 1 para prueba
byte Node2 = 0xCC;            //nodo 2 para prueba
byte Node3 = 0xDD;            //nodo 3 para prueba
byte Node4 = 0xEE;            //nodo 4 para prueba

String SenderNode = "";
String outgoing;              //mensaje a transmitirse

byte msgCount = 0;            //cuenta de mensajes enviados

int rssi;
int snr;
// Mide el tiempo desde la última vez que se envio un mensaje
unsigned long previousMillis=0;
unsigned long int previoussecs = 0; 
unsigned long int currentsecs = 0; 
unsigned long currentMillis = 0;
int interval= 1 ; // updated every 1 second
int Secs = 0; 

void Interrupcion(){         //Función que interrumpe en cualquier instancia el funcionamiento del "loop". En este caso aumenta el contador para caer en otra función.
    cont+=1;
    Serial.println("cambio");
}

void setup() {
Serial.begin(115200);       //La Comunicación es a 115200 baudios
pinMode(LED_BUILTIN, OUTPUT); //Led del ESP como output para validar que esta haciendo el setup
digitalWrite(LED_BUILTIN, HIGH);//LED del ESP32 encendido
delay(250);
digitalWrite(LED_BUILTIN, LOW);//LED del ESP32 apagado
delay(250);
pinMode(TRIG_PIN, OUTPUT);
pinMode(ECHO_PIN, INPUT);   // Modo de configuración en entrada
pinMode(a,OUTPUT);          //Pin del vibrador en modo salida.
pinMode(BUTTON_PIN, INPUT_PULLUP);    //Modo necesario para la lectura correcta del estado del boton.
attachInterrupt(digitalPinToInterrupt(BUTTON_PIN),Interrupcion,RISING);       //Declaración en setup de la condición para que el programa sea interrumpido.
delay(350);                 //Tiempo de espera para que se prepare el sensor.
////////////////////////////////////////////////////////////
//LoRa

while (!Serial);

  Serial.println("LoRa Duplex");

  // override the default CS, reset, and IRQ pins (optional)
  LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin
  LoRa.setGain(4);                      // El valor que ha presentado mejor desempeño. Puede ser de 1 a 6.
  if (!LoRa.begin(915E6)) {             // Iniciar en 915MHZ
    Serial.println("Fallo en el inicio de LoRa. Intente de nuevo  ");
    while (true);                       // si falla no hace nada
  }

  Serial.println("LoRa init succeeded.");


///////////////////////////////////////////////////////////
//BLE

  // Creacion del dispositivo bluetooth LE
  BLEDevice::init("Tesis ESP32"); // Nombre del dispositivo

  // Servidor del BLE
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Servicio de BLE
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Caracteristica de BLE encargada de llevar el mensaje
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
                    );
                      
  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setCallbacks(new MyCallbacks());

  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID_RX,
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pCharacteristic->setCallbacks(new MyCallbacks());

  // Inicio del servicio
  pService->start();

  // Inicio del advertising (visibilidad)
  pServer->getAdvertising()->start();
  Serial.println("Esperando para conectar a un dispositivo");
}

void loop(){
 if(cont==0){                     //Si el contador es 0 inicia la secuencia de cálculo, vibración e impresión (incluye audio) de distancia.
    Serial.println("Modo distancia");  //Prueba que revela en que condición está
    d();                          //Llamado a función de distancia
    Impresion(cDist,b);                  //Llamado a función de Impresión
    vibrar();                     //Llamado a función de vibrar                                               
    delay(100);                   //Tiempo de espera entre repetición del proceso para que los cálculos sean correctos en la distancia.
  }

 if(cont>0){                      //condición que desencadena las funciones LoRa
    cambio="Calculando ubicacion";
    cambio.toCharArray(sms,20);
    Impresion(sms,c);
  for(int k=0;k<100;k++){
    currentMillis = millis();
    currentsecs = currentMillis / 1000; 
    if ((unsigned long)(currentsecs - previoussecs) >= interval) {
     Secs = Secs + 1;
     //Serial.println(Secs);
     if ( Secs >= 8 )
    {
      Secs = 0; 
    }
    if ( (Secs >= 1) && (Secs <= 2) )
    {
     
    String message = "187"; 
    sendMessage(message,MasterNode, Node1);
    }
 
        if ( (Secs >= 3 ) && (Secs <= 4))
    {
     
    String message = "204"; 
    sendMessage(message,MasterNode, Node2);
    }

    if ( (Secs >= 4) && (Secs <= 5) )
    {
     
    String message = "221"; 
    sendMessage(message,MasterNode, Node3);
    }

    //test v1.4
    if ( (Secs >= 6) && (Secs <= 7) )
    {
     
    String message = "238"; //AGGREGAR A CODIGO DE TX4 para probar funcionamiento.
    sendMessage(message,MasterNode, Node4);
    }
    //final de test v1.4
    
    previoussecs = currentsecs;
    }
    Serial.println("sms enviado");
    // Mensaje enviado y esperando a recibir un mensaje de vuelta.
    onReceive(LoRa.parsePacket());
    delay(500);  
  }
  Condiciones();
  delay(1000);
  cont=0;
  }
}

void d(){                         //CALCULO DE LA DISTANCIA PARA SENSOR US.
//Ciclo para hacer promedio de los valores del voltaje que capta el sensor sharp.
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  duration_us = pulseIn(ECHO_PIN, HIGH);
  dist = 0.017 * duration_us; //US
  //dist=(5*(voltaje/0.0248)); infrared
  String Dist = String(dist);
  Dist.toCharArray(cDist,50);
  Serial.println(dist);            //En desarrollo>Esto en teoría debería asignarle un valor a una variable para luego ser usada en la función impresión.
}

void vibrar(){                    //Determinar periodo de vibracion
  if(dist>400){                   //Cada condición supone menor tiempo entre vibración de acuerdo a cúan lejos está el objeto a sensar
    vib=50;
    Serial.println("a"); 
   }
   if((400>=dist)&&(dist>300)){
    vib=100;
    Serial.println("b"); 
   }
    if((300>=dist)&&(dist>200)){
      vib=200;
      Serial.println("c");   
 }
    if((200>=dist)&&(dist>100)){
      vib=400;
      Serial.println("d");  
 }
 if((100>=dist)&&(dist>50)){
      vib=800;
      Serial.println("e");  
 }
 if(dist<=50){                   //Cada condición supone menor tiempo entre vibración de acuerdo a cúan lejos está el objeto a sensar
    vib=0;
    Serial.println("e");
 } 
                                   //Se encarga de dar la directriz de vibrar por X tiempo.
      digitalWrite(a, HIGH);
      delay(vib);
      digitalWrite(a, LOW);
      delay(500);        
}

void sendMessage(String outgoing, byte MasterNode, byte otherNode) {
  LoRa.beginPacket();                   // iniciar paquete
  LoRa.write(otherNode);              // agrega la dirección destino
  LoRa.write(MasterNode);             // agrega la dirección del remitente
  LoRa.write(msgCount);                 // agrega el ID del mesaje
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finaliza y envía el paquete
  msgCount++;                           // incrementa el contador de mensaje
}

void onReceive(int packetSize) {
  if (packetSize == 0) return;          // if there's no packet, return
 
  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
/*  if( sender == 0XBB )
  SenderNode = "Node1:";
  if( sender == 0XCC )
  SenderNode = "Node2:";
  if( sender == 0XDD )
  SenderNode = "Node3:";
  if( sender == 0XEE)
  SenderNode = "Node4:";*/
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length
  String incoming = "";
  rssi = (LoRa.packetRssi()) ;
  //test
  snr=(LoRa.packetSnr());
  digitalWrite(LED_BUILTIN, HIGH);// led que indica que envio mensaje.
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);//
  // fin del test
  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }
 
  if (incomingLength != incoming.length()) {   // check length for error
    //Serial.println("error: message length does not match length");
    return;                             // skip rest of function
  }
 
  // if the recipient isn't this device or broadcast,
  if (recipient != Node1 && recipient != MasterNode) {
   // Serial.println("This message is not for me.");
    return;                             // skip rest of function
  }
 
  // if message is for this device, or broadcast, print details:
  
  //Test 
  if(sender == 0XBB){ //Tx1
    Ant[0]= rssi;
    SenderNode = "Node1:";
    Serial.println(Ant[0]);
  }
  if(sender == 0XCC){ //Tx2
    Ant[3]= rssi;
    SenderNode = "Node2:";
    Serial.println(Ant[3]);
  }
  if(sender == 0XDD){ //Tx3
    Ant[1]= rssi;
    SenderNode = "Node3:";
    Serial.println(Ant[1]);
  }
  if(sender == 0XEE){ //Tx4
    Ant[2]= rssi;
    SenderNode = "Node4:";
    Serial.println(Ant[2]);
  }
  
  //FIN DEL TEST
  
  //Comentado debido a que ya no se necesita esto completo.
  //SenderNode +=rssi; //agregar el valor del RSSI al string
  //SenderNode += ";";
  //SenderNode +=snr;
  //SenderNode.toCharArray(Nodo,50); //convertir el string a char array para que sea imprimible en BLE.
  //Impresion(Nodo);//llamar a la funcion que se encarga de imprimir la variable tipo char*
}
void Condiciones(){
  for(j;j<=3;j++){
    Serial.print("Antena:");
    Serial.println(j);
    Serial.println(Ant[j]);
    if(j == 3){
      if(Ant[j]==0){u[j]=0;}
      if(-73<Ant[j] && Ant[j]<=-50){ u[j]=1;}
      if(-84<Ant[j] && Ant[j]<=-73){ u[j]=2; }
      if(-93<Ant[j] && Ant[j]<=-84){ u[j]=3; }
      if(-101<Ant[j] && Ant[j]<=-93){ u[j]=4; }
      if(-107<Ant[j] && Ant[j]<=-101){ u[j]=5; }
      if(-112<Ant[j] && Ant[j]<=-107){ u[j]=6; }
      if(-116<Ant[j] && Ant[j]<=-112){ u[j]=7; }
      if((-120<=Ant[j] && Ant[j]<=-116)){ u[j]=8; }
      if(Ant[j]<=-121){ u[j]=0; } //borrar si magicamente nada funca de nuevo  
    }
    else if(j<3){
      if(Ant[j]==0){u[j]=0;}
      if(-74<Ant[j] && Ant[j]<=-50){ u[j]=1;}
      if(-84<Ant[j] && Ant[j]<=-74){ u[j]=2; }
      if(-92<Ant[j] && Ant[j]<=-84){ u[j]=3; }
      if(-102<Ant[j] && Ant[j]<=-92){ u[j]=4; }
      if(-108<Ant[j] && Ant[j]<=-102){ u[j]=5; }
      if(-112<Ant[j] && Ant[j]<=-108){ u[j]=6; }
      if(-117<Ant[j] && Ant[j]<=-112){ u[j]=7; }
      if((-120<=Ant[j] && Ant[j]<=-117)){ u[j]=8; }
      if(Ant[j]<=-121){ u[j]=0; } //borrar si magicamente nada funca de nuevo 
  }
Serial.println(u[j]);  
}
  j=0;
  Resultados();
  Ant[0]=0;
  Ant[1]=0;
  Ant[2]=0;
  Ant[3]=0;// prueba
  u[0]=0;
  u[1]=0;
  u[2]=0;
  u[3]=0;
}
void Impresion(char* mensaje,int espera){
  Serial.println(mensaje);
  pCharacteristic->setValue(mensaje);
  pCharacteristic->notify();  // Send the value to the app!
  delay(espera); //quitar comentario para regresar a funcionamiento normal.
  //pCharacteristic->setValue(&txValue, 1); // To send the integer value
  //    pCharacteristic->setValue("Hello!"); // Sending a test message
/*  pCharacteristic->setValue("");
  pCharacteristic->notify();
  */
}

void Resultados(){
if((u[0] ==7 )&& (u[1] ==0 )&& (u[2] ==6 )&& (u[3] ==5 )){Serial.println("Estacionamientos Biblioteca Simon Bolivar");
ubicacion="Estacionamientos Biblioteca Simon Bolivar";
}
if((u[0] ==4 )&& (u[1] ==6 )&& (u[2] ==0 )&& (u[3] ==0 )){Serial.println("Estacionamientos Direccion asuntos internacionales");
ubicacion="Estacionamientos Direccion asuntos internacionales";
}
if((u[0] ==3 )&& (u[1] ==0 )&& (u[2] ==0 )&& (u[3] ==2 )){Serial.println("Estacionamientos Facultad de Derecho");
ubicacion="Estacionamientos Facultad de Derecho";
}
if((u[0] ==5 )&& (u[1] ==8 )&& (u[2] ==0 )&& (u[3] ==2 )){Serial.println("Estacionamientos Facultad de Ciencias");
ubicacion="Estacionamientos Facultad de Ciencias";
}
if((u[0] ==4 )&& (u[1] ==8 )&& (u[2] ==0 )&& (u[3] ==3 )){Serial.println("Estacionamientos Facultad de Ciencias");
ubicacion="Estacionamientos Facultad de Ciencias";
}
if((u[0] ==7 )&& (u[1] ==6 )&& (u[2] ==0 )&& (u[3] ==4 )){Serial.println("Estacionamientos de Enfermeria");
ubicacion="Estacionamientos de Enfermeria";
}
if((u[0] ==7 )&& (u[1] ==6 )&& (u[2] ==0 )&& (u[3] ==4 )){Serial.println("Estacionamientos de Enfermeria");
ubicacion="Estacionamientos de Enfermeria";
}

if((u[0] ==0 )&& (u[1] ==6 )&& (u[2] ==7 )&& (u[3] ==6 )){Serial.println("Logo Facultad de Enfermer");
ubicacion="Logo Facultad de Enfermeria";
}
if((u[0] ==8 )&& (u[1] ==4 )&& (u[2] ==7 )&& (u[3] ==6 )){Serial.println("Hidrante");
ubicacion="Hidrante";
}
if((u[0] ==6 )&& (u[1] ==6 )&& (u[2] ==8 )&& (u[3] ==4 )){Serial.println("Escalera facultad de ciencias y tecnologias");
ubicacion="Escalera facultad de ciencias y tecnologias";
}
if((u[0] ==8 )&& (u[1] ==6 )&& (u[2] ==7 )&& (u[3] ==4 )){Serial.println("Maquina de recarga metro");
ubicacion="Maquina de recarga metro";
}

if((u[0] ==0 )&& (u[1] ==0 )&& (u[2] ==0 )&& (u[3] ==3 )){Serial.println("Estatua Dr. Guillermo Patterson Jr");
ubicacion="Estatua Dr. Guillermo Patterson Jr";
}
if((u[0] ==5 )&& (u[1] ==0 )&& (u[2] ==0 )&& (u[3] ==5 )){Serial.println("Estatua Octavio Mendez Pereira");
ubicacion="Estatua Octavio Mendez Pereira";
}

if((u[0] ==6 )&& (u[1] ==0 )&& (u[2] ==0 )&& (u[3] ==5 )){Serial.println("Estatua Justo Arosemena");
ubicacion="Estatua Justo Arosemena";
}
if((u[0] ==6 )&& (u[1] ==0 )&& (u[2] ==0 )&& (u[3] ==4 )){Serial.println("Estatua Justo Arosemena");
ubicacion="Estatua Justo Arosemena";
}

if((u[0] ==6 )&& (u[1] ==0 )&& (u[2] ==7 )&& (u[3] ==5 )){Serial.println("Estatua Ascanio Arosemena");
ubicacion="Estatua Ascanio Arosemena";
}
if((u[0] ==6 )&& (u[1] ==0 )&& (u[2] ==7 )&& (u[3] ==4 )){Serial.println("Estatua Ascanio Arosemena");
ubicacion="Estatua Ascanio Arosemena";
}
if((u[0] ==6 )&& (u[1] ==0 )&& (u[2] ==0 )&& (u[3] ==5 )){Serial.println("Estatua Clara Gonzalez");
ubicacion="Estatua Clara Gonzalez";
}
if((u[0] ==0 )&& (u[1] ==6 )&& (u[2] ==5 )&& (u[3] ==6 )){Serial.println("Estatua Miguel de Cervantes");
ubicacion="Estatua Miguel de Cervantes";
}
if((u[0] ==3 )&& (u[1] ==8 )&& (u[2] ==0 )&& (u[3] ==2 )){Serial.println("Biblioteca Simon Bolivar");
ubicacion="Biblioteca Simon Bolivar";
}
if((u[0] ==2 )&& (u[1] ==0 )&& (u[2] ==0 )&& (u[3] ==2 )){Serial.println("Biblioteca Simon Bolivar");
ubicacion="Biblioteca Simon Bolivar";
}
if((u[0] ==4 )&& (u[1] ==0 )&& (u[2] ==8 )&& (u[3] ==3 )){Serial.println("Biblioteca Simon Bolivar");
ubicacion="Biblioteca Simon Bolivar";
}
if((u[0] ==3 )&& (u[1] ==0 )&& (u[2] ==0 )&& (u[3] ==3 )){Serial.println("Gestion y cooperacion inter");
ubicacion="Gestion y cooperacion inter";
}
if((u[0] ==5 )&& (u[1] ==0 )&& (u[2] ==0 )&& (u[3] ==4 )){Serial.println("Facultad de Derecho");
ubicacion="Facultad de Derecho";
}
if((u[0] ==7 )&& (u[1] ==0 )&& (u[2] ==0 )&& (u[3] ==5 )){Serial.println("Facultad de Derecho");
ubicacion="Facultad de Derecho";
}
if((u[0] ==7 )&& (u[1] ==8 )&& (u[2] ==0 )&& (u[3] ==4 )){Serial.println("Cafeteria Ciencias Exactas");
ubicacion="Cafeteria Ciencias Exactas";
}
if((u[0] ==0 )&& (u[1] ==0 )&& (u[2] ==0 )&& (u[3] ==4 )){Serial.println("Cafeteria Ciencias Exactas");
ubicacion="Cafeteria Ciencias Exactas";
}

if((u[0] ==0 )&& (u[1] ==4 )&& (u[2] ==0 )&& (u[3] ==6 )){Serial.println("Caja de ahorros");
ubicacion="Caja de ahorros";
}
if((u[0] ==0 )&& (u[1] ==5 )&& (u[2] ==7 )&& (u[3] ==7 )){Serial.println("Vicerrectoria de asuntos estu");
ubicacion="Vicerrectoria de asuntos estudiantil";
}
if((u[0] ==0 )&& (u[1] ==7 )&& (u[2] ==0 )&& (u[3] ==0 )){Serial.println("Vicerrectoria de asuntos estu");
ubicacion="Vicerrectoria de asuntos estudiantil";
}

if((u[0] ==0 )&& (u[1] ==6 )&& (u[2] ==0 )&& (u[3] ==7 )){Serial.println("Escuela de Matematicas");
ubicacion="Escuela de Matematicas";
}
if((u[0] ==0 )&& (u[1] ==7 )&& (u[2] ==0 )&& (u[3] ==8 )){Serial.println("Escuela de Matematicas");
ubicacion="Escuela de Matematicas"; 
}
if((u[0] ==0 )&& (u[1] ==5 )&& (u[2] ==0 )&& (u[3] ==8 )){Serial.println("Facultad de Enfermeria");
ubicacion="Facultad de Enfermeria";
}
if((u[0] ==8 )&& (u[1] ==3 )&& (u[2] ==7 )&& (u[3] ==5 )){Serial.println("Facultad de Enfermeria");
ubicacion="Facultad de Enfermeria";
}
if((u[0] ==0 )&& (u[1] ==4 )&& (u[2] ==0 )&& (u[3] ==6 )){Serial.println("Facultad de Enfermeria");
ubicacion="Facultad de Enfermeria";
}
if((u[0] ==0 )&& (u[1] ==0 )&& (u[2] ==3 )&& (u[3] ==7 )){Serial.println("La Colina");
ubicacion="La Colina";
}
if((u[0] ==0 )&& (u[1] ==0 )&& (u[2] ==7 )&& (u[3] ==0 )){Serial.println("La Colina");
ubicacion="La Colina";
}
if((u[0] ==0 )&& (u[1] ==7 )&& (u[2] ==0 )&& (u[3] ==0 )){Serial.println("Kiosko aleluya");
ubicacion="Kiosko aleluya";
}
if((u[0] ==0 )&& (u[1] ==7 )&& (u[2] ==8 )&& (u[3] ==7 )){Serial.println("Teatro al aire libre");
ubicacion="Teatro al aire libre";
}
if((u[0] ==8 )&& (u[1] ==7 )&& (u[2] ==6 )&& (u[3] ==7 )){Serial.println("Teatro al aire libre");
ubicacion="Teatro al aire libre";
}
if((u[0] ==0 )&& (u[1] ==6 )&& (u[2] ==6 )&& (u[3] ==6 )){Serial.println("Teatro al aire libre");
ubicacion="Teatro al aire libre";
}
if((u[0] ==8 )&& (u[1] ==6 )&& (u[2] ==7 )&& (u[3] ==5 )){Serial.println("Fuente del TUAL");
ubicacion="Fuente del TUAL";
}

if((u[0] ==0 )&& (u[1] ==8 )&& (u[2] ==6 )&& (u[3] ==0 )){Serial.println("Fuente de La Colina");
ubicacion="Fuente de La Colina";
}
ubicacion.toCharArray(MFinal,100);
Impresion(MFinal,c);
delay(500); //cambiar si no sirve la impresionde ubicacion
Impresion(MFinal,c);// borrar si no sirve xd
delay(500);
ubicacion="";
}
