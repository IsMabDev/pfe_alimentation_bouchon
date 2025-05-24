/*
 Controlling a servo position using a potentiometer (variable resistor)
 by Michal Rinott <http://people.interaction-ivrea.it/m.rinott>

 modified on 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Knob
*/
#include <Arduino.h>
#include <vector>
#include <ESP32Servo.h>
#include <AccelStepper.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <uri/UriBraces.h>

#define WIFI_SSID "Wokwi-GUEST"
#define WIFI_PASSWORD ""
// Defining the WiFi channel speeds up the connection:
#define WIFI_CHANNEL 6

WebServer server(80);


//definitions des pins
#define X_STEP_PIN 2
#define X_DIR_PIN 4
#define Y_STEP_PIN 16
#define Y_DIR_PIN 17
#define Z_STEP_PIN 18
#define Z_DIR_PIN 19
#define LED_PIN 5

#define ENDSTOP_X_PIN 32
#define ENDSTOP_Y_PIN 33
#define ENDSTOP_Z_PIN 26
#define homingButton 13
#define DCY_PIN 12 

//declaration des moteurs et servos
AccelStepper stepperX(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper stepperY(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);
AccelStepper stepperZ(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);
Servo myservo;  // create servo object to control a servo


//declaration des variables
int marche=false;
//nbre de pas par mm pour chaque moteur (prévoir de les acquerrer depuis l'interface web)
int nbre_pas_par_mm_pour_stepperX=10;
int nbre_pas_par_mm_pour_stepperY=5;
int nbre_pas_par_mm_pour_stepperZ=2;

// Debounce variables
unsigned long lastDebounceTime1 = 0;
unsigned long lastDebounceTime2 = 0;
const unsigned long debounceDelay = 50; 
bool bouchonAttrappe=false;

//dimension de la matrice A
int MA = 10;
int NA = 10;
std::vector<std::vector<int>> matriceA(MA, std::vector<int>(NA));

//dimension de la matrice B
int MB = 2;
int NB = 3;
std::vector<std::vector<int>> matriceB(MB, std::vector<int>(NB));

//les pas entre deux positions pour A et B en mm (prévoir de les calculer en pas) (prévoir de les acquérir depuis l'interface web)
int pasAX = 5;
int pasAY = 2;
int pasBX = 5;
int pasBY = 1;

//positions initiales pour les deux matrices en mm(prévoir de les calculer en pas) (prévoir de les acquérir depuis l'interface web)
int posAX = 50 ;
int posAY = 10;
int posBX = 150 ;
int posBY = 20;

// position actuelle dans matrice A
int posA_i = 0;
int posA_j = 0;

// position actuelle dans matrice B
int posB_i = 0;
int posB_j = 0;

bool enCours = false;

//declaration des fonctions
void initialisationHardware();
void initialisationMatrices();
void gotoPositionA(int x, int y);
void afficherPosition();
void monterPince();
void descendrePince();
void attraperBouchon();
void gotoPositionB(int x, int y);
void deposerBouchon();
void afficherMatrice(std::vector<std::vector<int>> matrice);
void verifierFinMatrices();
bool trouverProchainePositionDansB(int &i, int &j);
void fermerGripper();
void homing();
void sendHtml() ;
void initialisationWifi();


void setup() {
  Serial.begin(115200);
  initialisationWifi();
  initialisationHardware();
  initialisationMatrices();

}

void loop() {
  server.handleClient();

  //si on appuie sur le bouton DCY, on met la variable marche a true et le cycle commence ou continue selon les cas 
    if (digitalRead(homingButton) == LOW ) {
      marche = false;
      homing();
    }
    if (digitalRead(DCY_PIN) == LOW ) {
    marche = true;
    Serial.println("Démarrage du processus...");
  }

  if(!trouverProchainePositionDansB(posB_i, posB_j)&&digitalRead(DCY_PIN) == LOW){
    Serial.println("La matrice B:");
    posB_i=0;
    posB_j=0;
  for(int i=0; i<MB; i++){
    for(int j=0; j<NB; j++){

      //initialisation de la matrice B
      if ((i+j)%3==0 )
        matriceB[i][j]=1;
      else
        matriceB[i][j]=0;
        Serial.print(matriceB[i][j]);
        Serial.print(" ");
    }
    Serial.println();
  }
  }
  
  if (marche == true) {
    //Vérifier si la matrice A ou B est atteinte
    verifierFinMatrices();


    //Vérifier si la matrice A contient un bouchon à l'emplacement actuel
    if (matriceA[posA_i][posA_j] == 1) {
      
      // Aller à A[posA_i][posA_j]
      
      Serial.println("Aller à la position suivante dans la matrice A");
      Serial.print(posA_i);
      Serial.print(", "); 
      Serial.print(posA_j);
      Serial.println();

      if(bouchonAttrappe == false){
        gotoPositionA(posA_i, posA_j);
        attraperBouchon();
        bouchonAttrappe = true;
         // Marquer la case A comme vide
        matriceA[posA_i][posA_j] = 0;

        // Avancer dans matrice A
       
        posA_j++;
        if (posA_j >= NA) {
            posA_j = 0;
            posA_i++;
        }

      }
      

     

      // Chercher l'emplacement suivant avec un 1 dans matriceB
      if (trouverProchainePositionDansB(posB_i, posB_j)) {

        // Aller à B[posB_i][posB_j]
        Serial.println("Aller à la position suivante dans la matrice B");
        Serial.print(posB_i);
        Serial.print(", ");
        Serial.print(posB_j);
        Serial.println();

        gotoPositionB(posB_i, posB_j);

        descendrePince();
        deposerBouchon();
        bouchonAttrappe=false;
        monterPince();
        matriceB[posB_i][posB_j] = 2;
      
        // Préparer pour la prochaine recherche
        posB_j++;
        if (posB_j >= NB) {
          posB_j = 0;
          posB_i++;
        }
      } else {
        marche = false;
        afficherMatrice(matriceA);
        afficherMatrice(matriceB);
        Serial.println("Plus d'emplacements disponibles dans B.");
      }

     
    }

    
    delay(100); // pour voir le 
    
    

  }

  //si il y a un bouchon, faire descendre la pince (moteur Z)
    //attraper le bouchon (servo gripper)
    //faire monter la pince
    //chercher l'emplacement du bouchon dans la matrice B
    //aller vers la position B[i][j]   correspondante
    //faire descendre la pince
    //placer le bouchon
    //faire monter la pince
    //incrementer la position pour la matrice A pour préparer le prochain mouvement
    //incrementer la position pour la matrice B pour préparer le prochain mouvement
  //sinon, incrementer la position pour la matrice A pour préparer le prochain mouvement
  //si on arrive au bout de la matrice A, on met la variable marche a false et on affiche un message
  //si on arrive au bout de la matrice B, on met la variable marche a false et on affiche un message pour changer de matrice B
  
}

bool trouverProchainePositionDansB(int &i, int &j) {
  for (int x = i; x < MB; x++) {
    for (int y = (x == i ? j : 0); y < NB; y++) {
      if (matriceB[x][y] == 1) {
        i = x;
        j = y;
        return true;
      }
    }
  }
  return false;
}

void verifierFinMatrices(){
  if (posA_i >= MA || posB_i >= MB) {
    marche = false;
    if(posA_i >= MA){
      Serial.println("Fin de la matrice A atteinte.");
    }
    if(posB_i >= MB){
      Serial.println("Fin de la matrice B atteinte.");
    }
    Serial.println("Fin de la matrice A ou B atteinte.");
    afficherMatrice(matriceA);
    afficherMatrice(matriceB);
    return;
  }
}

void initialisationWifi(){
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD, WIFI_CHANNEL);
  Serial.print("Connecting to WiFi ");
  Serial.print(WIFI_SSID);
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
  }
  Serial.println(" Connected!");

  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  server.on("/", sendHtml);
  server.begin();
  
}
void initialisationHardware(){
  //initialisation des moteurs
  stepperX.setMaxSpeed(1000);
  stepperX.setAcceleration(500);
  stepperY.setMaxSpeed(1000);
  stepperY.setAcceleration(500);
  stepperZ.setMaxSpeed(1000);
  stepperZ.setAcceleration(500);
  
  //initialisation des boutons et leds
  pinMode(ENDSTOP_X_PIN, INPUT_PULLUP);
  pinMode(ENDSTOP_Y_PIN, INPUT_PULLUP);
  pinMode(ENDSTOP_Z_PIN, INPUT_PULLUP);
  pinMode(homingButton, INPUT_PULLUP);
  pinMode(DCY_PIN, INPUT_PULLUP);
  //initialisation des servos
  myservo.attach(27);
  
  // initialisation DCY button
  
}

void initialisationMatrices(){
  //initialisation des matrices
  //à faire par l'utilisateur pour indiquer les dimensions et les positions initiales
  //pour la simulation, on peut simplement donner des valeurs arbitraires (la matrice A est supposée pleine de bouchons)
  Serial.println("La matrice A:");

  for(int i=0; i<MA; i++){
    for(int j=0; j<NA; j++){
      //initialisation de la matrice A
      matriceA[i][j]=1;
      Serial.print(matriceA[i][j]);
        Serial.print(" ");
    }
    Serial.println();
  }

  Serial.println("La matrice B:");

  for(int i=0; i<MB; i++){
    for(int j=0; j<NB; j++){

      //initialisation de la matrice B
      if ((i+j)%3==0 )
        matriceB[i][j]=1;
      else
        matriceB[i][j]=0;
        Serial.print(matriceB[i][j]);
        Serial.print(" ");
    }
    Serial.println();
  }

  
}

//fonctions

void gotoPositionA(int x, int y){
  //aller vers la position A[x][y]
  //il faut calculer les positions en pas pour les moteurs X et Y
  stepperX.moveTo((posAX+x*pasAX)*nbre_pas_par_mm_pour_stepperX);
  stepperY.moveTo((posAY+y*pasAY)*nbre_pas_par_mm_pour_stepperY);
  while(stepperX.distanceToGo()!=0 || stepperY.distanceToGo()!=0) {
    stepperX.run();
    stepperY.run();
  }

  
}

void gotoPositionB(int x, int y){
  stepperX.moveTo((posBX + x * pasBX) * nbre_pas_par_mm_pour_stepperX);
  stepperY.moveTo((posBY + y * pasBY) * nbre_pas_par_mm_pour_stepperY);
  while (stepperX.distanceToGo() != 0 || stepperY.distanceToGo() != 0) {
    stepperX.run();
    stepperY.run();
  }
}


void afficherPosition(){

  Serial.print("la position du moteur est: ");

  Serial.print(stepperX.currentPosition());
  Serial.print(", ");
  Serial.println(stepperY.currentPosition());
  
}

void descendrePince() {
  stepperZ.moveTo(-30 * nbre_pas_par_mm_pour_stepperZ); // à adapter
  while (stepperZ.distanceToGo() != 0) {
    stepperZ.run();
  }
}

void monterPince() {
  stepperZ.moveTo(0); // remonte à la position 0
  while (stepperZ.distanceToGo() != 0) {
    stepperZ.run();
  }
}

void deposerBouchon() {
  Serial.println("Gripper ouvre (simulé)");
  myservo.write(0);
  delay(500);
}

void attraperBouchon() {
      delay(500);
      descendrePince();
      delay(500);
      fermerGripper();
      delay(500);
      monterPince();
      delay(500);
}

void afficherMatrice(std::vector<std::vector<int>> matrice){

  for(int i=0; i<matrice.size(); i++){
    for(int j=0; j<matrice[i].size(); j++){
      Serial.print(matrice[i][j]);
        Serial.print(" ");
    }
    Serial.println();
  }
}

void fermerGripper() {
  Serial.println("Gripper ferme");
  myservo.write(180);
  delay(500);
}

void homing(){

  stepperX.moveTo(0);
  stepperY.moveTo(0);
  stepperZ.moveTo(0);

  while(stepperX.distanceToGo() != 0 || stepperY.distanceToGo() != 0 || stepperZ.distanceToGo() != 0){
    stepperX.run();
    stepperY.run();
    stepperZ.run();
  }

  // Homing X
  while(digitalRead(ENDSTOP_X_PIN)==HIGH) {
    stepperX.move(-1);
    stepperX.run();
  }
  stepperX.setCurrentPosition(0);
  

  // Homing Y
  while(digitalRead(ENDSTOP_Y_PIN)==HIGH) {
    stepperY.move(-1);
    stepperY.run();
  }
  stepperY.setCurrentPosition(0);

  // Homing Z
  while(digitalRead(ENDSTOP_Z_PIN)==HIGH) {
    stepperZ.move(-1);
    stepperZ.run();
  }
  stepperZ.setCurrentPosition(0);

  Serial.println("Moteurs calibrés");
}

void sendHtml() {
  String response = R"(
    <!DOCTYPE html><html>
      <head>
        <title>ESP32 Web Server Demo</title>
        <meta name="viewport" content="width=device-width, initial-scale=1">
        <style>
          html { font-family: sans-serif; text-align: center; }
          body { display: inline-flex; flex-direction: column; }
          h1 { margin-bottom: 1.2em; } 
          h2 { margin: 0; }
          div { display: grid; grid-template-columns: 1fr 1fr; grid-template-rows: auto auto; grid-auto-flow: column; grid-gap: 1em; }
          .btn { background-color: #5B5; border: none; color: #fff; padding: 0.5em 1em;
                 font-size: 2em; text-decoration: none }
          .btn.OFF { background-color: #333; }
        </style>
      </head>
            
      <body>
        <h1>ESP32 Web Server</h1>

        <div>
          <h2>LED 1</h2>
          <a href="/toggle/1" class="btn LED1_TEXT">LED1_TEXT</a>
          <h2>LED 2</h2>
          <a href="/toggle/2" class="btn LED2_TEXT">LED2_TEXT</a>
        </div>
      </body>
    </html>
  )";
  // response.replace("LED1_TEXT", led1State ? "ON" : "OFF");
  // response.replace("LED2_TEXT", led2State ? "ON" : "OFF");
  server.send(200, "text/html", response);
}
