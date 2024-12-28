#include <Arduino.h>
#include <schSTM32.h>
#include <SPI.h>
#include <Keypad.h>
#include <Servo.h>
#include <LiquidCrystal.h>

Servo myServo;                      // Instance du servomoteur
const int servoPin = D2;            // Pin du signal du servomoteur
#define chipselectpin D1

const int ROW0 = D8;
const int ROW1 = D9;
const int ROW2 = D10;
const int ROW3 = D14;
const int COLS0 = D7;
const int COLS1 = D6;
const int COLS2 = D5;
const int COLS3 = D4;

const byte ROWS = 4;
const byte COLS = 4;

String correctPassword = "2003";  // Mot de passe correct
String inputString = "";          // Mot de passe saisi
const unsigned long timeout = 30000; 
int compteur = 0; 

enum State { 
    STATE_IDLE, 
    STATE_COLLECTING, //saisie
    STATE_CHECKING, //verification
    STATE_SUCCESS, //correct
    STATE_FAILURE //incorrect
}; 

State currentState = STATE_IDLE;  // Correction de l'état initial
unsigned long stateStartTime = 0;

// Définir les symboles sur les boutons du pavé numérique
char hexaKeys[ROWS][COLS] = {
  {'1', '2', '3', 'F'},
  {'4', '5', '6', 'E'},
  {'7', '8', '9', 'D'},
  {'A', '0', 'B', 'C'}
};

byte rowPins[ROWS] = {ROW0, ROW1, ROW2, ROW3}; // Connecter aux broches des lignes du pavé
byte colPins[COLS] = {COLS0, COLS1, COLS2, COLS3}; // Connecter aux broches des colonnes du pavé

// Initialiser une instance de Keypad
Keypad customKeypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);

// Variables pour le contrôle du servo
unsigned long lastUpdateTime = 0;          // Moment de la dernière mise à jour du servo
unsigned long rotationStartTime = 0;      // Moment où le mouvement de rotation a commencé
const unsigned long updateInterval = 15; // Intervalle de temps entre chaque mise à jour 
int rotationSpeed = 90;                   // Vitesse actuelle du servo (90 = position neutre, pas de mouvement)
static bool isMovingForward = true;       // Direction du mouvement (true = avant, false = arrière)
const unsigned long fullRotationTime = 180; // Durée pour une rotation complète

// Suivi de l'état du servo
bool isServoActive = false; // Indique si le servo est actuellement en mouvement

void Servo_rotate(bool direction) {
  if (!isServoActive) {
    isServoActive = true;                    // Indique que le servo est actif
    rotationSpeed = 90;                      // Réinitialise la vitesse à la position neutre
    isMovingForward = direction;             // Définit la direction du mouvement
    lastUpdateTime = millis();               // Réinitialise le minuteur de mise à jour
    rotationStartTime = millis();            // Réinitialise le minuteur de début de rotation
  }
}

void updateServo() {
  if (isServoActive) {
    unsigned long currentMillis = millis(); //depuis le demarrage

    if (currentMillis - lastUpdateTime >= updateInterval) { 
      lastUpdateTime = currentMillis; //Vérifie si suffisamment de temps s'est écoulé depuis la dernière mise à jour

      if (isMovingForward) {
        rotationSpeed--; 
      } else {            //rotationSpeed contrôle la vitesse et la direction du servomoteur
        rotationSpeed++; 
      }

      myServo.write(rotationSpeed); //envoie la nouvelle vitesse au servomoteur
    }

    if (millis() - rotationStartTime >= fullRotationTime) {
      isServoActive = false;      // servo inactif
      rotationSpeed = 90;         // Position neutre pour arrêter le servo
      myServo.write(90);   // Envoie la commande pour arrêter le servo
    }
  }
}

void openAndCloseServo() {
    myServo.write(90); // 90 deg
    delay(3000);       // 3s
    myServo.write(0);  // 0 deg
}

void lcd_write_byte(byte x) {
  digitalWrite(chipselectpin, LOW);//esclave selectionné = afficheur activé
  SPI.transfer(x); //envoie x via spi
  digitalWrite(chipselectpin, HIGH); //deselectionné l'esclave apres l'envoie
  delay(1);
}

void lcd_clear() {
  lcd_write_byte(0xFE); //prochaine données sera une instruction
  lcd_write_byte(0x51); //clear
  delay(5);
}

void lcd_write_string(String v) { //evoie une chaine de caractere
  for (size_t i = 0; i < v.length(); i++) { //calcule longueur de chaine
    lcd_write_byte(v[i]); //envoie octet par octet pour l'affichage
  }
}

void lcd_setCursor(uint8_t col, uint8_t row) {
  uint8_t offset = (row == 1) ? 0x40 : 0; //ajuster position du curseur
  lcd_write_byte(0xFE); 
  lcd_write_byte(0x45); //set cursor
  lcd_write_byte(col + offset); //ligne+colonne
}

bool nonBlockingDelay(unsigned long duration, unsigned long &startTime) { //Fournir une alternative non-bloquante
  if (millis() - startTime >= duration) {//le temps courant-temps de depart
    return true;// Le délai est écoulé
  }
  return false;
}

void resetStateTimer() { //Réinitialiser le minuteur d'état
  stateStartTime = millis();
}


bool isTimeout() {
  return (millis() - stateStartTime) > timeout;
}

void handleWaitingForCode() {
  lcd_clear();

  lcd_setCursor(0, 0);  // Positionner le curseur à la première ligne, première colonne
  lcd_write_string("Enter Code:");

  inputString = "";  // Réinitialiser la chaîne de texte actuelle pour une nouvelle entrée utilisateur

  currentState = STATE_COLLECTING; // Changer l'état actuel à COLLECTING pour commencer la collecte de l'entrée utilisateur

  resetStateTimer(); // Réinitialiser le chronomètre de l'état pour gérer les délais
}

void handleCollectingState(char key) {
  static unsigned long stateStartTime = 0;  //suivre le moment où l'état a été activé
  unsigned long currentTime = millis();   

  if (isTimeout()) {
    if (stateStartTime == 0){
      lcd_clear(); 
      lcd_write_string("Timeout!");  
      stateStartTime = millis(); 
    }
    if (nonBlockingDelay(2000, stateStartTime)) { //attendre 2 secondes avant de revenir à l'état STATE_IDLE
      currentState = STATE_IDLE;  
      stateStartTime = 0;       
      return;
    }
  }

  if (key != NO_KEY) {
    if (key == 'C') {
      if(inputString.length()) {
        inputString.remove(inputString.length() - 1);  
      }
      lcd_clear();
      lcd_write_string(inputString);
    } else if (key == '#') {
      currentState = STATE_CHECKING;  
      resetStateTimer();  
    } else {
      inputString += key;
      lcd_clear();
      lcd_write_string(inputString);
    }

    if (inputString.length() == 4) {
      currentState = STATE_CHECKING;
      resetStateTimer();
    }
  }
}

void handleValidateState(){
  if(inputString == correctPassword){
    currentState = STATE_SUCCESS;
  } else {
    compteur++;
    currentState = STATE_FAILURE;
  }

  resetStateTimer();
}

void handleAccessGrantedState(){
  static unsigned long stateStartTime = 0;  

  if (stateStartTime == 0){    
    lcd_clear();
    lcd_write_string("correct");
    Servo_rotate(true);
    openAndCloseServo(); 

    stateStartTime = millis();  // Enregistre l'heure à laquelle cet état a commencé
  }

  if(nonBlockingDelay(2000, stateStartTime)){
    compteur = 0;      // Réinitialise le compteur d'erreurs
    currentState = STATE_COLLECTING;  
    stateStartTime = 0;       
  }
}

void handleAccessDeniedState(){
  static unsigned long stateStartTime = 0;

  if (stateStartTime == 0) {
    lcd_clear(); 
    lcd_write_string("Access Denied");  
    stateStartTime = millis();
  }

  if (nonBlockingDelay(2000, stateStartTime)) {
    currentState = STATE_IDLE;
    stateStartTime = 0;
  }
}

void Update_state(void) {
  char key = customKeypad.getKey();
  updateServo();

  switch (currentState) {
    case STATE_IDLE:
      handleWaitingForCode();
      break; //Empêcher l'exécution en cascade
    case STATE_COLLECTING:
      handleCollectingState(key);
      break;
    case STATE_CHECKING:
      handleValidateState();
      break;
    case STATE_SUCCESS:
      handleAccessGrantedState();
      break;
    case STATE_FAILURE:
      handleAccessDeniedState();
      break;
    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  
  pinMode(chipselectpin, OUTPUT);
  SPI.begin();
  SPI.beginTransaction(SPISettings(100000 , MSBFIRST, SPI_MODE3)); //vitesse de comm + ordre de bit + mode de spi

  lcd_clear();
  lcd_setCursor(0, 0);
  lcd_write_string("Enter Code:");

  myServo.attach(servoPin);  // Connecter le servomoteur
  myServo.write(0);         

  Serial.println("Initializing Scheduler...");
  SCH_Init(2, HERTZ_FORMAT);  // 2Hz (1 / 2 Hz = 0.5 secondes) toutes les 500ms
  delay(2000);
  Serial.println("Scheduler initialized");

  Serial.println("Add Task : Update_state...");
  SCH_Add_Task(Update_state, 0, 1);  // un délai initial + l'intervalle d'exécution de la tâche en "ticks"
  Serial.println("Task, Update_state : added");
  Serial.println("Starting Scheduler...");
  SCH_Start();
  Serial.println("Scheduler Started");
}

void loop() {
  SCH_Dispatch_Tasks();
}
