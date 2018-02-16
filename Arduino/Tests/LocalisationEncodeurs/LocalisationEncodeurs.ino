/*
	Test du déplacement assisté par les encodeurs
	
	
	Matériel : pont en H L298, deux moteurs, deux encodeurs incrémentaux	

	Description fiche encodeur (inverted signal) :
	 _______________________________________   _________________________
	| X 	| Gris | Vert  | Bleu  | Marron | |  X  | B  | A  | 0  | 5V |
	| Blanc | Rose | Jaune | Rouge | X      | | GND | B/ | A/ | 0/ | X  |
	|---------------------------------------| |-------------------------|
	
	Branchements encodeurs : 
		- A gauche 18
		- B gauche 19
		- A droite 20
		- B droite 21
		
	Branchement du pont en H : 
		- E1 (violet)	pin 4
		- M1 (gris)		pin 6
		- E2 (blanc)	pin 3
		- A2 (bleu)		pin 5
		- fil rouge		5V
		
	
	Correspondance pin-interruption (mega) :		
	http://www.arduino.org/learning/reference/attachinterrupt
*/


#include "Definitions.h"


// PINS 

#define PIN_A_GAUCHE 18	// Pins encodeurs
#define PIN_B_GAUCHE 19
#define PIN_A_DROITE 20
#define PIN_B_DROITE 21

#define ENABLE_G 3  // blanc	// Pins moteurs
#define ENABLE_D 4  // violet
#define MOTEUR_G 5  // bleu
#define MOTEUR_D 6  // gris


// Dimensions
#define PI 3.14159265359

// MAXIMUM
#define MAX_SPEED 30

const double L = 11.75;	// demi-empattement en cm
const double e = 2*L;
const double Rcg = 5.05;	// rayon roue codeuse gauche en cm
const double Rcd = 5.05;	// rayon roue codeuse droite en cm
const int npas = 1024;	// nombre d'impulsions par tours encodeurs [en quadruple précision][?]
const double a = 10.0;	// distance point asservi axe des roues (sur l'axe de symétrie) en cm


// Variables globales

volatile long int compteurGauche = 0;
volatile long int compteurDroite = 0;
long int dernierCompteurGauche = 0;	// pour la détermination de la position
long int dernierCompteurDroite = 0;

position positionRobot = {0, 0, 0};


/*******************************************************************************************
	FONCTIONS UTILES
*******************************************************************************************/

// Fonctions appelées par les interruptions (les encodeurs ne sont pas dans le même sens)
// (Trop écrire dans le serial fait crash)
void incrementerCompteurDroite() {
	if (digitalRead(PIN_B_DROITE)) compteurDroite++;
	else compteurDroite--;
	
	/*Serial.print("compteurDroite = ");
	Serial.println(compteurDroite);*/
}

void incrementerCompteurGauche() {
	if (digitalRead(PIN_B_GAUCHE)) compteurGauche--;
	else compteurGauche++;
	
	/*Serial.print("compteurGauche = ");
	Serial.println(compteurGauche);*/
}

// **********************************************************************************************
// Update de la position à l'aide des codeurs
void actualiserPosition() {
  float deltaGauche = (compteurGauche - dernierCompteurGauche) * (2*PI*Rcg) / npas;  // en cm
  float deltaDroite = (compteurDroite - dernierCompteurDroite) * (2*PI*Rcd) / npas;

  float avancee = (deltaGauche + deltaDroite) / 2;
  float rotation = (deltaDroite - deltaGauche)  / e;

  positionRobot.x += avancee * cos(positionRobot.angle + rotation/2);
  positionRobot.y += avancee * sin(positionRobot.angle + rotation/2);
  positionRobot.angle += rotation; 

  dernierCompteurGauche = compteurGauche;  
  dernierCompteurDroite = compteurDroite;   
}



void afficherPosition(position M)
{
	Serial.print("CG=");
	Serial.print(compteurGauche*2*PI*Rcg/npas);
	Serial.print(" ; x=");
	Serial.print(M.x);
	Serial.print(" ; y=");
	Serial.print(M.y);
	Serial.print(" ; angle=");
	Serial.println(int(M.angle * 180 / PI) % 360);
  //Serial.println(M.angle);
}



/*******************************************************************************************
	CORPS DU PROGRAMME
*******************************************************************************************/

void setup() {	
	
	Serial.begin(9600);
	Serial.println("Debut !");
	
	// Ouverture des pins
	pinMode(INPUT, PIN_A_GAUCHE);
	pinMode(INPUT, PIN_B_GAUCHE);
	pinMode(INPUT, PIN_A_DROITE);
	pinMode(INPUT, PIN_B_DROITE);
	
	pinMode(MOTEUR_D, OUTPUT);
	pinMode(MOTEUR_G, OUTPUT);
	pinMode(ENABLE_D, OUTPUT);
	pinMode(ENABLE_G, OUTPUT);
	
	// Déclaration des interruptions
	attachInterrupt(digitalPinToInterrupt(PIN_A_GAUCHE), incrementerCompteurGauche, RISING);
	attachInterrupt(digitalPinToInterrupt(PIN_A_DROITE), incrementerCompteurDroite, RISING);
}


int compteurAffichage = 0;

void loop() {	
	actualiserPosition();

	if (compteurAffichage >= 100) {
		afficherPosition(positionRobot);
		compteurAffichage = 0;
	} else compteurAffichage++;

    delay(5); // 200 Hz

	
}


