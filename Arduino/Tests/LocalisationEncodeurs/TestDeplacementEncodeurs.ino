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


// PINS 

#define PIN_A_LEFT 18	// Pins encodeurs
#define PIN_B_LEFT 19
#define PIN_A_RIGHT 20
#define PIN_B_RIGHT 21

#define ENABLE_L 3  // blanc	// Pins moteurs
#define ENABLE_R 4  // violet
#define MOTEUR_L 5  // bleu
#define MOTEUR_R 6  // gris


// Structures
struct vitesse {
  float v; // vitesse linéaire point asservi en m/s 
  float Omega; //vitesse angulaire point asservi en rad/s
  float Omega_L; // vitesse angulaire roue gauche en rad/s
  float Omega_R;
};




// Dimensions
#define PI 3.14159265358979323

// MAXIMUM
#define MAX_SPEED 30

const double L = 0.1175;	// demi-empattement en m
const double e = 2*L;
const double Rcl = 0.0505;	// rayon roue codeuse gauche en m
const double Rcr = 0.0505;	// rayon roue codeuse droite en m
const double R = 3; //rayon d'une roue
const int npas = 1024;	// nombre d'impulsions par tours encodeurs [en quadruple précision][?]
const double a = 10.0;	// distance point asservi axe des roues (sur l'axe de symétrie) en cm



// Variables globales

volatile long int compteurLeft = 0;
volatile long int compteurRight = 0;
long int dernierCompteurLeft = 0;	// pour la détermination de la position
long int dernierCompteurRight = 0;
int puissanceMoteurMax =  30; // appartient à [-2047 ; 2047]
int puissanceMoteur_L = 0;
int puissanceMoteur_R = 0;
int gTref = 0; //sauvegarde le temps de mesure des compteurs
int dt; //pas de discrétisation

//Valeurs initiales
Pose positionRobot = {0, 0, 0};
vitesse vitesseRobot = {0, 0, 0, 0};
Pose positionPrecedenteRobot = {0, 0, 0};
Pose target = {0, 0, 0}; //position de la cible


/*******************************************************************************************
	FONCTIONS UTILES
*******************************************************************************************/

// Convertit un pin en un numéro d'interruption (la fonction est native ?)
/*int digitalPinToInterrupt(int pin) {
	// Pour une mega :
	if (true) {
		switch (pin) {
			case 2:  return 0; break;
			case 3:  return 1; break;
			case 21: return 2; break;
			case 20: return 3; break;
			case 19: return 4; break;
			case 18: return 5; break;
		}		
	}
	// Pour une Uno :
	else if (false) {
		return pin - 2;
	}
}*/

// Fonctions appelées par les interruptions (les encodeurs ne sont pas dans le même sens)
// (Trop écrire dans le serial fait crash)
void incrementerCompteurRight() {
	if (digitalRead(PIN_B_RIGHT)) compteurRight++;
	else compteurRight--;
	
	/*Serial.print("compteurDroite = ");
	Serial.println(compteurDroite);*/
}

void incrementerCompteurLeft() {
	if (digitalRead(PIN_B_LEFT)) compteurLeft--;
	else compteurLeft++;
	
	/*Serial.print("compteurGauche = ");
	Serial.println(compteurGauche);*/
}



// Update de la position à l'aide des codeurs 
void actualiserPositionVitesse() {

  dt = millis() - gTref ;
  gTref = millis() ;
  
  float deltaLeft = (compteurLeft - dernierCompteurLeft) * (2*PI) / npas;  
  float deltaRight = (compteurRight - dernierCompteurRight) * (2*PI) / npas;

  vitesseRobot.Omega_L = deltaLeft / dt * 1000;
  vitesseRobot.Omega_R = deltaRight / dt * 1000;
  vitesseRobot.Omega =  (Rcr*vitesseRobot.Omega_R - Rcl*vitesseRobot.Omega_L) / e; 
  vitesseRobot.v =  (Rcl*vitesseRobot.Omega_L + Rcr*vitesseRobot.Omega_R) / 2;

  /*float avancee = (deltaLeft + deltaRight) / 2;
  float rotation = (deltaRight - deltaLeft)  / e;

  positionPrecedenteRobot = positionRobot;
  
  positionRobot.x += avancee * cos(positionRobot.theta + rotation/2);
  positionRobot.y += avancee * sin(positionRobot.theta + rotation/2);
  positionRobot.theta += rotation; */

  dernierCompteurLeft = compteurLeft;  
  dernierCompteurRight = compteurRight;   
}



/*void afficherPosition(position M)
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
} */

/*void goTo(const Pose &_p)
{
    // error in robot frame
    Pose error = _p.transformInverse(positionRobot); //erreur de position

    int alpha = 20;
    int consigneV = error.x; //consigne en vitesse linéaire
    int consigneW = alpha*error.y; //consigne en vitesse angulaire

    int consigneOmega_L = (consigneV + e * consigneW) / R; //consigne vitesse angulaire roue gauche
    int consigneOmega_R = (consigneV - e * consigneW) / R; //consigne vitesse angulaire roue droite
    
    //try to do a straight line
    //commande des moteurs, asservissement en vitesse
    
    puissanceMoteur_L = puissanceMoteur_L * consigneOmega_L / vitesseRobot.Omega_L;
    puissanceMoteur_R = puissanceMoteur_R * consigneOmega_R / vitesseRobot.Omega_R;

    int a = max (puissanceMoteur_L/puissanceMoteurMax, puissanceMoteur_R/puissanceMoteurMax);
    if (a<1) {
      a = 1;
    }

    puissanceMoteur_L = puissanceMoteur_L / a;
    puissanceMoteur_R = puissanceMoteur_R / a; 

    //on suppose que LOW -> avancer, HIGH -> reculer
    
    digitalWrite(MOTEUR_L, !(error.x > 0));
    analogWrite(MOTEUR_L, puissanceMoteur_L);
    digitalWrite(MOTEUR_R, !(error.y > 0));
    analogWrite(MOTEUR_R, puissanceMoteur_R);
}; */

/*void rotate(const Pose &_p) {
  
    // error in robot frame
    Pose error = _p.transformInverse(positionRobot); //erreur d'angle
    
}; */
/*******************************************************************************************
	CORPS DU PROGRAMME
*******************************************************************************************/

void setup() {	
	
	Serial.begin(9600);
	Serial.println("Debut !");
	
	// Ouverture des pins
	pinMode(INPUT, PIN_A_LEFT);
	pinMode(INPUT, PIN_B_LEFT);
	pinMode(INPUT, PIN_A_RIGHT);
	pinMode(INPUT, PIN_B_RIGHT);
	
	pinMode(MOTEUR_R, OUTPUT); // avant - arrière
	pinMode(MOTEUR_L, OUTPUT);
	pinMode(ENABLE_R, OUTPUT); // puissance
	pinMode(ENABLE_L, OUTPUT);
	
	// Déclaration des interruptions
	attachInterrupt(digitalPinToInterrupt(PIN_A_LEFT), incrementerCompteurLeft, RISING);
	attachInterrupt(digitalPinToInterrupt(PIN_A_RIGHT), incrementerCompteurRight, RISING);
}


int compteurAffichage = 0;

void loop() {	
	actualiserPositionVitesse();
  compteurAffichage +=1;
  if (compteurAffichage % 10 == 0) {
    Serial.print(" v = ");
    Serial.print(vitesseRobot.v);
    Serial.print("    omega = ");
    Serial.println(vitesseRobot.Omega * 180 / PI);
  }
  delay(5); // 200 Hz
	
	// Moteurs
	//goTo(target); //va en (x,y) de la target
  //rotate(target); //s'oriente suivant le theta de la target
	
	/*if (true) {  //(compteurDroite < 500) {
		digitalWrite(MOTEUR_D, HIGH);
		analogWrite(ENABLE_D, 50);
	} else {
		digitalWrite(MOTEUR_D, LOW);
		analogWrite(ENABLE_D, 0);
	}*/

  /*int commandeMoteur = 100;
	
	if (compteurGauche < 500) {
		digitalWrite(MOTEUR_G, LOW);
		analogWrite(ENABLE_G, commandeMoteur);
	} else {
		digitalWrite(MOTEUR_G, LOW);
		analogWrite(ENABLE_G, 0);
	}*/
	

	/*digitalWrite(MOTEUR_D, LOW);
	digitalWrite(MOTEUR_G, LOW);
	analogWrite(ENABLE_D, 35);
	analogWrite(ENABLE_G, 35); */

	
	/*Serial.println(compteurGauche);
	delay(200);*/
	
}

