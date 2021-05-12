/***************************************************************
**  R2Ddemi - Robot Mobile Arduino autonome                   **
**  Auteur : B. Gillet                                        **
**  Version 14    - Mai 2021                                 **
**  Consulter : https://arduino-self.over-blog.com/           **
****************************************************************/


#define abs(x) ((x)>0?(x):-(x))

#include <FastLED.h>   //https://github.com/FastLED/FastLED

#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver ServeurServo = Adafruit_PWMServoDriver();  //Adresse par défaut 0x40

#include <math.h>
#include <Wire.h>
#include <SoftwareSerial.h>
SoftwareSerial HC05(10, 11); // Rx, Tx  --> Module BlueTooth

// ******* Capteur effet hall    ******/
#define Pin_Hall 1        // Broche analogique pour le signal capteur Hall
#define ClicChronoHall 40
#define SeuilHall 530     // Valeur à positionner en fonction du capteur utilisé
#define CorrHall 1   // 1.05  corrige, la distance reelle  : reelle = corrhall x mesuree

int ValHall = 0; 
int ValHallAvant = 0;
unsigned int ClicHall = 0;   
unsigned int DistParcourue = 0;
unsigned long MlssAvantHall = 0;
unsigned long MlssMaintenant = 0;

unsigned int ClicHallEvt = 0;   
unsigned int DistParcourueEvt = 0;

// ************** Lecture boussole   ********************
#define CMPS12_ADDRESS 0x60  // Address I2C CMPS12 
#define ANGLE  2           // Adresse debut lecture
#define CONTROL_Register 0
// Boussole et Cap
int azimuth;
int CapInit;
int CapCalcule;
int CapASuivre;
int CapActuel;


// **********  Echange Blue Tooth   *************
String messageRecu = "";
char c;


//******** Parametres servo roues Droite et gauche
#define VitssGaucheCons 1870   
#define VitssDroitCons  1200    
#define PtMortServoRoue 1500


// **** Lecture Tf Luna   et distances  **********
uint16_t DistCm     = 0; //distance
uint16_t Erreur     = 0; // Erreur
const byte TfLuna = 0x10; //Adresse I2C du capteur
int Distance = 0;
#define  MaxDistance 2500
int DistRoulEvt;
int DistParcLuna;
#define PointPivot 0.6   // % de la distance à parcourir avant de repivoter sur le cap initial (Arbitraire)

// Mesures de distances réelles
struct Radar {
  byte    Angle;
  int     Distance;
  int     XMes;
  int     YMes;
  float   b;
  float   a;
};
Radar TabRadar[37];

byte NbLectRadar = 36;    // Indic Max du tableau
#define IRadGauche 21     // Borne inf pour ScanMarche
#define IRadDroite 15     // Borne Sup pour ScanMarche
#define IRMedian   18     // I equiv. 90°
byte IRadMarcheEncours = IRadDroite;
float   AngleMesure;    // Pas d'incrementation en ° de TABRADAR'
boolean FSimuDist = false;
int DistMax = 0;


int DistDvt;    // Distance devant lors de l'évitement

// Mesures lissées
struct Lissage {
  byte    Irad;
  byte    Angle;
  int     XDrl;
  int     YDrl; 
  boolean DZO;
  int     YZO;
  int     Distance;
};

Lissage TabLissage[14];
#define SeuilLissage 270       // histo 300, 280,  250

// Action à faire
struct Actions {
  byte TypeAction;
  int  P1;
  int  P2;
  boolean Debut;
  boolean Fin;
};
  
Actions TabActions[5];

byte IActions = 0;
byte NActions = 0;
byte IActionEncours = 0;


// ******* Traitement des mesures
byte ILis = 0;
byte IYmax = 0;  // Indice du plus grand Y dans le tableau de Lissage
byte IYmaxReel = 0;  // Indice du plus grand Y dans le tableau des mesures réelles
int  Ymax = 0;
byte IradScan = 0;


//******* Variable de Chrono ********
#define ClicChronoScan 100  
#define ClicChronoScanMarche 100  
#define ClicChronoSurv 300  // Valeur timer de surveillance de la route
#define ClicChronoLed 300

unsigned long MlssAvantScan = 0;
unsigned long MlssAvantSurv = 0;
unsigned long MlssTop = 0;
unsigned long MlssAv = 0;
unsigned long MlssAvantLed = 0;

// ********* Variables d'actions
boolean ScanEntier = false;         // Au moins un scan entier a été fait
boolean ScanAFaire = true;          // Un scan est à faire
boolean ScanATraiter = true;        // Les données scan sont à traiter
boolean FObsATraiter = false;   
boolean DecisionAPrendre;           // Une décision est à prendre dans KessKonFait
boolean RobotRoule = false;         // True si le robot est en train de rouler
boolean FScanMarche = false;        // True = Au moins un scan complet Marche a été fait
boolean Go = false;                 //  True pour que le robot fonctionne
boolean EviterEMaitenant;           // True si obstacle a eviter, en cours d'evitement
int     KesKisPass;                // Contient les données d'évitement
byte    EvitPhase;                  // Phase de l'évitement
byte    OuEstObstacle;             // pour l'évitement
boolean FScanMarcheDemande = true;

// **** Affichage
String Coord;
byte icoord = 0;


// *** Mode simulation *****
const boolean FSimulation = false;     // true = simulation, les mesures sont chargées par la fonction Simultaion
const boolean FAllerDevant = true;      // Mode test, si false le robot tourne mais n'avance pas


//*** Obstacles en marche
#define ProfVision    700 // Limite en deça de laquelle on fait un evitement d'obstacle
#define LargCouloir 200 // Limite minimale pour trouver une voie libre pour un évitement
#define DistArrUrg  200  // Distance Arrêt Urgence

//*** Obstacles en marche
#define ProfVisionEvt    300 // Limite en deça de laquelle on fait un evitement d'obstacle
#define LargCouloirEvt   120 // Limite minimale pour trouver une voie libre pour un évitement

// Journal des erreurs et de la progression
#define NiveauProg  2
#define NiveauErr   3

//*** FastLed   ***/
#define NUM_STRIPS 1
#define NUM_LEDS 10
#define BRIGHTNESS 10
#define LED_TYPE WS2813
#define COLOR_ORDER RGB    //BRG//RGB
#define FASTLED_ALLOW_INTERRUPTS 0
#define FASTLED_INTERRUPT_RETRY_COUNT 1
#define FRAMES_PER_SECOND 20
#define COOLING 55
#define SPARKING 120
#define LedPin  22

byte IledEncours = 10;

CRGB BarreLeds[NUM_LEDS];
boolean Fleds = true;
boolean Clignote = false;
byte R = 0;         
byte G = 0;
byte B = 0;
byte ScenarioLed;   // Type d'animation barre de leds

/*******************/


void setup() {

  // **** Barre de Led    ****/
  if (Fleds) {
      FastLED.addLeds<LED_TYPE, LedPin, COLOR_ORDER>(BarreLeds, NUM_LEDS);
      FastLED.setBrightness(  BRIGHTNESS );
  }

  pinMode(Pin_Hall, INPUT);     // Capteur effet Hall
 
  Serial.begin(9600);    // Init liaison Série
  Wire.begin();         // Init I2C
  HC05.begin(9600);     // Init composant BlueTooth

  // ******   Init serveur de servo  PCA  ******
  ServeurServo.begin();
  ServeurServo.setPWMFreq(50);
  
  // ****************************************************

  ScanAFaire = true;          // Demande scan environnement
  ScanATraiter = true;        // Les données sont traitéees pour determiner un cap
  DecisionAPrendre = true;    //  force à prendre une décision au démarrage
  InitTabRadar ();            // Init du tableau TabRadar
  
  PositionServo(3,0);         // On met la tourelle à 0°
  MonDelay(50);
  InitMoteurs();              // Arrêts des moteurs par sécurité
  ClicHall = 0;
  DistParcourue = 0;

  Terminal(">",String(0));
  Terminal("@",F("OK pour demarrage"));

  R = 0; G = 0 ; B = 255;
  ScenarioLed = 4;   // Fixe
  GestionLeds(ScenarioLed,R,G,B);  //Bleu
  
  
  // Ecoute Bluetooth pour Démarrage
  while (!Go) {
   while (! VerifBT() ){ };
    DecodeBT ();
  }


  CapActuel = LectAzimuth();  
  CapInit =  CapActuel;
  
  Terminal("@",String(F("CapActuel ")) + String(CapActuel));
  
  R = 255; G = 0 ; B = 0;  // Vert
  ScenarioLed = 2;   // Chenillard
  
/*   Permet de vérifier le bon fonctionnement de le roue arrière (Distance) avant démarrage
  Terminal("@",String(F("Test roulette / Suite ")) );   // Pour vérifier que la roulette Hall fonctionne
  while(true)
  {
    if (VerifBT()) {break;} 
    LectureHall();
  }
*/
}

void loop() {


  if (VerifBT()) {DecodeBT();}   // Ecoute du terminal bluetooth


  Parametres();       
  

  if (Go) {    
  
      Regarder();       // Scan l'environnement avec le Tf-luna
      
      Decision();      // Décide et planifie des actions à faire
      
      Action();        // Lance les actions de déplacements

      Surveiller();    // Vérifie les données en cours de déplacement du robot 
      
      Eviter();      // Gere lévitement des obstacles

  }

}


/********************************************************************************
 *    Fonctions
********************************************************************************/

void Parametres () {

    CapActuel = LectAzimuth();  // Actualisation du Cap
    Terminal("z",String(CapActuel));
  
    LectureHall();    // Met à jour les distances parcourues
    
    GestionLeds(ScenarioLed,R,G,B); 
    
}

void Eviter () {     // Se charge des pahses 1 et 3 de l'évitement d'obstacle


  if (!EviterEMaitenant) {return;}
  
  int AngleVise;
  
  switch(EvitPhase) {
  case 1:    // Pivoter + demarrer   Phase 1
        JournalProgession(2,String(F("Evitement ph 1 ")) + String(OuEstObstacle));
        InitMoteurs(); 
        AngleVise = AngleAjout (CapActuel, KesKisPass);
        pivoter( CapActuel, AngleVise, 0);                            // pivote pour éviter l'obstacle
        DistDvt = MesureTourelle(TabRadar[IRMedian].Angle, true);     // Prend une mesure de distance pour controler distance à parcourir 
        MonDelay(200);
        DistParcourueEvt = DistParcourue;               
        AllerDevant();
        EvitPhase++;   // Passe à phase 2 =  surveiller
        FScanMarcheDemande = false;                                   // Désactive le scan marche (pour évier les rebonds)
        break;
        
  case 3:    // Pivoter pour se remettre dans l'axe   phase 3
        JournalProgession(2,String(F("Evitement ph 3 ")) + String(OuEstObstacle));
        InitMoteurs(); 
        //AngleVise = AngleAjout (CapActuel, KesKisPass);
        pivoter(LectAzimuth(),  CapASuivre,0);                   // pivote pour se remettre sur le cap initial
        Terminal("#", "Evit ph 3 " + String(CapASuivre));
        AllerDevant();
        EviterEMaitenant = false;
        FScanMarcheDemande = true;                              // Résactive le scan marche 
        R=255;G=0;B=0;   // Leds vertes
        break;

  default:
  
  break;
  }
  
}



void Regarder () {

  // Scan LIDAR de l'environnement et traitement des données si nécessaire et pour prise de décision

  if (ScanAFaire) {
        if ( ScanAutour() ) {   // =true --> Le scan est fait sur 180°

            TraiteDonneesScan ();            
            AfficheLissage(ILis);
            ScanAFaire = false;   // Scan complet fait et traité
            FScanMarche = false;   // Indique que les valeurs du ScanMarche ont été écrasées
            IRadMarcheEncours = IRadDroite;  // On se repositionne pour le ScanMarche

        }
  }
  else {
        // Scan LIDAR  si robot en mouvement pour controle trajectoire
        if ( (RobotRoule || !FScanMarche) && FScanMarcheDemande ) { ScanMarche(); }  
  }
  
}


boolean Decision () {   // Se charge du la partie trajectoire et planification des actions
  int CapRbot;
  

  if (!DecisionAPrendre ) {return false;}
  if (ScanAFaire) {return false;}

  JournalProgession(4,String(F("Kskonfait ")) + String(DecisionAPrendre));
  
  memset(TabActions, 0, sizeof(TabActions));
  IActions = 0; NActions = 0;IActionEncours=0;

    
  CapRbot = Trajectoire(IYmaxReel);   
  if (CapRbot == 9999) {
      InitMoteurs();
      JournalErreurs(3,F("1 - Calcul chemin Imposs - Arret"));
      return false;
  }
  // Si la distance mesurée pour le YMax > MaxDistance, on y va sans se poser d'autres questions
  if (TabRadar[IYmaxReel].Distance > MaxDistance) {
 
      // ACTION : Rouler tout droit sur CapASuivre, jusqu'au prochain obstacle
      Terminal("@",F("Choix Traj A1"));
      
      
      TabActions[IActions].TypeAction = 1;  //pivoter sur Cap a Suivre
      TabActions[IActions].P1 = CapASuivre;
      TabActions[IActions].P2 = TabRadar[IYmaxReel].Distance;
      TabActions[IActions].Debut = false;    
      TabActions[IActions].Fin = false;
      IActions++; NActions++;
      
      TabActions[IActions].TypeAction = 3;  // tout droit sur une certaine distance
      TabActions[IActions].P1 = TabRadar[IYmaxReel].Distance * PointPivot ;
      TabActions[IActions].Debut = false;   
      TabActions[IActions].Fin = false;
      IActions++; NActions++;
      
      TabActions[IActions].TypeAction = 1;  //pivoter sur le Cap Initial
      TabActions[IActions].P1 = CapInit;
      TabActions[IActions].Debut = false;   
      TabActions[IActions].Fin = false;
      IActions++; NActions++;

      DecisionAPrendre = false;
      AfficheCap(CapRbot);
      AfficheActions(F("A1"));
      Terminal("i",String(CapInit));
      Terminal("a",String(CapASuivre));
      
      return true;
       
  }
  
   // On regarde si des zones d'ombres sont définies de chaque coté du Ymax, si non pas la peine de creuser de ce coté là, c'est bouché'

   byte ILisAv = IYmax + 1;
   
     
  if (( TabLissage[IYmax].DZO ) || ( ILisAv <= 30 && TabLissage[ILisAv].DZO )) {
     
        // On detecte au moins une zone d'ombre, on peut explorer'  
        // On va chercher la distance de l'obstacle sur la trajectoire du YMax '

        
        // ACTION : Rouler tout droit sur une distance de "DistRoulage" et sur un cap de CapAsuivre et refaire un scan 
       
        Terminal("@",F("Choix Traj A2"));

        
        TabActions[IActions].TypeAction = 1;  //pivoter sur Cap a Suivre
        TabActions[IActions].P1 = CapASuivre;
        TabActions[IActions].P2 = TabRadar[IYmaxReel].Distance;
        TabActions[IActions].Debut = false;   
        TabActions[IActions].Fin = false;
        IActions++; NActions++;        

        
        TabActions[IActions].TypeAction = 3;  // tout droit sur une distance X
        TabActions[IActions].P1 = TabRadar[IYmaxReel].Distance * PointPivot;
        TabActions[IActions].Debut = false;   
        TabActions[IActions].Fin = false;
        IActions++; NActions++;       
        
        TabActions[IActions].TypeAction = 1;  //pivoter sur le Cap Initial
        TabActions[IActions].P1 = CapInit;
        TabActions[IActions].Debut = false;   
        TabActions[IActions].Fin = false;
        IActions++; NActions++;


        DecisionAPrendre = false;
        AfficheCap(CapRbot);
        AfficheActions(F("A2"));
        Terminal("i",String(CapInit));
        Terminal("a",String(CapASuivre));
        return true;
  } 
   
// Pas de YMAX exploitable , on change de stratégie

// On prend en compte la distance maximum mesurée

    int DistMT = 0;
    byte iDAngleDmax;
    for (byte iD = 0; iD <= ILis ; iD++) {
             if (TabLissage[iD].Distance > DistMT) {
                  DistMT = TabLissage[iD].Distance;
                  iDAngleDmax = iD;
             }
    }

// Si la distance mesurée maxi > MaxDistance, on y va sans se poser d'autres questions

  if (TabLissage[iDAngleDmax].Distance > MaxDistance) {
        CapRbot = Trajectoire(iDAngleDmax);    // TODO !!!!!!!!    Mettre le Y réel   !!!!!!!
        if (CapRbot == 9999) {
            InitMoteurs();
            JournalErreurs(3,F("1 - Calcul chemin Imposs - Arret"));
            return false; 
        } else {
            // ACTION : Rouler tout droit sur CapASuivre, jusqu'au prochain obstacle

            Terminal("$",F("Choix Traj A3"));

            
            TabActions[IActions].TypeAction = 1;  //pivoter sur Cap a Suivre
            TabActions[IActions].P1 = CapASuivre;
            TabActions[IActions].P2 = TabLissage[iDAngleDmax].Distance;
            TabActions[IActions].Debut = false;   
            TabActions[IActions].Fin = false;
            IActions++; NActions++;
            
            TabActions[IActions].TypeAction = 3;  // tout droit sur une distance X
            TabActions[IActions].P1 = TabLissage[iDAngleDmax].Distance * PointPivot;
            TabActions[IActions].Debut = false;   
            TabActions[IActions].Fin = false;
            IActions++; NActions++; 
            
            TabActions[IActions].TypeAction = 1;  //pivoter sur le Cap Initial
            TabActions[IActions].P1 = CapInit;
            TabActions[IActions].P2 = 0;
            TabActions[IActions].Debut = false;   
            TabActions[IActions].Fin = false;
            IActions++; NActions++;

            DecisionAPrendre = false;
            AfficheCap(CapRbot);
            AfficheActions(F("A3"));
            Terminal("i",String(CapInit));
            Terminal("a",String(CapASuivre));
            return true;
        }
  }

// Arrivé ici, on a pas trouvé de solution


    JournalErreurs(1,F("Pas de trajectoire trouvee"));
    DecisionAPrendre = false;
    Go = false;
    return false;
}

void Action () {   // Cette fonction lance les actions planifiées par KesKonFait

  
  if (DecisionAPrendre) {  return ; }     // Il y a une décision à prendre
  if (IActionEncours == NActions ) {
          DecisionAPrendre=true;
          return;
          }   // Toutes les actions sont terminées
  if (TabActions[IActionEncours].Debut  && !TabActions[IActionEncours].Fin) {  return ; }              // Il y a une action en cours


  JournalProgession(4,F("FaireActions "));

  //Type Action
  //1 --> pivoter P1 = CapASuivre
  //2 --> Tout droit jusqu'au prochain obstacle  , pas utilisé
  //3 --> Tout droit sur une distance  P1 = Distance
  
  
  switch (TabActions[IActionEncours].TypeAction) {
      case 1:
          JournalProgession(1,String(F("Action 1")) + String(CapActuel) + " " + String( CapASuivre));
          InitMoteurs();
          PositionServo(3,90);    // On met la tourelle à 90°
          pivoter(CapActuel, TabActions[IActionEncours].P1, TabActions[IActionEncours].P2);
          TabActions[IActionEncours].Fin = true;
          IActionEncours++;                  
          break;
      case 2:   
          AllerDevant();
          TabActions[IActionEncours].Debut = true;
          break;
      case 3:
          ClicHall=0;    // Raz Distance parcourue
          TabActions[IActionEncours].Debut = true;
          AllerDevant();
          break;

  }
  
    AfficheActions("");
    return;
}


void Surveiller () {

// Surveille la progression du robot. Clos les actions terminées

  if (DecisionAPrendre) { return ; }
  if (!FScanMarche)     { return ; }
  if (IActionEncours == NActions ) { // Plus d'actions à faire
          DecisionAPrendre=true;
          ScanAFaire=true;
          return;
  }  


   JournalProgession(4,F("Surveillance 1 ") );
   
   
  //******************  Surveillance de l'évitement   **************
 
  if (EviterEMaitenant && EvitPhase == 2) {   
      JournalProgession(4,F("Surveillance Evitement") );


      Terminal(";",String(DistParcourue - DistParcourueEvt));
      
      if (FSimuDist) {            // Simule le fait que l'on est atteint la distance à rouler après obstacle
          EvitPhase++;    // passe en phase 3
          FSimuDist = false ; 
          return; }    
          
          
      if ( (DistParcourue - DistParcourueEvt) < min(DistDvt, DistRoulEvt) ) {    // Minimum entre distance calculéee et distance mesurée 
          JournalProgession(4,String(F("Surveil. Evit. encore obstacle ")) + String(DistParcourue - DistParcourueEvt) + " " + String( DistRoulEvt));

          return;    // on a pas encore dépassé l'obstacle
      }
      else {
          JournalProgession(4,F("Surveil. Evit. obstacle depass") );
          
          EvitPhase++;  // passe en phase 3
          return;
      }
  }

  //******************  Surveillance obstacles   **************
     JournalProgession(4,F("Surveillance 2") );
  
  if ( ObstacleScanMarche() && !FObsATraiter ) {    // Surveillance obstacles
      InitMoteurs();
      FScanMarche = false; // on force a reprendre les mesures devant le robot
      FObsATraiter = true;  
      R=255;G=255;B=0;   // Leds Jaune
      JournalProgession(4,F("Dem Scan March. Complet ") );
      return;
  }
  
  KesKisPass = TraiteDonneesScanMarche();   // Analyse des données du scan Marche
  FObsATraiter = false;
  
  JournalProgession(4,String(F("Surveillance 3")) + String(KesKisPass));
       
  //******************  Surveillance des actions   **************
  
  switch (TabActions[IActionEncours].TypeAction) {
      case 3:
          if (DistParcourue >= TabActions[IActionEncours].P1  || FSimuDist ) {  // On s'arrete quand on atteint la distance parcourue ou on a simuler ce résultat
              JournalProgession(4,F("S K3 "));
              InitMoteurs();
              TabActions[IActionEncours].Fin = true;  // Clos l'action
              IActionEncours++;
              AfficheActions("");
              FSimuDist = false;
          }
          break;
      default:
          if (KesKisPass == 1) { // obstacle détecté à moins de DistArrUrg, mais sans actions identifiée
              JournalProgession(4,F("S DF"));
              InitMoteurs();
          }
          break;      
  }
  
  switch (KesKisPass) { 
        case 2000:     // Scan non terminé
            JournalProgession(3,String(F("KesKisPass 2000 ")) + String(KesKisPass));
            break;
        case 3000:     // Pas de trajecoire d'évitement
            JournalProgession(3,String(F("KesKisPass 3000")) + String(KesKisPass));
            AllerDevant();
            break;
        case 1000:     // Arret d'urgence
            InitMoteurs();
            JournalErreurs(1,F("ARRET Urgence"));
            break;    
        default:       // Evitement à faire
            JournalProgession(3,String(F("Prepa Evitement ")) + String(KesKisPass));
            EviterEMaitenant = true;
            EvitPhase = 1;
            AffObs();
            //Etape();
            break;             
            
  }

  return;
}


boolean ObstacleScanMarche () {
// vérifie si obstacle sur 3 points, si obstacles on forcera un scan plus précis

  if (!EviterEMaitenant) {
           if  ( ( TabRadar[IRadGauche].Distance < ProfVision &&  abs(TabRadar[IRadGauche].XMes) < LargCouloir )  ||  
                 ( TabRadar[IRadDroite].Distance < ProfVision &&  abs(TabRadar[IRadDroite].XMes) < LargCouloir )  || 
                   TabRadar[IRMedian].Distance   < DistArrUrg ) {
                  JournalProgession(1,String(F("Detect. obst. ")) + String(abs(TabRadar[IRadGauche].XMes)) + " " + String(abs(TabRadar[IRadDroite].XMes)) + " " + String(TabRadar[IRMedian].Distance)  );      
                  return true;
           }
           else {
                  return false;
           } 
  }
  else {   // Si on est en mode evitement , alors on a une vision moins profonde des obstacles
           if  ( ( TabRadar[IRadGauche].Distance < ProfVisionEvt &&  abs(TabRadar[IRadGauche].XMes) < LargCouloirEvt )  ||  
                 ( TabRadar[IRadDroite].Distance < ProfVisionEvt &&  abs(TabRadar[IRadDroite].XMes) < LargCouloirEvt )  || 
                   TabRadar[IRMedian].Distance   < DistArrUrg ) {
                  JournalProgession(1,String(F("Detect. obst. Evt")) + String(abs(TabRadar[IRadGauche].XMes)) + " " + String(abs(TabRadar[IRadDroite].XMes)) + " " + String(TabRadar[IRMedian].Distance)  );      
                  R=0;G=255;B=0;   // Leds rouges
                  return true;
           }
           else {
                  return false;
           } 
  }
  
}


int TraiteDonneesScanMarche () {

// Traite les données du scan Marche pour la detection d'obstacles
    
    boolean FObsGauche;
    int DistG, DistD;
    JournalProgession(4,String(F("Trait. Donnees Scan 0" ))  );
    if (FScanMarche) {    // Uniquement si au moins un scan marche complet a été fait
    
            JournalProgession(4,String(F("Trait. Donnees Scan 1 " ))  );
            if  (  TabRadar[IRMedian].Distance   < DistArrUrg ) {        
                  return 1000;     // Arrêt
            }
            
            if ( (TabRadar[IRadGauche].Distance < ProfVision && abs(TabRadar[IRadGauche].XMes) < LargCouloir ) ||   ( TabRadar[IRadDroite].Distance < ProfVision && abs(TabRadar[IRadDroite].XMes) ) ) 
            {
                              
                 
                    if ( TabRadar[IRadGauche].Distance < ProfVision && TabRadar[IRadDroite].Distance < ProfVision) {
                    // toutes les mesures sont inferieures à la profondeur de vision, du coup, on ne sait pas ou aller
                    // On étend le cadran 
                          DistG = MesureTourelle(TabRadar[IRadGauche + 1].Angle, true);  // Un cran de plus à gauche
                          MonDelay(200);
                          DistD = MesureTourelle(TabRadar[IRadDroite - 1].Angle, true);  // un cran de plus à droite
                          MonDelay(200);
                          
                          JournalProgession(1,String(F("Trait. Donnees Scan Fobs  " ))  + String(DistG)  + " " +  String(DistD) );
                          
                          if (DistG < DistD) { 
                                FObsGauche = true;    // Obstacle à gauche
                          }
                          else {
                                FObsGauche = false;   // Obstacle à droite
                          }
                          
                    }
                    else
                    {
                        if (TabRadar[IRadGauche].Distance < ProfVision) {     
                             FObsGauche = true;    // Obstacle à gauche     
                        }
                        else {
                             FObsGauche = false;   // Obstacle à droite
                        }
                    }
 
                     JournalProgession(1,String(F("Trait. Donnees Scan Fobs  " ))  + String(FObsGauche)  );

                    
                    if  (  FObsGauche ) {   
                                         
                    
                          JournalProgession(2,String(F("S G 1 ")) + String(TabRadar[IRadGauche].Distance) + "  " + String(abs(TabRadar[IRadGauche].XMes)) );
                          
                          // Calcul de l'angle de la droite entre point extreme et médian. Servira de trajectoire d'évitement
                          float AngleDroite = RadVersDeg(atan(1.0 * (TabRadar[IRadGauche].YMes - min(TabRadar[IRMedian].YMes,ProfVision)) / (TabRadar[IRadGauche].XMes - TabRadar[IRMedian].XMes)   )) ; 

                          JournalProgession(2,String(F("S G 2 " )) + String(AngleDroite)  );
                          
                          // Calcul distance à rouler : 2 fois (Arbitraire) la distance entre les points extremes et médian                    
                          DistRoulEvt =  2 * sqrt(  pow(TabRadar[IRadGauche].XMes,2) +  pow(min(TabRadar[IRMedian].YMes,ProfVision) - TabRadar[IRadGauche].YMes,2));
                          
                          Terminal(":",String(DistRoulEvt));
                          Terminal(";",String(0));
                                    
                          Terminal ("@","Angle Dist Evt " + String(AngleDroite) + " " + String(DistRoulEvt));
                          
                          // Si l'angle est trs faible (Mut plat devant), on force à 45°
                          if (abs(AngleDroite) < 10) {  AngleDroite = signe(AngleDroite) * 45;  }  // Angle tres perpendiculaire (Mur) , on force à 45°
                          
                          if (AngleDroite >= 0)  {return  90 - AngleDroite ;}
                          else { return  (90 - abs(AngleDroite )) * -1;}


                    }

                    
                    
                    if  ( !FObsGauche ) {   
                          JournalProgession(2,String(F("S D 1 ")) + String(TabRadar[IRadDroite].Distance) + "  " + String(abs(TabRadar[IRadDroite].XMes)) );
                          // Obstacle a moins de x mm a droite

                          // Calcul de l'angle de la droite entre point extreme et médian. Servira de trajectoire d'évitement
                          float AngleDroite = RadVersDeg(atan(1.0 * (TabRadar[IRadDroite].YMes - min(TabRadar[IRMedian].YMes,ProfVision)) / (TabRadar[IRadDroite].XMes - TabRadar[IRMedian].XMes)   )) ; 

                          // Calcul distance à rouler : 2 fois (Arbitraire) la distance entre les points extremes et médian
                          DistRoulEvt =  2 * sqrt(  pow(TabRadar[IRadDroite].XMes,2) +  pow(min(TabRadar[IRMedian].YMes,ProfVision) - TabRadar[IRadDroite].YMes,2));
                        
                          Terminal(":",String(DistRoulEvt));
                          Terminal(";",String(0));
                                    
                          JournalProgession(2,String(F("S D 2 " )) + String(AngleDroite) );             
                          Terminal ("@","Angle Evt dist" + String(AngleDroite) + " " + String(DistRoulEvt));
                          
                          if (abs(AngleDroite) < 10) {  AngleDroite = signe(AngleDroite) * 45;  }  // Angle tres perpendiculaire (Mur) , on force à 45°
                          
                          if (AngleDroite >= 0)  {return  90 - AngleDroite ; }
                          else {return  (90 - abs(AngleDroite )) * -1;}  
                         
                    }
            
            
            }
            return 3000;
    }
    return 2000;
}

void ScanMarche () {

  // scan les distances entre 2 bornes pendant que le robot roule, 1 scan par passage dans le loop

  boolean UnDelai = false;

  MlssMaintenant=millis();
  if((MlssMaintenant- MlssAvantScan) >  ClicChronoScanMarche) {  // Il faut au moins N millisecondes entre chaque mesure

        JournalProgession(4,F("ScanMarche 2") );
        if (IRadMarcheEncours > IRadGauche ) { 
            IRadMarcheEncours = IRadDroite;  
            FScanMarche = true;   // Un scan Marche a été fait au moins une fois
        }

        if (IRadMarcheEncours == IRadDroite ) {UnDelai = true;}  // delai supplémentaire pour que le servo est le temps de revenir à doite
        
        for (byte Nb = 0; Nb <= 3; Nb++) {
            TabRadar[IRadMarcheEncours].Distance = MesureTourelle(TabRadar[IRadMarcheEncours].Angle, UnDelai );
            if ( TabRadar[IRadMarcheEncours].Distance != -1 ) {break;}
        }
        
        if (TabRadar[IRadMarcheEncours].Distance == -1) {
            JournalErreurs(1,F("SCM Err prise de mesure"));
            TabRadar[IRadMarcheEncours].Distance = 500;    // Valeur par défaut
        }
        TabRadar[IRadMarcheEncours].XMes = TabRadar[IRadMarcheEncours].Distance * cos(DegVersRad(TabRadar[IRadMarcheEncours].Angle));
        TabRadar[IRadMarcheEncours].YMes = TabRadar[IRadMarcheEncours].Distance * sin(DegVersRad(TabRadar[IRadMarcheEncours].Angle));
        

        IRadMarcheEncours++;
        MlssAvantScan=millis();
        
  }

return;
}

boolean ScanAutour () {

  // Scan les distances sur 180°.   1 mesure par passage dans le loop

  boolean UnDelai = false;

  MlssMaintenant=millis();
  if((MlssMaintenant- MlssAvantScan) >  ClicChronoScan) {  // Il faut au moins N millisecondes entre chaque mesure

        if (IradScan == 0) { 
            DistMax = 0;
            if (FSimulation) {Simulation ();} else {InitTabRadar;}           
        }
    
        if (IradScan <= NbLectRadar ) { 
            if (!FSimulation) {
                if (TabRadar[IradScan].Angle == 0) {UnDelai=true;}  // delai supplémentaire pour que le servo ai le temps de revenir à doite
                TabRadar[IradScan].Distance = MesureTourelle(TabRadar[IradScan].Angle,UnDelai);
                
            }
            if (TabRadar[IradScan].Distance > DistMax) {DistMax = TabRadar[IradScan].Distance;}
            
            TabRadar[IradScan].XMes = TabRadar[IradScan].Distance * cos(DegVersRad(TabRadar[IradScan].Angle)) ;
            TabRadar[IradScan].YMes = TabRadar[IradScan].Distance * sin(DegVersRad(TabRadar[IradScan].Angle)) ;
            

            Coord = Coord + String(IradScan) + " " + String(TabRadar[IradScan].Distance) + " / ";
            if (icoord == 3) {
                 Terminal("#", Coord);
                  Coord = "";
                  icoord=0;
            }
            else {
              icoord++;
              };


            IradScan++;
            MlssAvantScan=millis();
            return false;
        }
        else { 
            IradScan = 0;  
            Terminal("#", Coord);
            return true;
        }

  }

}


byte Lissage(float Seuil) {   
    // Lisse les données scannées pour réduire le nombre de segments
    // Erreur si on obtient 15 ou + segments

    byte DebCalcReg;
    float DistADroite = 0;
    int NbLis = -1;
    float XLis;
    float YLis;


    memset(TabLissage, 0, sizeof(TabLissage));

    DebCalcReg = 0;
    ILis = 0;
    

    HC05.print("*HX" +  String(TabRadar[0].XMes  )+"Y"+String(TabRadar[0].YMes )+ ",X" + String(TabRadar[0].XMes)+"Y"+String(TabRadar[0].YMes));
    
    for (byte Irad = 1; Irad <= NbLectRadar; Irad++ ) {
        
        RegLineaire(DebCalcReg, Irad);
        
        if (Irad > 1 ) {   

            // Calcul de la distance à la droite de regression
            DistADroite = 1.0 * abs (  ( -1 * TabRadar[Irad - 1].a * TabRadar[Irad].XMes )     +  TabRadar[Irad].YMes   + (-1 *   TabRadar[Irad - 1].b ) ) / sqrt(  (TabRadar[Irad - 1].a * TabRadar[Irad - 1].a) + 1 );

            if (!isnan(DistADroite) && DistADroite >= Seuil ) {

              if (ILis == 0) {
                TabLissage[ILis].Irad = 0;
                TabLissage[ILis].Angle = 0;
                TabLissage[ILis].XDrl = TabRadar[0].XMes;
                TabLissage[ILis].YDrl = TabRadar[0].YMes;
                ILis++; 
              }
              
              if (ILis < 15 ) {
                TabLissage[ILis].Irad = Irad;
                TabLissage[ILis].Angle = TabRadar[Irad].Angle;
                TabLissage[ILis].Distance = TabRadar[Irad].Distance;
                TabLissage[ILis].XDrl = TabRadar[Irad ].XMes;
                TabLissage[ILis].YDrl = (TabRadar[Irad ].a * TabRadar[Irad ].XMes) + TabRadar[Irad ].b;               
                         
                NbLis++;

                ILis++;

                // On a detecté une rupture de direction sur le nuaghe de points
                DebCalcReg = Irad;
              }
              else
              {
                  JournalErreurs(3,F("Liss- Depassmt Nb Segments"));
                  return ILis;}              
            }
        }
    }
    // reccopie de la derniere mesure
    TabLissage[ILis].Irad = NbLectRadar;
    TabLissage[ILis].XDrl = TabRadar[NbLectRadar].XMes;
    TabLissage[ILis].YDrl = TabRadar[NbLectRadar].YMes;
    
    Ymax = 0;
    for (int iMax = 0; iMax <= ILis; iMax++) {
        // on conserve l'indice contenant le plus grand Y
        if (TabLissage[iMax].YDrl > Ymax ) { 
              Ymax = TabLissage[iMax].YDrl ; 
              IYmax = iMax; 
              IYmaxReel =  TabLissage[iMax].Irad;

        }
    
    }
    
    return ILis;
}


void RegLineaire(int Debut, int fin) {
    // Calcul de la droite de regression linéaire pour les points entre deux bornes du tableau
    int i; 
    double xsomme, ysomme, xysomme, xxsomme;
    double ai,bi;
    xsomme = 0.0; 
    ysomme = 0.0;
    xysomme = 0.0; 
    xxsomme = 0.0;

    for (byte PosTab=Debut;PosTab<=fin;PosTab++)
    { 
    xsomme = xsomme + TabRadar[PosTab].XMes; 
    ysomme = ysomme + TabRadar[PosTab].YMes;
    xysomme = xysomme + TabRadar[PosTab].XMes * TabRadar[PosTab].YMes;
    xxsomme = xxsomme + TabRadar[PosTab].XMes * TabRadar[PosTab].XMes;ai = (((fin-Debut+1)*xysomme) - (xsomme*ysomme))/(((fin-Debut+1) * xxsomme) - (xsomme*xsomme));
    TabRadar[fin].a = (((fin-Debut+1)*xysomme) - (xsomme*ysomme))/(((fin-Debut+1) * xxsomme) - (xsomme*xsomme));
    TabRadar[fin].b = TabRadar[fin].YMes - (ai * TabRadar[fin].XMes);
    }
    return;
}



boolean TraiteDonneesScan() {
// Traite les donnees du scan sur 180°

  byte IRadMax = 99;
  int precision = 200;   //en mm , on pixelise la mesure par tranche de 200 mm
  float Seuil = SeuilLissage;     // Distance a la droite mini pour être incorporer à la DRL
  byte  ILis;
  byte  Nbtentatives = 0;
  boolean CycleOK = true;


    //*********************************************************************
    // Calculs des segments de lissages par itération (Moins de N segments et plus de 4 )
    // on fait au max 10 tentatives en baissant le seuil de lissage à chaque fois
    
    while ( true ) {
        Nbtentatives++;
        if (Nbtentatives > 10) {
          CycleOK = false;
          break;
        }
        ILis = Lissage(Seuil);
        if (ILis > 14 || ILis < 4 ) {
              Seuil = Seuil - 0.1;
        }
        else
        {
          break;
        }
    }
    if (!CycleOK) {return false;}

 
    // ************  calcul des zones d'ombres  ****
  
    
    for (byte Irad = 1; Irad <= ILis;Irad++) {

        // Calcul de la pente du segment de droite et conversion en un angle en °
        float PenteSD = 0;
        float AngleSD = 0;
        
        if (TabLissage[Irad].XDrl - TabLissage[Irad - 1].XDrl == 0)  {
            AngleSD = 90;
        }
        else if (TabLissage[Irad].YDrl - TabLissage[Irad - 1].YDrl <= 3) { // On va éliminer les "couloirs de 3 x pixel"
            AngleSD = 0;
        }
        
        else {
            PenteSD = 1.0 * (TabLissage[Irad].YDrl - TabLissage[Irad - 1].YDrl) / (TabLissage[Irad].XDrl - TabLissage[Irad - 1].XDrl);
            AngleSD = RadVersDeg(atan( PenteSD));
            if (AngleSD <  0) {AngleSD = AngleSD + 180;}
              
        }

        // Calcule de la pente entre le robot et la moitié du segment de droite
        
        float PenteDSD = 0;
        float AngleDSD = 0;
        
        if (   ((TabLissage[Irad].XDrl + TabLissage[Irad - 1].XDrl) / 2 ) == 0    )  {
            AngleDSD = 90;
        }
        else if ((    1.0 *  TabLissage[Irad].YDrl + TabLissage[Irad - 1].YDrl     )/ 2 == 0) {

             AngleDSD = 0;
        }
        
        else {
            PenteDSD = 1.0 * (( (  1.0 *  TabLissage[Irad].YDrl + TabLissage[Irad - 1].YDrl )/ 2 ) / (1.0 * (TabLissage[Irad].XDrl + TabLissage[Irad - 1].XDrl )/ 2 ));
            AngleDSD = RadVersDeg(atan( PenteDSD));
            if (AngleDSD <  0) {AngleDSD = AngleDSD + 180;}
        }
 
        if (AngleSD < 0) {AngleSD =  AngleSD +180;}
        if (AngleDSD < 0) {AngleDSD =  AngleDSD +180;}
        
        if (abs(AngleSD - AngleDSD ) <= 30 && AngleSD != 0 && AngleDSD != 0 ) {   // On élimine les horizontales
                  //Serial.println("Detection ombre");
                  TabLissage[Irad].DZO = true;
                  TabLissage[Irad].YZO =  abs(TabLissage[Irad].YDrl - TabLissage[Irad - 1].YDrl) / 2;

        }       
    }
 

return true;
}  // fin traiteDonneesScan


int Trajectoire (int ICouloir)  {

    // on fait le calcul de trajectoire avec les données non lissées
       
    // calcule de la pente de la droite pointant vers le point souhaité
    int CapScan;
    int AnglePoint;
    float Pente = 1.0 * TabRadar[ ICouloir].YMes / TabRadar[ ICouloir].XMes;
    
    AnglePoint = RadVersDeg(atan(Pente));
        
    if ( AnglePoint < 0) {
          CapScan = (90 - abs(AnglePoint)) * -1;
    }
    else {
         CapScan = 90 - AnglePoint;  // exprimé par rapport à la verticale (le 90] LIDAR)
    }

    CapASuivre = AngleAjout( CapActuel, CapScan);

    Terminal("@","Irad reel du Ymax " + String(ICouloir) );
    Terminal("@","Cap Scan " + String(CapScan));
    Terminal("@","Cap A Suivre " + String(CapASuivre));
    //Serial.println(F("*******    Fin Calcul trajectoire ***** "));
    return CapScan;  
}



int AngleAjout (int Initial, int Ajout) {  // Additionne deux angles

  // Ajout : 0° face au robot, 90° A droite, -90° a gauche
  int NouvelAngle = 0;
  NouvelAngle = (Initial + Ajout) % 360;
  return NouvelAngle;
}

int AngleSens (int AngleDepart, int AngleArrivee) {  // Compare deux angles et donne le sens pour aller de l'un à l'autre

      // Retourne -1 si sens horaire (vers la droite) , 1 si sens anti Horaire (vers la gauche)

      if ( (AngleArrivee - AngleDepart) > 180 ) {
          return   (AngleArrivee - AngleDepart - 360 ) / abs ((AngleArrivee - AngleDepart - 360 ));
      }
      else if ( (AngleArrivee - AngleDepart) < -180 ) {
          return   (AngleArrivee - AngleDepart + 360 ) / abs ((AngleArrivee - AngleDepart + 360 ));
      }
      else {
          return (AngleArrivee - AngleDepart) / abs (AngleArrivee - AngleDepart);
      }
}

void pivoter (int AngleDepart, int AngleArrivee, int DistAttendue) {

    JournalProgession(1,String(F("Pivoter 0 ")) + String(AngleDepart)  + " " + String(AngleArrivee) + " " + String(DistAttendue));
   
  //  parametrage vitesse roue fonction sens du pivot
  int SensPivot = AngleSens(AngleDepart,AngleArrivee);

  if ( SensPivot  == -1 ) {
        PivoterGauche();   
  }
  else {
        PivoterDroite();       
  }

  MonDelay(10);  // Doit être suffisant pour que arrivé dans la bouche l'azimuth ne soit plus égal à l'angle de départ


  while (  AngleSens(AngleArrivee,LectAzimuth()) != SensPivot ) {
      // On boucle tant que l'on a pas atteint l'angle d'arrivée

    Distance = Mesure();

     if  (  DistAttendue != 0 && (Distance  > DistAttendue * 0.98 &&   Distance < DistAttendue * 1.03 )) {
        // On test le cap mais aussi la distance 
        JournalProgession(1,String(F("Pivoter fin D ")) + String(LectAzimuth()) + " " + String(Distance ) );
        InitMoteurs();
        break;
     }
      MonDelay(20);
  }
  InitMoteurs();
  Terminal("z",String(LectAzimuth()));

}

void Simulation() {

//  Remplir le tableau manuellement
TabRadar[10].Distance=1042;

}

void InitMoteurs () {
            // Moteurs à l'arrêt
            ServeurServo.writeMicroseconds(2,PtMortServoRoue);  // Gauche
            ServeurServo.writeMicroseconds(1,PtMortServoRoue ); // Droite
            RobotRoule = false;
}
void AllerDevant () {
            if (FAllerDevant) {  // mode tests les moteurs ne démarre pas ensemble
                ServeurServo.writeMicroseconds(2,VitssGaucheCons);  // Gauche
                ServeurServo.writeMicroseconds(1,VitssDroitCons ); // Droite
            }
            RobotRoule = true;           
}

void PivoterDroite () {
            ServeurServo.writeMicroseconds(2,PtMortServoRoue * 1.08);  // Gauche, doucement
            ServeurServo.writeMicroseconds(1,PtMortServoRoue * 1.08); // Droite à l'envers
            RobotRoule = true;
}
void PivoterGauche () {
            ServeurServo.writeMicroseconds(2,PtMortServoRoue * 0.98);  // Gauche, à l'envers
            ServeurServo.writeMicroseconds(1,PtMortServoRoue * 0.98); // Droite, doucement
            RobotRoule = true;
}


void LectureHall() {

  MlssMaintenant=millis();

  if((MlssMaintenant- MlssAvantHall) >  ClicChronoHall) {  // on ne fait une mesure qu'au moins tous les "ClicChronoHall" millisecondes

            ValHall = analogRead(Pin_Hall);   // Lecture valeur analogique sur la Pin A1 - Attention très dépendant de la charge des batteries !!!!!

            if (ValHall > SeuilHall && ValHallAvant <= SeuilHall) { // On ne fait +1 que si la value lue et celle d'avant sont de part et d'autre de "Seuil"
                ClicHall++;
                DistParcourue = ((PI * 30) / 4) * ClicHall *  CorrHall;  // Distance en mm, la roue a un diametre de 30 mm  
                Terminal(">",String(ClicHall)+"/"+String(DistParcourue));
                JournalProgession(4,String(F("hall ")) + " " +String(ClicHall));
            }
            ValHallAvant = ValHall;
            MlssAvantHall=millis();
  }
}


int MesurePoint() {   //prend la mesure de la distance

  int distance;
  byte MesStatus;
  boolean Erreur = false;

  // Tf-Luna
  
  if (MesureDistLuna(TfLuna) == true)
  {
      DistCm = DistCm * 10; // Tf luna donne une messure en cm, conversion en mm
  }
  else {

    return -1;
  }

  return DistCm;
}

int Mesure() {

  byte NbOk = 0;
  unsigned int  Som = 0;
  int  DistMesur = 0;
  for (byte Nb = 1; Nb <= 4; Nb++) {    // 4 mesures consecutives
      Distance = MesurePoint();
      if (Distance > 0 ) {
        NbOk++;
        Som=Som+Distance;
      }
      MonDelay(20);
  }
    
  if (NbOk == 0 ) {    // Pas de mesure Correcte
      return -1;
  }
  else 
  { 
      return (Som / NbOk);  // Retourne la moyenne des mesures
  }

}


int MesureTourelle(byte Angle, boolean UnDelai) {

    int Distance;

    PositionServo(3,Angle); 
    if ( UnDelai ) { MonDelay(200);}     // Laisse plus de temps au servo pour faire un demi tour
    MonDelay(20);
        
    Distance = Mesure();
        
    if (Distance == -1 ) {   // SI mesure erronee on en prend une autre
            Distance = Mesure();
    }
    return (Distance);
}


void PositionServo(byte Nservo, byte AngleServo) {       // Met le servo sur un angle donné
    ServeurServo.setPWM(Nservo, 0, map(AngleServo, 0, 180, 158, 450));    // A calibrer selon servo
}


void InitTabRadar () {    // Initialise les angles à scanern prérempli les angles

  AngleMesure = 180.0 / NbLectRadar;

  for (byte i=0; i<= NbLectRadar ; i++) {
       TabRadar[i].Angle = i * AngleMesure;
       TabRadar[i].Distance = 0;
       TabRadar[i].XMes = 0;
       TabRadar[i].YMes = 0;
       TabRadar[i].b = 0.0;
       TabRadar[i].a - 0.0;
  }
  
}

boolean MesureDistLuna(uint8_t TfLunaAddress)   // LECTURE DU tf luna
{ 
  // Lire les 5 premieères informations (10 octets) sur le capteur Tf-Luna
  
  Wire.beginTransmission(TfLunaAddress);
  Wire.write(0x00); // Adresse début lecture

  if (Wire.endTransmission(false) != 0) {
    return (false); //Sensor did not ACK
  }
  Wire.requestFrom(TfLunaAddress, (uint8_t) 4 ); // lire 10 octets 

  if (Wire.available()) {
        uint8_t Buffer = Wire.read();
        DistCm = Buffer; //octet faible Distance
        Buffer = Wire.read();
        DistCm |= Buffer << 8; //octet fort distance
        Buffer = Wire.read();
        Erreur = Buffer; // Erreur
        Buffer = Wire.read();
        Erreur  |= Buffer << 8; // Erreur
  }
  else
  {
    JournalErreurs(1,F("Erreur Lecture Tf-Luna"));
    return (false);
  }
  return (true);
}


boolean VerifBT () {  // Vérifie si présence message BlueTooth
    while(HC05.available()>0)
    {
      MonDelay(3);
      c = HC05.read();
      messageRecu += c;
    }
    if (messageRecu.length() >0)
      {return true;}
    else
      {return false;}
}

void DecodeBT () {   // Decodage du message BlueTooth
    if (messageRecu.length() >0)
    {
      switch (messageRecu.charAt(0)) {
      case 'k': 
          if (Go) {      // Arrête le robot
            Go = false;
            InitMoteurs(); 
            Terminal("@",F("Arret pris en compte"));
            messageRecu = "";
            }
          else
          {  // Démarre le robot
            Go = true;
            Terminal("@",F("Demarage pris en compte"));
            messageRecu = "";
          }
          break;
      case 's': // Stockage du profil de la boussole
          StockeProfil();
          Terminal("@","Profil stocké");
          messageRecu = "";
          break;
      case 'e': 
          Terminal("@",F("Suite"));     // Fait une pause
          messageRecu = "";
          break;
      case 'r': 
          Terminal("@",F("Reset"));     // Fait un reset
          PushReset();
          messageRecu = "";
          break;
      case 'd': 
          Terminal("@",F("Simu Distance"));     // Simule distance parcourue atteinte
          FSimuDist = true;
          messageRecu = "";
          break;
      default:
          Serial.println("Pas de trt du mess bluetooth");
          break;
      }
    messageRecu = "";
    }
}

void Etape() {   // Met le robot en stand by (boucle)
       //InitMoteurs();
       Terminal("@",F("Validez etape "));
       while (!VerifBT()) {};
       //AllerDevant();
}

void Terminal (String NoTerm, String Mess) {    // Affiche un message dans un champ parametré de blueTooth electronics
     HC05.flush();
     HC05.print("*" + NoTerm + Mess + "\r\n*");
}

void MonDelay(int Delai) {
    MlssAv=millis();
    MlssTop=MlssAv;
    while (  (MlssTop - MlssAv) < Delai ) {
         MlssTop=millis();
  }
}


int LectAzimuth () {      // Lit la valeur du cap boussole

  unsigned char high_byte, low_byte;
  unsigned int angle16;   // CMPS12 angle sur 2 octets, a diviser par 10 (1 decimale)
  
  Wire.beginTransmission(CMPS12_ADDRESS);  //Début communication avec CMPS12
  Wire.write(ANGLE);                     // Envoi adresse de début
  Wire.endTransmission();
 
  // demande 2 octets
  Wire.requestFrom(CMPS12_ADDRESS, 2);    // requete des données
  
  while(Wire.available() < 2);        //  Attente réponse
  
  high_byte = Wire.read();
  low_byte = Wire.read();
  
  angle16 = high_byte;                 // Lecture angle sur 16 bits
  angle16 <<= 8;
  angle16 += low_byte;
  return angle16 / 10;

}



double DegVersRad (float deg) {     // Convertion angle de Degrés vers Radians
  double result;
  result = PI * deg / 180;
  return result;
}

double RadVersDeg (float rad) {      // Convertion angle de  Radians vers Degrés 
  double result;
  result = rad * 180 / PI;
  return result;
}

void JournalProgession (byte Niveau, String Message) {   // filtre et affiche les éléments de debug
// Niveau    1- Indispensable , 2 - Moyen, 3 - Detail
      
      if (Niveau <= NiveauProg) { 
      
          String Complement; 
          Complement.concat(DecisionAPrendre);
          Complement.concat(ScanAFaire);
          //Complement.concat(ScanMarche);
          Complement.concat(" ");
          Complement.concat(IActionEncours);     
          Complement.concat("/");
          Complement.concat(NActions -1); 
          Complement.concat(" ");           
          Complement.concat(Message); 

          Terminal ("-",Complement); 
          //MonDelay(500); //Pour ralentir le traitement 
      } ;
}

// retourne le signe d'un nombre
int signe (float Valeur) {
  if (Valeur  >=0 ) {
    return 1;
  }
  else
  {
    return -1;
  }
}

void JournalErreurs (byte Niveau, String Message) {   // Filtre et affiche les erreurs
// Niveau    1- Haut , 2 - Moyen, 3 - Bas
      if (Niveau <= NiveauErr) { Terminal ("@",Message); } ;
      ScenarioLed = 4;
      R = 0; G = 255 ; B = 0;  //Rouge
}

void StockeProfil()
{

  //Inspiré par :  https://roboticboat.uk/Arduino/Due/ArduinoDueCompass12.html
  // Après calibrage de la boussole, permet de strocker le profil

  // Cf DataSheet : To store a profile write the following to the command register 0xF0, 0xF5, 0xF6 with a 20ms MonDelay after each of the three bytes


  
  Wire.beginTransmission(CMPS12_ADDRESS);
  Wire.write(CONTROL_Register);
  Wire.write(byte(0xF0));
  
  // End the transmission
  int nackCatcher = Wire.endTransmission();
  String Retour = String(nackCatcher);
  MonDelay(50);

  // Begin communication
  Wire.beginTransmission(CMPS12_ADDRESS);
  Wire.write(CONTROL_Register);
  Wire.write(byte(0xF5));

  // End the transmission
  nackCatcher = Wire.endTransmission();
  Retour += String(nackCatcher);
  // Return if we have a connection problem 
  if(nackCatcher != 0){Terminal("@","Stockage Profil " + Retour); return;}

  MonDelay(50);

  // Begin communication
  Wire.beginTransmission(CMPS12_ADDRESS);
  Wire.write(CONTROL_Register);
  Wire.write(byte(0xF6));

  // End the transmission
  nackCatcher = Wire.endTransmission();
  Retour += String(nackCatcher);
  // Return if we have a connection problem 
  if(nackCatcher != 0){ Terminal("@","Stockage Profil " + Retour); return;}

  MonDelay(50);

  Terminal("@","Stockage Profil " + Retour);
  
  
}


void GestionLeds(byte Scenario, byte R, byte G, byte B ) {    // Animation barre Leds
  if (!Fleds) { return ; }
      switch(Scenario) 
      {
      case 1:    // remplissage par un coté
              MlssMaintenant=millis();
              if((MlssMaintenant- MlssAvantLed) >  ClicChronoLed) {
                if (IledEncours > 9 ) {
                    EffaceLeds();
                    IledEncours = 0;
                }
                else { IledEncours++;}
                
                BarreLeds[IledEncours].setRGB(R, G, B);
                FastLED.show();
                MlssAvantLed=millis();
              } 
      break;
      case 2:    // Chenillard 1 led
              MlssMaintenant=millis();
              if((MlssMaintenant- MlssAvantLed) >  ClicChronoLed) {
              
                  BarreLeds[IledEncours]= CRGB::Black;
                  
                  if (IledEncours > 9) { IledEncours = 0;} else {IledEncours++;}
                  
                  BarreLeds[IledEncours].setRGB(R, G, B);

                  FastLED.show();                    

              MlssAvantLed=millis();
              } 
      break;
      case 3:    // tout clignote
              MlssMaintenant=millis();
              if((MlssMaintenant- MlssAvantLed) >  ClicChronoLed) {
              
                  if (Clignote) {

                      for (byte ld = 0; ld < 10;ld++)  {
                          BarreLeds[ld].setRGB(R, G, B);
                      }
                      FastLED.show(); 

                  }  else
                  {
                      EffaceLeds();
                  }
                  Clignote = !Clignote;
              MlssAvantLed=millis();
              } 
              break;        
      case 4:    // tout fixe
                  EffaceLeds();
                  for (byte ld = 0; ld < 10;ld++)  {
                      BarreLeds[ld].setRGB(R, G, B);
                  }
                  FastLED.show(); 
      break;  
      }  
     
}

void EffaceLeds () {
      for (int i = 0; i < 10 ; i++) {
          BarreLeds[i] = CRGB::Black;
          FastLED.show();
      }
}

void PushReset()     // https://arduino103.blogspot.com/2013/06/comment-un-reset-darduino-par-logiciel.html
{     // Fait un reset du microcontroleur (voir schéma de cablage)
  int pin=52; 
  // active la broche en sortie (OUTPUT)  
  pinMode(pin, OUTPUT); 
  // Déactive le reset forçant la sortie au niveau bas 
  digitalWrite(pin, LOW);
}


void AfficheActions(String Action) {      // Affiche les actions planifiées
    Terminal("+", "------------------");
    for (byte i = 0; i<5 ; i++) {
        Terminal("+", Action + " " + String(TabActions[i].TypeAction) + " " +  String(TabActions[i].P1) + " " +  String(TabActions[i].P2) + " " +  String(TabActions[i].Debut) + " " +  String(TabActions[i].Fin));
    }

}

void AfficheLissage(byte ILIS) {      // Affiche le graphique des segments lissés
  
    String MessageHC05;

     MessageHC05 = "*LX0Y0,X0,Y0";
     HC05.print(MessageHC05);
       
    for (byte i=0; i <= NbLectRadar ; i++) {
         
         if  (i <= ILIS) {
         HC05.print("*LX" +  String(TabRadar[i].XMes  )+"Y"+String(TabRadar[i].YMes )  + ",X" +   String(TabLissage[i].XDrl  )+"Y"+String(TabLissage[i].YDrl )     );
         Terminal("$", String(i) + " " + String(TabLissage[i].Irad) + " " + String(TabLissage[i].Angle) + " " + String(TabLissage[i].Distance) + " " + String(TabLissage[i].XDrl)+ " " + String(TabLissage[i].YDrl) + " " + String(TabLissage[i].DZO)  + " " + String(IYmax) +  " " + String(IYmaxReel));
         }
         else {
         HC05.print("*LX" +  String(TabRadar[i].XMes  )+"Y"+String(TabRadar[i].YMes )  + ",X0Y0"     );         
         }
         
     }  
}

void AfficheScanMarche () {     // Afiiche les données du scan marche
  
    String MessageHC05;
    
    for (byte i=IRadDroite; i <= IRadGauche ; i++) {
          Terminal("$", String(i) + " " + String(TabRadar[i].Angle ) + " " + String(TabRadar[i].Distance) + " " + String(TabRadar[i].XMes)+ " " + String(TabRadar[i].YMes) );
   }
}

void AfficheCap ( int Cap) {    // Affiche sur le graphique la trajectoire calculée

    String MessageHC05;
    int Y = DistMax * sin(DegVersRad(90 - Cap));
    int X = DistMax * cos(DegVersRad(90 - Cap));  

     MessageHC05 = "*LX0Y0,X0Y0,X0Y0";
     HC05.print(MessageHC05);  
     MessageHC05 = "*LX0Y0,X0Y0,X" + String(X) + "Y" + String(Y);
     HC05.print(MessageHC05);    
    
}


boolean AffObs () {   // affiche sur le deuxième graphqique la courbe des obstacles

  // Scan les distances sur 180°.   1 mesure par passage dans le loop
  
     String MessageHC05;
     MessageHC05 = "*ZX0Y0,X0Y0";
     HC05.print(MessageHC05);
     Coord = "";
     
  Terminal("#", "--------------------");
  for (int IradScan = IRadDroite -1; IradScan <= IRadGauche +1 ; IradScan++ ) {
  
  
         MessageHC05 = "*ZX" +  String(TabRadar[IradScan].XMes  )+"Y"+String(TabRadar[IradScan].YMes ) ;
         if (IradScan == IRadGauche +1)  {MessageHC05 =MessageHC05 + ",X" + String(TabRadar[IradScan].Distance * cos(DegVersRad(TabRadar[IradScan].Angle))  )   + ",Y" + String(TabRadar[IradScan].Distance * sin(DegVersRad(TabRadar[IradScan].Angle))  ) ;  }
         HC05.print(MessageHC05); 

            Coord = Coord + String(IradScan) + " " + String(TabRadar[IradScan].Angle) + " " + String(TabRadar[IradScan].Distance) + " / ";
            if (icoord == 3) {
                 Terminal("#", Coord);
                  Coord = "";
                  icoord=0;
            }
            else {
                  icoord++;
            };
  }
  Terminal("#", Coord);
  Terminal("#", "--------------------");

}
