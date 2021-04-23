/***************************************************************
**  R2Ddemi - Robot Mobile Arduino autonome                   **
**  Auteur : B. Gillet                                        **
**  Version 8    - Avril 2021                                 **
**  Consulter : https://arduino-self.over-blog.com/           **
****************************************************************/


#define abs(x) ((x)>0?(x):-(x))

#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver ServeurServo = Adafruit_PWMServoDriver();  //Adresse par défaut 0x40

#include <math.h>
#include <Wire.h>
#include <SoftwareSerial.h>
SoftwareSerial HC05(8, 9); // Rx, Tx  --> Module BlueTooth

// ******* Capteur effet hall    ******/
#define Pin_Hall 1        // Broche analogique pour le signal capteur Hall
#define ClicChronoHall 20
#define SeuilHall 580     // Valeur à positionner en fonction du capteur utilisé
int ValHall = 0; 
int ValHallAvant = 0;
unsigned int ClicHall = 0;   
unsigned int DistParcourue = 0;
unsigned long MlssAvantHall = 0;
unsigned long MlssMaintenant = 0;

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
int DistRoulage;

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
#define IRadGauche 22     // Borne inf pour ScanMarche
#define IRadDroite 14     // Borne Sup pour ScanMarche
#define IRMedian   18     // I equiv. 90°
byte IRadMarcheEncours = IRadDroite;
float   AngleMesure;    // Pas d'incrementation en ° de TABRADAR'

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
byte IYmax = 0;
byte Ymax = 0;
byte IradScan = 0;


//******* Variable de Chrono ********
#define ClicChronoScan 40  
#define ClicChronoScanMarche 200  
#define ClicChronoSurv 500  // Valeur timer de surveillance de la route

unsigned long MlssAvantScan = 0;
unsigned long MlssAvantSurv = 0;
unsigned long MlssTop = 0;
unsigned long MlssAv = 0;


// ********* Variables d'actions
boolean ScanEntier = false;     // Au moins un scan entier a été fait
boolean ScanAFaire = true;      // Un scan est à faire
boolean ScanATraiter = true;    // Les données scan sont à traiter
boolean FObsATraiter = false;   
boolean DecisionAPrendre;       // Une décision est à prendre dans KessKonFait
boolean RobotRoule = false;  // True si le robot est en train de rouler
boolean FScanMarche = false; // True = Au moins un scan complet Marche a été fait
boolean Go = false;          //  True pour que le robot fonctionne

// **** Affichage
String Coord;
byte icoord = 0;

// ******* A trier
int CapYmax;
boolean MemCap;
int DistMax = 0;

// *** Mode simulation *****
const boolean FSimulation = false;     // true = simulation, les mesures sont chargées par la fonction Simultaion
const boolean FAllerDevant = true;   // Mode test, si false le robot tourne mais n'avance pas


//*** Obstacles en marche
#define ProfVision      300 // Limite en deça de laquelle on fait un evitement d'obstacle
#define ProfVisionObs   400 // Limite minimale pour trouver une voie libre pour un évitement
#define DistArrUrg  200  // Distance Arrêt Urgence

// Journal des erreurs et de la progression
#define NiveauProg  2
#define NiveauErr   3

void setup() {


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
  Terminal("<",String(0));
  Terminal("@",F("OK pour demarrage"));


  // Ecoute Bluetooth pour Démarrage
  while (!Go) {
   while (! VerifBT() ){ };
    DecodeBT ();
  }

  CapActuel = LectAzimuth();  
  CapInit =  CapActuel;
  Terminal("@",String(F("CapActuel ")) + String(CapActuel));
    
}

void loop() {


  if (VerifBT()) {DecodeBT();}   // Ecoute du terminal bluetooth

  CapActuel = LectAzimuth();  // Actualisation du Cap
  LectureHall();    // Met à jour les distances parcourues
  
  if (Go) {    

      JeZieute();       // Scan l'environnement avec le Tf-luna
      
      Keskjefait();      // Décide et planifie des actions à faire
      
      JeBosse();        // Lance les actions de déplacements

      JeTaiAloeil() ;    // Vérifie les données en cours de déplacement du robot toutes les M millisecondes

  }
 
}

/********************************************************************************
 *    Fonctions
********************************************************************************/

void JeZieute () {

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
        // Scan LIDAR sur +/- 30° si robot en mouvement pour controle trajectoire
        if (RobotRoule || !FScanMarche) { ScanMarche(); }  
  }
  
}


boolean Keskjefait () {
  int CapRbot;
  

  if (!DecisionAPrendre ) {return false;}
  if (ScanAFaire) {return false;}

  JournalProgession(3,String(F("Kskonfait")) + String(DecisionAPrendre));
  
  memset(TabActions, 0, sizeof(TabActions));
  IActions = 0; NActions = 0;IActionEncours=0;

    
  CapRbot = Trajectoire(IYmax);   /
  if (CapRbot == 9999) {
      InitMoteurs();
      JournalErreurs(3,F("1 - Calcul chemin Imposs - Arret"));
      return false;
  }
// Si la distance mesurée pour le YMax > MaxDistance, on y va sans se poser d'autres questions
  if (TabRadar[TabLissage[IYmax].Irad].Distance > MaxDistance) {
 
      // ACTION : Rouler tout droit sur CapASuivre, jusqu'au prochain obstacle
      JournalProgession(3,F("Kskonfait A1"));
      
      
      TabActions[IActions].TypeAction = 1;  //pivoter sur Cap a Suivre
      TabActions[IActions].P1 = CapASuivre;
      TabActions[IActions].P2 = TabRadar[TabLissage[IYmax].Irad].Distance;
      TabActions[IActions].Debut = false;    
      TabActions[IActions].Fin = false;
      IActions++; NActions++;
      
      TabActions[IActions].TypeAction = 3;  // tout droit sur une certaine distance
      TabActions[IActions].P1 = TabRadar[TabLissage[IYmax].Irad].Distance * 0.75 ;
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
      return true;
       
  }
  
// On regarde si des zones d'ombres sont définies de chaque coté du Ymax, si non pas la peine de creuser de ce coté là, c'est bouché'

   byte ILisAv = IYmax + 1;
   
     
  if (( TabLissage[IYmax].DZO ) || ( ILisAv <= 30 && TabLissage[ILisAv].DZO )) {
     
        // On detecte au moins une zone d'ombre, on peut explorer'  
        // On va chercher la distance de l'obstacle sur la trajectoire du YMax '

 
        DistRoulage = TabRadar[   TabLissage[IYmax].Irad     ].Distance / 3;     // On fixe 1/3 du chemin à parcourir
        
        // ACTION : Rouler tout droit sur une distance de "DistRoulage" et sur un cap de CapAsuivre et refaire un scan 
        JournalProgession(3,F("Kskonfait A2"));

        
        TabActions[IActions].TypeAction = 1;  //pivoter sur Cap a Suivre
        TabActions[IActions].P1 = CapASuivre;
        TabActions[IActions].P2 = TabRadar[TabLissage[IYmax].Irad].Distance;
        TabActions[IActions].Debut = false;   
        TabActions[IActions].Fin = false;
        IActions++; NActions++;        

        
        TabActions[IActions].TypeAction = 3;  // tout droit sur une distance X
        TabActions[IActions].P1 = DistRoulage;
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
            JournalProgession(3,F("Kskonfait A3"));

            
            TabActions[IActions].TypeAction = 1;  //pivoter sur Cap a Suivre
            TabActions[IActions].P1 = CapASuivre;
            TabActions[IActions].P2 = TabLissage[iDAngleDmax].Distance;
            TabActions[IActions].Debut = false;   
            TabActions[IActions].Fin = false;
            IActions++; NActions++;
            
            TabActions[IActions].TypeAction = 3;  // tout droit sur une distance X
            TabActions[IActions].P1 = TabLissage[iDAngleDmax].Distance * 0.75;
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
            return true;
        }
  }

// Arrivé ici, on a pas trouvé de solution


    JournalErreurs(1,F("Pas de trajectoire trouvee"));
    DecisionAPrendre = false;
    Go = false;
    return false;
}

void JeBosse () {   // Cette fonction lance les actions planifiées par KesKonFait

  
  if (DecisionAPrendre) {  return ; }                                 // Il y a une décision à prendre
  if (IActionEncours == NActions ) {
          DecisionAPrendre=true;
          return;
          }   // Toutes les actions sont terminées
  if (TabActions[IActionEncours].Debut  && !TabActions[IActionEncours].Fin) {  return ; }              // Il y a une action en cours


  JournalProgession(3,F("FaireActions "));

  //1 --> pivoter P1 = CapASuivre
  //2 --> Tout droit jusqu'au prochain obstacle
  //3 --> Tout droit sur une distance  P1 = Distance
  
  
  switch (TabActions[IActionEncours].TypeAction) {
      case 1:
          InitMoteurs();
          PositionServo(3,90);    // On met la tourelle à 90°
          pivoter(CapActuel, CapASuivre, TabActions[IActionEncours].P2);
          TabActions[IActionEncours].Fin = true;
          IActionEncours++;
          break;
      case 2:   
          AllerDevant();
          TabActions[IActionEncours].Debut = true;
          break;
      case 3:
          ClicHall=0;    // Raz Distance parcourue
          AllerDevant();
          TabActions[IActionEncours].Debut = true;
          break;

  }
  
    AfficheActions("");
    return;
}


void JeTaiAloeil () {

// Surveille la progression du robot. Clos les actions terminées

  if (DecisionAPrendre) {  return ; }
  if (!FScanMarche) { return ; }
  if (IActionEncours == NActions ) { // Plus d'actions à faire
          DecisionAPrendre=true;
          ScanAFaire=true;
          return;
  }  

  JournalProgession(3,F("Surveillance ") );

  if ( ObstacleScanMarche() && !FObsATraiter ) { 
      InitMoteurs();
      FScanMarche = false; // on force a reprendre les mesures devant le robot
      FObsATraiter = true;  
      JournalProgession(2,F("Dem Scan Marc. Complet ") );
      return;
  }

  int KesKisPass = TraiteDonneesScanMarche();
  FObsATraiter = false;

  switch (TabActions[IActionEncours].TypeAction) {
      case 2:   
          if (KesKisPass == 1000) { 
              JournalProgession(3,F("S K2 "));
              InitMoteurs();
              TabActions[IActionEncours].Fin = true;
              IActionEncours++;
              AfficheActions("");
          }
          break;
      case 3:
          if (DistParcourue >= TabActions[IActionEncours].P1  ) {  // On s'arrete quand on atteint la distance parcourue
              JournalProgession(3,F("S K3 "));
              InitMoteurs();
              TabActions[IActionEncours].Fin = true;
              IActionEncours++;
              AfficheActions("");
          }
          break;
      default:
          if (KesKisPass == 1) { // obstacle détecté à moins de DistArrUrg, mais sans actions identifiée
              JournalProgession(3,F("S DF"));
              InitMoteurs();
          }
          break;      
  }
  

  
  switch (KesKisPass) {
        case 2000:
            break;
        case 1000:
            InitMoteurs();
            JournalErreurs(1,F("Arrêt Urgence Surv"));
            break;    
        default:    
            JournalProgession(2,String(F("S Evitement ")) + String(KesKisPass));
            Evitement (KesKisPass);
            break;             
            
  }

  return;
}


boolean ObstacleScanMarche () {
// vérifie si obstacle sur 3 points, si obstacles on forcera un scan plus précis

 if  ( abs(TabRadar[IRadGauche].XMes) < ProfVision  ||  abs(TabRadar[IRadDroite].XMes) < ProfVision || TabRadar[IRMedian].Distance   < DistArrUrg ) {
      return true;
 }
 else {
      return false;
  } 
      
}


int TraiteDonneesScanMarche () {

// Traite les données du scan Marche pour la detection d'obstacles

    if (FScanMarche) {
            if  (  TabRadar[IRMedian].Distance   < DistArrUrg ) {
                  // la moyenne des 3 mesures centrale est < à ProfVision - Obstacle au centre --> Arrêt          
                  return 1000;     // Arrêt
            }
            
            if  ( abs(TabRadar[IRadGauche].XMes) < ProfVision ) {   
                  // Obstacle a moins de x mm a gauche
                  // on cherche la voie libre à + de ProfVision mm
                  for (int Iobs = IRadGauche; Iobs >= IRadDroite; Iobs--) {
                      if (abs(TabRadar[Iobs].XMes) > ProfVisionObs ) {
                            JournalProgession(2,String(F("S G ")) + String(TabRadar[Iobs].Angle) + " " + String(TabRadar[Iobs].Distance) );
                            return( 90 - TabRadar[Iobs].Angle);
                      }
                  }
                  return 1000;     // Pas de voie libre
            }
            
            if  ( abs(TabRadar[IRadDroite].XMes) < ProfVision ) {   
                  // Obstacle a moins de x mm a droite
                  for (int Iobs = IRadDroite; Iobs <= IRadGauche; Iobs++) {
                      if (abs(TabRadar[Iobs].XMes) > ProfVisionObs  ) {
                            JournalProgession(2,String(F("S D ")) + String(TabRadar[Iobs].Angle) + " " + String(TabRadar[Iobs].Distance) );
                            return( 90 - TabRadar[Iobs].Angle);
                      }
                  }
                  return 1000;     // Pas de voie libre                  

            }
            
    }
    return 2000;
}

void ScanMarche () {

  // scan les distances entre 2 bornes pendant que le robot roule, 1 scan par passage dans le loop

  boolean UnDelai = false;

  MlssMaintenant=millis();
  if((MlssMaintenant- MlssAvantScan) >  ClicChronoScanMarche) {  // Il faut au moins N millisecondes entre chaque mesure

        JournalProgession(3,F("ScanMarche 2") );
        if (IRadMarcheEncours > IRadGauche ) { 
            IRadMarcheEncours = IRadDroite;  
            FScanMarche = true;   // Un scan Marche a été fait au moins une fois
        }

        if (IRadMarcheEncours == IRadDroite ) {UnDelai = true;}  // delai supplémentaire pour que le servo est le temps de revenir à doite
        
        TabRadar[IRadMarcheEncours].Distance = MesureTourelle(TabRadar[IRadMarcheEncours].Angle, UnDelai );
        TabRadar[IRadMarcheEncours].XMes = TabRadar[IRadMarcheEncours].Distance * cos(DegVersRad(TabRadar[IRadMarcheEncours].Angle));
        TabRadar[IRadMarcheEncours].YMes = TabRadar[IRadMarcheEncours].Distance * sin(DegVersRad(TabRadar[IRadMarcheEncours].Angle));
        
        
        IRadMarcheEncours++;
        MlssAvantScan=millis();
        
  }

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
                if (TabRadar[IradScan].Angle == 0) {UnDelai=true;}  // delai supplémentaire pour que le servo est le temps de revenir à doite
                TabRadar[IradScan].Distance = MesureTourelle(TabRadar[IradScan].Angle,UnDelai);
                
            }
            if (TabRadar[IradScan].Distance > DistMax) {DistMax = TabRadar[IradScan].Distance;}
            
            TabRadar[IradScan].XMes = TabRadar[IradScan].Distance * cos(DegVersRad(TabRadar[IradScan].Angle)) ;
            TabRadar[IradScan].YMes = TabRadar[IradScan].Distance * sin(DegVersRad(TabRadar[IradScan].Angle)) ;
            
            HC05.print("*LX" +  String(TabRadar[IradScan].XMes  )+"Y"+String(TabRadar[IradScan].YMes ));
            
            TabRadar[IradScan].XMes = TabRadar[IradScan].Distance * cos(DegVersRad(TabRadar[IradScan].Angle)) / 200;
            TabRadar[IradScan].YMes = TabRadar[IradScan].Distance * sin(DegVersRad(TabRadar[IradScan].Angle)) / 200;

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
                TabLissage[ILis].XDrl = TabRadar[Irad - 1].XMes;
                TabLissage[ILis].YDrl = (TabRadar[Irad - 1].a * TabRadar[Irad - 1].XMes) + TabRadar[Irad - 1].b;
                
                NbLis++;

                // on conserve l'indice contenant le plus grand Y
                if (TabLissage[ILis].YDrl > Ymax ) { 
                    Ymax = TabLissage[ILis].YDrl ; 
                    IYmax = ILis; 
                }
                   
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

    TabLissage[ILis].Irad = NbLectRadar;
    TabLissage[ILis].XDrl = TabRadar[NbLectRadar].XMes;
    TabLissage[ILis].YDrl = TabRadar[NbLectRadar].YMes;
    
    return ILis;
}


void RegLineaire(int Debut, int fin) {
    // Calcu de la droite de regression linéaire pour les points entre deux bornes du tableau
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
  float Seuil = 1.7;     // Distance a la droite mini pour être incorporer à la DRL
  byte  ILis;
  byte  Nbtentatives = 0;
  boolean CycleOK = true;


    //*********************************************************************
    // Calculs des segments de lissages par itération (Moins de 10 segments et plus de 4 )
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

       //Serial.println(";");
        
    }
 

return true;
}  // fin traiteDonneesScan


int Trajectoire (int ICouloir)  {

    // on fait le calcul de trajectoire avec les données non lissées
       
    // calcule de la pente de la droite pointant vers le point souhaité
    int CapScan;
    int AnglePoint;
    float Pente = 1.0 * TabRadar[ TabLissage[ICouloir].Irad - 1].YMes / TabRadar[ TabLissage[ICouloir].Irad -1].XMes;
    
    AnglePoint = RadVersDeg(atan(Pente));
        
    if ( AnglePoint < 0) {
          CapScan = (90 - abs(AnglePoint)) * -1;
    }
    else {
         CapScan = 90 - AnglePoint;  // exprimé par rapport à la verticale (le 90] LIDAR)
    }

    CapASuivre = AngleAjout( CapActuel, CapScan);

    Terminal("@","Irad du Ymax " + String(TabLissage[ICouloir].Irad -1));
    Terminal("@","Cap Scan " + String(CapScan));
    Terminal("@","Cap A Suivre " + String(CapASuivre));
    //Serial.println(F("*******    Fin Calcul trajectoire ***** "));
    return CapScan;  
}



int AngleAjout (int Initial, int Ajout) {

  // Ajout : 0° face au robot, 90° A droite, -90° a gauche
  int NouvelAngle = 0;
  NouvelAngle = (Initial + Ajout) % 360;
  return NouvelAngle;
}

int AngleSens (int AngleDepart, int AngleArrivee) {

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

  int M;
  //  parametrage vitesse roue fonction sens du pivot
  int SensPivot = AngleSens(AngleDepart,AngleArrivee);

  if ( SensPivot  == -1 ) {
        PivoterGauche();   
  }
  else {
        PivoterDroite();       
  }

  MonDelay(30);  // Doit être suffisant pour que arrivé dans la bouche l'azimuth ne soit plus égal à l'angle de départ


  while (  AngleSens(AngleArrivee,LectAzimuth()) != SensPivot ) {
      // On boucle tant que l'on a pas atteint l'angle d'arrivée
 
    Distance = Mesure();

     if  (  DistAttendue != 0 && (Distance  > DistAttendue * 0.95 &&   Distance < DistAttendue * 1.05 )) {
      break;
     }
      MonDelay(20);
  }
  InitMoteurs();
  
  CapASuivre = AngleArrivee;
 
}

void Simulation() {

//  Remplir le tableau manuellement
TabRadar[0].Distance=1042;

}

void AfficheActions(String Action) {
    Terminal("+", "------------------");
    for (byte i = 0; i<5 ; i++) {
        Terminal("+", Action + " " + String(TabActions[i].TypeAction) + " " +  String(TabActions[i].P1) + " " +  String(TabActions[i].P2) + " " +  String(TabActions[i].Debut) + " " +  String(TabActions[i].Fin));
    }

}

void AfficheLissage(byte ILIS) {
  
    String MessageHC05;

     MessageHC05 = "*HX0Y0,X0Y0";
     HC05.print(MessageHC05);
       
    for (byte i=0; i <= ILIS ; i++) {

         MessageHC05 = "*HX" +  String(TabLissage[i].XDrl  )+"Y"+String(TabLissage[i].YDrl ) ;
         HC05.print(MessageHC05);      
         Terminal("$", String(i) + " " + String(TabLissage[i].Irad) + " " + String(TabLissage[i].Angle) + " " + String(TabLissage[i].XDrl)+ " " + String(TabLissage[i].YDrl) + " " + String(TabLissage[i].DZO)  + " " + String(IYmax)  + "/");
     }
}

void AfficheCap ( int Cap) {

    String MessageHC05;
    int Y = DistMax * sin(DegVersRad(90 - Cap));
    int X = DistMax * cos(DegVersRad(90 - Cap));  

     MessageHC05 = "*LX0Y0,X0Y0";
     HC05.print(MessageHC05);  
     MessageHC05 = "*LX0Y0,X" + String(X) + "Y" + String(Y);
     HC05.print(MessageHC05);    
    
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
            ServeurServo.writeMicroseconds(2,VitssGaucheCons * 0.9);  // Gauche, doucement
            ServeurServo.writeMicroseconds(1,PtMortServoRoue ); // Droite
            RobotRoule = true;
}
void PivoterGauche () {
            ServeurServo.writeMicroseconds(2,PtMortServoRoue);  // Gauche
            ServeurServo.writeMicroseconds(1,VitssDroitCons + 200); // Droite, doucement
            RobotRoule = true;
}


void Evitement (int Angle) {

// Evite un obstacle (par passe successive si besoin)

    InitMoteurs(); 
    int AngleVise = AngleAjout (CapActuel, Angle);
    pivoter( CapActuel, AngleVise, 0);
    AllerDevant();
    MonDelay(2000);
    InitMoteurs(); 
    pivoter(AngleVise,  CapActuel,0);
    AllerDevant();
}

void JournalProgession (byte Niveau, String Message) {
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
          Complement.concat(Message); 

          Terminal ("-",Complement); 
          //MonDelay(500); //Pour ralentir le traitement 
      } ;
}

void JournalErreurs (byte Niveau, String Message) {
// Niveau    1- Haut , 2 - Moyen, 3 - Bas
      if (Niveau <= NiveauErr) { Terminal ("@",Message); } ;
}

void LectureHall() {

  MlssMaintenant=millis();

  if((MlssMaintenant- MlssAvantHall) >  ClicChronoHall) {  // on ne fait une mesure qu'au moins tous les "ClicChronoHall" millisecondes
            ValHall = analogRead(Pin_Hall);   // Lecture valeur analogique sur la Pin A1
            if (ValHall > SeuilHall && ValHallAvant <= SeuilHall) { // On ne fait +1 que si la value lue et celle d'avant sont de part et d'autre de "Seuil"
                ClicHall++;
                DistParcourue = ((PI * 30) / 4) * ClicHall;  // Distance en mm, la roue a un diametre de 30 mm  
                Terminal(">",String(ClicHall)+"/"+String(DistParcourue));
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
    Serial.println(F("Tf Luna Echec mesure"));
    return -1;
  }
  //Terminal ("+","Mesure " + String(DistCm));
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
      return (Som / NbOk);
  }

}


int MesureTourelle(byte Angle, boolean UnDelai) {

    int Distance;

    PositionServo(3,Angle); 
    if ( UnDelai ) { MonDelay(200);}
        
    Distance = Mesure();
        
    if (Distance == -1 ) {   // SI mesure erronee on en prend une autre
            Distance = Mesure();
    }
    return (Distance);
}


void PositionServo(byte Nservo, byte AngleServo) {       // Met le servo sur un angle donné
    ServeurServo.setPWM(Nservo, 0, map(AngleServo, 0, 180, 158, 450));
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

void InitTabRadar () {

  // Initialise les angles à scaner
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

boolean MesureDistLuna(uint8_t TfLunaAddress)
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
      default:
          Serial.println("Pas de trt du mess bluetooth");
          break;
      }
    messageRecu = "";
    }
}

void Etape() {   // Met le robot en stand by (boucle)
       Terminal("@",F("Validez etape "));
       while (!VerifBT()) {};
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



int LectAzimuth () {      // Lit la veur du cap boussole

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


void StockeProfil()
{

  //Inspiré par :  https://roboticboat.uk/Arduino/Due/ArduinoDueCompass12.html
  // Après calibrage de la boussole, permet de strocker le profil

  // Cf DataSheet : To store a profile write the following to the command register 0xF0, 0xF5, 0xF6 with a 20ms MonDelay after each of the three bytes

/**********    Décommenter le code pour utilisation (Libère de la mémoire)    ***********

  
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
  
  */
}

