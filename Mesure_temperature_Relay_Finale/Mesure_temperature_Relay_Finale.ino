/* MISSION:
 *  - Afficher la température sur un écran LCD - Grove - LCD RGB Backlight - Temperature Sensor V1.1/1.2.
 *  - Définir une température sur ce même écran lcd - Grove - Rotary angle sensor.
 *  - Allumer un relais qui fait office de radiateur. 
 *  --------------------------------
 *  Vincent Marais | 12/01/2022
 *  github: https://github.com/Betawolfy
 *  --------------------------------
 *  NOTES TECHNIQUES: 
 *  le Grove - Temperature Sensor V1.1/1.2 doit être branché sur le port A0
 *  le Grove - LCD RGB Backlight doit être connecté sur le port I2C
 *  le Grove - Rotary angle sensor doit être branché sur A1
 *  Le Grove - Relay doit être branché en D3
 */

//ci dessous, les librairies nessesaire pour que le code fonctionne.

// les deux ci-dessous sont natifs dans Arduino (cela veut dire qu'il sont déjà installé). 
#include <Wire.h>
#include <math.h>

// La bibliothèque ci-dessous doit être installé: https://github.com/Seeed-Studio/Grove_LCD_RGB_Backlight/archive/master.zip
#include "rgb_lcd.h" 

//configuration du Grove - LCD RGB Backlight : dans cette exemple: l'écran sera bleu clair.
rgb_lcd lcd;

const int colorR = 0;
const int colorG = 255;
const int colorB = 255;

//configuration du Temperature Sensor V1.1/1.2 du Rotary angle sensor et du Relay. 

const int B = 4275;               //  Valeur B de la thermistance
const int R0 = 100000;            // R0 = 100k
const int pinTempSensor = A0;     // le capteur de thempérature est connecté sur A0
const int ROTARY_ANGLE_SENSOR = A1; // le capteur d'angle est connecté en A1
const int RELAY = 3;              // Le relais est connecté en D3

#if defined(ARDUINO_ARCH_AVR)
#define debug  Serial
#elif defined(ARDUINO_ARCH_SAMD) ||  defined(ARDUINO_ARCH_SAM)
#define debug  SerialUSB
#else
#define debug  Serial
#endif

#define ADC_REF 5 //la tension de référence de l'ADC est de 5v. Si l'interrupteur Vcc sur la carte arduino
                    //la carte arduino passe à 3V3, l'ADC_REF devrait être à 3.3                    
#define GROVE_VCC 5 // Le voltage en courant continue du grove est normalement de 5V
#define FULL_ANGLE 300 //La valeur totale de l'angle rotatif est de 300 degrés.

void setup() {
// Partie du Temperature Sensor V1.1/1.2
    
    Serial.begin(9600); //nombre de données par secondes. ici, environ 9500 bytes par secondes.

// Partie du Grove - LCD RGB Backlight
  // définir le nombre de colonnes et de lignes de l'écran LCD (nombre de colonnes, nombres de ligne)
    lcd.begin(16, 2);
    
    lcd.setRGB(colorR, colorG, colorB);

 // Partie du Grove - Rotary angle sensor
  // on va dire que le Grove - Rotary angle sensor est une entrée (on entre une donnée)
  // On va aussi dire le Grove - Relay est une sortie (on communique une donnée)
  
 pinMode(ROTARY_ANGLE_SENSOR, INPUT);     
 pinMode(RELAY, OUTPUT);
}

void loop() {
  
//Partie du Temperature Sensor V1.1/1.2  
  int a = analogRead(pinTempSensor);
 
    float R = 1023.0/a-1.0;
    R = R0*R;
 
    int temperature = 1.0/(log(R/R0)/B+1/298.15)-273.15; // définition de la variable "temperature" via l'opération du code. 
    Serial.print("T_MES : "); 
    Serial.println(temperature); // la valeur de la température est affiché dans le moniteur série

//Partie du Grove - Rotary angle sensor

 float voltage; // création d'une variable décimale appelé "voltage".
    int sensor_value = analogRead(ROTARY_ANGLE_SENSOR); //création d'une variable entière appelé "sensor_value". celle-ci est égale à l'angle du Grove - Rotary angle sensor
    voltage = (float)sensor_value*ADC_REF/1023; 
    int consigne = ((voltage*FULL_ANGLE)/GROVE_VCC)/10; //on calcule la variable consigne. on divise par 10 pour obtenir un résultat uniquement entre 0 et 30

// condition pour afficher si le relais est allumé ou non. 
// L'utilisation d'une structure en "if" est plus adapté que les autres. Le while aussi, mais nous sommes déjà dans une boucle. 

if(consigne > temperature){  // on pose une contition : si la température de consigne est inférieur à la température mesurée 
  lcd.setCursor(14, 0);
  lcd.print("on");    // on marque sur l'écran que le relay et on 
  digitalWrite(RELAY, HIGH);   // on intègre une tension haute au relay. 
}
else{       // dans le cas ou la température de consigne estsupérieur à la température mesurée.
  lcd.setCursor(13, 0);
  lcd.print("off");     // on marque sur l'écran que le relay est off
  digitalWrite(RELAY, LOW);     // on intègre une tension basse au relay. 
}

// partie pour le Grove - LCD RGB Backlight

  // placer le curseur à la colonne 0, ligne 1
    // (remarque : la ligne 1 est la deuxième ligne, puisque le comptage commence par 0) :
    lcd.setCursor(0, 0);
    // affichage de la température sous la forme "temp_mes : " <variable> "°" "C"
    lcd.print("T_MES:");
    lcd.print(temperature);
    lcd.print(char(223));
    lcd.print("C");

//on saute une ligne.
 lcd.setCursor(0, 1);
 
  // affichage de la température sous la forme "temp_cons : " <variable> "°" "C"
        lcd.print("T_CONS:");
    lcd.print(consigne);
    lcd.print(char(223));
    lcd.print("C");
    delay(100); // le programme ce répète tout les 100 ms
    lcd.clear(); 
}
/* code utilisé: 
 *  Wikiseeed : https://wiki.seeedstudio.com/Grove-LCD_RGB_Backlight/
 *              https://wiki.seeedstudio.com/Grove-Temperature_Sensor_V1.2/
 *              https://wiki.seeedstudio.com/Grove-Rotary_Angle_Sensor/
 *              https://wiki.seeedstudio.com/Grove-Relay/
 * 
 * Ressources complémentaires:
 *              https://www.arduino.cc/reference/en/language/structure/control-structure/else/
 * Travail réalisé pour un TP
 */
