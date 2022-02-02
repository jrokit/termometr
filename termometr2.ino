#include <Bounce2.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define BUTTON_PIN_B 3

#define LED_PIN_A 5
#define LED_PIN_B 6

#define TER_PIN_A 7
//Interwal pomiedzy odczytem danych z czujnikow
#define T_REFRESH 10000

#define BUZZER_PIN 8
#define BUZZER_HZ 3600

#define POMPA_PIN 9
//temp zalaczania pompki
#define POMPA_TEMP 40.0
//temp alarmu
#define ALARM_TEMP 80.0

#define BEEP_X 5 //pikniecie co x czasu
#define BEEP_MS 500 //dlugosc pikniecia
#define BEEP_PAUSE 300000 //dlugosc pauzy po nacisnieciu 300000- 5 minut

OneWire oneWire(TER_PIN_A); //Podłączenie do A5
DallasTemperature sensors(&oneWire); //Przekazania informacji do biblioteki

LiquidCrystal_I2C lcd(0x27,16,2);

Bounce2::Button buttonB = Bounce2::Button();

//unsigned long buttonPushedMillis; // when button was released
unsigned long cichyAlarm; // po ilu sekundach od wyciszenia Alarmu ma znowu wyć - okolo 5 minut 300000 ms

//unsigned long czasBB; //dlugie przytrzymanie przycisku B

unsigned long StartTime;
//unsigned long CurrentTime;

byte customChar[8] = {
  0b01000,
  0b10100,
  0b10100,
  0b01000,
  0b00000,
  0b00000,
  0b00000,
  0b00000
};

//bool praca; // true jesli nie wcisniety przycisk
bool alarmON;
bool alarmOFF;
float tempC, tempC_old;
float tempD;
//int czasWA;
unsigned long st;
//int tonB, l, pMax;
int l;
int err;

void setup() {
  //Serial.begin(9600);
  sensors.begin();
  
  //Konfiguracja wyswietlacza
  lcd.init();                                     
  lcd.backlight();                      
  lcd.clear();
  lcd.print("Termometr");            
  lcd.setCursor(0, 1);                   
  lcd.print("startuje"); 
  lcd.createChar(0, customChar);
  
  //Konfiguracja przycisku  
  buttonB.attach( BUTTON_PIN_B ,  INPUT_PULLUP );
  buttonB.interval(5); 
  buttonB.setPressedState(LOW); 

  //Konfiguracja pinu diody LED :
  pinMode(LED_PIN_A,OUTPUT);
  pinMode(LED_PIN_B,OUTPUT);

  //Konfiguracja przekaznika pompy
  pinMode(POMPA_PIN,OUTPUT);
  digitalWrite(POMPA_PIN, LOW); 
  
  //Konfiguracja buzzera
  pinMode(BUZZER_PIN, OUTPUT);
  
  StartTime = millis();
  st = millis();
  //praca = true;
  alarmON = false;
  alarmOFF = false;
  //domyslna temperatura
  tempC = POMPA_TEMP - 1.0; 
  tempD = 20;
  l=0; //zmienna licznika pikniec
  err = 0;
  
}

void loop() {
  buttonB.update();
 // Zalaczanie pompki
    if (tempC >= POMPA_TEMP ) {
        pompa(true);
        diodaB(true);
      }
    else {
        pompa(false);
        diodaB(false);
     }  
      
  

  
  // Alarm
  if ( alarmON && tempC >= ALARM_TEMP && !alarmOFF) {
    dzwiek(BEEP_MS, &st, &l, BEEP_X);
    }
  else {
     noTone(BUZZER_PIN);
  }
  //Odczyt czujnikow co X sekund
  if ( opozniacz(T_REFRESH, &StartTime) ) {    //&& praca
      pobierzTemp(sensors, &tempC, &tempD);

   

    if ( tempC >= ALARM_TEMP ) {
        alarmON=true;
        diodaA(true);
      }
    else   {
        alarmON=false;
        diodaA(false);
      }

    if ( tempC != DEVICE_DISCONNECTED_C && tempD != DEVICE_DISCONNECTED_C )    {
        wyswietlTemp();
        err = 0;
        tempC_old = tempC;
      } 
    else    {
        tempC = tempC_old;
        String error = "Blad czujnika";
       // Serial.println(error);
        //alarmOFF = false;
        //alarmON=true;
        //bzz(true);
        err++;
    //    tempC = POMPA_TEMP;
        wyswietlError(lcd, error);

      }

  }
  
  if (err >= 3 ) {

    //alarmOFF = false;
    //alarmON=true;
    //bzz(true);
    tempC = POMPA_TEMP;
    
  }
  
  if ( buttonB.pressed() ) {

      cichyAlarm = millis(); 
      alarmOFF = true;
 
  }
  
 if ( alarmOFF && millis() - cichyAlarm >= BEEP_PAUSE ) {
     alarmOFF = false;
   }

}


void diodaA(bool dA){
  if (dA){
  digitalWrite(LED_PIN_A, HIGH);
  }
  else {
  digitalWrite(LED_PIN_A, LOW);  
  }
  
}
void diodaB(bool dB){

  if (dB){
  digitalWrite(LED_PIN_B, HIGH);
  }
  else {
  digitalWrite(LED_PIN_B, LOW);  
  }
}
void wyswietlError(LiquidCrystal_I2C l, String error){
    l.clear();
    l.setCursor(0, 0);
    l.print(error);
}

void wyswietlTemp(){

   // Serial.print("Temperature for the device 1 (index 0) is: ");
    //Serial.println(tempC);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Piec:   ");
    lcd.print(tempC);
    lcd.write((uint8_t)0);
    lcd.print("C");
    //Serial.print("Temperature for the device 2 (index 1) is: ");
    //Serial.println(tempD);

    lcd.setCursor(0, 1);
    lcd.print("Bojler: ");
    lcd.print(tempD);
    lcd.write((uint8_t)0);
    lcd.print("C");
  
}
bool opozniacz(unsigned long ms, unsigned long *wst) {
// wywolanie funkcji (opozniacz(1000, &StartTime))
  if ( millis() - *wst >= ms ) {
    
    *wst = millis();
    
    return true;
  }
    else {
      return false;
    }
}
void pobierzTemp(DallasTemperature s, float *tC, float *tD) {
  
    s.requestTemperatures();
    *tC = sensors.getTempCByIndex(0);
    *tD = sensors.getTempCByIndex(1);
}
//funkcja bzz t- wlacz,
void bzz(bool t) {
  if (t) {
       tone(BUZZER_PIN, BUZZER_HZ);
  }
  else {
       noTone(BUZZER_PIN);
  }

}
// (ile sekund, czas, przerwalicznik, przerwa, port, hz)
void dzwiek(unsigned long ms, unsigned long *wst, int *p, int pmax) {

  if (opozniacz(ms, wst)){
   if (*p == pmax) {
        bzz(true);
        *p=0;
      }
   else {
        bzz(false);
        *p=*p+1;
      }
   }
 
}

void pompa(bool b){
  if(b) {
      digitalWrite(POMPA_PIN, HIGH);
  }
  else{
      digitalWrite(POMPA_PIN, LOW);

  }
  
}
