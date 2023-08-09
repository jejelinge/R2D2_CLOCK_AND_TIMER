#include <DFMiniMp3.h>
#include <SoftwareSerial.h>
#include "WiFiManager.h"
#include "NTPClient.h"
#include "TM1637Display.h"
#include "DFRobotDFPlayerMini.h"

//========================USEFUL VARIABLES=============================
const char *ssid     = "SSID"; 
const char *password = "PASSWORD";
const long utcOffsetInSeconds = 7200; // UTC + 2H / Offset in second
uint16_t notification_volume= 15;
int Display_backlight = 3; // Set displays brightness 0 to 7;
//=====================================================================


#define CLK 25
#define DT 26
#define SW 27
#define RED_LED 32
#define WHITE_LED 33

const byte RXD2 = 16; // Connects to module's RX 
const byte TXD2 = 17; // Connects to module's TX 

#if (defined(ARDUINO_AVR_UNO) || defined(ESP8266))   // Using a soft serial port
SoftwareSerial softSerial(/*rx =*/4, /*tx =*/5);
#define FPSerial softSerial
#else
#define FPSerial Serial1
#endif

DFRobotDFPlayerMini myDFPlayer;
void printDetail(uint8_t type, int value);

float counter = 0;
int currentStateCLK;
int lastStateCLK;
String currentDir = "";
unsigned long lastButtonPress = 0;
int btnState = 0;

int secondes = 0;
int minutes = 0;
float inc_red_led = 0;




// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);
TM1637Display red1(21, 22);


void setup() {

  #if (defined ESP32)
  FPSerial.begin(9600, SERIAL_8N1, RXD2, TXD2);
#else
  FPSerial.begin(9600);
#endif

  pinMode(CLK, INPUT);
  pinMode(DT, INPUT);
  pinMode(SW, INPUT_PULLUP);
  pinMode(RED_LED, OUTPUT);
  pinMode(WHITE_LED, OUTPUT);
    // configure LED PWM functionalitites
  ledcSetup(0, 5000, 8);
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(RED_LED, 0);
  red1.setBrightness(Display_backlight);
  Serial.begin(115200);

  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));
  
  if (!myDFPlayer.begin(FPSerial, /*isACK = */true, /*doReset = */true)) {  //Use serial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while(true){
      delay(0); // Code to compatible with ESP8266 watch dog.
    }
  }
  Serial.println(F("DFPlayer Mini online."));
  
  myDFPlayer.volume(20);  //Set volume value. From 0 to 30
  myDFPlayer.play(1);  //Play the first mp3

  Serial.println("\n Starting");

  WiFi.begin(ssid, password);


  while ( WiFi.status() != WL_CONNECTED ) {
    delay ( 500 );
    Serial.print ( "." );
  }

  timeClient.begin();

  // Read the initial state of CLK
  lastStateCLK = digitalRead(CLK);

}

void loop() {

 timeClient.update();
 red1.showNumberDecEx(timeClient.getHours(),0b01000000,true,2,0);
 red1.showNumberDecEx(timeClient.getMinutes(),0b01000000,true,2,2);

  if (myDFPlayer.available()) {
    printDetail(myDFPlayer.readType(), myDFPlayer.read()); //Print the detail message from DFPlayer to handle different errors and states.
  }

  btnState = digitalRead(SW);
  ledcWrite(0, inc_red_led);
  digitalWrite(WHITE_LED, LOW);

  //If we detect LOW signal, button is pressed
  if (btnState == LOW) {
    //if 50ms have passed since last LOW pulse, it means that the
    //button has been pressed, released and pressed again
    if (millis() - lastButtonPress > 50) {
      Serial.println("Button pressed!");
      Setup_timer();
    }

    // Remember last button press event
    lastButtonPress = millis();
  }

if ( inc_red_led <= 254)
   inc_red_led = inc_red_led + 0.1;
else inc_red_led = 0;

  delay(1);

}

void Setup_timer() {

  red1.showNumberDecEx(88,0b01000000,true,2,0);
  red1.showNumberDecEx(88,0b01000000,true,2,2);

  digitalWrite(RED_LED, HIGH);
  digitalWrite(WHITE_LED, HIGH);
  myDFPlayer.play(2);
  delay(500);

  btnState = digitalRead(SW);
  while (btnState == HIGH) {
    // Read the current state of CLK
    currentStateCLK = digitalRead(CLK);
    btnState = digitalRead(SW);
    // Serial.println("Dans la boucle");
    // If last and current state of CLK are different, then pulse occurred
    // React to only 1 state change to avoid double count
    if (currentStateCLK != lastStateCLK  && currentStateCLK == 1) {
      // If the DT state is different than the CLK state then
      // the encoder is rotating CCW so decrement
      if (digitalRead(DT) != currentStateCLK) {
        counter = counter - 10;
        currentDir = "CCW";
      } else {
        // Encoder is rotating CW so increment
        counter = counter + 10;
        currentDir = "CW";
      }

      if (counter < 0){counter = 0;}
      
      Serial.print("Direction: ");
      Serial.print(currentDir);
      Serial.print(" | Counter: ");
      Serial.println(counter);
    }

    // Remember last CLK state
    lastStateCLK = currentStateCLK;

    minutes = counter / 60;
    secondes =  ((counter / 60) - minutes) * 60;
    red1.showNumberDecEx(minutes,0b01000000,true,2,0);
    red1.showNumberDecEx(secondes,0b01000000,true,2,2);
    //delay(1);

  }
  Countdown(counter);
  Serial.println("Sortie de la boucle");
}


void Countdown (float timer_counter) {
  
  myDFPlayer.play(8);;
  delay(1000);
  btnState = digitalRead(SW);

  while (btnState == HIGH )
  {
    btnState = digitalRead(SW);

    for (int i = 10 ; i > 0; i--) {

      timer_counter = timer_counter - 0.1;
      minutes = timer_counter / 60;
      secondes =  ((timer_counter / 60) - minutes) * 60;
  red1.showNumberDecEx(minutes,0b01000000,true,2,0);
  red1.showNumberDecEx(secondes,0b01000000,true,2,2);
      delay(50);
      digitalWrite(RED_LED, LOW);
    }

    if (timer_counter <= 0) {

      myDFPlayer.play(3);

      for ( int i = 0 ; i < 9 ; i++) {
        ledcWrite(0, 255);
        waitMilliseconds(random(10, 150));
        digitalWrite(WHITE_LED, HIGH);
        waitMilliseconds(random(10, 150));
        ledcWrite(0, 0);
        waitMilliseconds(random(10, 150));
        digitalWrite(WHITE_LED, LOW);
        waitMilliseconds(random(10, 150));
      }

      myDFPlayer.play(5);
      
      for ( int i = 0 ; i < 9 ; i++) {
        ledcWrite(0, 255);
        waitMilliseconds(random(10, 150));
        digitalWrite(WHITE_LED, HIGH);
        waitMilliseconds(random(10, 150));
        ledcWrite(0, 0);
        waitMilliseconds(random(10, 150));
        digitalWrite(WHITE_LED, LOW);
        waitMilliseconds(random(10, 150));
      }

      myDFPlayer.play(7);

      for ( int i = 0 ; i < 9 ; i++) {
        ledcWrite(0, 255);
        waitMilliseconds(random(10, 150));
        digitalWrite(WHITE_LED, HIGH);
        waitMilliseconds(random(10, 150));
        ledcWrite(0, 0);
        waitMilliseconds(random(10, 150));
        digitalWrite(WHITE_LED, LOW);
        waitMilliseconds(random(10, 150));
      }


      btnState = LOW;
      counter = 0;
    };
  }

}


void waitMilliseconds(uint16_t msWait)
{
  uint32_t start = millis();

  while ((millis() - start) < msWait)
  {
    // calling mp3.loop() periodically allows for notifications
    // to be handled without interrupts
    delay(1);
  }
}

void printDetail(uint8_t type, int value){
  switch (type) {
    case TimeOut:
      Serial.println(F("Time Out!"));
      break;
    case WrongStack:
      Serial.println(F("Stack Wrong!"));
      break;
    case DFPlayerCardInserted:
      Serial.println(F("Card Inserted!"));
      break;
    case DFPlayerCardRemoved:
      Serial.println(F("Card Removed!"));
      break;
    case DFPlayerCardOnline:
      Serial.println(F("Card Online!"));
      break;
    case DFPlayerUSBInserted:
      Serial.println("USB Inserted!");
      break;
    case DFPlayerUSBRemoved:
      Serial.println("USB Removed!");
      break;
    case DFPlayerPlayFinished:
      Serial.print(F("Number:"));
      Serial.print(value);
      Serial.println(F(" Play Finished!"));
      break;
    case DFPlayerError:
      Serial.print(F("DFPlayerError:"));
      switch (value) {
        case Busy:
          Serial.println(F("Card not found"));
          break;
        case Sleeping:
          Serial.println(F("Sleeping"));
          break;
        case SerialWrongStack:
          Serial.println(F("Get Wrong Stack"));
          break;
        case CheckSumNotMatch:
          Serial.println(F("Check Sum Not Match"));
          break;
        case FileIndexOut:
          Serial.println(F("File Index Out of Bound"));
          break;
        case FileMismatch:
          Serial.println(F("Cannot Find File"));
          break;
        case Advertise:
          Serial.println(F("In Advertise"));
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
  
}