#include "WiFiManager.h"
#include "NTPClient.h"
#include "TM1637Display.h"
#include "DFRobotDFPlayerMini.h"
#include "AiEsp32RotaryEncoder.h"

//========================USEFUL VARIABLES=============================
uint16_t notification_volume= 15;
const char *ssid     = "SSID"; // put your SSID between the quotes mark
const char *password = "PASSWORD"; // put your wifi password between the quotes mark
int UTC = 2; // UTC + value in hour - Summer time
int Display_backlight = 3; // Set displays brightness 0 to 7;
//=====================================================================

#define ROTARY_ENCODER_A_PIN 25
#define ROTARY_ENCODER_B_PIN 26
#define ROTARY_ENCODER_BUTTON_PIN 27
#define RED_LED 32
#define WHITE_LED 33
#define ROTARY_ENCODER_STEPS 4
#define ROTARY_ENCODER_VCC_PIN -1

const byte RXD2 = 16; // Connects to module's TX 
const byte TXD2 = 17; // Connects to module's RX
const long utcOffsetInSeconds = 3600; // UTC + 2H / Offset in second
bool res;

AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS);

void IRAM_ATTR readEncoderISR()
{
    rotaryEncoder.readEncoder_ISR();
}

#define FPSerial Serial1
DFRobotDFPlayerMini myDFPlayer;
void printDetail(uint8_t type, int value);

float counter = 0;
String currentDir = "";
unsigned long lastButtonPress = 0;
int btnState = 0;

int secondes = 0;
int minutes = 0;
float inc_red_led = 0;


// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds*UTC);
TM1637Display red1(21, 22);

void setup() {

  pinMode(RED_LED, OUTPUT);
  pinMode(WHITE_LED, OUTPUT);
    // configure LED PWM functionalitites
  ledcSetup(0, 5000, 8);
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(RED_LED, 0);
  red1.setBrightness(Display_backlight);
  Serial.begin(9600);
  WiFi.begin(ssid, password);

  while ( WiFi.status() != WL_CONNECTED ) {
    delay ( 500 );
    Serial.print ( "." );
  }

  timeClient.begin();

  FPSerial.begin(9600, SERIAL_8N1, RXD2, TXD2);
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

  rotaryEncoder.begin();
  rotaryEncoder.setup(readEncoderISR);
  rotaryEncoder.setBoundaries(0, 3500, true); //minValue, maxValue, circleValues true|false (when max go to min and vice versa)
  rotaryEncoder.setAcceleration(250);

}

void loop() {

 timeClient.update();
 red1.showNumberDecEx(timeClient.getHours(),0b01000000,true,2,0);
 red1.showNumberDecEx(timeClient.getMinutes(),0b01000000,true,2,2);

  Serial.print("Time: ");
  Serial.println(timeClient.getFormattedTime());
  unsigned long epochTime = timeClient.getEpochTime();
  struct tm *ptm = gmtime ((time_t *)&epochTime); 
  int currentYear = ptm->tm_year+1900;
  Serial.print("Year: ");
  Serial.println(currentYear);
  
  int monthDay = ptm->tm_mday;
  Serial.print("Month day: ");
  Serial.println(monthDay);

  int currentMonth = ptm->tm_mon+1;
  Serial.print("Month: ");
  Serial.println(currentMonth);

  if (myDFPlayer.available()) {
    printDetail(myDFPlayer.readType(), myDFPlayer.read()); //Print the detail message from DFPlayer to handle different errors and states.
  }

  btnState = digitalRead(ROTARY_ENCODER_BUTTON_PIN);
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
if((currentMonth*30 + monthDay) >= 121 && (currentMonth*30 + monthDay) < 331){
timeClient.setTimeOffset(utcOffsetInSeconds*UTC);} // Change daylight saving time - Summer - change 31/03 at 00:00
else {timeClient.setTimeOffset((utcOffsetInSeconds*UTC) - 3600);} // Change daylight saving time - Winter - change 31/10 at 00:00

}

void Setup_timer() {

  red1.showNumberDecEx(88,0b01000000,true,2,0);
  red1.showNumberDecEx(88,0b01000000,true,2,2);

  digitalWrite(RED_LED, HIGH);
  digitalWrite(WHITE_LED, HIGH);
  myDFPlayer.play(2);
  delay(500);

  btnState = digitalRead(ROTARY_ENCODER_BUTTON_PIN);
  while (digitalRead(ROTARY_ENCODER_BUTTON_PIN) == HIGH) {
  
    if (rotaryEncoder.encoderChanged())
    {
        Serial.println(rotaryEncoder.readEncoder());
        counter = rotaryEncoder.readEncoder();
    }
    if (rotaryEncoder.isEncoderButtonClicked())
    {
        Serial.println("button pressed");
    }

    minutes = counter / 60;
    secondes =  ((counter / 60) - minutes) * 60;
    red1.showNumberDecEx(minutes,0b01000000,true,2,0);
    red1.showNumberDecEx(secondes,0b01000000,true,2,2);
    

  }
  Countdown(counter);
  Serial.println("Sortie de la boucle");
}


void Countdown (float timer_counter) {
  
  myDFPlayer.play(8);;
  delay(1000);
  btnState = digitalRead(ROTARY_ENCODER_BUTTON_PIN);

  while (btnState == HIGH )
  { 
    btnState = digitalRead(ROTARY_ENCODER_BUTTON_PIN);
    for (int i = 10 ; i > 0; i--) {

      timer_counter = timer_counter - 0.1;
      minutes = timer_counter / 60;
      secondes =  ((timer_counter / 60) - minutes) * 60;
  red1.showNumberDecEx(minutes,0b01000000,true,2,0);
  red1.showNumberDecEx(secondes,0b01000000,true,2,2);
      delay(70);
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