#include <DFMiniMp3.h>
#include <SoftwareSerial.h>
#include "WiFiManager.h"
#include "NTPClient.h"
#include "TM1637Display.h"

//========================USEFUL VARIABLES=============================
const char *ssid     = "SSID"; 
const char *password = "WIFI PASSWORD";
const long utcOffsetInSeconds = 7200; // UTC + 2H / Offset in second
uint16_t notification_volume= 15;
int Display_backlight = 3; // Set displays brightness 0 to 7;
//=====================================================================


#define CLK 25
#define DT 26
#define SW 27
#define RED_LED 17
#define WHITE_LED 16

float counter = 0;
int currentStateCLK;
int lastStateCLK;
String currentDir = "";
unsigned long lastButtonPress = 0;
int btnState = 0;

int secondes = 0;
int minutes = 0;
float inc_red_led = 0;

// forward declare the notify class, just the name
//
class Mp3Notify; 

// define a handy type using serial and our notify clas
// Some arduino boards only have one hardware serial port, so a software serial port is needed instead.
// comment out the above definitions and use these
SoftwareSerial secondarySerial(18, 19); // RX, TX
typedef DFMiniMp3<SoftwareSerial, Mp3Notify> DfMp3;
DfMp3 dfmp3(secondarySerial);


class Mp3Notify
{
public:
  static void PrintlnSourceAction(DfMp3_PlaySources source, const char* action)
  {
    if (source & DfMp3_PlaySources_Sd) 
    {
        Serial.print("SD Card, ");
    }
    if (source & DfMp3_PlaySources_Usb) 
    {
        Serial.print("USB Disk, ");
    }
    if (source & DfMp3_PlaySources_Flash) 
    {
        Serial.print("Flash, ");
    }
    Serial.println(action);
  }
  static void OnError([[maybe_unused]] DfMp3& mp3, uint16_t errorCode)
  {
    // see DfMp3_Error for code meaning
    Serial.println();
    Serial.print("Com Error ");
    Serial.println(errorCode);
  }
  static void OnPlayFinished([[maybe_unused]] DfMp3& mp3, [[maybe_unused]] DfMp3_PlaySources source, uint16_t track)
  {
    Serial.print("Play finished for #");
    Serial.println(track);  

    // start next track
    track += 1;
    // this example will just start back over with 1 after track 3
    if (track > 3) 
    {
      track = 1;
    }
    dfmp3.playMp3FolderTrack(track);  // sd:/mp3/0001.mp3, sd:/mp3/0002.mp3, sd:/mp3/0003.mp3
  }
  static void OnPlaySourceOnline([[maybe_unused]] DfMp3& mp3, DfMp3_PlaySources source)
  {
    PrintlnSourceAction(source, "online");
  }
  static void OnPlaySourceInserted([[maybe_unused]] DfMp3& mp3, DfMp3_PlaySources source)
  {
    PrintlnSourceAction(source, "inserted");
  }
  static void OnPlaySourceRemoved([[maybe_unused]] DfMp3& mp3, DfMp3_PlaySources source)
  {
    PrintlnSourceAction(source, "removed");
  }
};

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);
TM1637Display red1(21, 22);


void setup() {
  
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

  dfmp3.begin();

  uint16_t version = dfmp3.getSoftwareVersion();
  Serial.print("version ");
  Serial.println(version);

  uint16_t volume = dfmp3.getVolume();
  Serial.print("volume ");
  Serial.println(volume);
  dfmp3.setVolume(notification_volume);
  
  uint16_t count = dfmp3.getTotalTrackCount(DfMp3_PlaySource_Sd);
  Serial.print("files ");
  Serial.println(count);
  
  Serial.println("starting...");

  // start the first track playing
  dfmp3.playMp3FolderTrack(1);  // sd:/mp3/0001.mp3


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
  dfmp3.playMp3FolderTrack(2);
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
  
  dfmp3.playMp3FolderTrack(8);
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

      dfmp3.playMp3FolderTrack(3);

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

      dfmp3.playMp3FolderTrack(5);
      
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

      dfmp3.playMp3FolderTrack(7);

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
    dfmp3.loop();
    delay(1);
  }
}
