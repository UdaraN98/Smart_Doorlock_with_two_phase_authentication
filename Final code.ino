// Include required libraries
#include "model.h"
#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h> // include i/o class header

hd44780_I2Cexp lcd;

#include <Keypad.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include <SPI.h>
#define MIC A0
#define INTERVAL 5
#define SOUND_THRESHOLD 115
#define NUM_SAMPLES 32
#include <arduinoFFT.h>

// Create instances
arduinoFFT fft;

float backgroundSound = 0;

float features[NUM_SAMPLES];
double featuresForFFT[NUM_SAMPLES];
Eloquent::ML::Port::RandomForest classifier;


SoftwareSerial SIM900(9, 10); // SoftwareSerial SIM900(Rx, Tx)
SoftwareSerial Voice(A4,A5);
//LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo sg90;

// Initialize Pins for led's, servo and buzzer

constexpr uint8_t servoPin = 11;
constexpr uint8_t buzzerPin = 8;

char initial_password[4] = {'1', '2', '3', '4'};  // Variable to store initial password

char password[4];   // Variable to store users password
boolean VoiceMode = true; // boolean to change modes
boolean NormalMode = true; // boolean to change modes
char key_pressed = 0; // Variable to store incoming keys
uint8_t i = 0;  // Variable used for counter

// defining how many rows and columns our keypad have
const byte rows = 4;
const byte columns = 4;

// Keypad pin map
char hexaKeys[rows][columns] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

// Initializing pins for keypad
byte row_pins[rows] = {A1, A2, A3,3};
byte column_pins[columns] = {2, 1, 0};

// Create instance for keypad
Keypad keypad_key = Keypad( makeKeymap(hexaKeys), row_pins, column_pins, rows, columns);

void setup() {
  // Arduino Pin configuration
  Voice.begin(115200);
  //Serial.begin(9600);
  pinMode(MIC, INPUT);
  pinMode(buzzerPin, OUTPUT);

  //pinMode(12, INPUT);

  sg90.attach(servoPin);  //Declare pin 8 for servo
  sg90.write(20); // Set initial position at 0 degrees

  lcd.init();   // LCD screen
  lcd.backlight();
  SPI.begin();      // Init SPI bus
  //mfrc522.PCD_Init();   // Init MFRC522

  // Arduino communicates with SIM900 GSM shield at a baud rate of 19200
  
  SIM900.begin(19200);

  // AT command to set SIM900 to SMS mode
  SIM900.print("AT+CMGF=1\r");
  delay(100);
  // Set module to send SMS data to serial out upon receipt
  SIM900.print("AT+CNMI=2,2,0,0,0\r");
  delay(100);

  lcd.clear(); // Clear LCD screen
  digitalWrite(buzzerPin,HIGH);
}

void loop() {
  if (NormalMode == false) {
    // Function to receive message
    receive_message();
  }

  else if (NormalMode == true) {
    // System will first look for mode
    if (VoiceMode == true) {
      // Function to receive message
      receive_message();

      lcd.setCursor(0, 0);
      lcd.print("Door Lock");
      lcd.setCursor(0, 1);
      lcd.print("Voice Recognition");

       if (!soundDetected()) {
        captureWord();
        
        Serial.println(classifier.predictLabel(features));
        if (classifier.predictLabel(features) == "Allowed"){
          
          lcd.clear();
          lcd.print("Voice Matched");

          delay(3000);

          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Enter Password:");
          lcd.setCursor(0, 1);
          VoiceMode = false; // Make RFID mode false
          }

        else{

          // If Voice is not matched.
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Unauthorized");
          lcd.setCursor(0, 1);
          lcd.print("Access Denied");
          digitalWrite(buzzerPin, LOW);

          send_message("Voice Authentication failed \nType 'close' to halt the system.");
          delay(3000);
          digitalWrite(buzzerPin, HIGH);

          lcd.clear();
          
          
          
          }
      }
      
     
        //delay(1000);




 
    }

    // If Voice mode is false, it will look for keys from keypad
    if (VoiceMode == false) {
      //Serial.begin(9600);
      //lcd.setCursor(0, 0);
      //lcd.print("Enter Password:");
      key_pressed = keypad_key.getKey();
      //lcd.setCursor(0, 1);// Storing keys
      if (key_pressed)
      {
        password[i++] = key_pressed;
        // Storing in password variable
        lcd.print("*");
      }
      if (i == 4) // If 4 keys are completed
      {
        delay(200);
        if (!(strncmp(password, initial_password, 4))) // If password is matched
        { 
            opendoor();
//          lcd.clear();
//          lcd.print("Pass Accepted");
//          sg90.write(90); // Door Opened
//          digitalWrite(greenLed, HIGH);
//          send_message("Door Opened \nIf it was't you, type 'close' to halt the system.");
//          delay(3000);
//          digitalWrite(greenLed, LOW);
//          sg90.write(0); // Door Closed
//          lcd.clear();
//          i = 0;
//          VoiceMode = true; 
        }
        else    // If password is not matched
        {
          lcd.clear();
          lcd.print("Wrong Password");
          digitalWrite(buzzerPin, LOW);

          send_message("Someone Tried with the wrong Password \nType 'close' to halt the system.");
          delay(3000);
          digitalWrite(buzzerPin, HIGH);

          lcd.clear();
          i = 0;
          VoiceMode = true;  // Make RFID mode true
        }
      }
    }
  }
}

// Receiving the message
void receive_message()
{
  char incoming_char = 0; //Variable to save incoming SMS characters
  String incomingData;   // for storing incoming serial data
  
  if (SIM900.available() > 0)
  {
    incomingData = SIM900.readString(); // Get the incoming data.
    delay(10);
  }

  // if received command is to open the door
  if (incomingData.indexOf("open") >= 0)
  {
    sg90.write(90);
    NormalMode = true;
    send_message("Opened");
    delay(10000);
    sg90.write(0);
  }

  // if received command is to halt the system
  if (incomingData.indexOf("close") >= 0)
  {
    NormalMode = false;
    send_message("Closed");
  }
  incomingData = "";
}

// Function to send the message
void send_message(String message)
{
  SIM900.println("AT+CMGF=1");    //Set the GSM Module in Text Mode
  delay(100);
  SIM900.println("AT+CMGS=\"+94773111987\""); // Replace it with your mobile number
  delay(100);
  SIM900.println(message);   // The SMS text you want to send
  delay(100);
  SIM900.println((char)26);  // ASCII code of CTRL+Z
  delay(100);
  SIM900.println();
  delay(1000);
}

bool VoiceAuth(){
  if (!soundDetected()) {
        captureWord();
        
        Serial.println(classifier.predictLabel(features));
        if (classifier.predictLabel(features) == "Allowed"){
            return true;
        }
        else{
          return false;
          
        }
        
        


  
  
  }
}
bool soundDetected() {
    return abs(readMic() - backgroundSound) >= SOUND_THRESHOLD;
}

void captureWord() {
    for (uint16_t i = 0; i < NUM_SAMPLES; i++) {
        featuresForFFT[i] = readMic();
        delay(INTERVAL);
    }

    fft.Windowing(featuresForFFT, NUM_SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);

    for (int i = 0; i < NUM_SAMPLES; i++){
        features[i] = featuresForFFT[i];
    }



    
}
int16_t readMic() {
    // this translated the analog value to a proper interval
    return  (analogRead(MIC) - 512) >> 2;
}

void calibrate() {
    for (int i = 0; i < 200; i++)
        backgroundSound += readMic();

    backgroundSound /= 200;

    Serial.print("Background sound level is ");
    Serial.println(backgroundSound);
}





void printFeatures() {
    const uint16_t numFeatures = sizeof(features) / sizeof(float);
    
    for (int i = 0; i < numFeatures; i++) {
        Serial.print(features[i]);
        Serial.print(i == numFeatures - 1 ? 'n' : ',');
    }
}

void opendoor(){
          lcd.clear();
          lcd.print("Pass Accepted");
          sg90.write(90); // Door Opened

          send_message("Door Opened \nIf it was't you, type 'close' to halt the system.");
          delay(3000);

          sg90.write(0); // Door Closed
          lcd.clear();
          i = 0;
          VoiceMode = true; 
  
  
  
  
  
  }
