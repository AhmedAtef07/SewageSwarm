//Libraries
#include <DHT.h>;
#include <SoftwareSerial.h>
#include <TinyGPS.h>
//Constants
#define PIN_GATE_IN 4
#define IRQ_GATE_IN  0
#define PIN_LED_OUT 13
#define PIN_ANALOG_IN A0
#define ProximityPIN 7
#define LED 6
#define gasPin A3
#define DHTPIN 2     // what pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302)
DHT dht(DHTPIN, DHTTYPE); //// Initialize DHT sensor for normal 16mhz Arduino


//Variables
int chk;
float hum;  //Stores humidity value
float temp; //Stores temperature value

int redLed = 12;
int greenLed = 11;
//int buzzer = 10;
int smokeA0 = A5;
// Your threshold value
int sensorThres = 200;


int prox ;

void soundISR()
{
  int pin_val;

  pin_val = digitalRead(PIN_GATE_IN);
  digitalWrite(PIN_LED_OUT, pin_val);   
}



SoftwareSerial mySerial(10, 11);
TinyGPS gps;

void gpsdump(TinyGPS &gps);
void printFloat(double f, int digits = 2);


void setup()
{
    Serial.begin(9600);
  dht.begin();


  pinMode(redLed, OUTPUT);
  pinMode(greenLed, OUTPUT);
//  pinMode(buzzer, OUTPUT);
  pinMode(smokeA0, INPUT);

  
  pinMode(ProximityPIN, INPUT);
  pinMode(LED, OUTPUT);


    //  Configure LED pin as output
  pinMode(PIN_LED_OUT, OUTPUT);

  // configure input to interrupt
  pinMode(PIN_GATE_IN, INPUT);
  attachInterrupt(IRQ_GATE_IN, soundISR, CHANGE);

  // Display status
//  Serial.println("Initialized");


  mySerial.begin(9600);
  delay(1000);
  //Serial.println("uBlox Neo 6M");
  //Serial.print("Testing TinyGPS library v. "); 
  //Serial.println(TinyGPS::library_version());
  //Serial.println();
  //Serial.print("Sizeof(gpsobject) = "); 
  //Serial.println(sizeof(TinyGPS));
  //Serial.println();
}

void loop()
{
  ////////////////////////////////////TEMRATURE AND HUMIDITY////////////////////////////////////

    //Read data and store it to variables hum and temp
    hum = dht.readHumidity();
    temp= dht.readTemperature();
    //Print temp and humidity values to serial monitor
   // Serial.println();
    Serial.print("Humidity:");
    Serial.print(hum);
    Serial.println();
    Serial.print("Temp:");
    Serial.print(temp);
    //Serial.println();
    //Serial.println(" Celsius");
    Serial.println();
    delay(200); //Delay 2 sec.

////////////////////////////////////////////SMOKE////////////////////////////////////////////

    int analogSensor = analogRead(smokeA0);

  Serial.print("Smoke:");
  Serial.print(analogSensor);
  Serial.println();
  // Checks if it has reached the threshold value
 /* if (analogSensor > sensorThres)
  {
    digitalWrite(redLed, HIGH);
    digitalWrite(greenLed, LOW);
    //tone(buzzer, 1000, 200);
  }
  else
  {
    digitalWrite(redLed, LOW);
    digitalWrite(greenLed, HIGH);
    //noTone(buzzer);
  }
  */
  delay(200);

//////////////////////////////////////////CO///////////////////////////////////////////
  Serial.print("Carbon Monoxide:");
  Serial.println(analogRead(gasPin));
  delay(200); // Print value every 1 sec.
//////////////////////////////////////////PROXIMITY///////////////////////////////////////////

    prox=digitalRead(ProximityPIN);
      Serial.print("Object Detection:");
      Serial.print(!prox);
      Serial.println();
      delay (200);

/////////////////////////////////////////SOUNDDETECTOR/////////////////////////////////////////
int value;

  // Check the envelope input
  value = analogRead(PIN_ANALOG_IN);

  // Convert envelope value into a message
  Serial.print("Sound:");
  /*if(value <= 10)
  {
    Serial.println("Quiet.");
  }
  else if( (value > 10) && ( value <= 300) )
  {
    Serial.println("Moderate.");
  }
  else if(value > 300)
  {
    Serial.println("Loud.");
  }*/
    Serial.print(value);
    Serial.println();
  // pause for 1 second
  delay(200);
/////////////////////////////////////////////GPS/////////////////////////////////////////////
 bool newdata = false;
  unsigned long start = millis();
  // Every 5 seconds we print an update
  while (millis() - start < 5000) 
  {
    if (mySerial.available()) 
    
    {
      char c = mySerial.read();
      //Serial.print(c);  // uncomment to see raw GPS data
      if (gps.encode(c)) 
      {
        newdata = true;
        break;  // uncomment to print new data immediately!
      }
    }
  }
  
  if (newdata) 
  {
   // Serial.println("Acquired Data");
   // Serial.println("-------------");
    gpsdump(gps);
   // Serial.println("-------------");
   // Serial.println();
  }
  
}

void gpsdump(TinyGPS &gps)
{
  long lat, lon;
  float flat, flon;
  unsigned long age, date, time, chars;
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned short sentences, failed;

  gps.get_position(&lat, &lon, &age);
  Serial.print("Latitude ");
  Serial.print(lat); 
  Serial.print("Longitude");
  Serial.print(lon); 
 // Serial.print(" Fix age: "); Serial.print(age); Serial.println("ms.");
  
  // On Arduino, GPS characters may be lost during lengthy Serial.print()
  // On Teensy, Serial prints to USB, which has large output buffering and
  //   runs very fast, so it's not necessary to worry about missing 4800
  //   baud GPS characters.

  gps.f_get_position(&flat, &flon, &age);
  //Serial.print("Lat/Long(float): "); printFloat(flat, 5); Serial.print(", "); printFloat(flon, 5);
    //Serial.print(" Fix age: "); Serial.print(age); Serial.println("ms.");

  gps.get_datetime(&date, &time, &age);
  //Serial.print("Date(ddmmyy): "); Serial.print(date); Serial.print(" Time(hhmmsscc): ");
   // Serial.print(time);
  //Serial.print(" Fix age: "); Serial.print(age); Serial.println("ms.");

  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  //Serial.print("Date: "); Serial.print(static_cast<int>(month)); Serial.print("/"); 
    //Serial.print(static_cast<int>(day)); Serial.print("/"); Serial.print(year);
  //Serial.print("  Time: "); Serial.print(static_cast<int>(hour+8));  Serial.print(":"); //Serial.print("UTC +08:00 Malaysia");
    //Serial.print(static_cast<int>(minute)); Serial.print(":"); Serial.print(static_cast<int>(second));
    //Serial.print("."); Serial.print(static_cast<int>(hundredths)); Serial.print(" UTC +08:00 Malaysia");
  //Serial.print("  Fix age: ");  Serial.print(age); Serial.println("ms.");

  //Serial.print("Alt(cm): "); Serial.print(gps.altitude()); Serial.print(" Course(10^-2 deg): ");
    //Serial.print(gps.course()); Serial.print(" Speed(10^-2 knots): "); Serial.println(gps.speed());
  //Serial.print("Alt(float): "); printFloat(gps.f_altitude()); Serial.print(" Course(float): ");
    //printFloat(gps.f_course()); Serial.println();
  //Serial.print("Speed(knots): "); printFloat(gps.f_speed_knots()); Serial.print(" (mph): ");
    //printFloat(gps.f_speed_mph());
  //Serial.print(" (mps): "); printFloat(gps.f_speed_mps()); Serial.print(" (kmph): ");
   // printFloat(gps.f_speed_kmph()); Serial.println();

  gps.stats(&chars, &sentences, &failed);
 // Serial.print("Stats: characters: "); Serial.print(chars); Serial.print(" sentences: ");
 //   Serial.print(sentences); Serial.print(" failed checksum: "); Serial.println(failed);
}

void printFloat(double number, int digits)
{
  // Handle negative numbers
  if (number < 0.0) 
  {
     Serial.print('-');
     number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;
  
  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  Serial.print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    Serial.print("."); 

  // Extract digits from the remainder one at a time
  while (digits-- > 0) 
  {
    remainder *= 10.0;
    int toPrint = int(remainder);
    Serial.print(toPrint);
    remainder -= toPrint;
  }

}

   
