#include <Wire.h> //MQ7 Sensor
#include "DFRobot_OxygenSensor.h" //Oxygen Sensor
#include <LiquidCrystal_I2C.h> //LCD I2C
#define Oxygen_IICAddress ADDRESS_3
#define COLLECT_NUMBER  10 // collect number, the collection range is 1-100.
#define RL 10.0            // Nilai resistansi beban (RL) dalam kilo-ohm
#define AIR_CLEAN_RATIO 27.0 // Ratio Rs/Ro di udara bersih (dari datasheet MQ-7)/************************Hardware Related Macros************************************/
#define         MG_PIN                       (34)    //define which analog input channel you are going to use (GPIO 34 for ESP32)
#define         BOOL_PIN                     (2)     // Digital pin for BOOL
#define         DC_GAIN                      (8.5)   //define the DC gain of amplifier

/***********************Software Related Macros************************************/
#define         READ_SAMPLE_INTERVAL         (50)    //define how many samples you are going to take in normal operation
#define         READ_SAMPLE_TIMES            (5)     //define the time interval(in millisecond) between each samples in
                                                     //normal operation

/**********************Application Related Macros**********************************/
//These two values differ from sensor to sensor. user should derermine this value.
#define         ZERO_POINT_VOLTAGE           (0.220) //define the output of the sensor in volts when the concentration of CO2 is 400PPM
#define         REACTION_VOLTGAE             (0.030) //define the voltage drop of the sensor when move the sensor from air into 1000ppm CO2

/*****************************Globals***********************************************/
float           CO2Curve[3]  =  {2.602,ZERO_POINT_VOLTAGE,(REACTION_VOLTGAE/(2.602-3))};
float Ro = 10.0;           // Nilai awal Ro (akan dikalibrasi)
DFRobot_OxygenSensor oxygen;

// BH1750 lightMeter;

int analogPin   = 33;   // Dust Sensor, analog Pin
int digitalPin  = 5;    // Dust Sensor, digital Pin

const int MQ7_PIN = 4;

LiquidCrystal_I2C lcd(0x27, 20, 4); // Ganti 0x27 dengan alamat I2C yang sesuai jika diperlukan

void Sensor_02(){
  float oxygenData = oxygen.getOxygenData(COLLECT_NUMBER);

  Serial.print("Nilai O2  : ");
  Serial.print(oxygenData);
  Serial.println(" % PPM");
  Serial.println();
  lcd.setCursor(0, 0); // Set cursor ke kolom 0, baris 0
  lcd.print("Nilai O2  : "); // Tampilkan pesan di baris pertama
  lcd.print(oxygenData);
  lcd.print(" %");
  delay(500);
}

float MGRead(int mg_pin)
{
    int i;
    float v = 0;

    for (i = 0; i < READ_SAMPLE_TIMES; i++) {
        v += analogRead(mg_pin);
        delay(READ_SAMPLE_INTERVAL);
    }
    v = (v / READ_SAMPLE_TIMES) * 3.3 / 4095; // ESP32 analogRead returns values from 0 to 4095 for 0-3.3V
    return v;
}

int  MGGetPercentage(float volts, float *pcurve)
{
    if ((volts / DC_GAIN) >= ZERO_POINT_VOLTAGE) {
        return -1;
    } else {
        return pow(10, ((volts / DC_GAIN) - pcurve[1]) / pcurve[2] + pcurve[0]);
    }
}

void Sensor_CO2(){ //Gas CO2 (Karbon Dioksida)
 int percentage;
    float volts;

    volts = MGRead(MG_PIN);
    percentage = MGGetPercentage(volts, CO2Curve);
    Serial.print("Nilai CO2 : ");
    if (percentage == -1) {
        Serial.print("<400");
    } else {
        Serial.print(percentage);
    }

    Serial.println(" PPM");
    Serial.println();

  lcd.setCursor(0, 1); // Set cursor ke kolom 0, baris 0
  lcd.print("Nilai CO2 : "); // Tampilkan pesan di baris pertama
  if (percentage == -1) {
        lcd.print("<400");
    } else {
        lcd.print(percentage);
    }
  delay(500);  
}

// Fungsi untuk kalibrasi sensor
float calibrateSensor() {
  float val = 0.0;
  Serial.println("Tempatkan sensor di udara bersih untuk kalibrasi.");
  for (int i = 0; i < 50; i++) {
    val += calculateRs(analogRead(MQ7_PIN));
    delay(100); // Tunggu antara pengukuran
  }
  val = val / 50.0; // Nilai rata-rata Rs
  return val / AIR_CLEAN_RATIO; // Menghitung Ro berdasarkan Rs di udara bersih
}

// Fungsi untuk menghitung Rs
float calculateRs(int raw_adc) {
  float voltage = (raw_adc / 4095.0) * 5.0; // Konversi ADC ke tegangan (dalam volt)
  float Rs = ((5.0 - voltage) / voltage) * RL; // Menghitung Rs berdasarkan tegangan
  return Rs;
}

void Sensor_PM(){
  int outBits; //AD-converter raw output
  int calcVoltage;
  int dustDensity; //dust density, based on the formula (see later

  digitalWrite(digitalPin,LOW); //turn ON the LED
 
  delayMicroseconds(280); // wait 0.28 ms = 280 us
 
  outBits = analogRead(analogPin); //measure the peak of the output pulse  

  delayMicroseconds(40);
 
  digitalWrite(digitalPin,HIGH); //turn OFF the LED

  delayMicroseconds(9680);

  // If you want to get the converted data on the Arduino terminal,
  //uncomment this part and replace the outbits to dustDensity in printFormattedData()
  calcVoltage = outBits * (5.05 / 4095.0);
  dustDensity = (0.17 * calcVoltage - 0.1) * 1000.0;
  
  Serial.print("Nilai PM  : ");
  Serial.print(dustDensity);
  Serial.println(" ug/m3");
  Serial.println();
  lcd.setCursor(0, 2); // Set cursor ke kolom 0, baris 0
  lcd.print("Nilai PM  : "); // Tampilkan pesan di baris pertama
  lcd.print(dustDensity);
  delay(500);
}

void Sensor_CO(){ //Gas CO (Karbon Monoksida)
  float Rs = calculateRs(analogRead(MQ7_PIN)); // Menghitung Rs dari nilai analog sensor
  int analogValue = Rs / Ro; // Menghitung rasio Rs/Ro

  Serial.print("Nilai CO  : ");
  Serial.print(analogValue);
  Serial.println(" PPM");
  Serial.println();  
  lcd.setCursor(0, 3); // Set cursor ke kolom 0, baris 0
  lcd.print("Nilai CO  : "); // Tampilkan pesan di baris pertama
  lcd.print(analogValue);
  delay(500);
}

// void Sensor_Lux(){ //Intensitas Cahaya
//   float lux = lightMeter.readLightLevel();

//   Serial.print("Nilai Lux: ");
//   Serial.print(lux);
//   Serial.println(" Lux");
//   Serial.println();
//   delay(1000);
//   lcd.setCursor(0, 4); // Set cursor ke kolom 0, baris 0
//   lcd.print("Nilai Lux: "); // Tampilkan pesan di baris pertama
//   lcd.print(lux);
//   lcd.print(" Lux");
// }

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while(!oxygen.begin(Oxygen_IICAddress)){
    Serial.println("I2c device number error !");
    delay(1000);
  }
  Serial.println("I2c connect success !");
  Wire.begin();
  // lightMeter.begin();
  
  Ro = calibrateSensor();
  Serial.print("Kalibrasi selesai. Ro = ");
  Serial.println(Ro, 2); // Menampilkan nilai Ro dengan 2 desimal
  pinMode(BOOL_PIN, INPUT);                        //set pin to input
  digitalWrite(BOOL_PIN, HIGH);                    //turn on pullup resistors
  Serial.print("MG-811 Demonstration\n");
  pinMode(digitalPin,OUTPUT); //the pin for the dust sensor's LED is set as an output
  pinMode(analogPin,INPUT); //the pin for the dust sensor's LED is set as an output
  pinMode(MQ7_PIN, INPUT);
  // Memulai komunikasi dengan LCD
  lcd.init();                      // initialize the lcd 
  lcd.backlight();
}

void loop() {
  // put your main code here, to run repeatedly:
  Sensor_02();
  delay(500);
  Sensor_CO2();
  delay(500);
  Sensor_PM();
  delay(500);
  Sensor_CO();
  delay(500);
  // Sensor_Lux();
  // delay(500);
  Serial.println("--------------------------------------------------------------");
}
