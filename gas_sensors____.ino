#include "esp32-mqtt.h"
#include <DHT.h>;
#include "ArduinoJson.h"
DHT dht(33,DHT22 ); //// Initialize DHT sensor for normal 16mhz Arduino
int chk;
float hum;  //Stores humidity value
float temp; //Stores temperature value
DynamicJsonDocument doc(1024) ;
DynamicJsonDocument ht(512);
char charBuf[150] ;

#define         MQ_PIN                       (25)     //define which analog input channel you are going to use h2
#define         RL_VALUE                     (10)    //define the load resistance on the board, in kilo ohms
#define         RO_CLEAN_AIR_FACTOR          (9.21)  //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
                                                     //which is derived from the chart in datasheet

/***********************Software Related Macros************************************/
#define         CALIBARAION_SAMPLE_TIMES     (20)    //define how many samples you are going to take in the calibration phase
#define         CALIBRATION_SAMPLE_INTERVAL  (200)   //define the time interal(in milisecond) between each samples in the
                                                     //cablibration phase
#define         READ_SAMPLE_INTERVAL         (50)    //define how many samples you are going to take in normal operation
#define         READ_SAMPLE_TIMES            (5)     //define the time interal(in milisecond) between each samples in
                                                     //normal operation

/**********************Application Related Macros**********************************/
#define         GAS_H2                      (1)

/*****************************Globals***********************************************/
float           H2Curve[3]  =  {2.3, 0.93,-1.44};    //two points are taken from the curve in datasheet.
                                                     //data format:{ x, y, slope}; point1: (lg200, lg8.5), point2: (lg10000, lg0.03)

float           Ro           =  10;                  //Ro is initialized to 10 kilo ohms


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
    dht.begin();

    
    Serial.print("Calibrating...\n");               
  Ro = MQCalibration(MQ_PIN);
  
  //Calibrating the sensor. Please make sure the sensor is in clean air
                                                     //when you perform the calibration                   
  Serial.print("Calibration is done...\n");
  Serial.print("Ro=");
  Serial.print(Ro);
  Serial.print("kohm");
  Serial.print("\n");
    

  

  setupCloudIoT();
}

unsigned long lastMillis = 0;
void loop() {
    delay(2000);
    //Read data and store it to variables hum and temp
   


  mqtt->loop();
  delay(10);  // <- fixes some issues with WiFi stability

  if (!mqttClient->connected()) {
    connect();
    
  }

  // TODO: replace with your code

Serial.print("H2:");
   Serial.println(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_H2) );
   
   //Serial.print( "ppm" );
   Serial.print("\n");
   delay(200);
   doc["H2"] = MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_H2);
   delay(1000);


   doc["humiditya"]=dht22().substring(0,5);
   doc["tempa"] = dht22().substring(6,11);
   doc["airpurity"] = Airquality();
   //doc["watertemp"] =  water_temp();//== NULL ?float(0):water_temp() ;
   //doc["ph"] = pH_value();//== NULL ?float(0):pH_value() ; 
     serializeJson(doc,charBuf);
    serializeJson(doc,Serial);
  // publish a message roughly every second.
  if (millis() - lastMillis > 60000) {
    lastMillis = millis();
    //publishTelemetry(mqttClient, "/sensors", getDefaultSensor());
    publishTelemetry(charBuf);
   // free(charBuf);
  }
}
String dht22()
{
 
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  //float f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) )
  {
    Serial.println(F("Failed to read from DHT sensor!"));
    return "None";
  }

  Serial.print(F("Humidity: "));
 Serial.println(h);
 Serial.print(F("Temperature: "));
 Serial.println(t);
 

 return String(String(h)+"-"+String(t)+"-");
}
String Airquality( )
{
  float ppm;
  ppm = analogRead(32);
  Serial.print(F("airquality: "));
  Serial.println(ppm);
  return String(String(ppm));

  
}

float MQResistanceCalculation(int raw_adc)
{
  return ( ((float)RL_VALUE*(1023-raw_adc)/raw_adc));
}

float MQCalibration(int mq_pin)
{
  int i;
  float val=0;

  for (i=0;i<CALIBARAION_SAMPLE_TIMES;i++) {            //take multiple samples
    val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val/CALIBARAION_SAMPLE_TIMES;                   //calculate the average value

  val = val/RO_CLEAN_AIR_FACTOR;                        //divided by RO_CLEAN_AIR_FACTOR yields the Ro
                                                        //according to the chart in the datasheet

  return val;
}

float MQRead(int mq_pin)
{
  int i;
  float rs=0;

  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }

  rs = rs/READ_SAMPLE_TIMES;

  return rs; 
}

 String MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
  if ( gas_id == GAS_H2) {
    Serial.println("H2");
    
     return String(MQGetPercentage(rs_ro_ratio,H2Curve));
  } 
   
     
  }
  return "NONE";
}

int  MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
  return (pow(10,( ((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}
