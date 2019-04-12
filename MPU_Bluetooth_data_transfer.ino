#include "I2Cdev.h"
#include <MPU6050.h>
#include <BluetoothSerial.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// Uncomment to enable debuginfo on serial output. 
//#define ENABLE_SERIAL_DEBUG_OUTPUT
MPU6050 mpu;
BluetoothSerial esp_bt; //Object for Bluetooth

uint8_t ff_dur=20;
uint8_t ff_thr=35;
int16_t ax, ay, az;

int dataInn1=0;
int dataInn2=0;
volatile bool outputState=false;
volatile int timeOfAcivation=0;
int timeToSend=5000; // 5 sec. 
int samplesNr=100;
int axAv = 0;
int ayAv = 0;
int azAv = 0;
String outputString =String("");
/* LED pin */
byte ledPin = 2;
/* pin that is attached to interrupt */
byte interruptPin = 5;

void setup() {
  Wire.begin(21,22,400000);
  #ifdef ENABLE_SERIAL_DEBUG_OUTPUT
  Serial.begin(115200); //Start Serial monitor in 115200
  Serial.println("Initializing Accelerometer (mpu6050)...");
  Serial.println("Testing device connections...");
  #endif
  /** Power on and prepare for general usage.
  * This will activate the device and take it out of sleep mode (which must be done
  * after start-up). This function also sets both the accelerometer and the gyroscope
  * to their most sensitive settings, namely +/- 2g and +/- 250 degrees/sec, and sets
  * the clock source to use the X Gyro for reference, which is slightly better than
  * the default internal clock source.
  */
  mpu.initialize();
  // verify connection
  do
  {
    delay(500);
    #ifdef ENABLE_SERIAL_DEBUG_OUTPUT
    Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    #endif
  }while(!mpu.testConnection());

  esp_bt.begin("The_Eir"); //Name of your Bluetooth Signal

 
  mpu.setIntFreefallEnabled(true);
  mpu.setFreefallDetectionDuration(ff_dur); // set interupt duration to 2ms.
  mpu.setFreefallDetectionThreshold(ff_thr); // set interupt theshold to 25mg.

  pinMode(ledPin, OUTPUT);
  // set the input of the mpu interrupt as an interruput for the "arduino".
  pinMode(interruptPin, INPUT_PULLUP);
  /* attach interrupt to the pin
  function blink will be invoked when interrupt occurs
  interrupt occurs whenever the pin change value */
  attachInterrupt(digitalPinToInterrupt(interruptPin), activate, RISING);
}

void loop() 
{
  if (esp_bt.available()) //Check if we receive anything from Bluetooth
  {
    dataInn1 = esp_bt.read(); //Read what we recevive
    dataInn2 = esp_bt.read(); //remove last character. 
    
    #ifdef ENABLE_SERIAL_DEBUG_OUTPUT
    Serial.print("Received:"); 
    Serial.print(dataInn1);
    Serial.print(",");
    Serial.println(dataInn2);
    #endif
    
    if(dataInn1 == 1)
    {
      // 1 (or more) is recived
      activate();
    }
    else if(dataInn1 == 2)
    {
      timeToSend=dataInn2*1000; // change time to send. in secs. 
    }
    else if(dataInn1 == 3)
    {
      samplesNr=dataInn2*10; //number of samples in multiples of 10. 
    }
    else
    {
      // 0 is recived
      deactivate();
    }
  }

  #ifdef ENABLE_SERIAL_DEBUG_OUTPUT
  if(outputState) 
  {
  Serial.print("#");
  Serial.print(dataInn1);
  Serial.print(',');
  Serial.println(outputState);
  Serial.print("#");
  Serial.print(ax);
  Serial.print(',');
  Serial.print(ay);
  Serial.print(',');
  Serial.println(az);
  }
  #endif
  
  if(outputState)
  {
    digitalWrite(ledPin, HIGH);  
    for(int i=0; i<samplesNr;i++)
    {
     mpu.getAcceleration(&ax, &ay, &az);
     axAv+=ax;
     ayAv+=ay;
     azAv+=az; 
    }
    outputString = String(String(axAv/samplesNr) + String(',') + 
    String(ayAv/samplesNr) + String(',') + String(azAv/samplesNr));
    esp_bt.println(outputString);
    //outputState=false;
    axAv = 0;
    ayAv = 0;
    azAv = 0;
    if(millis() - timeOfAcivation > timeToSend)
    {
      deactivate();
    }
  }
}

void activate()
{
  outputState=true;
  timeOfAcivation=millis();
}

void deactivate()
{
  outputState=false;
  digitalWrite(ledPin, LOW);
}
