const int tempSensorPin = 1;

//For unit testing:
//void setup() {
//  // put your setup code here, to run once:
//  Serial.begin(9600);
//}
//
//void loop() {
//  // put your main code here, to run repeatedly:
//  
//  float temp = getExtTemp();
//  Serial.print(temp,1);
//  delay(500);
//  
//  Serial.println();
//}

/**
* The main sensor function
* Returns the external temperature in degrees Celcius to a single decimal place
* Accuracy: +-3/4 degrees
**/
float getExtTemp()
{
  return (analogRead(tempSensorPin)*(5000.0/1024.0)-500.0)/10.0;
}
