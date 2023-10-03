void setup()
{
  Serial.begin(9600);
}
String rxed="";
String rtc_string="";
String vrms_string="";
String irms_string="";
String power_string="";
String powerfactor_string="";
String energy_string="";

  float vol[400]={0.0};
   float cur[400]={0.0};
    float power=0.0;
     float powerfactor=0.0;
     float irms=0.0;
     float vrms=0.0;
     long int rtc =0;
     float energy=0.0;





void loop()
{


 Serial.write(Serial.read());*/


rxed = Serial.readStringUntil('\n');
vrms_string = rxed.substring(0,6);
irms_string= rxed.substring(6,12); 
powerfactor_string= rxed.substring(12,18);
power_string = rxed.substring(18,24); 
energy_string = rxed.substring(24,30);



delay(10);



vrms = vrms_string.toFloat();
irms = irms_string.toFloat();
power = power_string.toFloat();
powerfactor = powerfactor_string.toFloat();
energy = energy_string.toFloat();



// Serial.print("Vrms: ");
Serial.print(vrms);
Serial.print(", ");

//Serial.print("Irms: ");
Serial.print(irms);
Serial.print(", ");

//Serial.print("Power factor: ");
Serial.print(powerfactor);
Serial.print(", ");

//Serial.print("Power: ");
Serial.print(power);
Serial.print(", ");

//Serial.print("Energy: ");
Serial.println(energy);

Serial.flush();
delay(2000);



  
}