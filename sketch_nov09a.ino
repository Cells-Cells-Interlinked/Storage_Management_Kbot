#include <Servo.h>

Servo myservo; //ประกาศตัวแปรแทน Servo

void setup()
{
Serial.begin(9600);
myservo.attach(9); // กำหนดขา 9 ควบคุม Servo
}
void loop()
{
    char input = Serial.read();  

    if (input == 'e')
    {
    myservo.writeMicroseconds(3000); 
    delay(50); // หน่วงเวลา 2000ms
    }
    if (input == 'q'){
    myservo.writeMicroseconds(1450); 
    delay(50); // หน่วงเวลา 2000ms
    }
    if (input == 'r'){
    myservo.writeMicroseconds(0); 
    delay(50); // หน่วงเวลา 2000ms
    }

}
