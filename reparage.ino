

#include <Arduino.h>
#include <Wire.h>
#include "SoftwareSerial.h"

#include <MeAuriga.h>

//Encoder Motor
MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);

void isr_process_encoder1(void)
{
      if(digitalRead(Encoder_1.getPortB()) == 0){
            Encoder_1.pulsePosMinus();
      }else{
            Encoder_1.pulsePosPlus();
      }
}

void isr_process_encoder2(void)
{
      if(digitalRead(Encoder_2.getPortB()) == 0){
            Encoder_2.pulsePosMinus();
      }else{
            Encoder_2.pulsePosPlus();
      }
}

void move(int direction, int speed)
{
      int leftSpeed = 0;
      int rightSpeed = 0;
      if(direction == 1){
            leftSpeed = -speed;
            rightSpeed = speed;
      }else if(direction == 2){
            leftSpeed = speed;
            rightSpeed = -speed;
      }else if(direction == 3){
            leftSpeed = -speed;
            rightSpeed = -speed;
      }else if(direction == 4){
            leftSpeed = speed;
            rightSpeed = speed;
      }
      Encoder_1.setTarPWM(leftSpeed);
      Encoder_2.setTarPWM(rightSpeed);
}
void moveDegrees(int direction,long degrees, int speed_temp)
{
      speed_temp = abs(speed_temp);
      if(direction == 1)
      {
            Encoder_1.move(-degrees,(float)speed_temp);
            Encoder_2.move(degrees,(float)speed_temp);
      }
      else if(direction == 2)
      {
            Encoder_1.move(degrees,(float)speed_temp);
            Encoder_2.move(-degrees,(float)speed_temp);
      }
      else if(direction == 3)
      {
            Encoder_1.move(-degrees,(float)speed_temp);
            Encoder_2.move(-degrees,(float)speed_temp);
      }
      else if(direction == 4)
      {
            Encoder_1.move(degrees,(float)speed_temp);
            Encoder_2.move(degrees,(float)speed_temp);
      }
}
const float diametreRoue = 6.5f;
double angle_rad = PI/180.0;
double angle_deg = 180.0/PI;
void updatePositionRobot(double encIG, double encID, double encFG, double encFd);
double distance;
double coefG;
double coefD;
double angleDepl;
void avancer(double vitesse);
double encI1;
double encI2;
double encF1;
double encF2;
void tournerGauche(double vitesse);
void tournerDroite(double vitesse);
void affichageRobot(double xr, double yr, double angler);
double angleAffichage;
double xRobot;
double yRobot;
double orientationRobot;
char TempString[10];
int turns = 0;
MeLEDMatrix ledMtx_7(7);

void updatePositionRobot(double encIG, double encID, double encFG, double encFD)
{
    distance = ((((encFG) - (encIG)) * (coefG)) + (((encFD) - (encID)) * (coefD))) / (2);
    angleDepl = ((encFD-encID) - (encFG - encIG))*coefG/14.4;
    xRobot += (distance) * (cos(angle_rad*angleDepl));
    yRobot += (distance) * (sin(angle_rad*angleDepl));
    orientationRobot += angleDepl;
}

void avancer(double vitesse)
{
    encI1 = (-1) * (Encoder_1.getCurPos());
    encI2 = Encoder_2.getCurPos();
    move(1,vitesse);
    encF1 = (-1) * (Encoder_1.getCurPos());
    encF2 = Encoder_2.getCurPos();
    updatePositionRobot(encI1,encI2,encF1,encF2);
}

void tournerGauche(double vitesse)
{
    encI1 = (-1) * (Encoder_1.getCurPos());
    encI2 = Encoder_2.getCurPos();
    move(3,vitesse);
    encF1 = (-1) * (Encoder_1.getCurPos());
    encF2 = Encoder_2.getCurPos();
    updatePositionRobot(encI1,encI2,encF1,encF2);
}

void tournerDroite(double vitesse)
{
    encI1 = (-1) * (Encoder_1.getCurPos());
    encI2 = Encoder_2.getCurPos();
    move(4,vitesse);
    encF1 = (-1) * (Encoder_1.getCurPos());
    encF2 = Encoder_2.getCurPos();
    updatePositionRobot(encI1,encI2,encF1,encF2);
}

void affichageRobot(double xr, double yr, double angler)
{
    if((angler) < (180)){
        angleAffichage = (90) - (angler);
    }else{
        angleAffichage = (90) + (fmod((-1) * (angler),180));
    }
}

void setup(){
    attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
    attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
    //Set Pwm 8KHz
    TCCR1A = _BV(WGM10);
    TCCR1B = _BV(CS11) | _BV(WGM12);
    TCCR2A = _BV(WGM21) | _BV(WGM20);
    TCCR2B = _BV(CS21);
     ledMtx_7.setColorIndex(1);
    ledMtx_7.setBrightness(6);
    Encoder_1.reset(SLOT1);
    Encoder_2.reset(SLOT2);
    xRobot = 0;
    yRobot = 0;
    orientationRobot = 0;
    distance = 0;
    coefG = ((diametreRoue) * (3.1415)) / (360);
    coefD = ((diametreRoue) * (3.1415)) / (360);
    encI1 = 0;
    encI2 = 0;
    Serial.begin(9600);

}


double absolute(double value)
{
  if (value < 0)
  {
    value = -value;
  }

  return value;

}

void loop(){
//  turns++;
//  for(int i = 1;i<=10; i++ ){
//    //avancer(100);
//    move(1,200);
//    _delay(0.5);
//    _loop();
//    encF1 = (-1) * (Encoder_1.getCurPos());
//    encF2 = Encoder_2.getCurPos();
//    updatePositionRobot(encI2,encI1,encF2,encF1);
//    encI1 = encF1;
//    encI2 = encF2;
//  }

  if (turns == 0){
    while (absolute(xRobot - 50.0) > 0.1 || (abs(yRobot)>0.1)){
      //Serial.println("En deplacement");
      Serial.print("xRobot : ");
      Serial.print(xRobot);
      Serial.print(" xRobot - 10 = ");
      Serial.print(absolute(xRobot - 10.0));
      Serial.print(" ; condition : ");
      bool b = (abs(xRobot-50.0)>0.1);
      Serial.println(b); 
      double speed = xRobot - 50;
      int coef = 3;
      //move(1,50);
      if (speed < 0 )
      {
        move(1,-speed * coef + 20);
      }
      else
      {
        move(2,speed * coef + 20);
      }
      /*double speed = xRobot - 10;
      Encoder_1.setTarPWM(speed * 7);
      Encoder_2.setTarPWM(speed * 7);*/
      _loop();
      encF1 = (-1) * (Encoder_1.getCurPos());
      encF2 = Encoder_2.getCurPos();
      updatePositionRobot(encI2,encI1,encF2,encF1);
      encI1 = encF1;
      encI2 = encF2;
    }
    move(1,0);
    _loop();
    Serial.print(" : x = ");
    Serial.print(xRobot);
    Serial.print(" ; y = ");
    Serial.print(yRobot);
    Serial.print(" ; orientation = ");
    Serial.print(orientationRobot);
    Serial.print(" ; enc1 = ");
    Serial.print(-Encoder_1.getCurPos());
    Serial.print(" ; enc2 = ");
    Serial.println(Encoder_2.getCurPos());
    turns++;
  }
  else{
    Encoder_1.setMotorPwm(0);
    Encoder_2.setMotorPwm(0);
  }
}

void _delay(float seconds){
    long endTime = millis() + seconds * 1000;
    while(millis() < endTime)_loop();
}

void _loop(){
    Encoder_1.loop();
    Encoder_2.loop();
}