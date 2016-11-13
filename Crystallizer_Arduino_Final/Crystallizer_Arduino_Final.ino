#include <Time.h>
#include <TimeLib.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Servo.h>
#include <PIDControl.h>
#include <Crystallizer.h>

//Determine Cycle Parameters
double k_Proportional = 60;
double k_Integral = 0.025;
double k_Derivative = 0;
double I_Limit = 200;
double Dif_Cycle = 1;

//REMEMBER TO CHANGE NUMBER OF STEPS

int t_steps = 10;
double s_temp[10] = {87,87,52,52,28,28,55,55,28,28};
double s_time[10] = {30,900,2700,7200,21600,7200,3600,7200,21600,259200};
double h_limit = 275;
double servo_lim = 100;
double t_delay = 10000;
double t_offset = 0.5;

void setup(void)
{

  Serial.begin(9600);
  Serial.println("Press and Enter any Key to Continue");
}

  //Crystallizer_Arduino Corresponds to COM 4
 //COM 4: LEFT System
 //Impeller: Marine
 //Servo: Blue

void loop(void)
{
  Crystallizer Hello_v3(k_Proportional, k_Integral, k_Derivative, I_Limit, Dif_Cycle, t_steps, s_temp, s_time, h_limit, servo_lim, t_delay, t_offset);
  //Hello_v3.Calibrate();
  Hello_v3.RunCycle_v3();
}
