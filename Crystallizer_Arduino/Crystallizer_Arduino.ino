#include <Time.h>
#include <TimeLib.h>

#include <OneWire.h>
#include <DallasTemperature.h>
#include <Servo.h>

// Data wire is plugged into pin 2 on the Arduino
#define ONE_WIRE_BUS 2
 
// Setup a oneWire instance to communicate with any OneWire devices 
// (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
 
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

Servo servo1;
time_t t;


const int max_steps = 100;

#ifndef _PID_CNTL_H_
#define _PID_CNTL_H_

class PidControl {
private:
  double   KProportional,
    KIntegral,
    KDifferential,
    IntegralLimit,
    ProportionalGain,
    IntegralGain,
    DifferentialGain,
    Setpoint,
    Gain;
  unsigned DifferentialCycle;
  unsigned DifferentialCycleCount;
  double   ErrorSum,
    LastError;
protected:
  PidControl(double kP, double kI, double kD,
    double lI, unsigned Dc)
  {
    KProportional = kP;
    KIntegral = kI;
    KDifferential = kD;
    IntegralLimit = lI;
    ProportionalGain = 0;
    IntegralGain = 0;
    DifferentialGain = 0;
    DifferentialCycle = Dc;
    DifferentialCycleCount = 0;
    ErrorSum = 8000;
    LastError = 0;
  }
public:
  double CalculateGain(double position);
  virtual void   SetSetpoint(double setpoint)
  {
    Setpoint = setpoint;
  };
  double GetProportionalGain() { return ProportionalGain; }
  double GetIntegralGain() { return IntegralGain; }
  double GetDifferentialGain() { return DifferentialGain; }
  void SetProportionalConstant(double kP)
  {
    KProportional = kP;
  };
  void SetIntegralConstant(double kI)
  {
    KIntegral = kI;
  }
  void SetDifferentialConstant(double kD)
  {
    KDifferential = kD;
  }
  void SetDifferentialCycle(unsigned Dc)
  {
    DifferentialCycle = Dc;
  }
  void SetIntegralLimit(unsigned lI)
  {
    IntegralLimit = lI;
  }
  double GetProportionalConstant() { return KProportional; }
  double GetIntegralConstant() { return KIntegral; }
  double GetDifferentialConstant() { return KDifferential; }
  double GetIntegralLimit() { return IntegralLimit; }
  double GetSetpoint() { return Setpoint; }
};

#endif
/* End of File */


double PidControl::CalculateGain(double position)
{
  double error = Setpoint - position;
  ErrorSum += error;
  ProportionalGain = KProportional * error;
  IntegralGain = KIntegral * ErrorSum;
  if (IntegralGain > IntegralLimit) 
  {
    IntegralGain = IntegralLimit;
    ErrorSum -= error;
  }
  else if (IntegralGain < 28.0)
  {
    IntegralGain = 28.0;
    ErrorSum -= error;
  }
  if (++DifferentialCycleCount >= DifferentialCycle) {
    DifferentialGain = KDifferential * (error - LastError);
    DifferentialCycleCount = 0;
  }
  LastError = error;
  Gain = ProportionalGain + IntegralGain + DifferentialGain;
  return Gain;
}
//End of File


class Crystallizer : public PidControl
{
public:
  Crystallizer(double kP, double kI, double kD,
    double lI, unsigned Dc, int steps, double *m_temp, double *m_time, double h_limit, double s_limit, double t_delay, double temp_buffer) : PidControl(kP, kI, kD, lI, Dc)
  {
    n_steps = steps;
    HeaterLimit = h_limit;
    ServoLimit = s_limit;
    TimeDelay = t_delay;
    temp_offset = temp_buffer;
    for (int i = 0; i < n_steps; i++)
    {
      n_temp[i] = m_temp[i];
      n_time[i] = m_time[i];
    }
  }
  void output_header(int stepcounter,bool ramp)
  {
    if (ramp)
    {
      Serial.print("Step ");
      Serial.print(stepcounter + 1);
      Serial.print(": System Ramp To ");
      Serial.print(n_temp[stepcounter]);
    }
    else
    {
      Serial.print("Step ");
      Serial.print(stepcounter + 1);
      Serial.print(": System Hold At ");
      Serial.print(n_temp[stepcounter]);
    }
    Serial.print(" Start Time/Current Time: ");
    Serial.print(StartTime);
    Serial.print(" End Time: ");
    Serial.print(EndTime);
    Serial.print(" Current Temperature: ");
    Serial.print(SysTemp);
    Serial.print('\n');
  }
  void output_display(bool ramp)
  {
    Serial.print(" Setpoint: ");
    Serial.print(this->GetSetpoint());
    Serial.print(" Time: ");
    Serial.print(t);
    Serial.print(" Temp: ");
    Serial.print(SysTemp);
    Serial.print(" Gain: ");
    Serial.print(gain);
    Serial.print(" PGain: ");
    Serial.print(this->GetProportionalGain());
    Serial.print(" IGain: ");
    Serial.print(this->GetIntegralGain());
    Serial.print(" DGain: ");
    Serial.print(this->GetDifferentialGain());
    Serial.print(" Pre-Position: ");
    Serial.print(preAdj_servopos);
    Serial.print(" Adjusted-Position: ");
    Serial.print(servo_pos);
    if (ramp)
    {
      Serial.print(" System Ramping...   ");
    }
    else
    {
      Serial.print(" System Hold...   Endtime: ");
      Serial.print(EndTime);
    }
    Serial.print('\n');
  }
  void adjustservo()
  {
    sensors.requestTemperatures();
    SysTemp = sensors.getTempCByIndex(0);
    gain = this->CalculateGain(SysTemp);
    servo_pos = map(gain, 5, HeaterLimit, 0, ServoLimit);
    preAdj_servopos = servo_pos;
    if (servo_pos > ServoLimit)
    {
      servo_pos = ServoLimit;
    }
    else if (servo_pos < 0)
    {
      servo_pos = 0;
    }
    servo1.write(servo_pos);
  }
  
  void RunCycle_v3()
  {
    while(Serial.available() == 0){}
    Serial.println("Start Cycle!");
    for (int stepcounter = 0; stepcounter < n_steps; stepcounter++)
    {
      if (stepcounter == 0)
      {
        sensors.requestTemperatures();
        StartSysTemp = sensors.getTempCByIndex(0);
        this->SetSetpoint(0);
        adjustservo();
        t = now();
        StartTime = t;
        EndTime = t + n_time[stepcounter];
        output_header(stepcounter,true);
        while (this->GetSetpoint() != n_temp[stepcounter]) 
        {
          t = now();
          NewSetpoint = map(t,StartTime,EndTime,0,n_temp[stepcounter]);
          if (NewSetpoint > n_temp[stepcounter])
          {
            NewSetpoint = n_temp[stepcounter];
          }
          this->SetSetpoint(NewSetpoint);
          adjustservo();
          output_display(true);
          delay(TimeDelay);
        }
        sensors.requestTemperatures();
        SysTemp = sensors.getTempCByIndex(0);
        while (SysTemp <= this->GetSetpoint() - temp_offset || SysTemp >= this->GetSetpoint() + temp_offset)
        {
          t = now();
          adjustservo();
          output_display(true);
          delay(TimeDelay);
        }
      }
      else
      {
        if (n_temp[stepcounter] == n_temp[stepcounter - 1])
        {
          t = now();
          StartTime = t;
          EndTime = t + n_time[stepcounter];
          output_header(stepcounter,false);
          while (t <= EndTime)
          {
            t = now();
            adjustservo();
            output_display(false);
            delay(TimeDelay);
          }
        }
        else
        {
          sensors.requestTemperatures();
          StartSysTemp = sensors.getTempCByIndex(0);
          t = now();
          StartSetpoint = this->GetSetpoint();
          StartTime = t;
          EndTime = t + n_time[stepcounter];
          output_header(stepcounter,true);
          while (this->GetSetpoint() != n_temp[stepcounter])
          {
            t = now();
            NewSetpoint = map(t,StartTime,EndTime,StartSetpoint,n_temp[stepcounter]);
            if (StartSetpoint < n_temp[stepcounter] && NewSetpoint > n_temp[stepcounter])
            {
              NewSetpoint = n_temp[stepcounter];
            }
            else if (StartSetpoint > n_temp[stepcounter] && NewSetpoint < n_temp[stepcounter])
            {
              NewSetpoint = n_temp[stepcounter];
            }
            this->SetSetpoint(NewSetpoint);
            adjustservo();
            output_display(true);
            delay(TimeDelay);
          }
          sensors.requestTemperatures();
          SysTemp = sensors.getTempCByIndex(0);
          while (SysTemp <= this->GetSetpoint() - temp_offset || SysTemp >= this->GetSetpoint() + temp_offset)
          {
            t = now();
            adjustservo();
            output_display(true);
            delay(TimeDelay);
          }
        }
      }
    }
  }
  void Calibrate()
  {
    sensors.requestTemperatures();
    SysTemp = sensors.getTempCByIndex(0);
    Serial.println("Calibrating! Enter a Servo Position: ");
    while(Serial.available() == 0){}
    servo_pos = Serial.parseFloat();
    servo1.write(servo_pos);
    Serial.print("Servo Position is: ");
    Serial.print(servo_pos);
    Serial.print('\n');
    Serial.print("System Temperature is: ");
    Serial.print(SysTemp);
    Serial.print('\n');
  }
private:
  int n_steps;
  double n_temp[max_steps];
  double n_time[max_steps];
  double gain;
  double servo_pos;
  double EndTime;
  double StartTime;
  double NewSetpoint;
  double StartSetpoint;
  double SysTemp;
  double StartSysTemp;
  double preAdj_servopos;
  double HeaterLimit;
  double ServoLimit;
  double TimeDelay;
  double temp_offset;
};

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
  sensors.begin();
  servo1.attach(9);
  t = now();
  servo1.write(0);
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
