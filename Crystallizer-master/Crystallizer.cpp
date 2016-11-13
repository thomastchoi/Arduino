#include <Arduino.h>
#include <Time.h>
#include <TimeLib.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Servo.h>
#include <PIDControl.h>
#include <Crystallizer.h>

void Crystallizer::output_header(int stepcounter,bool ramp)
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

void Crystallizer::output_display(bool ramp)
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

void Crystallizer::adjustservo()
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

void Crystallizer::RunCycle_v3()
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

void Crystallizer::Calibrate()
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
