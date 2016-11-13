#ifndef _CRYSTALLIZER_
#define _CRYSTALLIZER_

#define ONE_WIRE_BUS 2
const int max_steps = 100;

class Crystallizer : public PidControl
{
public:
  Crystallizer(double kP, double kI, double kD,
    double lI, unsigned Dc, int steps, double *m_temp, double *m_time, double h_limit, double s_limit, double t_delay, double temp_buffer) : PidControl(kP, kI, kD, lI, Dc), oneWire(ONE_WIRE_BUS), sensors(&oneWire)
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
    servo1.attach(9);
    t = now();
    servo1.write(0);
    sensors.begin();
  }
  void output_header(int stepcounter,bool ramp);
  void output_display(bool ramp);
  void adjustservo();
  void RunCycle_v3();
  void Calibrate();

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
  Servo servo1;
  time_t t;
  OneWire oneWire;
  DallasTemperature sensors;
};

#endif
