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
