#include "PIDControl.h"

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
