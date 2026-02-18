package frc.robot.libs;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.controller.PIDController;

public class PIDLoggerAutoLogged implements LoggableInputs {
  public double P;
  public double I;
  public double D;

  public PIDController pid;

  public PIDLoggerAutoLogged(PIDController pid) {
    this.pid = pid;
    P = pid.getP();
    I = pid.getI();
    D = pid.getD();
  }

  public void toLog(LogTable table) {
    table.put("P", P);
    table.put("I", I);
    table.put("D", D);
  }

  public void fromLog(LogTable table) {
    P = table.get("P", P);
    I = table.get("I", I);
    D = table.get("D", D);

    pid.setPID(P, I, D);
  }
}
