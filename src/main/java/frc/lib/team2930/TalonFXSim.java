package frc.lib.team2930;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TalonFXSim {

  private DCMotorSim motor;
  private double gearing;

  private ControlMode control = ControlMode.VOLTAGE;

  private double volts;

  private double targetOutput;

  private double kP;
  private double kI;
  private double kD;
  private double kV;
  private double kS;
  private double kG;

  private double lastError;

  private double velSum;
  private double lastVel;

  private double posSum;
  private double lastPos;

  public TalonFXSim(DCMotor motorType, double gearing, double MOI) {
    motor = new DCMotorSim(motorType, gearing, MOI);
    this.gearing = gearing;
  }

  public void update(double dtSeconds) {
    motor.update(dtSeconds);
    double error;
    double deltaError;

    double adjustedVelocity = getVelocity();
    double adjustedPosition = getPosition();
    switch (control) {
      case VOLTAGE:
        volts = targetOutput;
        break;
      case VELOCITY:
        error = targetOutput - adjustedVelocity;
        deltaError = (error - lastError) / dtSeconds;
        velSum += (adjustedVelocity + lastVel) / 2.0 * dtSeconds;
        volts += kV * error + kS * velSum + kP * deltaError;
        lastError = error;
        break;
      case POSITION:
        error = targetOutput - (adjustedPosition * gearing);
        deltaError = (error - lastError) / dtSeconds;
        posSum += (adjustedPosition + lastPos) / 2.0 * dtSeconds;
        volts = kP * error + kI * posSum + kD * deltaError + kG;
        lastError = error;
        break;
      default:
        volts = 0;
        break;
    }
    motor.setInputVoltage(volts);
    lastVel = adjustedVelocity;
    lastPos = adjustedPosition;

    if (volts == 0.0)
      motor.setState(motor.getAngularPositionRad(), motor.getAngularVelocityRadPerSec() / 2.0);
  }

  public void setControl(VoltageOut request) {
    targetOutput = request.Output;
    control = ControlMode.VOLTAGE;
  }

  public void setControl(VelocityVoltage request) {
    targetOutput = request.Velocity;
    control = ControlMode.VELOCITY;
  }

  public void setControl(PositionDutyCycle request) {
    targetOutput = request.Position;
    control = ControlMode.POSITION;
  }

  public void setConfig(TalonFXConfiguration config) {
    kP = config.Slot0.kP;
    kI = config.Slot0.kI;
    kD = config.Slot0.kD;
    kV = config.Slot0.kV;
    kS = config.Slot0.kS;
    kG = config.Slot0.kG;
  }

  public double getVoltage() {
    return volts;
  }

  public double getVelocity() {
    return motor.getAngularVelocityRPM() * gearing;
  }

  public double getPosition() {
    return motor.getAngularPositionRad() * gearing;
  }

  public void setState(double angularPositionRad, double angularVelocityRadPerSec) {
    motor.setState(angularPositionRad, angularVelocityRadPerSec);
  }

  public enum ControlMode {
    VOLTAGE,
    VELOCITY,
    POSITION
  }
}
