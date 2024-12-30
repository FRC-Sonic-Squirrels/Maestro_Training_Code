package frc.lib.team2930;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.lib.team2930.TalonFXSim.ControlMode;

public class TalonFXElevatorSim {

  private final ElevatorSim sim;
  private ControlMode controlMode;

  private double targetVoltage;
  private double targetHeight;

  private double output;

  private ProfiledPIDController controller;
  private double kG;

  public TalonFXElevatorSim(ElevatorSim coreSim) {
    sim = coreSim;
    controller = new ProfiledPIDController(0, 0, 0, new Constraints(0, 0));
  }

  public void update(double dtSeconds) {
    sim.update(dtSeconds);
    if (controlMode == ControlMode.VOLTAGE) {
      output = targetVoltage;
    } else {
      output =
          controller.calculate(
                  Units.Meters.of(sim.getPositionMeters()).in(Units.Inches), targetHeight)
              + kG;
    }
    sim.setInputVoltage(output);
  }

  public void setControl(VoltageOut request) {
    targetVoltage = request.Output;
    controlMode = ControlMode.VOLTAGE;
  }

  /** Note: this uses INCHES for position control */
  public void setControl(MotionMagicVoltage request) {
    targetHeight = request.Position;
    controlMode = ControlMode.POSITION;
  }

  public double getVoltage() {
    return output;
  }

  public double getVelocityInchesPerSecond() {
    return Units.MetersPerSecond.of(sim.getVelocityMetersPerSecond()).in(Units.InchesPerSecond);
  }

  public double getPositionInches() {
    return Units.Meters.of(sim.getPositionMeters()).in(Units.Inches);
  }

  public void setConfig(TalonFXConfiguration config) {
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs = config.Slot0;

    controller.setPID(slot0Configs.kP, slot0Configs.kI, slot0Configs.kD);
    kG = slot0Configs.kG;
    controller.setConstraints(
        new Constraints(
            config.MotionMagic.MotionMagicCruiseVelocity,
            config.MotionMagic.MotionMagicAcceleration));
  }

  public void setState(double positionMeters, double velocityMetersPerSecond) {
    sim.setState(positionMeters, velocityMetersPerSecond);
  }

  public void setSensorPosition(double positionInches) {
    sim.setState(positionInches, sim.getVelocityMetersPerSecond());
  }
}
