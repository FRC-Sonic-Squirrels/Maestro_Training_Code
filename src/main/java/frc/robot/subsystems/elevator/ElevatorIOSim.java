package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.lib.team2930.ControlMode;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOSim implements ElevatorIO {
  private ElevatorSim sim =
      new ElevatorSim(
          DCMotor.getFalcon500Foc(1),
          Constants.ElevatorConstants.GEAR_RATIO,
          Constants.ElevatorConstants.CARRIAGE_MASS,
          Constants.ElevatorConstants.PULLEY_DIAMETER / 2,
          0.0,
          Constants.ElevatorConstants.MAX_HEIGHT.in(Units.Inches),
          false,
          0.1);

  private Measure<Distance> targetHeight = Units.Meters.zero();

  private final ProfiledPIDController feedback =
      new ProfiledPIDController(0.0, 0.0, 0.0, new Constraints(0.0, 0.0));

  private double kG = 0.0;

  private ControlMode controlMode = ControlMode.OPEN_LOOP;

  private double openLoopVolts = 0.0;

  private double appliedVolts = 0.0;

  public ElevatorIOSim() {}

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    sim.update(0.02);
    Logger.recordOutput("Elevator/actualTargetHeight", targetHeight);
    if (controlMode.equals(ControlMode.CLOSED_LOOP)) {
      appliedVolts = feedback.calculate(inputs.heightInches, targetHeight.in(Units.Inches)) + kG;
    } else {
      appliedVolts = openLoopVolts;
    }

    Logger.recordOutput("Elevator/error", feedback.getPositionError());

    sim.setInputVoltage(appliedVolts);

    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = sim.getCurrentDrawAmps();
    inputs.heightInches = Units.Meters.of(sim.getPositionMeters()).in(Units.Inches);
    inputs.velocityInchesPerSecond =
        Units.Meters.of(sim.getVelocityMetersPerSecond()).in(Units.Inches);
  }

  @Override
  public void setVoltage(double volts) {
    openLoopVolts = volts;
    controlMode = ControlMode.OPEN_LOOP;
  }

  @Override
  public void setHeight(Measure<Distance> height) {
    targetHeight = height;
    controlMode = ControlMode.CLOSED_LOOP;
  }

  @Override
  public void setSensorPosition(Measure<Distance> position) {
    sim.setState(position.in(Units.Meters), 0.0);
  }

  @Override
  public void setPIDConstraints(double kP, double kD, double kG, Constraints constraints) {
    feedback.setPID(kP, 0.0, kD);
    this.kG = kG;
    feedback.setConstraints(constraints);
  }
}
