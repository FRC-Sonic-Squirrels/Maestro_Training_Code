package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.lib.team2930.TalonFXElevatorSim;
import frc.robot.Constants;

public class ElevatorIOSim implements ElevatorIO {

  private final TalonFXElevatorSim sim =
      new TalonFXElevatorSim(
          new ElevatorSim(
              DCMotor.getKrakenX60Foc(1),
              Constants.ElevatorConstants.GEAR_RATIO,
              Constants.ElevatorConstants.CARRIAGE_MASS,
              Constants.ElevatorConstants.PULLEY_DIAMETER / 2.0,
              0.0,
              Units.Inches.of(Constants.ElevatorConstants.MAX_HEIGHT).in(Units.Meter),
              true,
              0));

  private VoltageOut openLoopControl = new VoltageOut(0);
  private MotionMagicVoltage closedLoopControl = new MotionMagicVoltage(0);

  public ElevatorIOSim() {}

  @Override
  public void updateInputs(Inputs inputs) {
    sim.update(Constants.kDefaultPeriod);
    inputs.appliedVolts = sim.getVoltage();
    inputs.velocityInchesPerSecond = sim.getVelocityInchesPerSecond();
    inputs.heightInches = sim.getPositionInches();
  }

  @Override
  public void setVoltage(double volts) {
    sim.setControl(openLoopControl.withOutput(volts));
  }

  @Override
  public void setHeightInches(double heightInches) {
    sim.setControl(closedLoopControl.withPosition(heightInches));
  }

  @Override
  public void setSensorPosition(double heightInches) {
    sim.setSensorPosition(heightInches);
  }

  @Override
  public void setClosedLoopConstants(
      double kP, double kD, double kG, MotionMagicConfigs mmConfigs) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    Slot0Configs slot0Configs = new Slot0Configs();

    slot0Configs.kP = kP;
    slot0Configs.kD = kD;
    slot0Configs.kG = kG;

    config.Slot0 = slot0Configs;
    config.MotionMagic = mmConfigs;

    sim.setConfig(config);
  }
}
