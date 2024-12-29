// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endEffector;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.team2930.TalonFXSim;
import frc.robot.Constants;

public class EndEffectorIOSim implements EndEffectorIO {

  private TalonFXSim motor =
      new TalonFXSim(
          DCMotor.getKrakenX60Foc(1),
          Constants.EndEffectorConstants.GEAR_RATIO,
          Constants.EndEffectorConstants.MOI);

  private VoltageOut closedLoopControl = new VoltageOut(0);
  private VelocityVoltage openLoopControl = new VelocityVoltage(0);

  /** Creates a new EndEffector. */
  public EndEffectorIOSim() {}

  @Override
  public void updateInputs(Inputs inputs) {
    motor.update(Constants.kDefaultPeriod);

    inputs.appliedVolts = motor.getVoltage();
    inputs.velocityRPM = motor.getVelocity();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(closedLoopControl.withOutput(volts));
  }

  @Override
  public void setVelocity(double velocityRPM) {
    motor.setControl(openLoopControl.withVelocity(velocityRPM));
  }

  @Override
  public void setClosedLoopConstants(double kP, double kV, double mmAcceleration) {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0.kP = kP;
    config.Slot0.kV = kV;

    motor.setConfig(config);
  }
}
