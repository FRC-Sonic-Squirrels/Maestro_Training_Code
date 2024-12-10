// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endEffector;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EndEffectorIOReal implements EndEffectorIO {

  private final TalonFX motor = new TalonFX(Constants.CanIDs.END_EFFECTOR_CAN_ID);

  private final VoltageOut openLoopControl = new VoltageOut(0).withEnableFOC(true);

  private final StatusSignal<Double> deviceTemp;
  private final StatusSignal<Double> appliedVolts;
  private final StatusSignal<Double> currentAmps;
  private final StatusSignal<Double> velocityRPS;

  /** Creates a new EndEffector. */
  public EndEffectorIOReal() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 30.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.Feedback.SensorToMechanismRatio =
        Constants.MotorConstants.EndEffectorConstants.GEAR_RATIO;

    motor.getConfigurator().apply(config);

    deviceTemp = motor.getDeviceTemp();
    appliedVolts = motor.getMotorVoltage();
    currentAmps = motor.getStatorCurrent();
    velocityRPS = motor.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setVoltage(double volts) {
    motor.setControl(openLoopControl.withOutput(volts));
  }

  public Subsystem getRequirements() {
    return this;
  }
}
