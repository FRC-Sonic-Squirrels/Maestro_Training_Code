// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

public class IntakeIOReal implements IntakeIO {

  private final TalonFX motor = new TalonFX(Constants.CanIDs.INTAKE_CAN_ID);

  private final VoltageOut openLoopControl = new VoltageOut(0).withEnableFOC(true);

  private final MotionMagicVelocityVoltage closedLoopControl =
      new MotionMagicVelocityVoltage(0).withEnableFOC(true);

  private final StatusSignal<Double> deviceTemp;
  private final StatusSignal<Double> appliedVolts;
  private final StatusSignal<Double> currentAmps;
  private final StatusSignal<Double> velocityRPS;

  /** Creates a new EndEffector. */
  public IntakeIOReal() {
    // Motor Config
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit = 30.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.Feedback.SensorToMechanismRatio = Constants.IntakeConstants.GEAR_RATIO;

    motor.getConfigurator().apply(config);

    deviceTemp = motor.getDeviceTemp();
    appliedVolts = motor.getMotorVoltage();
    currentAmps = motor.getStatorCurrent();
    velocityRPS = motor.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(100, appliedVolts);
    BaseStatusSignal.setUpdateFrequencyForAll(50, currentAmps, velocityRPS);
    BaseStatusSignal.setUpdateFrequencyForAll(1, deviceTemp);

    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(Inputs inputs) {
    // Motor
    BaseStatusSignal.refreshAll(appliedVolts, currentAmps, velocityRPS, deviceTemp);

    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
    inputs.velocityRPM = velocityRPS.getValueAsDouble() * 60.0;
    inputs.deviceTemp = deviceTemp.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(openLoopControl.withOutput(volts));
  }

  @Override
  public void setVelocity(double velocityRPM) {
    motor.setControl(closedLoopControl.withVelocity(velocityRPM / 60.0));
  }

  @Override
  public void setClosedLoopConstants(double kP, double kV, double mmAcceleration) {
    TalonFXConfigurator configurator = motor.getConfigurator();
    Slot0Configs pidConfig = new Slot0Configs();
    MotionMagicConfigs mmConfig = new MotionMagicConfigs();

    configurator.refresh(pidConfig);
    configurator.refresh(mmConfig);

    pidConfig.kP = kP;
    pidConfig.kV = kV;

    mmConfig.MotionMagicAcceleration = mmAcceleration;

    configurator.apply(pidConfig);
    configurator.apply(mmConfig);
  }
}
