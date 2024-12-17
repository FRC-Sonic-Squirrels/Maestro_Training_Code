// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endEffector;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import edu.wpi.first.units.Units;
import frc.robot.Constants;

public class EndEffectorIOReal implements EndEffectorIO {

  private final TalonFX motor = new TalonFX(Constants.CanIDs.END_EFFECTOR_CAN_ID);
  private final TimeOfFlight intakeSide_tof =
      new TimeOfFlight(Constants.CanIDs.END_EFFECTOR_INTAKE_SIDE_TOF_CAN_ID);

  private final VoltageOut openLoopControl = new VoltageOut(0).withEnableFOC(true);

  private final StatusSignal<Double> deviceTemp;
  private final StatusSignal<Double> appliedVolts;
  private final StatusSignal<Double> currentAmps;
  private final StatusSignal<Double> velocityRPS;

  /** Creates a new EndEffector. */
  public EndEffectorIOReal() {
    // Motor Config
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit = 30.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.Feedback.SensorToMechanismRatio = Constants.EndEffectorConstants.GEAR_RATIO;

    motor.getConfigurator().apply(config);

    deviceTemp = motor.getDeviceTemp();
    appliedVolts = motor.getMotorVoltage();
    currentAmps = motor.getStatorCurrent();
    velocityRPS = motor.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(100, appliedVolts);
    BaseStatusSignal.setUpdateFrequencyForAll(50, currentAmps, velocityRPS);
    BaseStatusSignal.setUpdateFrequencyForAll(1, deviceTemp);

    motor.optimizeBusUtilization();

    // Time of Flight Config
    intakeSide_tof.setRangeOfInterest(6, 6, 10, 10);

    intakeSide_tof.setRangingMode(RangingMode.Short, 25);
  }

  @Override
  public void updateInputs(Inputs inputs) {
    // Motor
    BaseStatusSignal.refreshAll(appliedVolts, currentAmps, velocityRPS, deviceTemp);

    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
    inputs.velocityRPM = velocityRPS.getValueAsDouble() * 60.0;
    inputs.deviceTemp = deviceTemp.getValueAsDouble();

    // Time of Flight
    inputs.intakeSideTOFDistanceInches =
        Units.Millimeter.of(intakeSide_tof.getRange()).in(Units.Inch);
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(openLoopControl.withOutput(volts));
  }
}
