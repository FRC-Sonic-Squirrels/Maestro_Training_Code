// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.RobotMode.RobotType;

public class Intake extends SubsystemBase {

  private final IntakeIO io;
  private final IntakeIO.Inputs inputs = new IntakeIO.Inputs();

  private static final LoggerGroup loggerGroup = LoggerGroup.build(Constants.IntakeConstants.NAME);
  private static final LoggerEntry.Decimal loggerAppliedVolts =
      loggerGroup.buildDecimal("AppliedVolts");
  private static final LoggerEntry.Decimal loggerDeviceTemp =
      loggerGroup.buildDecimal("DeviceTemp");
  private static final LoggerEntry.Decimal loggerCurrentAmps =
      loggerGroup.buildDecimal("CurrentAmps");
  private static final LoggerEntry.Decimal loggerVelocityRPM =
      loggerGroup.buildDecimal("VelocityRPM");
  private static final LoggerEntry.Decimal loggerTargetRPM = loggerGroup.buildDecimal("TargetRPM");

  private static final TunableNumberGroup tunableGroup =
      new TunableNumberGroup(Constants.IntakeConstants.NAME);

  private static final LoggedTunableNumber kP = tunableGroup.build("motorConfig/kP");
  private static final LoggedTunableNumber kV = tunableGroup.build("motorConfig/kV");
  private static final LoggedTunableNumber mmAcceleration =
      tunableGroup.build("motorConfig/mmAcceleration", 300);

  static {
    if (Constants.RobotMode.getRobot() == RobotType.ROBOT_2024_MAESTRO) {
      kP.initDefault(0.8);
      kV.initDefault(0.15);
      mmAcceleration.initDefault(300);
    } else {
      kP.initDefault(0.08);
      kV.initDefault(0.015);
      mmAcceleration.initDefault(0);
    }
  }

  /** Creates a new EndEffector. */
  public Intake(IntakeIO io) {
    this.io = io;
    setConstants();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);

    // Motor Logging
    loggerAppliedVolts.info(inputs.appliedVolts);
    loggerDeviceTemp.info(inputs.deviceTemp);
    loggerCurrentAmps.info(inputs.currentAmps);
    loggerVelocityRPM.info(inputs.velocityRPM);

    // Update motor constants
    int hc = hashCode();
    if (kP.hasChanged(hc) || kV.hasChanged(hc) || mmAcceleration.hasChanged(hc)) setConstants();
  }

  // Motor methods

  public void setPercentOut(double percent) {
    io.setVoltage(percent * Constants.MAX_VOLTAGE);
  }

  public void setVelocity(double velocityRPM) {
    io.setVelocity(velocityRPM);
    loggerTargetRPM.info(velocityRPM);
  }

  public double getRPM() {
    return inputs.velocityRPM;
  }

  private void setConstants() {
    io.setClosedLoopConstants(kP.get(), kV.get(), mmAcceleration.get());
  }
}
