// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endEffector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.robot.Constants;

public class EndEffector extends SubsystemBase {

  private final EndEffectorIO io;
  private final EndEffectorIO.Inputs inputs = new EndEffectorIO.Inputs();

  private static final LoggerGroup loggerGroup =
      LoggerGroup.build(Constants.EndEffectorConstants.NAME);
  private static final LoggerEntry.Decimal loggerAppliedVolts =
      loggerGroup.buildDecimal("AppliedVolts");
  private static final LoggerEntry.Decimal loggerDeviceTemp =
      loggerGroup.buildDecimal("DeviceTemp");
  private static final LoggerEntry.Decimal loggerCurrentAmps =
      loggerGroup.buildDecimal("CurrentAmps");
  private static final LoggerEntry.Decimal loggerVelocityRPM =
      loggerGroup.buildDecimal("VelocityRPM");
  private static final LoggerEntry.Decimal loggerTofDistance =
      loggerGroup.buildDecimal("TOFDistanceInches");

  /** Creates a new EndEffector. */
  public EndEffector(EndEffectorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);

    loggerAppliedVolts.info(inputs.appliedVolts);
    loggerDeviceTemp.info(inputs.deviceTemp);
    loggerCurrentAmps.info(inputs.currentAmps);
    loggerVelocityRPM.info(inputs.velocityRPM);
  }

  public void setPercentOut(double percent) {
    io.setVoltage(percent * Constants.MAX_VOLTAGE);
  }

  public double getRPM(){
    return inputs.velocityRPM;
  }

  public double getTOFDistanceInches(){
    return inputs.tofDistanceInches;
  }
}
