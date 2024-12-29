// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endEffector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
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
  private static final LoggerEntry.Decimal loggerIntakeSideTofDistance =
      loggerGroup.buildDecimal("IntakeSideTOFDistanceInches");
  private static final LoggerEntry.Decimal loggerShooterSideTofDistance =
      loggerGroup.buildDecimal("ShooterSideTOFDistanceInches");
  private static final LoggerEntry.Bool loggerIntakeSideTof =
      loggerGroup.buildBoolean("IntakeSideTOF");
  private static final LoggerEntry.Bool loggerShooterSideTof =
      loggerGroup.buildBoolean("ShooterSideTOF");

  private static final TunableNumberGroup tunableGroup =
      new TunableNumberGroup(Constants.EndEffectorConstants.NAME);
  private static final LoggedTunableNumber distanceToTriggerNoteDetection =
      tunableGroup.build("DistanceToTriggerNoteDetection", 11);

  private static final LoggedTunableNumber kP = tunableGroup.build("motorConfig/kP", 0.4);
  private static final LoggedTunableNumber kV = tunableGroup.build("motorConfig/kV", 0.15);
  private static final LoggedTunableNumber mmAcceleration =
      tunableGroup.build("motorConfig/mmAcceleration", 300);

  /** Creates a new EndEffector. */
  public EndEffector(EndEffectorIO io) {
    this.io = io;
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

    // Time of Flight Logging
    loggerIntakeSideTofDistance.info(inputs.intakeSideTOFDistanceInches);
    loggerShooterSideTofDistance.info(inputs.shooterSideTOFDistanceInches);

    loggerIntakeSideTof.info(intakeSideTimeOfFlight());
    loggerShooterSideTof.info(shooterSideTimeOfFlight());

    // Update motor constants
    int hc = hashCode();
    if (kP.hasChanged(hc) || kV.hasChanged(hc) || mmAcceleration.hasChanged(hc))
      io.setClosedLoopConstants(kP.get(), kV.get(), mmAcceleration.get());
  }

  // Motor methods

  public void setPercentOut(double percent) {
    io.setVoltage(percent * Constants.MAX_VOLTAGE);
  }

  public void setVelocity(double velocityRPM) {
    io.setVelocity(velocityRPM);
  }

  public double getRPM() {
    return inputs.velocityRPM;
  }

  // TOF methods

  public double getIntakeSideTOFDistanceInches() {
    return inputs.intakeSideTOFDistanceInches;
  }

  public double getShooterSideTOFDistanceInches() {
    return inputs.shooterSideTOFDistanceInches;
  }

  public boolean intakeSideTimeOfFlight() {
    return inputs.intakeSideTOFDistanceInches <= distanceToTriggerNoteDetection.get();
  }

  public boolean shooterSideTimeOfFlight() {
    return inputs.shooterSideTOFDistanceInches <= distanceToTriggerNoteDetection.get();
  }

  public boolean noteInEndEffector() {
    return intakeSideTimeOfFlight() && shooterSideTimeOfFlight();
  }
}
