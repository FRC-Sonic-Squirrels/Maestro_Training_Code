// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.mechanism;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;

public class HomeMechanism extends Command {
  private Elevator elevator;
  private Arm arm;
  private boolean homeElevator = false;
  private boolean homeArm = false;
  private boolean armReset = false;
  private boolean elevatorReset = false;
  private boolean beginHomingArm = false;
  private final double safeElevatorHeightForArmResetting = 5.0;
  private final Rotation2d safeArmAngle =
      Constants.ArmConstants.HOME_POSITION.plus(Rotation2d.fromDegrees(2.0));
  private final double safeElevatorHeight = 1.0;

  /** Creates a new HomeMechanism. */
  public HomeMechanism(Elevator elevator, Arm arm) {
    this.elevator = elevator;
    this.arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator, arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (armReset) {
      if (!elevatorReset) {
        if (homeElevator) {
          elevator.setVoltage(-0.1);
          if (elevator.getVoltage() <= -0.05 && elevator.getVelocity() >= -0.02) {
            elevator.resetSensorToHomePosition();
            elevatorReset = true;
          }
        } else {
          elevator.setHeight(safeElevatorHeight);
          if (elevator.isAtTarget(safeElevatorHeight)) {
            homeElevator = true;
          }
        }
      }
    } else {
      if (beginHomingArm) {
        if (homeArm) {
          arm.setVoltage(-0.1);
          if (arm.getVoltage() <= -0.05 && arm.getVelocity() >= -0.02) {
            arm.resetSensorToHomePosition();
            armReset = true;
          }
        } else {
          if (arm.isAtTargetAngle(safeArmAngle)) {
            homeArm = true;
          }
          arm.setAngle(safeArmAngle);
        }
      } else {
        elevator.setHeight(safeElevatorHeightForArmResetting);
        if (elevator.isAtTarget(safeElevatorHeightForArmResetting)) {
          beginHomingArm = true;
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return armReset && elevatorReset;
  }
}