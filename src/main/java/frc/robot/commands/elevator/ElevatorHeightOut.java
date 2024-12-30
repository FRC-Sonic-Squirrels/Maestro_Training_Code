// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import java.util.function.DoubleSupplier;

public class ElevatorHeightOut extends Command {

  private Elevator elevator;
  private DoubleSupplier heightInches;

  /** Creates a new EndEffectorVelocityOut. */
  public ElevatorHeightOut(Elevator elevator, DoubleSupplier heightInches) {
    this.elevator = elevator;
    this.heightInches = heightInches;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  public ElevatorHeightOut(Elevator elevator, double heightInches) {
    this(elevator, () -> heightInches);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.setHeight(heightInches.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.setPercentOut(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
