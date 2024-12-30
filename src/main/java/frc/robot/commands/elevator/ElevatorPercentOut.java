// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import java.util.function.DoubleSupplier;

public class ElevatorPercentOut extends Command {

  private Elevator elevator;
  private DoubleSupplier percent;

  /** Creates a new EndEffectorPercentOut. */
  public ElevatorPercentOut(Elevator elevator, DoubleSupplier percent) {
    this.elevator = elevator;
    this.percent = percent;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  public ElevatorPercentOut(Elevator elevator, double percent) {
    this(elevator, () -> percent);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.setPercentOut(percent.getAsDouble());
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
