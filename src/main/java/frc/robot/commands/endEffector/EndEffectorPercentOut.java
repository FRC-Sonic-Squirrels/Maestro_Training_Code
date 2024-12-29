// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.endEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.endEffector.EndEffector;
import java.util.function.DoubleSupplier;

public class EndEffectorPercentOut extends Command {

  private EndEffector endEffector;
  private DoubleSupplier percent;

  /** Creates a new EndEffectorPercentOut. */
  public EndEffectorPercentOut(EndEffector endEffector, DoubleSupplier percent) {
    this.endEffector = endEffector;
    this.percent = percent;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(endEffector);
  }

  public EndEffectorPercentOut(EndEffector endEffector, double percent) {
    this(endEffector, () -> percent);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    endEffector.setPercentOut(percent.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    endEffector.setPercentOut(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
