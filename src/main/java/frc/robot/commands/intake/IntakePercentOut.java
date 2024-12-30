// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import java.util.function.DoubleSupplier;

public class IntakePercentOut extends Command {

  private Intake intake;
  private DoubleSupplier percent;

  /** Creates a new EndEffectorPercentOut. */
  public IntakePercentOut(Intake intake, DoubleSupplier percent) {
    this.intake = intake;
    this.percent = percent;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  public IntakePercentOut(Intake intake, double percent) {
    this(intake, () -> percent);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setPercentOut(percent.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setPercentOut(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
