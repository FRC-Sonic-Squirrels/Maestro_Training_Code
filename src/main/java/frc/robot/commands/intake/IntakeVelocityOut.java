// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import java.util.function.DoubleSupplier;

public class IntakeVelocityOut extends Command {

  private Intake intake;
  private DoubleSupplier velocityRPM;

  /** Creates a new EndEffectorVelocityOut. */
  public IntakeVelocityOut(Intake intake, DoubleSupplier velocityRPM) {
    this.intake = intake;
    this.velocityRPM = velocityRPM;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  public IntakeVelocityOut(Intake intake, double velocityRPM) {
    this(intake, () -> velocityRPM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setVelocity(velocityRPM.getAsDouble());
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
