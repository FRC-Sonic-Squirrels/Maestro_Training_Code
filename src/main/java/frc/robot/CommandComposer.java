package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team2930.GeometryUtil;
import frc.robot.commands.AutoClimb;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.commands.endEffector.EndEffectorCenterNoteBetweenToFs;
import frc.robot.commands.endEffector.EndEffectorPercentOut;
import frc.robot.commands.mechanism.MechanismActions;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class CommandComposer {
  public static Command autoClimb(
      DrivetrainWrapper drivetrainWrapper,
      Elevator elevator,
      Arm arm,
      EndEffector endEffector,
      Shooter shooter,
      Intake intake) {
    /*
     * Step 1: drive to in front of the chain,
     * at the same time, if the robot is within 2 meters of the stage:
     * if the robot is under the stage
     * bring mech into under stage prep,
     * else
     * bring the mech into climb prep positon,
     * if we are farther than 2 meters,
     * stay in stow/loading position
     * Step 2: run climb command
     */

    DriveToPose driveToClimbPos =
        new DriveToPose(
            drivetrainWrapper,
            () -> AutoClimb.getTargetPose(drivetrainWrapper.getPoseEstimatorPose(true)));

    BooleanSupplier withinRangeOfStage =
        () ->
            GeometryUtil.getDist(
                    Constants.FieldConstants.getStageCenter(),
                    drivetrainWrapper.getPoseEstimatorPose(false).getTranslation())
                <= 2.0;

    BooleanSupplier underStage =
        () -> AutoClimb.underStage(drivetrainWrapper.getPoseEstimatorPose(false));

    Supplier<Command> prepForClimb =
        () ->
            new ConditionalCommand(
                MechanismActions.climbPrepUnderStagePosition(elevator, arm)
                    .until(() -> !underStage.getAsBoolean())
                    .andThen(
                        MechanismActions.climbPrepPosition(
                            elevator, arm, endEffector, shooter, intake)),
                MechanismActions.climbPrepPosition(elevator, arm, endEffector, shooter, intake),
                underStage);

    Command climbCommand =
        (driveToClimbPos.alongWith(
                new ConditionalCommand(
                    prepForClimb.get(),
                    MechanismActions.loadingPosition(elevator, arm)
                        .until(withinRangeOfStage)
                        .andThen(prepForClimb.get()),
                    withinRangeOfStage)))
            .until(driveToClimbPos::atGoal)
            .andThen(new AutoClimb(drivetrainWrapper, elevator, arm, endEffector));

    climbCommand.setName("AutoClimb");
    return climbCommand;
  }

  public static Command scoreAmp(
      EndEffector endEffector,
      DrivetrainWrapper drivetrainWrapper,
      Elevator elevator,
      Arm arm,
      Intake intake,
      Shooter shooter,
      boolean doDrive,
      Trigger confirmation) {
    /*
     * Step 1: drive to amp
     * at the same time, if we are within a distance move mech into position
     * Step 2: run end effector
     */
    Trigger noGamepieceInEE =
        new Trigger(
                () ->
                    !endEffector.intakeSideTOFDetectGamepiece()
                        && !endEffector.shooterSideTOFDetectGamepiece())
            .debounce(0.4);

    DriveToPose driveToAmp =
        new DriveToPose(drivetrainWrapper, Constants.FieldConstants::getAmpScoringPose);

    Measure<Distance> distToElevateMech = Units.Meters.of(3.5);

    BooleanSupplier withinRangeOfAmp =
        () ->
            GeometryUtil.getDist(
                    drivetrainWrapper.getPoseEstimatorPose(false),
                    Constants.FieldConstants.getAmpScoringPose())
                <= distToElevateMech.in(Units.Meters);

    Command scoreAmp =
        new ConditionalCommand(
                driveToAmp.until(
                    () ->
                        driveToAmp.withinTolerance(0.1, Rotation2d.fromDegrees(10))
                            || confirmation.getAsBoolean()),
                Commands.waitUntil(confirmation::getAsBoolean),
                () -> doDrive)
            .asProxy()
            .alongWith(MechanismActions.ampPrepPosition(elevator, arm))
            .deadlineWith(new EndEffectorCenterNoteBetweenToFs(endEffector, intake, shooter))
            .andThen(
                Commands.run(() -> endEffector.setVelocity(2500), endEffector)
                    .alongWith(MechanismActions.ampPosition(elevator, arm))
                    .until(noGamepieceInEE));
    // .andThen(cancelScoreAmp(drivetrainWrapper, endEffector, elevator, arm));

    scoreAmp.setName("ScoreAmp");

    return scoreAmp;
  }

  public static Command cancelScoreAmp(
      DrivetrainWrapper drivetrainWrapper, EndEffector endEffector, Elevator elevator, Arm arm) {
    Command cancelScoreAmp =
        MechanismActions.ampPositionToLoadPosition(elevator, arm)
            .until(
                () ->
                    GeometryUtil.getDist(
                            drivetrainWrapper.getPoseEstimatorPose(true),
                            Constants.FieldConstants.getAmpScoringPose())
                        >= 0.5)
            .andThen(MechanismActions.loadingPosition(elevator, arm))
            .alongWith(new EndEffectorPercentOut(endEffector, 0.0));

    cancelScoreAmp.setName("CancelScoreAmp");
    return cancelScoreAmp;
  }

  public static Command stageAlign(DrivetrainWrapper wrapper, Supplier<Double> driveMagnitudeX, Supplier<Double> driveMagnitudeY) {
    PIDController rotationalPID = new PIDController(5.0, 0, 0);
    Supplier<Pose2d> targetPose =
        () -> AutoClimb.getTargetPose(wrapper.getPoseEstimatorPose(false));
    DriveToPose driveCommand = new DriveToPose(wrapper, targetPose);
    return driveCommand
        .until(() -> driveCommand.withinTolerance(0.05, new Rotation2d(0.05)))
        .andThen(
            Commands.run(
                () ->
                    wrapper.setVelocityOverride(
                        new ChassisSpeeds(
                            driveMagnitudeX.get(),
                            driveMagnitudeY.get(),
                            rotationalPID.calculate(
                                wrapper.getPoseEstimatorPose(false).getRotation().getRadians(),
                                targetPose.get().getRotation().getRadians())))))
        .finallyDo(wrapper::resetVelocityOverride);
  }
}
