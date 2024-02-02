// Copyright 2021-2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.team2930.ArrayUtil;
import frc.lib.team2930.GeometryUtil;
import frc.robot.Constants.RobotMode.Mode;
import frc.robot.Constants.RobotMode.RobotType;
import frc.robot.RobotState.ScoringMode;
import frc.robot.autonomous.AutoCommand;
import frc.robot.autonomous.AutosManager;
import frc.robot.commands.drive.DriveToGamepiece;
import frc.robot.commands.drive.DrivetrainDefaultTeleopDrive;
import frc.robot.commands.drive.RotateToSpeaker;
import frc.robot.commands.intake.EjectGamepiece;
import frc.robot.commands.intake.IntakeGamepiece;
import frc.robot.commands.shooter.ShooterShootMode;
import frc.robot.commands.shooter.ShooterStowMode;
import frc.robot.configs.SimulatorRobotConfig;
import frc.robot.mechanismVisualization.SimpleMechanismVisualization;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOReal;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.endEffector.EndEffectorIO;
import frc.robot.subsystems.endEffector.EndEffectorIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroIOPigeon2;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.subsystems.vision.VisionModuleConfiguration;
import frc.robot.subsystems.visionGamepiece.VisionGamepiece;
import frc.robot.subsystems.visionGamepiece.VisionGamepieceIO;
import frc.robot.subsystems.visionGamepiece.VisionGamepieceIOReal;
import java.util.ArrayList;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final Drivetrain drivetrain;
  private final Vision vision;
  private final Arm arm;
  private final Elevator elevator;
  private final Intake intake;
  private final Shooter shooter;
  private final EndEffector endEffector;
  private final LED led;
  private final VisionGamepiece visionGamepiece;

  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  private final ShuffleBoardLayouts shuffleBoardLayouts;

  private final LoggedDashboardChooser<Supplier<AutoCommand>> autoChooser =
      new LoggedDashboardChooser<Supplier<AutoCommand>>("Auto Routine");
  private final AutosManager autoManager;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    RobotType robotType = Constants.RobotMode.getRobot();
    Mode mode = Constants.RobotMode.getMode();

    Logger.recordOutput("RobotType", robotType);
    Logger.recordOutput("RobotMode", mode);

    var config = robotType.config.get();
    AprilTagFieldLayout aprilTagLayout;

    try {
      aprilTagLayout = config.getAprilTagFieldLayout();
    } catch (Exception e) {
      // FIXME:
      aprilTagLayout = new AprilTagFieldLayout(new ArrayList<AprilTag>(), 0.0, 0.0);
    }

    if (mode == Mode.REPLAY) {
      drivetrain = new Drivetrain(config, new GyroIO() {}, config.getReplaySwerveModuleObjects());

      vision =
          new Vision(
              aprilTagLayout,
              drivetrain::getPoseEstimatorPose,
              drivetrain::addVisionEstimate,
              config.getReplayVisionModules());

      arm = new Arm(new ArmIO() {});
      elevator = new Elevator(new ElevatorIO() {});
      intake = new Intake(new IntakeIO() {});
      shooter = new Shooter(new ShooterIO() {});
      endEffector = new EndEffector(new EndEffectorIO() {});
      led = new LED();
      visionGamepiece =
          new VisionGamepiece(new VisionGamepieceIO() {}, drivetrain::getPoseEstimatorPose);

    } else { // REAL and SIM robots HERE
      switch (robotType) {
        case ROBOT_SIMBOT:
          com.ctre.phoenix6.unmanaged.Unmanaged.setPhoenixDiagnosticsStartTime(0.0);

          drivetrain = new Drivetrain(config, new GyroIO() {}, config.getSwerveModuleObjects());
          VisionModuleConfiguration[] visionModules = {
            new VisionModuleConfiguration(
                new VisionIOSim(
                    config,
                    drivetrain::getPoseEstimatorPose,
                    SimulatorRobotConfig.FRONT_LEFT_ROBOT_TO_CAMERA,
                    SimulatorRobotConfig.FRONT_LEFT_CAMERA_NAME),
                SimulatorRobotConfig.FRONT_LEFT_CAMERA_NAME,
                SimulatorRobotConfig.FRONT_LEFT_ROBOT_TO_CAMERA),
            new VisionModuleConfiguration(
                new VisionIOSim(
                    config,
                    drivetrain::getPoseEstimatorPose,
                    SimulatorRobotConfig.FRONT_RIGHT_ROBOT_TO_CAMERA,
                    SimulatorRobotConfig.FRONT_RIGHT_CAMERA_NAME),
                SimulatorRobotConfig.FRONT_RIGHT_CAMERA_NAME,
                SimulatorRobotConfig.FRONT_RIGHT_ROBOT_TO_CAMERA),
            // new VisionModule(
            //     new VisionIOSim(
            //         config,
            //         drivetrain::getPose,
            //         SimulatorRobotConfig.BACK_ROBOT_TO_CAMERA,
            //         SimulatorRobotConfig.BACK_CAMERA_NAME),
            //     SimulatorRobotConfig.BACK_CAMERA_NAME,
            //     SimulatorRobotConfig.BACK_ROBOT_TO_CAMERA),
          };

          vision =
              new Vision(
                  aprilTagLayout,
                  drivetrain::getPoseEstimatorPose,
                  drivetrain::addVisionEstimate,
                  visionModules);
          arm = new Arm(new ArmIOSim());
          elevator = new Elevator(new ElevatorIOSim());
          intake = new Intake(new IntakeIOSim());
          shooter = new Shooter(new ShooterIOSim());
          endEffector = new EndEffector(new EndEffectorIOSim());
          led = new LED();
          visionGamepiece =
              new VisionGamepiece(new VisionGamepieceIOReal(), drivetrain::getPoseEstimatorPose);
          break;

        case ROBOT_2023_RETIRED_ROBER:
          drivetrain =
              new Drivetrain(config, new GyroIOPigeon2(config), config.getSwerveModuleObjects());

          vision =
              new Vision(
                  aprilTagLayout,
                  drivetrain::getPoseEstimatorPose,
                  drivetrain::addVisionEstimate,
                  config.getVisionModuleObjects());
          arm = new Arm(new ArmIO() {});
          elevator = new Elevator(new ElevatorIO() {});
          intake = new Intake(new IntakeIO() {});
          shooter = new Shooter(new ShooterIO() {});
          endEffector = new EndEffector(new EndEffectorIO() {});
          led = new LED();
          visionGamepiece =
              new VisionGamepiece(new VisionGamepieceIO() {}, drivetrain::getPoseEstimatorPose);
          break;

        case ROBOT_2024:
          // FIXME:
          // drivetrain =
          //     new Drivetrain(config, new GyroIOPigeon2(config), config.getSwerveModuleObjects());

          // vision = new Vision(aprilTagLayout, drivetrain, config.getVisionModuleObjects());
          // arm = new Arm(new ArmIOReal());
          // elevator = new Elevator(new ElevatorIOReal());
          // intake = new Intake(new IntakeIOReal());
          // shooter = new Shooter(new ShooterIOReal());
          // wrist = new Wrist(new WristIOReal());
          // endEffector = new EndEffector(new EndEffectorIOReal());

          DriverStation.silenceJoystickConnectionWarning(true);

          drivetrain =
              new Drivetrain(config, new GyroIO() {}, config.getReplaySwerveModuleObjects());

          vision =
              new Vision(
                  aprilTagLayout,
                  drivetrain::getPoseEstimatorPose,
                  drivetrain::addVisionEstimate,
                  config.getReplayVisionModules());
          arm = new Arm(new ArmIOReal());
          elevator = new Elevator(new ElevatorIO() {});
          intake = new Intake(new IntakeIO() {});
          shooter = new Shooter(new ShooterIO() {});
          endEffector = new EndEffector(new EndEffectorIO() {});
          led = new LED();
          visionGamepiece =
              new VisionGamepiece(new VisionGamepieceIOReal(), drivetrain::getPoseEstimatorPose);
          break;

        default:
          drivetrain =
              new Drivetrain(config, new GyroIO() {}, config.getReplaySwerveModuleObjects());
          vision =
              new Vision(
                  aprilTagLayout,
                  drivetrain::getPoseEstimatorPose,
                  drivetrain::addVisionEstimate,
                  config.getReplayVisionModules());
          arm = new Arm(new ArmIO() {});
          elevator = new Elevator(new ElevatorIO() {});
          intake = new Intake(new IntakeIO() {});
          shooter = new Shooter(new ShooterIO() {});
          endEffector = new EndEffector(new EndEffectorIO() {});
          led = new LED();
          visionGamepiece =
              new VisionGamepiece(new VisionGamepieceIO() {}, drivetrain::getPoseEstimatorPose);
          break;
      }
    }

    autoManager = new AutosManager(drivetrain, shooter, intake, endEffector, config, autoChooser);

    drivetrain.setDefaultCommand(
        new DrivetrainDefaultTeleopDrive(
            drivetrain,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    intake.setDefaultCommand(new IntakeGamepiece(intake, driverController.getHID()));

    shooter.setDefaultCommand(new ShooterStowMode(shooter));

    // Set up named commands for PathPlanner
    // NamedCommands.registerCommand(
    //     "Run Flywheel",
    //     Commands.startEnd(
    //         () -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop, flywheel));

    // Set up auto routines
    // autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up FF characterization routines
    // autoChooser.addOption(
    //     "Drive FF Characterization",
    //     new FeedForwardCharacterization(
    //         drive, drive::runCharacterizationVolts, drive::getCharacterizationVelocity));
    // autoChooser.addOption(
    //     "Flywheel FF Characterization",
    //     new FeedForwardCharacterization(
    //         flywheel, flywheel::runCharacterizationVolts,
    // flywheel::getCharacterizationVelocity));

    shuffleBoardLayouts =
        new ShuffleBoardLayouts(arm, elevator, endEffector, intake, shooter, drivetrain);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /*
    Button binding summary:

      Driver:

        right trigger: spin intake

        left trigger: drive to GP

        right bumper: speaker mode

        A button: confirm speaker shot

        left bumper: score in amp (may have A button confirmation)

        B button: force stow everything

        X button: move all position controlled subsystems to hard stops to reset sensor position

        Y button: toggle climb mode (A: confirm climb)

        Start: reset odometry

        POV down: eject gamepiece (through intake)

      Operator:

        B: (when not connected to FMS) run systems check command: press B to continue onto next stage of systems check
          Drivetrain
            1. Forward
            2. Spin
          Intake
            3. Spin
          Shooter
            4. Pivot
            5. Flywheel + kicker
          Elevator
            6. up
          Arm
            7. Rotate to amp or trap position
          Wrist
            8. move through range of motion
          End Effector
            9. (after moving mech back to stow position) spin
          ----- end command -----
          Vision
            10. check localization vision (preferably with april tag)

            12. look at LEDs
            13. double check connection from pigeon

        X: cancel systems check

        Right trigger (plus right stick): manual control of elevator

        POV down: initiate no vision mode?


    */
    // ----------- DRIVER CONTROLS ------------
    driverController.leftBumper().whileTrue(new EjectGamepiece(intake));
    driverController
        .leftTrigger()
        .whileTrue(
            new DriveToGamepiece(
                visionGamepiece::getClosestGamepiece, drivetrain, intake::getBeamBreak));

    driverController
        .rightBumper()
        .onTrue(
            new InstantCommand(() -> RobotState.getInstance().setScoringMode(ScoringMode.SPEAKER)))
        .onFalse(
            new InstantCommand(() -> RobotState.getInstance().setScoringMode(ScoringMode.NONE)))
        .whileTrue(
            new RotateToSpeaker(
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () ->
                    DriverStation.getAlliance().isPresent()
                        ? new Translation2d(
                            DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)
                                ? 0.03950466960668564
                                : 16.281435012817383,
                            5.498747638702393)
                        : new Translation2d(0.03950466960668564, 5.498747638702393),
                drivetrain,
                () -> false,
                new Rotation2d(Math.PI),
                shooter::getRPM));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public LoggedDashboardChooser<Supplier<AutoCommand>> getAutonomousChooser() {
    return autoChooser;
  }

  public void setPose(Pose2d pose) {
    drivetrain.setPose(pose);
  }

  public double[] getCurrentDraws() {
    double[] current =
        new double[] {
          intake.getCurrentDraw(),
        };

    current = ArrayUtil.concatWithArrayCopy(current, drivetrain.getCurrentDrawAmps());
    return current;
  }

  public void updateVisualization() {
    SimpleMechanismVisualization.updateVisualization(
        new Rotation2d(),
        shooter.getPitch(),
        Math.hypot(
            GeometryUtil.getDist(
                drivetrain.getPoseEstimatorPose().getTranslation(),
                DriverStation.getAlliance().isPresent()
                    ? (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)
                        ? Constants.FieldConstants.BLUE_SPEAKER_TRANSLATION
                        : Constants.FieldConstants.RED_SPEAKER_TRANSLATION)
                    : Constants.FieldConstants.BLUE_SPEAKER_TRANSLATION),
            Constants.FieldConstants.SPEAKER_HEIGHT_METERS),
        shooter.getRPM());
    SimpleMechanismVisualization.logMechanism();
  }

  public void updateRobotState() {
    Command shootModeCommand = new ShooterShootMode(shooter, drivetrain, shooter::getRPM);
    if (RobotState.getInstance().getScoringMode().equals(ScoringMode.SPEAKER)
        && !CommandScheduler.getInstance().isScheduled(shootModeCommand))
      CommandScheduler.getInstance().schedule(shootModeCommand);
  }
}
