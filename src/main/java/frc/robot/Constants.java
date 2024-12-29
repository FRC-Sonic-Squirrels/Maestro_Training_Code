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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.team6328.Alert;
import frc.lib.team6328.Alert.AlertType;
import frc.robot.configs.RobotConfig;
import frc.robot.configs.RobotConfig2023Rober;
import frc.robot.configs.RobotConfig2024;
import frc.robot.configs.SimulatorRobotConfig;
import java.util.function.Supplier;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double kDefaultPeriod = 0.02;
  public static final Translation2d zeroTranslation2d = new Translation2d();
  public static final Rotation2d zeroRotation2d = new Rotation2d();
  public static final Pose2d zeroPose2d = new Pose2d();
  public static final Pose3d zeroPose3d = new Pose3d();

  public static boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
  }

  public static class RobotMode {
    private static final RobotType ROBOT = RobotType.ROBOT_2024_MAESTRO;

    private static final Alert invalidRobotAlert =
        new Alert("Invalid robot selected, using competition robot as default.", AlertType.ERROR);

    public static boolean isSimBot() {
      switch (getRobot()) {
        case ROBOT_SIMBOT:
        case ROBOT_SIMBOT_REAL_CAMERAS:
          return true;

        default:
          return false;
      }
    }

    public static Mode getMode() {
      if (isSimBot()) {
        return Mode.SIM;
      }

      return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
    }

    public static RobotType getRobot() {
      if (!RobotBase.isReal()) {
        return RobotType.ROBOT_SIMBOT;
      }

      if (ROBOT != RobotType.ROBOT_2024_MAESTRO) {
        invalidRobotAlert.set(true);
        return RobotType.ROBOT_2024_MAESTRO;
      }

      return ROBOT;
    }

    // FIXME: update for various robots
    public enum RobotType {
      // use supplier because if we just create the object, the fields in the
      // config classes are also created. Meaning tunableNumber values are stuck to
      // the first
      // object that is created. In this case ExampleRobotConfig. Suppliers solve this
      // by only creating the specific config object corresponding to the robot type
      ROBOT_SIMBOT(SimulatorRobotConfig::new),
      ROBOT_SIMBOT_REAL_CAMERAS(SimulatorRobotConfig::new),
      ROBOT_2023_RETIRED_ROBER(RobotConfig2023Rober::new),
      ROBOT_2024_MAESTRO(RobotConfig2024::new);

      public final Supplier<RobotConfig> config;

      RobotType(Supplier<RobotConfig> config) {
        this.config = config;
      }
    }

    public enum Mode {
      REAL,
      REPLAY,
      SIM
    }
  }

  public static double MAX_VOLTAGE = 12.0;

  public static class MotorConstants {
    public static class KrakenConstants {
      public static final double MAX_RPM = 6000.0;
      public static final double NOMINAL_VOLTAGE_VOLTS = 12.0;
      public static final double STALL_TORQUE_NEWTON_METERS = 7.09;
      public static final double STALL_CURRENT_AMPS = 40.0;
      public static final double FREE_CURRENT_AMPS = 30.0;
      public static final double FREE_SPEED_RPM = 6000.0;
    }
  }

  public static class EndEffectorConstants {
    public static final String NAME = "EndEffector";
    public static final double GEAR_RATIO = 1.33;
    public static final double MOI = 0.5;
  }

  public static class CanIDs {
    // READ ME: CAN ID's THAT ARE NOT VALID TO USE
    // 1, 11, 21, 31
    // 2, 12, 22, 32
    // 3, 13, 23, 33
    // 4, 14, 24, 34
    // all these CAN ID's are reserved for the Drivetrain

    public static final int INTAKE_CAN_ID = 34;

    public static final int SHOOTER_LEAD_CAN_ID = 33;
    public static final int SHOOTER_FOLLOW_CAN_ID = 36;
    public static final int SHOOTER_PIVOT_CAN_ID = 32;
    public static final int SHOOTER_KICKER_CAN_ID = 35;
    public static final int SHOOTER_TOF_CAN_ID = 38;

    public static final int ARM_CAN_ID = 17;

    public static final int ELEVATOR_CAN_ID = 37;
    public static final int REACTION_ARM_CAN_ID = 7;

    public static final int END_EFFECTOR_CAN_ID = 30;
    public static final int END_EFFECTOR_INTAKE_SIDE_TOF_CAN_ID = 39;
    public static final int END_EFFECTOR_SHOOTER_SIDE_TOF_CAN_ID = 40;

    public static final int GYRO_2_CAN_ID = 41;
  }

  public static boolean unusedCode = false;
}
