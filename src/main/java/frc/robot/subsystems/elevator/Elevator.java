// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.RobotMode.RobotType;

public class Elevator extends SubsystemBase {

  private final ElevatorIO io;
  private final ElevatorIO.Inputs inputs = new ElevatorIO.Inputs();

  private static final TunableNumberGroup tunableGroup =
      new TunableNumberGroup(Constants.ElevatorConstants.NAME);

  private static final LoggedTunableNumber kP = tunableGroup.build("motorConfig/kP");
  private static final LoggedTunableNumber kD = tunableGroup.build("motorConfig/kD");
  private static final LoggedTunableNumber kG = tunableGroup.build("motorConfig/kG");
  private static final LoggedTunableNumber mmVelocity =
      tunableGroup.build("motorConfig/mmVelocity");
  private static final LoggedTunableNumber mmAcceleration =
      tunableGroup.build("motorConfig/mmAcceleration");

  static {
    if (Constants.RobotMode.getRobot() == RobotType.ROBOT_2024_MAESTRO) {
      kP.initDefault(1.0);
      kD.initDefault(0);
      kG.initDefault(0);
      mmVelocity.initDefault(640.0);
      mmAcceleration.initDefault(640.0);
    } else {
      kP.initDefault(12.0);
      kD.initDefault(0);
      kG.initDefault(0);
      mmVelocity.initDefault(1000);
      mmAcceleration.initDefault(2000);
    }
  }

  /** Creates a new Elevator. */
  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);

    int hc = hashCode();
    if (kP.hasChanged(hc)
        || kD.hasChanged(hc)
        || kG.hasChanged(hc)
        || mmVelocity.hasChanged(hc)
        || mmAcceleration.hasChanged(hc)) updateConstants();
  }

  private void updateConstants() {
    MotionMagicConfigs mmConfigs = new MotionMagicConfigs();
    mmConfigs.MotionMagicAcceleration = mmAcceleration.get();
    mmConfigs.MotionMagicCruiseVelocity = mmVelocity.get();
    io.setClosedLoopConstants(kP.get(), kD.get(), kG.get(), mmConfigs);
  }
}
