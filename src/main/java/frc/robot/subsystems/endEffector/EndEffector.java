// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endEffector;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.CANSparkLowLevel.FollowConfig.Config;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EndEffector extends SubsystemBase {
  /** Creates a new EndEffector. */
  
    private final TalonFX motor = new TalonFX(Constants.CanIDs.END_EFFECTOR_CAN_ID);

  public EndEffector() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 30.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
  }
  
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
