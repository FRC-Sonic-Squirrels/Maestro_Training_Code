package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIO.Inputs;

public class ElevatorIOReal implements ElevatorIO {

  private TalonFX motor = new TalonFX(Constants.CanIDs.ELEVATOR_CAN_ID);

  private StatusSignal<Double> positionRotations;
  private StatusSignal<Double> velocityRPS;
  private StatusSignal<Double> appliedVolts;
  private StatusSignal<Double> currentAmps;
  private StatusSignal<Double> tempCelsius;

  private VoltageOut openLoopControl = new VoltageOut(0).withEnableFOC(true);
  private MotionMagicVoltage closedLoopControl = new MotionMagicVoltage(0).withEnableFOC(true);

  public ElevatorIOReal() {
    // Motor Config
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.Feedback.SensorToMechanismRatio = Constants.IntakeConstants.GEAR_RATIO;

    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        ElevatorConstants.MAX_HEIGHT * ElevatorConstants.INCHES_TO_MOTOR_ROT;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    config.Voltage.SupplyVoltageTimeConstant = 0.02;

    motor.getConfigurator().apply(config);

    // Motor inputs

    positionRotations = motor.getPosition();
    velocityRPS = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    currentAmps = motor.getSupplyCurrent();
    tempCelsius = motor.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(100, positionRotations, velocityRPS);
    BaseStatusSignal.setUpdateFrequencyForAll(50, appliedVolts, currentAmps);
    BaseStatusSignal.setUpdateFrequencyForAll(1, tempCelsius);
  }

  @Override
  public void updateInputs(Inputs inputs) {
    inputs.heightInches =
        positionRotations.getValueAsDouble() / ElevatorConstants.INCHES_TO_MOTOR_ROT;
    inputs.velocityInchesPerSecond =
        velocityRPS.getValueAsDouble() / ElevatorConstants.INCHES_TO_MOTOR_ROT;
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
    inputs.tempCelsius = tempCelsius.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(openLoopControl.withOutput(volts));
  }

  @Override
  public void setHeightInches(double heightInches) {
    motor.setControl(
        closedLoopControl.withPosition(heightInches * ElevatorConstants.INCHES_TO_MOTOR_ROT));
  }

  @Override
  public void setSensorPosition(double heightInches) {
    motor.setPosition(heightInches * ElevatorConstants.INCHES_TO_MOTOR_ROT);
  }

  @Override
  public void setClosedLoopConstants(
      double kP, double kD, double kG, MotionMagicConfigs mmConfigs) {
    TalonFXConfigurator configurator = motor.getConfigurator();
    Slot0Configs pidConfig = new Slot0Configs();

    configurator.refresh(pidConfig);
    configurator.refresh(mmConfigs);

    pidConfig.kP = kP;
    pidConfig.kD = kD;
    pidConfig.kG = kG;

    configurator.apply(pidConfig);
    configurator.apply(mmConfigs);
  }
}
