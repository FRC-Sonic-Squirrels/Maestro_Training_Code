package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;

public interface ElevatorIO {
  /** Contains all of the input data received from hardware. */
  public static class Inputs {
    public double heightInches;
    public double velocityInchesPerSecond;
    public double appliedVolts;
    public double currentAmps;
    public double tempCelsius;
  }

  public default void updateInputs(Inputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setHeightInches(double heightInches) {}

  public default void setSensorPosition(double heightInches) {}

  public default void setClosedLoopConstants(
      double kP, double kD, double kG, MotionMagicConfigs mmConfigs) {}
}
