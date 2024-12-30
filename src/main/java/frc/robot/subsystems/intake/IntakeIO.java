package frc.robot.subsystems.intake;

public interface IntakeIO {
  /** Contains all of the input data received from hardware. */
  public static class Inputs {
    // Motor
    public double deviceTemp;
    public double appliedVolts;
    public double currentAmps;
    public double velocityRPM;
  }

  public default void updateInputs(Inputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setVelocity(double velocityRPM) {}

  public default void setClosedLoopConstants(double kP, double kV, double mmAcceleration) {}
}
