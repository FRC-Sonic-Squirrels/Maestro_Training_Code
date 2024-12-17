package frc.robot.subsystems.endEffector;

public interface EndEffectorIO {
  /** Contains all of the input data received from hardware. */
  public static class Inputs {
    public double deviceTemp;
    public double appliedVolts;
    public double currentAmps;
    public double velocityRPM;
    public double intakeSideTOFDistanceInches;
  }

  public default void updateInputs(Inputs inputs) {}

  public default void setVoltage(double volts) {}
}
