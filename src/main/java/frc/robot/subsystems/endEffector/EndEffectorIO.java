package frc.robot.subsystems.endEffector;

public interface EndEffectorIO {
  /** Contains all of the input data received from hardware. */
  public static class Inputs {
    // Motor
    public double deviceTemp;
    public double appliedVolts;
    public double currentAmps;
    public double velocityRPM;

    // Time of Flights
    public double intakeSideTOFDistanceInches;
    public double shooterSideTOFDistanceInches;
  }

  public default void updateInputs(Inputs inputs) {}

  public default void setVoltage(double volts) {}
}
