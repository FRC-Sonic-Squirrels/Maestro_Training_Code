package frc.robot.subsystems.endEffector;

public interface EndEffectorIO {

  public static class Inputs {
    double deviceTemp;
    double appliedVolts;
    double currentAmps;
    double velocityRPS;
  }

  public default void updateInputs(Inputs inputs) {}

  public default void setVoltage(double volts) {}
}
