package frc.lib.team2930;

import org.littletonrobotics.junction.Logger;

public class ExecutionTiming implements AutoCloseable {
  private static final String ROOT_TABLE = "ExecutionTiming/";

  private final String context;
  private long startTime;

  public ExecutionTiming(String context) {
    this.context = context;
  }

  public ExecutionTiming start() {
    startTime = Logger.getRealTimestamp();
    return this;
  }

  @Override
  public void close() {
    var endTime = Logger.getRealTimestamp();

    Logger.recordOutput(ROOT_TABLE + context, endTime - startTime);
  }
}
