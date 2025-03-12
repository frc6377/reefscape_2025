// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.RobotController;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

/**
 * A class for keeping track of how much time it takes for different parts of code to execute. This
 * is done with epochs, that are added by calls to {@link #addEpoch(String)}, and can be printed
 * with a call to {@link #printEpochs()}.
 *
 * <p>Epochs are a way to partition the time elapsed so that when overruns occur, one can determine
 * which parts of an operation consumed the most time.
 */
public class NTTracer {
  private static final long kMinPrintPeriod = 1000000; // microseconds

  private long m_lastEpochsPrintTime; // microseconds
  private long m_startTime; // microseconds

  private final Map<String, Long> m_epochs = new HashMap<>(); // microseconds

  /** Tracer constructor. */
  public NTTracer() {
    resetTimer();
  }

  /** Clears all epochs. */
  public void clearEpochs() {
    m_epochs.clear();
    resetTimer();
  }

  /** Restarts the epoch timer. */
  public final void resetTimer() {
    m_startTime = RobotController.getFPGATime();
  }

  /**
   * Adds time since last epoch to the list printed by printEpochs().
   *
   * <p>Epochs are a way to partition the time elapsed so that when overruns occur, one can
   * determine which parts of an operation consumed the most time.
   *
   * <p>This should be called immediately after execution has finished, with a call to this method
   * or {@link #resetTimer()} before execution.
   *
   * @param epochName The name to associate with the epoch.
   */
  public void addEpoch(String epochName) {
    long currentTime = RobotController.getFPGATime();
    m_epochs.put(epochName, currentTime - m_startTime);
    m_startTime = currentTime;
  }

  /** Prints list of epochs added so far and their times to the DriverStation. */
  public void logEpochs() {
    long now = RobotController.getFPGATime();
    if (now - m_lastEpochsPrintTime > kMinPrintPeriod) {
      m_lastEpochsPrintTime = now;
      m_epochs.forEach(
          (key, value) -> Logger.recordOutput("subsystem times/" + key + " (ms)", value / 1.0e3));
    }
  }
}
