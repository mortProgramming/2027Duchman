package frc.robot.util;

import java.io.File;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Keeps CTRE .hoot logs off the roboRIO's internal flash.
 *
 * <p>Phoenix 6 writes .hoot logs to the first USB stick if one is mounted, otherwise to
 * /home/lvuser/logs on the rio's 386 MB internal flash, and never deletes old ones. In Nov 2026
 * that filled the disk to 12 KB free and every deploy failed with "SFTP error (SSH_FX_FAILURE)".
 *
 * <p>There were two things starting the logger, and both have to be gated: Phoenix's own
 * auto-logging, and the {@code SignalLogger.start()} in the Tuner X generated {@code Telemetry}
 * constructor. Disabling auto-logging alone did nothing because of the second one.
 *
 * <p>Simulation is unaffected: logs land in the project folder on the laptop, which is fine.
 */
public final class HootLogStorage {
  private HootLogStorage() {}

  /**
   * Whether it is safe to write .hoot logs. On this rio, /u only exists while a USB stick is
   * plugged in, and that is where CTRE writes by default when it does.
   */
  public static boolean canLog() {
    return RobotBase.isSimulation() || new File("/u").isDirectory();
  }

  /** Call once, first thing in the Robot constructor, before Phoenix's 1 s auto-log window. */
  public static void configure() {
    if (!canLog()) {
      SignalLogger.enableAutoLogging(false);
    }
  }
}
