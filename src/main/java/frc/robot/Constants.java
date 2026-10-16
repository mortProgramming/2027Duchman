package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import frc.robot.generated.TunerConstants;

/**
 * FIRST Team 11 M.O.R.T. — Dutchman, kept running as the 2027 test and driver-tryout robot.
 * Platform: roboRIO on WPILib 2026 (NOT SystemCore — this chassis predates it).
 * Drivetrain: 4x SDS MK4i, Kraken X60 drive + steer, CANcoder azimuth, Pigeon 2, all on the
 * CANivore named "drivetrain".
 *
 * <p>Layout follows EVE (the 2027 skeleton): constants live here in nested {@code *Constants}
 * classes and are pulled in at the use site with static imports. The one deliberate difference is
 * that per-module hardware constants are NOT duplicated here — CAN IDs, encoder offsets, gear
 * ratios, inversions and gains live in {@link TunerConstants}, which Phoenix Tuner X generates and
 * will overwrite. Copying those values into this file would create a second source of truth that
 * silently goes stale the next time anyone re-runs the swerve generator.
 */
public final class Constants {
  private Constants() {}

  public static final class OperatorConstants {
    private OperatorConstants() {}

    public static final int DRIVER_CONTROLLER_PORT = 0;
    // TODO: no operator controller is bound yet — this is a drivetrain-only build. Add
    // OPERATOR_CONTROLLER_PORT = 1 when the elevator/algae/tiki-torch subsystems get ported over.

    /** Fraction of stick travel treated as zero, applied once in TeleopSwerve. */
    public static final double DRIVE_DEADBAND = 0.1;
  }

  public static final class DrivetrainConstants {
    private DrivetrainConstants() {}

    /**
     * Fraction of the robot's measured free speed available to the driver. Start tryouts well
     * under 1.0 — this robot's real top speed is quick enough to hurt someone or a wall.
     */
    public static final double SPEED_SCALE = 0.6;

    /**
     * Top translational speed, in m/s. Derived from {@code kSpeedAt12Volts} in TunerConstants
     * (3.41 m/s, measured on this robot by Tuner X) rather than typed in again here.
     */
    public static final double MAX_SPEED_MPS =
        TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * SPEED_SCALE;

    /** Top rotational speed, in rad/s. 0.75 rotations/sec is the CTRE generator's default. */
    public static final double MAX_ANGULAR_RAD_PER_SEC =
        RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    /** Creep speed for the drive-forward auto, in m/s. Deliberately slow — this is a test robot. */
    public static final double AUTO_CREEP_SPEED_MPS = 0.5;

    /** How long the drive-forward auto creeps, in seconds. */
    public static final double AUTO_DRIVE_SECONDS = 5.0;

    // TODO — UNRESOLVED HARDWARE CONFLICT, do not paper over this by picking one:
    // TunerConstants and the 2025 code disagree about this chassis.
    //   drive gear ratio   TunerConstants 8.143 (SDS MK4i L1)  vs  2025 code 5.357 (from tooth counts)
    //   wheel diameter     TunerConstants 3.6 in               vs  2025 code 3.99 in
    //   front-back         TunerConstants 27 in                vs  2025 code 23.75 in
    //   left-right         TunerConstants 22.5 in              vs  2025 code 27.75 in
    // The steer ratio (21.43) matches MK4i in both, so the modules are MK4i. A 52% drive-ratio
    // error would make odometry and any speed-based auto badly wrong. Count teeth against the SDS
    // build sheet and put a tape measure on the frame, then correct TunerConstants in Tuner X —
    // not here. Teleop driving works regardless, because it is open-loop voltage.
  }
}
