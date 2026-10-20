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

    // Drivetrain geometry is Jaden's measurements, captured in TunerConstants by Phoenix Tuner X:
    // drive ratio 8.143 (SDS MK4i L1), steer 21.43, wheel radius 1.8 in, modules at +/-13.5 in
    // front-back and +/-11.25 in left-right, free speed 3.41 m/s.
    //
    // These supersede the 2025 project's numbers (ratio 5.357, wheel 3.99 in, 23.75 x 27.75 in),
    // which disagreed and are used nowhere in this project. Resolved 2026-10-19 in favour of the
    // measured set.
    //
    // Change these in Tuner X and re-export. Editing TunerConstants by hand works until the next
    // swerve-generator run silently reverts it.
  }
}
