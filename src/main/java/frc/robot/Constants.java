package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

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
    public static final int OPERATOR_CONTROLLER_PORT = 1;

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

  /** Battery-voltage reference. Percent outputs are scaled by this and sent as setVoltage(). */
  public static final double ROBOT_VOLTAGE = 12.0;

  /**
   * Elevator: Vortex on CAN 11, REV through-bore absolute encoder on DIO 0, two limit switches.
   *
   * <p>Ported verbatim from the Worlds code. Two things here look wrong and are deliberately NOT
   * corrected, because the mechanism was tuned around them and the robot is mechanically unchanged:
   * POS_KG is applied as a raw percent-output bias rather than through a feedforward, and
   * UPPER_LIMIT_SWITCH_HEIGHT is negative while every setpoint is positive. Changing either would
   * change how the elevator behaves on a robot that currently works.
   */
  public static final class ElevatorConstants {
    private ElevatorConstants() {}

    public static final int MOTOR_ID = 11;
    public static final int ENCODER_DIO = 0;
    public static final int UPPER_LIMIT_SWITCH_DIO = 2;
    public static final int LOWER_LIMIT_SWITCH_DIO = 3;

    public static final double POS_KP = 0.08;
    public static final double POS_KI = 0.0;
    public static final double POS_KD = 0.003;

    /** Percent-output bias added to every elevator command. NOT a feedforward kG. */
    public static final double POS_KG = -0.05;

    public static final Constraints POS_CONSTRAINTS = new Constraints(80, 300);
    public static final Constraints POS_TELEOP_CONSTRAINTS = new Constraints(87, 360);

    /** Inches of carriage travel per encoder rotation. The 3 is the elevator stage count. */
    public static final double ROTATIONS_TO_INCHES = 5.642 * 3;

    /** Per-cycle jump that means the absolute encoder wrapped. Must stay below half a rotation. */
    public static final double MAXIMUM_INCH_CHANGE = 5;

    /** Seeds rotationsCompleted at boot. Bring-up value, re-derive if the mechanism is rebuilt. */
    public static final double ELEVATOR_OFFSET = -23.11;

    public static final double LOWER_LIMIT_SWITCH_HEIGHT = 0;
    public static final double UPPER_LIMIT_SWITCH_HEIGHT = -69.4;

    public static final double REST_HEIGHT = 5.6;
    public static final double L1_HEIGHT = 0;
    /** Floor pickup and "pop" were both literally 0 in the Worlds code. Kept as-is. */
    public static final double FLOOR_HEIGHT = 0;
    public static final double POP_HEIGHT = 0;
    public static final double L2_HEIGHT = 27.6;
    public static final double L3_HEIGHT = 43.6;
    public static final double L4_HEIGHT = 68;
    public static final double INTAKE_HEIGHT = 17.25;
    public static final double LOW_ALGAE_HEIGHT = 4.4;
    public static final double HIGH_ALGAE_HEIGHT = 22.3;
    /** 71.5 is above L4 (68) and above the upper limit switch magnitude (69.4). As at Worlds. */
    public static final double BARGE_HEIGHT = 71.5;

    /** Open-loop speed for manual elevator jogging. The Worlds testing controller used 0.2. */
    public static final double JOG_SPEED = 0.2;
  }

  /**
   * Algae arm: NEO 550 on CAN 13, absolute encoder on the SparkMax data port, geared 1:1 to the
   * pivot. WRAP_UPPER and WRAP_LOWER are NOT soft limits — they bound the dead band used to unwrap
   * the absolute encoder. This arm has no travel protection in software, exactly as at Worlds.
   */
  public static final class AlgaeArmConstants {
    private AlgaeArmConstants() {}

    public static final int MOTOR_ID = 13;

    public static final double ROT_KP = 0.0065;
    public static final double ROT_KI = 0.0;
    public static final double ROT_KD = 0.0;
    /** All four feedforward gains are zero, so the ArmFeedforward contributes nothing today. */
    public static final double ROT_KS = 0.0;
    public static final double ROT_KG = 0.0;
    public static final double ROT_KV = 0.0;
    public static final double ROT_KA = 0.0;
    public static final Constraints ARM_ROT_CONSTRAINTS = new Constraints(200, 750);

    public static final double OFFSET = -257;
    public static final double WRAP_UPPER = -120;
    public static final double WRAP_LOWER = -300;

    public static final double REST = 95;
    public static final double REEF_INTAKE = 45;
    public static final double FLOOR_INTAKE = -27;
    public static final double POP_INTAKE = 18;
    public static final double BARGE_SCORE = 45;

    // Software travel limits. The Worlds code had NONE, which is how a sign error turned into an
    // arm driving into a hard stop. Set generously outside the real setpoint range (-27 to 95).
    // TODO verify against the physical stops before trusting these to protect the mechanism.
    public static final double MIN_DEGREES = -40;
    public static final double MAX_DEGREES = 110;

  }

  /** Algae roller: NEO 550 on CAN 12. Percent output. REST_SPEED is a light hold, not zero. */
  public static final class AlgaeRollerConstants {
    private AlgaeRollerConstants() {}

    public static final int MOTOR_ID = 12;

    public static final double INTAKE_SPEED = -1;
    public static final double OUTTAKE_SPEED = 0.8;
    public static final double REST_SPEED = -0.05;
  }

  /**
   * Tiki torch arm (coral end effector): NEO 550 on CAN 15, absolute encoder on the data port.
   * Unlike the algae arm this one drives the motor with set(percent) rather than setVoltage(),
   * preserved from Worlds. ROT_KG is the only non-zero arm feedforward gain in the project.
   */
  public static final class TikiTorchArmConstants {
    private TikiTorchArmConstants() {}

    public static final int MOTOR_ID = 15;

    public static final double ROT_KP = 0.006;
    public static final double ROT_KI = 0.001;
    public static final double ROT_KD = 0.0;
    public static final double ROT_KS = 0.0;
    public static final double ROT_KG = -0.03;
    public static final double ROT_KV = 0.0;
    public static final double ROT_KA = 0.0;
    public static final Constraints ARM_ROT_CONSTRAINTS = new Constraints(300, 1000);

    public static final double OFFSET = 224;
    public static final double WRAP_UPPER = 45;
    public static final double WRAP_LOWER = 0;

    public static final double REST = 59.3;
    public static final double L4_SCORE = -65;
    public static final double L3_SCORE = -60;
    public static final double L2_SCORE = -60;
    public static final double L1_SCORE = 35;
    public static final double INTAKE = 35;
    /** Clearance pose. Every composite endeffector move starts by going here. */
    public static final double ALGAE_CLEAR = -89;

    // Software travel limits, set outside the real setpoint range (-89 to 59.3).
    // TODO verify against the physical stops before trusting these.
    public static final double MIN_DEGREES = -100;
    public static final double MAX_DEGREES = 75;

  }

  /** Tiki torch roller: NEO 550 on CAN 14. Percent output. */
  public static final class TikiTorchRollerConstants {
    private TikiTorchRollerConstants() {}

    public static final int MOTOR_ID = 14;

    public static final double INTAKE_SPEED = -1;
    public static final double OUTTAKE_SPEED = 1;
    public static final double REST_SPEED = -0.1;
    /** Slower score used for L1 so the coral does not launch. */
    public static final double L1_SPEED = 0.65;
  }

  /**
   * Climber: pneumatic, not a motor. A double solenoid latches onto the cage for the Reefscape
   * deep climb. CTRE PCM on CAN 32 — the Worlds source carries a "module type might be wrong"
   * note, so this is the first thing to check if the pistons do not fire.
   */
  public static final class ClimberConstants {
    private ClimberConstants() {}

    public static final int PCM_CAN_ID = 32;
    public static final int SOLENOID_UP = 7;
    public static final int SOLENOID_DOWN = 6;
  }

  /** Lights: REV Blinkin on PWM 0. Values are Blinkin pattern codes, not brightness. */
  public static final class LightsConstants {
    private LightsConstants() {}

    public static final int PWM_PORT = 0;

    public static final double PINK = 0.57;
    public static final double RED = 0.61;
    public static final double ORANGE = 0.65;
    public static final double GOLD = 0.67;
    public static final double YELLOW = 0.69;
    public static final double GREEN = 0.77;
    public static final double DARK_BLUE = 0.85;
    public static final double BLUE = 0.87;
    public static final double PURPLE = 0.91;
    public static final double WHITE = 0.93;
    public static final double BLACK = 0.99;

    /** Elevator heights that drive the idle colour. Were magic numbers inside TeleopLights. */
    public static final double HEIGHT_YELLOW = 60;
    public static final double HEIGHT_DARK_BLUE = 35;
    public static final double HEIGHT_WHITE = 10;
  }

  /**
   * Limelight 4, named "limelight-front" in its web UI. That name is the NetworkTables table
   * name, so it must match exactly or every read silently returns a default.
   *
   * <p>Pose estimation uses MegaTag2, which needs the robot yaw fed to the camera every loop —
   * see Vision.periodic(). MegaTag2 is far more stable than MegaTag1 when only one tag is in
   * view, which is the common case on this field.
   */
  public static final class VisionConstants {
    private VisionConstants() {}

    public static final String CAMERA_NAME = "limelight-front";

    /**
     * Whether vision corrections are fed into the drivetrain pose estimator.
     *
     * <p>Starts FALSE deliberately. A miscalibrated camera injecting poses will make a drivetrain
     * that currently works start fighting itself, and that is a miserable thing to debug mid
     * session. Verify the numbers on the dashboard first, then turn this on.
     */
    public static final boolean USE_VISION_ODOMETRY = false;

    /** Reject estimates further than this from the current odometry pose, in meters. */
    public static final double MAX_POSE_JUMP_METERS = 1.0;

    /** Ignore tags further away than this, in meters. Distant single tags are noisy. */
    public static final double MAX_TAG_DISTANCE_METERS = 4.0;
  }
}
