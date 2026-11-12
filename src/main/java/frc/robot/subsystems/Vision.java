package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.CAMERA_ELASTIC_NAME;
import static frc.robot.Constants.VisionConstants.CAMERA_FORWARD_METERS;
import static frc.robot.Constants.VisionConstants.CAMERA_NAME;
import static frc.robot.Constants.VisionConstants.CAMERA_PITCH_DEG;
import static frc.robot.Constants.VisionConstants.CAMERA_RELAY_PORT;
import static frc.robot.Constants.VisionConstants.CAMERA_RIGHT_METERS;
import static frc.robot.Constants.VisionConstants.CAMERA_ROLL_DEG;
import static frc.robot.Constants.VisionConstants.CAMERA_STREAM_URL;
import static frc.robot.Constants.VisionConstants.CAMERA_UP_METERS;
import static frc.robot.Constants.VisionConstants.CAMERA_YAW_DEG;
import static frc.robot.Constants.VisionConstants.MAX_POSE_JUMP_METERS;
import static frc.robot.Constants.VisionConstants.MAX_TAG_DISTANCE_METERS;
import static frc.robot.Constants.VisionConstants.MAX_YAW_RATE_DEG_PER_SEC;
import static frc.robot.Constants.VisionConstants.USE_VISION_ODOMETRY;
import static frc.robot.Constants.VisionConstants.VISION_HEADING_STD_DEV_RAD;
import static frc.robot.Constants.VisionConstants.VISION_XY_STD_DEV_METERS;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.util.LimelightHelpers;

/**
 * Limelight 4 AprilTag vision.
 *
 * <p>Uses MegaTag2, which needs the robot yaw pushed to the camera every loop — that happens in
 * {@link #periodic()}. MegaTag2 uses the gyro to resolve the tag ambiguity that makes MegaTag1
 * jittery with a single tag in view, which is most of the time on this field. The cost is that
 * the yaw has to be correct <i>in field terms</i>: 0 degrees = facing the red alliance wall. The
 * gyro reads 0 at boot whichever way the robot faces, so the driver must press R1 with the robot
 * facing away from their own driver station before vision positions mean anything. If the
 * heading is wrong, the pose is wrong, and it will look like a camera problem.
 *
 * <p><b>Seeding.</b> Odometry starts at (0, 0) — the field corner — at boot. The jump guard below
 * would reject every real estimate against that, so vision would silently never correct anything.
 * Instead, the first trustworthy estimate after boot (or after R1 re-zeroes the heading) snaps the
 * robot's position straight to it, and while disabled it keeps snapping, since the robot is not
 * moving. Only once seeded and enabled does the jump guard apply, with estimates blended in
 * through the pose estimator.
 *
 * <p>Feeding the drivetrain is gated behind {@code USE_VISION_ODOMETRY}. Everything is published
 * regardless — including {@code Vision/WouldAccept} and {@code Vision/RejectReason} — so the
 * camera can be verified before it is trusted.
 *
 * <p>Depends on the camera being named exactly {@code limelight-front} in its web UI. That string
 * is the NetworkTables table name; get it wrong and every read returns a default with no error.
 */
public class Vision extends SubsystemBase {
  private final CommandSwerveDrivetrain drivetrain;

  private static final Matrix<N3, N1> VISION_STD_DEVS =
      VecBuilder.fill(VISION_XY_STD_DEV_METERS, VISION_XY_STD_DEV_METERS, VISION_HEADING_STD_DEV_RAD);

  /** Vision estimate as a second robot on the existing "Pose" Field2d that Telemetry publishes. */
  private final DoubleArrayPublisher fieldVisionPose =
      NetworkTableInstance.getDefault().getTable("Pose").getDoubleArrayTopic("Vision").publish();

  // Camera relay and its Elastic entries. Held as fields so they live as long as the robot code.
  private HttpCamera limelightCamera;
  private MjpegServer limelightRelay;
  private StringPublisher sourcePublisher;
  private StringArrayPublisher streamsPublisher;
  private StringPublisher modePublisher;
  private StringArrayPublisher modesPublisher;

  private boolean seeded = false;
  private int acceptedCount = 0;
  private int rejectedCount = 0;

  public Vision(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;

    // Push the measured camera pose every boot, so git rather than the camera holds it.
    // NetworkTables retains the value, so a Limelight that finishes booting after the robot code
    // still receives it when it connects.
    LimelightHelpers.setCameraPose_RobotSpace(
        CAMERA_NAME,
        CAMERA_FORWARD_METERS,
        CAMERA_RIGHT_METERS,
        CAMERA_UP_METERS,
        CAMERA_ROLL_DEG,
        CAMERA_PITCH_DEG,
        CAMERA_YAW_DEG);

    publishCameraStream();
  }

  /**
   * Makes the Limelight's video show in Elastic whether the laptop is on the radio or only on the
   * rio's USB cable.
   *
   * <p>The rio relays the stream on port 1181, and the Elastic camera entry
   * ({@code CAMERA_ELASTIC_NAME}) lists two addresses: the Limelight directly, then the relay over
   * USB. Elastic moves to the next address when one fails, so on the radio it uses the camera
   * directly and on USB it uses the relay.
   *
   * <p><b>Why not just "limelight-front":</b> the Limelight publishes its own
   * /CameraPublisher/limelight-front entry (confirmed via NetworkTables' publisher list, Nov 2026),
   * with only its direct address, and re-sends it every time it reconnects. Anything the robot
   * publishes under that same name gets overwritten — which is why WPILib's own relay address
   * never reached Elastic either. So ours uses a different name. In Elastic, pick
   * {@code limelight-front-rio}; the plain {@code limelight-front} entry is the Limelight's own
   * and only works on the radio.
   *
   * <p>Elastic recognises a camera by the mode/modes/source/streams entries, so all four are
   * published.
   *
   * <p>The relay only pulls frames from the Limelight while someone is watching: measured about
   * 18 fps over USB for about 10% of the rio's CPU, with no loop overruns.
   */
  private void publishCameraStream() {
    limelightCamera = new HttpCamera(CAMERA_NAME, CAMERA_STREAM_URL);
    limelightRelay = new MjpegServer("serve_" + CAMERA_NAME, CAMERA_RELAY_PORT);
    limelightRelay.setSource(limelightCamera);

    NetworkTable table =
        NetworkTableInstance.getDefault()
            .getTable("CameraPublisher")
            .getSubTable(CAMERA_ELASTIC_NAME);
    sourcePublisher = table.getStringTopic("source").publish();
    streamsPublisher = table.getStringArrayTopic("streams").publish();
    modePublisher = table.getStringTopic("mode").publish();
    modesPublisher = table.getStringArrayTopic("modes").publish();

    sourcePublisher.set("ip:" + CAMERA_STREAM_URL);
    streamsPublisher.set(
        new String[] {
          "mjpg:" + CAMERA_STREAM_URL,
          "mjpg:http://172.22.11.2:" + CAMERA_RELAY_PORT + "/?action=stream"
        });
    modePublisher.set("");
    modesPublisher.set(new String[0]);
  }

  /**
   * Forget the current seed, so the next trustworthy estimate snaps the position again. Call when
   * the heading is re-zeroed: every MegaTag2 position depends on the heading, so a heading change
   * makes the old position disagree with new estimates by more than the jump guard allows.
   */
  public void requestReseed() {
    seeded = false;
  }

  @Override
  public void periodic() {
    SwerveDriveState state = drivetrain.getState();

    // MegaTag2 needs the current heading before the estimate is read, every loop.
    LimelightHelpers.SetRobotOrientation(
        CAMERA_NAME, state.Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

    LimelightHelpers.PoseEstimate estimate =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(CAMERA_NAME);

    String rejectReason = rejectReason(estimate, state);
    boolean wouldAccept = rejectReason.isEmpty();

    if (wouldAccept && USE_VISION_ODOMETRY) {
      if (!seeded || DriverStation.isDisabled()) {
        // Heading is untouched: it comes from the gyro, and MegaTag2's heading is that same gyro.
        drivetrain.resetTranslation(estimate.pose.getTranslation());
        seeded = true;
      } else {
        drivetrain.addVisionMeasurement(estimate.pose, estimate.timestampSeconds, VISION_STD_DEVS);
      }
      acceptedCount++;
    } else if (!wouldAccept && estimate != null && estimate.tagCount > 0) {
      rejectedCount++;
    }

    publishDiagnostics(estimate, wouldAccept, rejectReason);
  }

  /**
   * Why an estimate should not be believed, or an empty string if it should.
   *
   * <p>No tags, tags too far to be precise, spinning too fast, or — once seeded and driving — a
   * pose that disagrees with odometry by more than the robot could plausibly have moved. That last
   * one catches a tag misidentified across the field, the failure that throws a robot into a wall.
   */
  private String rejectReason(LimelightHelpers.PoseEstimate estimate, SwerveDriveState state) {
    if (estimate == null || estimate.tagCount == 0) {
      return "No tags";
    }
    if (estimate.avgTagDist > MAX_TAG_DISTANCE_METERS) {
      return "Tag too far";
    }
    if (Math.abs(Math.toDegrees(state.Speeds.omegaRadiansPerSecond)) > MAX_YAW_RATE_DEG_PER_SEC) {
      return "Spinning too fast";
    }
    if (seeded && DriverStation.isEnabled()) {
      double jump = estimate.pose.getTranslation().getDistance(state.Pose.getTranslation());
      if (jump > MAX_POSE_JUMP_METERS) {
        return "Jump from odometry too big";
      }
    }
    return "";
  }

  private void publishDiagnostics(
      LimelightHelpers.PoseEstimate estimate, boolean wouldAccept, String rejectReason) {
    SmartDashboard.putBoolean("Vision/HasTarget", hasTarget());
    SmartDashboard.putNumber("Vision/TagID", getTagId());
    SmartDashboard.putNumber("Vision/TX", LimelightHelpers.getTX(CAMERA_NAME));
    SmartDashboard.putNumber("Vision/TY", LimelightHelpers.getTY(CAMERA_NAME));

    boolean hasEstimate = estimate != null && estimate.tagCount > 0;
    SmartDashboard.putNumber("Vision/TagCount", estimate != null ? estimate.tagCount : 0);
    if (hasEstimate) {
      SmartDashboard.putNumber("Vision/AvgTagDistance", estimate.avgTagDist);
      SmartDashboard.putNumber("Vision/EstimatedX", estimate.pose.getX());
      SmartDashboard.putNumber("Vision/EstimatedY", estimate.pose.getY());
      SmartDashboard.putNumber("Vision/EstimatedHeadingDeg", estimate.pose.getRotation().getDegrees());
      fieldVisionPose.set(
          new double[] {
            estimate.pose.getX(), estimate.pose.getY(), estimate.pose.getRotation().getDegrees()
          });
    } else {
      fieldVisionPose.set(new double[0]); // hides the vision robot on the Field widget
    }

    // WouldAccept/RejectReason work with odometry feeding off, so the guards can be checked first.
    SmartDashboard.putBoolean("Vision/WouldAccept", wouldAccept);
    SmartDashboard.putString("Vision/RejectReason", rejectReason);
    SmartDashboard.putBoolean("Vision/Seeded", seeded);
    SmartDashboard.putBoolean("Vision/FeedingOdometry", USE_VISION_ODOMETRY);
    SmartDashboard.putNumber("Vision/Accepted", acceptedCount);
    SmartDashboard.putNumber("Vision/Rejected", rejectedCount);
  }

  public boolean hasTarget() {
    return LimelightHelpers.getTV(CAMERA_NAME);
  }

  /** Primary tag id, or -1 when nothing is in view. */
  public int getTagId() {
    return hasTarget() ? (int) LimelightHelpers.getFiducialID(CAMERA_NAME) : -1;
  }

  /** Horizontal offset to the target in degrees. Zero when centred. */
  public double getTx() {
    return LimelightHelpers.getTX(CAMERA_NAME);
  }

  /** Vertical offset to the target in degrees. */
  public double getTy() {
    return LimelightHelpers.getTY(CAMERA_NAME);
  }
}
