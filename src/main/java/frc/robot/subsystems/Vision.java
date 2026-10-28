package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.CAMERA_NAME;
import static frc.robot.Constants.VisionConstants.MAX_POSE_JUMP_METERS;
import static frc.robot.Constants.VisionConstants.MAX_TAG_DISTANCE_METERS;
import static frc.robot.Constants.VisionConstants.USE_VISION_ODOMETRY;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.util.LimelightHelpers;

/**
 * Limelight 4 AprilTag vision.
 *
 * <p>Uses MegaTag2, which needs the robot yaw pushed to the camera every loop — that happens in
 * {@link #periodic()}. MegaTag2 uses the gyro to resolve the tag ambiguity that makes MegaTag1
 * jittery with a single tag in view, which is most of the time on this field. The cost is that
 * the yaw has to be correct: if the gyro is wrong, the pose is wrong, and it will look like a
 * camera problem.
 *
 * <p>Feeding corrections into the drivetrain is gated behind {@code USE_VISION_ODOMETRY}, off by
 * default. Everything here publishes to the dashboard regardless, so the camera can be verified
 * before it is trusted with odometry.
 *
 * <p>Depends on the camera being named exactly {@code limelight-tigers} in its web UI. That string
 * is the NetworkTables table name; get it wrong and every read returns a default with no error.
 */
public class Vision extends SubsystemBase {
  private final CommandSwerveDrivetrain drivetrain;

  private int acceptedCount = 0;
  private int rejectedCount = 0;

  public Vision(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  @Override
  public void periodic() {
    // MegaTag2 needs the current heading before the estimate is read, every loop.
    double yawDegrees = drivetrain.getState().Pose.getRotation().getDegrees();
    LimelightHelpers.SetRobotOrientation(CAMERA_NAME, yawDegrees, 0, 0, 0, 0, 0);

    LimelightHelpers.PoseEstimate estimate =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(CAMERA_NAME);

    publishDiagnostics(estimate);

    if (USE_VISION_ODOMETRY && isTrustworthy(estimate)) {
      drivetrain.addVisionMeasurement(estimate.pose, estimate.timestampSeconds);
      acceptedCount++;
    } else if (estimate != null && estimate.tagCount > 0) {
      rejectedCount++;
    }
  }

  /**
   * Whether an estimate is worth believing.
   *
   * <p>Three independent reasons to reject: no tags at all, tags too far away to be precise, or a
   * pose that disagrees with odometry by more than the robot could plausibly have moved. That
   * last one is the important guard — it catches a tag misidentified across the field, which is
   * the failure that throws a robot into a wall.
   */
  private boolean isTrustworthy(LimelightHelpers.PoseEstimate estimate) {
    if (estimate == null || estimate.tagCount == 0) {
      return false;
    }

    if (estimate.avgTagDist > MAX_TAG_DISTANCE_METERS) {
      return false;
    }

    double jump =
        estimate.pose.getTranslation().getDistance(drivetrain.getState().Pose.getTranslation());

    return jump <= MAX_POSE_JUMP_METERS;
  }

  private void publishDiagnostics(LimelightHelpers.PoseEstimate estimate) {
    SmartDashboard.putBoolean("Vision/HasTarget", hasTarget());
    SmartDashboard.putNumber("Vision/TagID", getTagId());
    SmartDashboard.putNumber("Vision/TX", LimelightHelpers.getTX(CAMERA_NAME));
    SmartDashboard.putNumber("Vision/TY", LimelightHelpers.getTY(CAMERA_NAME));

    if (estimate != null) {
      SmartDashboard.putNumber("Vision/TagCount", estimate.tagCount);
      SmartDashboard.putNumber("Vision/AvgTagDistance", estimate.avgTagDist);
      SmartDashboard.putNumber("Vision/EstimatedX", estimate.pose.getX());
      SmartDashboard.putNumber("Vision/EstimatedY", estimate.pose.getY());
      SmartDashboard.putNumber("Vision/EstimatedHeadingDeg", estimate.pose.getRotation().getDegrees());
    }

    // Watch these while calibrating. Rejections climbing with tags in view means the camera and
    // the odometry disagree, which is usually the robot-to-camera transform in the web UI.
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
