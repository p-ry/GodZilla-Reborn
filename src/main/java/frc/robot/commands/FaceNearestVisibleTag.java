package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import frc.robot.InitLogger;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.function.Supplier;

public class FaceNearestVisibleTag extends Command {
  private static final double kP = 2.0;                       // Tune 1.0–3.0
  private static final double TOL_DEG = 2.0;                  // Finish when |error| ≤ 2°
  private static final double MAX_OMEGA_RAD = Math.toRadians(180.0); // Clamp 180°/s
  private static final double[] FIELD_TAG_HEADINGS_DEG = { 0, 60, 120, 180, -60, -120 };

  private final CommandSwerveDrivetrain drivetrain;
  private final Supplier<Pose2d> robotPose;
  private final String[] cams;

  private Rotation2d desiredHeading; // updated in execute()

  public FaceNearestVisibleTag(
      CommandSwerveDrivetrain drivetrain,
      Supplier<Pose2d> robotPose,
      String... limelightNames
  ) {
    this.drivetrain = drivetrain;
    this.robotPose = robotPose;
    this.cams = (limelightNames != null && limelightNames.length > 0)
        ? limelightNames
        : new String[] { "limelight-left", "limelight-right" };
    addRequirements(drivetrain);
    setName("FaceNearestVisibleTag");
  }

  @Override
  public void initialize() {
    desiredHeading = null;
    InitLogger.logMessage("FaceTag", InitLogger.Level.INFO, "Rotate-to-face START");
  }

  @Override
  public void execute() {
    Pose2d pose = robotPose.get();
    desiredHeading = computeFacingHeadingFromNearestVisibleTag(pose, cams);

    if (desiredHeading == null) {
      // No tag this tick → hold still
      drivetrain.setControl(
          drivetrain.m_pathApplyRobotSpeeds.withSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0))
      );
      InitLogger.logBoolean("FaceTag", "visible", false);
      return;
    }
    InitLogger.logBoolean("FaceTag", "visible", true);

    Rotation2d cur = pose.getRotation();
    double errRad = cur.minus(desiredHeading).getRadians(); // shortest signed diff
    double omega = kP * errRad;

    // Clamp ω
    if (omega >  MAX_OMEGA_RAD) omega =  MAX_OMEGA_RAD;
    if (omega < -MAX_OMEGA_RAD) omega = -MAX_OMEGA_RAD;

    // If your robot spins the wrong way, flip exactly once here:
    // omega = -omega;

    // Rotate in place using YOUR drivetrain API
    drivetrain.setControl(
        drivetrain.m_pathApplyRobotSpeeds.withSpeeds(new ChassisSpeeds(0.0, 0.0, omega))
    );

    // Telemetry
    InitLogger.logDouble("FaceTag", "targetHeadingDeg", desiredHeading.getDegrees());
    InitLogger.logDouble("FaceTag", "robotHeadingDeg", cur.getDegrees());
    InitLogger.logDouble("FaceTag", "headingErrDeg", Math.toDegrees(errRad));
    InitLogger.logDouble("FaceTag", "omegaCmd", omega);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop so we never "lock" the robot
    drivetrain.setControl(
        drivetrain.m_pathApplyRobotSpeeds.withSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0))
    );
    InitLogger.logMessage("FaceTag", InitLogger.Level.INFO,
        interrupted ? "Rotate-to-face CANCEL" : "Rotate-to-face DONE");
  }

  @Override
  public boolean isFinished() {
    if (desiredHeading == null) return false; // allow .whileTrue() to interrupt on release
    double errDeg = wrapDeg(robotPose.get().getRotation().minus(desiredHeading).getDegrees());
    boolean aligned = Math.abs(errDeg) <= TOL_DEG;
    InitLogger.logBoolean("FaceTag", "finish.aligned", aligned);
    return aligned;
  }

  // ----------------- Private helpers -----------------

  private static double wrapDeg(double deg) {
    double d = deg % 360.0;
    if (d <= -180.0) d += 360.0;
    if (d > 180.0) d -= 360.0;
    return d;
  }

  private static double angDiffDeg(double a, double b) {
    return wrapDeg(a - b);
  }

  private static double snapToFieldHeadings(double deg) {
    double best = FIELD_TAG_HEADINGS_DEG[0];
    double bestAbs = Math.abs(angDiffDeg(deg, best));
    for (int i = 1; i < FIELD_TAG_HEADINGS_DEG.length; i++) {
      double cand = FIELD_TAG_HEADINGS_DEG[i];
      double err = Math.abs(angDiffDeg(deg, cand));
      if (err < bestAbs) { bestAbs = err; best = cand; }
    }
    return wrapDeg(best);
  }

  /**
   * Compute the ABSOLUTE field heading the robot should hold to face the
   * nearest visible tag (no field layout required).
   *
   * desired = robotHeading + (tagYawRobot + 180°), snapped to {0, ±60, ±120, 180}.
   * Returns null if no camera sees a tag.
   */
  private static Rotation2d computeFacingHeadingFromNearestVisibleTag(Pose2d robotPose, String[] limelightNames) {
    Pose3d bestRS = null;
    String usedCam = null;
    double bestRange = Double.POSITIVE_INFINITY;

    for (String name : limelightNames) {
      try {
        boolean tv = LimelightHelpers.getTV(name);
        InitLogger.logBoolean("FaceTag." + name, "hasTarget", tv);
        if (!tv) continue;

        Pose3d rs = LimelightHelpers.getTargetPose3d_RobotSpace(name);
        if (rs == null) continue;

        double dx = rs.getX(), dy = rs.getY();
        double range = Math.hypot(dx, dy);
        InitLogger.logDouble("FaceTag." + name, "dxRobot", dx);
        InitLogger.logDouble("FaceTag." + name, "dyRobot", dy);
        InitLogger.logDouble("FaceTag." + name, "range", range);

        if (range < bestRange) {
          bestRange = range;
          bestRS = rs;
          usedCam = name;
        }
      } catch (Throwable t) {
        InitLogger.logMessage("FaceTag." + name, InitLogger.Level.ERROR,
            "RobotSpace read exception: " + t.getMessage());
      }
    }

    if (bestRS == null) {
      InitLogger.logMessage("FaceTag", InitLogger.Level.WARN, "No visible tag on any camera");
      return null;
    }

    // Tag yaw in ROBOT frame (Δ = θ_tag - θ_robot), degrees
    double tagYawRobotDeg = Math.toDegrees(bestRS.getRotation().getZ());
    tagYawRobotDeg = wrapDeg(tagYawRobotDeg);

    // Absolute desired heading = θ_robot + (Δ + 180°)
    double robotHeadingDeg = robotPose.getRotation().getDegrees();
    double deltaDeg = wrapDeg(tagYawRobotDeg + 180.0);
    double desiredRawDeg = wrapDeg(robotHeadingDeg + deltaDeg);

    // Snap to nearest allowed field heading
    double desiredSnappedDeg = snapToFieldHeadings(desiredRawDeg);

    // Telemetry
    InitLogger.logMessage("FaceTag", InitLogger.Level.INFO, "Using camera=" + usedCam);
    InitLogger.logDouble("FaceTag", "robotHeadingDeg", robotHeadingDeg);
    InitLogger.logDouble("FaceTag", "tagYawRobotDeg", tagYawRobotDeg);
    InitLogger.logDouble("FaceTag", "deltaAddDeg", deltaDeg);
    InitLogger.logDouble("FaceTag", "desiredRawDeg", desiredRawDeg);
    InitLogger.logDouble("FaceTag", "desiredSnappedDeg", desiredSnappedDeg);

    return Rotation2d.fromDegrees(desiredSnappedDeg);
  }
}
