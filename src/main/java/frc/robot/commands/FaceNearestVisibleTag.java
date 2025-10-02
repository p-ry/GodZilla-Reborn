package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import frc.robot.Constants;
import frc.robot.InitLogger;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.function.Supplier;

import static java.lang.Math.*;

/**
 * Phase 1: rotate in place until aligned to facing heading (<= 2 deg).
 * Phase 2: schedule pathfindToPose(where, constraints, 0.0) and finish when it does.
 *
 * “Facing heading” is computed from nearest visible tag using robot-space yaw,
 * then snapped to {0, ±60, ±120, 180}. The goal pose is the tag field position
 * shifted by (Constants.forwardOffset, Constants.rightOffset) in the facing frame.
 */
public class FaceNearestVisibleTag extends Command {
  // Allowed field headings (deg) for your tags
  private static final double[] FIELD_TAG_HEADINGS_DEG = { 0, 60, 120, 180, -60, -120 };

  // ROTATE phase params
  private static final double kP_ROT = 2.0;                // tune 1.0–3.0
  private static final double TOL_DEG = 2.0;               // finish rotate when |err| ≤ 2°
  private static final double MAX_OMEGA_RAD = Math.toRadians(180.0); // clamp 180°/s
  private static final double ROTATE_TIMEOUT_SEC = 2.5;    // safety timeout for rotate

  private enum Phase { ROTATE, DRIVE, DONE }

  private final CommandSwerveDrivetrain drivetrain;
  private final Supplier<Pose2d> robotPose;
  private final String[] cams;
  private final PathConstraints constraints;

  // Latched at initialize()
  private Rotation2d faceHeading = null;
  private Pose2d goalPose = null;

  // State
  private Phase phase = Phase.ROTATE;
  private edu.wpi.first.wpilibj2.command.Command pathCmd = null;
  private final edu.wpi.first.wpilibj.Timer timer = new edu.wpi.first.wpilibj.Timer();

  public FaceNearestVisibleTag(
      CommandSwerveDrivetrain drivetrain,
      Supplier<Pose2d> robotPose,
      PathConstraints constraints,     // pass your preferred constraints
      String... limelightNames         // "limelight-left", "limelight-right"
  ) {
    this.drivetrain = drivetrain;
    this.robotPose = robotPose;
    this.constraints = (constraints != null)
        ? constraints
        : new PathConstraints(2.0, 2.0, 3.0, 3.0);
    this.cams = (limelightNames != null && limelightNames.length > 0)
        ? limelightNames
        : new String[] { "limelight-left", "limelight-right" };

    // Parent does not addRequirements so the child path command can own the drivetrain.
    // If you prefer, you can addRequirements(drivetrain); it also works with child owning too.
    setName("FaceNearestVisibleTag_RotateThenPathfind");
  }

  @Override
  public void initialize() {
    phase = Phase.ROTATE;
    pathCmd = null;
    timer.restart();

    // 1) Pick nearest visible tag (robot-space)
    Pose2d pose = robotPose.get();
    Rotation2d robotHdg = pose.getRotation();

    Pose3d bestRS = null;
    String usedCam = null;
    double bestRange = Double.POSITIVE_INFINITY;

    for (String name : cams) {
      try {
        boolean tv = LimelightHelpers.getTV(name);
        InitLogger.logBoolean("FaceTag." + name, "hasTarget", tv);
        if (!tv) continue;

        Pose3d rs = LimelightHelpers.getTargetPose3d_RobotSpace(name);
        if (rs == null) continue;

        double dx = rs.getX(), dy = rs.getY();
        double range = hypot(dx, dy);
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
      // No tag → nothing to do
      InitLogger.logMessage("FaceTag", InitLogger.Level.WARN, "No visible tag on any camera");
      phase = Phase.DONE;
      return;
    }

    // 2) Tag FIELD position = robotXY + rotate((dx,dy)_robot, robotHeading)
    Translation2d robotToTag_robot = new Translation2d(bestRS.getX(), bestRS.getY()); // +X fwd, +Y left
    Translation2d robotToTag_field = robotToTag_robot.rotateBy(robotHdg);
    Translation2d tagField = pose.getTranslation().plus(robotToTag_field);

    InitLogger.logMessage("FaceTag", InitLogger.Level.INFO, "Using camera=" + usedCam);
    InitLogger.logDouble("FaceTag", "robot.x", pose.getX());
    InitLogger.logDouble("FaceTag", "robot.y", pose.getY());
    InitLogger.logDouble("FaceTag", "tagField.x", tagField.getX());
    InitLogger.logDouble("FaceTag", "tagField.y", tagField.getY());

    // 3) Compute facing heading from tag yaw in ROBOT frame, snapped to {0,±60,±120,180}
    double tagYawRobotDeg = wrapDeg(Math.toDegrees(bestRS.getRotation().getZ()));
    double desiredRawDeg = wrapDeg(robotHdg.getDegrees() + wrapDeg(tagYawRobotDeg + 180.0));
    double desiredSnappedDeg = snapToFieldHeadings(desiredRawDeg);
    faceHeading = Rotation2d.fromDegrees(desiredSnappedDeg);

    InitLogger.logDouble("FaceTag", "robotHeadingDeg", robotHdg.getDegrees());
    InitLogger.logDouble("FaceTag", "tagYawRobotDeg", tagYawRobotDeg);
    InitLogger.logDouble("FaceTag", "desiredRawDeg", desiredRawDeg);
    InitLogger.logDouble("FaceTag", "desiredSnappedDeg", desiredSnappedDeg);

    // 4) Build goal pose shifted by your constants in the facing frame
    double fwd = Constants.forwardOffset;  // meters
    double rgt = Constants.rightOffset;    // meters

    // Forward along facing; right is robot's right relative to facing
    Translation2d fwdVec = new Translation2d(cos(faceHeading.getRadians()), sin(faceHeading.getRadians()));
    Translation2d rgtVec = new Translation2d(sin(faceHeading.getRadians()), -cos(faceHeading.getRadians()));

    Translation2d goalXY = tagField.plus(fwdVec.times(fwd)).plus(rgtVec.times(rgt));
    goalPose = new Pose2d(goalXY, faceHeading);

    InitLogger.logDouble("FaceTag", "goal.x", goalPose.getX());
    InitLogger.logDouble("FaceTag", "goal.y", goalPose.getY());
    InitLogger.logDouble("FaceTag", "goal.headingDeg", faceHeading.getDegrees());
  }

  @Override
  public void execute() {
    switch (phase) {
      case ROTATE -> {
        if (faceHeading == null) { phase = Phase.DONE; return; }
        Pose2d pose = robotPose.get();
        Rotation2d cur = pose.getRotation();

        double errRad = cur.minus(faceHeading).getRadians(); // shortest signed diff
        double omega = kP_ROT * errRad;

        // Clamp ω
        if (omega >  MAX_OMEGA_RAD) omega =  MAX_OMEGA_RAD;
        if (omega < -MAX_OMEGA_RAD) omega = -MAX_OMEGA_RAD;

        // If your chassis spins the wrong direction, flip once here:
        // omega = -omega;

        // Rotate in place using YOUR API
        drivetrain.setControl(
            drivetrain.m_pathApplyRobotSpeeds.withSpeeds(
                new ChassisSpeeds(0.0, 0.0, omega))
        );

        // Logs
        InitLogger.logDouble("FaceTag", "robotHeadingDeg", cur.getDegrees());
        InitLogger.logDouble("FaceTag", "headingErrDeg", Math.toDegrees(errRad));
        InitLogger.logDouble("FaceTag", "omegaCmd", omega);

        // Exit condition for ROTATE
        double errDeg = wrapDeg(cur.minus(faceHeading).getDegrees());
        boolean aligned = Math.abs(errDeg) <= TOL_DEG;
        boolean timedOut = timer.hasElapsed(ROTATE_TIMEOUT_SEC);

        InitLogger.logBoolean("FaceTag", "rotate.aligned", aligned);
        InitLogger.logBoolean("FaceTag", "rotate.timeout", timedOut);

        if (aligned || timedOut) {
          // Stop rotation before starting path
          drivetrain.setControl(
              drivetrain.m_pathApplyRobotSpeeds.withSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0))
          );
          // Kick off pathfind
          if (goalPose != null) {
            pathCmd = AutoBuilder.pathfindToPose(goalPose, constraints, 0.0);
            if (pathCmd != null) {
              pathCmd.schedule();
              InitLogger.logBoolean("FaceTag", "path.scheduled", true);
              phase = Phase.DRIVE;
            } else {
              InitLogger.logMessage("FaceTag", InitLogger.Level.ERROR, "Pathfind command was null");
              phase = Phase.DONE;
            }
          } else {
            phase = Phase.DONE;
          }
        }
      }

      case DRIVE -> {
        if (pathCmd == null) { phase = Phase.DONE; return; }
        // Keep a light eye on the child status
        InitLogger.logBoolean("FaceTag", "path.isScheduled", pathCmd.isScheduled());
        InitLogger.logBoolean("FaceTag", "path.isFinished", pathCmd.isFinished());
        if (pathCmd.isFinished()) {
          phase = Phase.DONE;
        }
      }

      case DONE -> { /* nothing */ }
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Cancel child so releasing button stops motion immediately
    if (pathCmd != null) pathCmd.cancel();
    // Hard stop to be safe
    drivetrain.setControl(
        drivetrain.m_pathApplyRobotSpeeds.withSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0))
    );
    InitLogger.logMessage("FaceTag", InitLogger.Level.INFO,
        interrupted ? "Face+Path CANCEL" : "Face+Path DONE");
  }

  @Override
  public boolean isFinished() {
    return phase == Phase.DONE;
  }

  // ----------------- helpers -----------------

  private static double wrapDeg(double deg) {
    double d = deg % 360.0;
    if (d <= -180.0) d += 360.0;
    if (d > 180.0) d -= 360.0;
    return d;
  }

  private static double angDiffDeg(double a, double b) { return wrapDeg(a - b); }

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
}
