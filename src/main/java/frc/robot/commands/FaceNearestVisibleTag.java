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

public class FaceNearestVisibleTag extends Command {
  // Allowed field headings (deg) for your tags
  private static final double[] FIELD_TAG_HEADINGS_DEG = { 0, 60, 120, 180, -60, -120 };

  private final CommandSwerveDrivetrain drivetrain;
  private final Supplier<Pose2d> robotPose;
  private final String[] cams;
  private final PathConstraints constraints;

  private edu.wpi.first.wpilibj2.command.Command pathCmd = null;

  public FaceNearestVisibleTag(
      CommandSwerveDrivetrain drivetrain,
      Supplier<Pose2d> robotPose,
      PathConstraints constraints,                 // pass your desired constraints
      String... limelightNames                     // "limelight-left", "limelight-right"
  ) {
    this.drivetrain = drivetrain;
    this.robotPose = robotPose;
    this.constraints = (constraints != null)
        ? constraints
        : new PathConstraints(2.0, 2.0, 3.0, 3.0); // safe default
    this.cams = (limelightNames != null && limelightNames.length > 0)
        ? limelightNames
        : new String[] { "limelight-left", "limelight-right" };

    // IMPORTANT: do NOT addRequirements(drivetrain) here,
    // because we'll schedule a child command that requires drivetrain.
    setName("FaceNearestVisibleTag+Pathfind");
  }

  @Override
  public void initialize() {
    // 1) Find nearest visible tag (robot-space)
    Pose3d bestRS = null;
    String usedCam = null;
    double bestRange = Double.POSITIVE_INFINITY;

    Pose2d pose = robotPose.get();
    Rotation2d robotHdg = pose.getRotation();

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
          bestRange = range; bestRS = rs; usedCam = name;
        }
      } catch (Throwable t) {
        InitLogger.logMessage("FaceTag." + name, InitLogger.Level.ERROR,
            "RobotSpace read exception: " + t.getMessage());
      }
    }

    if (bestRS == null) {
      InitLogger.logMessage("FaceTag", InitLogger.Level.WARN, "No visible tag on any camera");
      // Nothing to do: finish immediately
      pathCmd = null;
      return;
    }

    // 2) Compute tag FIELD position = robotXY + rotate((dx,dy)_robot, robotHeading)
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
    Rotation2d faceHeading = Rotation2d.fromDegrees(desiredSnappedDeg);

    InitLogger.logDouble("FaceTag", "robotHeadingDeg", robotHdg.getDegrees());
    InitLogger.logDouble("FaceTag", "tagYawRobotDeg", tagYawRobotDeg);
    InitLogger.logDouble("FaceTag", "desiredRawDeg", desiredRawDeg);
    InitLogger.logDouble("FaceTag", "desiredSnappedDeg", desiredSnappedDeg);

    // 4) Build "where" pose shifted by your constants (forward along facing, right is robot's right)
    double fwd = Constants.forwardOffset;  // meters
    double rgt = Constants.rightOffset;    // meters

    // Unit vectors for forward (heading) and right (heading - 90°)
    Translation2d fwdVec = new Translation2d(cos(faceHeading.getRadians()), sin(faceHeading.getRadians()));
    Translation2d rgtVec = new Translation2d(sin(faceHeading.getRadians()), -cos(faceHeading.getRadians()));

    Translation2d goalXY = tagField.plus(fwdVec.times(fwd)).plus(rgtVec.times(rgt));
    Pose2d where = new Pose2d(goalXY, faceHeading);

    InitLogger.logDouble("FaceTag", "goal.x", where.getX());
    InitLogger.logDouble("FaceTag", "goal.y", where.getY());
    InitLogger.logDouble("FaceTag", "goal.headingDeg", faceHeading.getDegrees());

    // 5) Create & schedule PathPlanner command
    pathCmd = AutoBuilder.pathfindToPose(where, constraints, 0.0);
    if (pathCmd != null) {
      pathCmd.schedule();
      InitLogger.logMessage("FaceTag", InitLogger.Level.INFO, "Pathfind scheduled");
    } else {
      InitLogger.logMessage("FaceTag", InitLogger.Level.ERROR, "Pathfind command was null");
    }
  }

  @Override
  public void execute() {
    // Nothing to do here; AutoBuilder child command is doing the driving.
    // (Optional: you could monitor and log here if desired.)
  }

  @Override
  public void end(boolean interrupted) {
    // Cancel the child so releasing button stops motion
    if (pathCmd != null) pathCmd.cancel();

    // Hard stop to be safe (uses your API)
    drivetrain.setControl(
        drivetrain.m_pathApplyRobotSpeeds.withSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0))
    );

    InitLogger.logMessage("FaceTag", InitLogger.Level.INFO,
        interrupted ? "Face+Path CANCEL" : "Face+Path DONE");
  }

  @Override
  public boolean isFinished() {
    // Finish if the child path command is done (or we never started it because no tag)
    return pathCmd == null || pathCmd.isFinished();
  }

  // ----------------- helpers -----------------

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
}
