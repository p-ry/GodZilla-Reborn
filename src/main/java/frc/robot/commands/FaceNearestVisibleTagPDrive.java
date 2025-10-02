package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import frc.robot.Constants;
import frc.robot.InitLogger;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.function.Supplier;

import static java.lang.Math.*;

/**
 * Single-phase proportional drive to face the tag (NO tag yaw usage).
 * - Nearest visible tag from Limelight ROBOT-SPACE pose.
 * - Desired heading = robotHeading + atan2(dy_robot, dx_robot)  (bearing to tag).
 * - Goal pose = tag field position + (forwardOffset, rightOffset) in the facing frame.
 * - P-control on field X, field Y and heading; converted via fromFieldRelativeSpeeds().
 */
public class FaceNearestVisibleTagPDrive extends Command {
  // Tolerances & timeout
  private static final double POS_TOL_M    = 0.05;                 // 5 cm
  private static final double HDG_TOL_DEG  = 3.0;                  // 3°
  private static final double TIMEOUT_S    = 6.0;                  // safety

  // Proportional gains (tune as needed)
  private static final double kVX   = 1.6;     // m/s per m error (field X)
  private static final double kVY   = 1.6;     // m/s per m error (field Y)
  private static final double kOMEGA= 2.2;     // rad/s per rad error (heading)

  // Speed caps
  private static final double MAX_V_MPS     = 2.0;
  private static final double MAX_OMEGA_RAD = Math.toRadians(180.0);

  // Optional polarity flips if your drive conventions differ
  private static final boolean INVERT_TRANSL = false;
  private static final boolean INVERT_OMEGA  = false;

  private final CommandSwerveDrivetrain drivetrain;
  private final Supplier<Pose2d> robotPose;
  private final String[] cams;

  private Pose2d goalPose = null;
  private Rotation2d faceHeading = null;

  private final edu.wpi.first.wpilibj.Timer timer = new edu.wpi.first.wpilibj.Timer();

  public FaceNearestVisibleTagPDrive(
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
    setName("FaceNearestVisibleTagPDrive");
  }

  @Override
  public void initialize() {
    timer.restart();
    goalPose = null;
    faceHeading = null;

    // --- Pick nearest visible tag (robot-space) ---
    Pose2d pose = robotPose.get();
    Rotation2d robotHdg = pose.getRotation();

    Pose3d bestRS = null;
    String usedCam = null;
    double bestRange = Double.POSITIVE_INFINITY;

    for (String name : cams) {
      try {
        boolean tv = LimelightHelpers.getTV(name);
        InitLogger.logBoolean("FaceP." + name, "hasTarget", tv);
        if (!tv) continue;

        Pose3d rs = LimelightHelpers.getTargetPose3d_RobotSpace(name);
        if (rs == null) continue;

        double dx_r = rs.getX();  // +X forward (robot)
        double dy_r = rs.getY();  // +Y left (robot)
        double range = hypot(dx_r, dy_r);
        InitLogger.logDouble("FaceP." + name, "dxRobot", dx_r);
        InitLogger.logDouble("FaceP." + name, "dyRobot", dy_r);
        InitLogger.logDouble("FaceP." + name, "range", range);

        if (range < bestRange) {
          bestRange = range; bestRS = rs; usedCam = name;
        }
      } catch (Throwable t) {
        InitLogger.logMessage("FaceP." + name, InitLogger.Level.ERROR,
            "RobotSpace read exception: " + t.getMessage());
      }
    }

    if (bestRS == null) {
      InitLogger.logMessage("FaceP", InitLogger.Level.WARN, "No visible tag on any camera");
      return; // no goal → command will end immediately
    }

    // --- Bearing to tag (ROBOT frame) → absolute desired heading (FIELD frame) ---
    double dx_r = bestRS.getX();
    double dy_r = bestRS.getY();
    double bearingRobotToTagRad = atan2(dy_r, dx_r); // + left, - right in robot frame
    double desiredHeadingRad = wrapRad(robotHdg.getRadians() + bearingRobotToTagRad);
    faceHeading = new Rotation2d(desiredHeadingRad);

    InitLogger.logMessage("FaceP", InitLogger.Level.INFO, "Using camera=" + usedCam);
    InitLogger.logDouble("FaceP", "robotHeadingDeg", robotHdg.getDegrees());
    InitLogger.logDouble("FaceP", "bearingRobotToTagDeg", Math.toDegrees(bearingRobotToTagRad));
    InitLogger.logDouble("FaceP", "desiredHeadingDeg", faceHeading.getDegrees());

    // --- Compute tag FIELD position: robotXY + rotate((dx,dy)_robot, robotHeading) ---
    Translation2d robotToTag_robot = new Translation2d(dx_r, dy_r);
    Translation2d robotToTag_field = robotToTag_robot.rotateBy(robotHdg);
    Translation2d tagField = pose.getTranslation().plus(robotToTag_field);

    InitLogger.logDouble("FaceP", "tagField.x", tagField.getX());
    InitLogger.logDouble("FaceP", "tagField.y", tagField.getY());

    // --- Goal pose with offsets in the FACING frame (toward tag) ---
    double fwd = Constants.forwardOffset;   // meters along faceHeading
    double rgt = Constants.rightOffset;     // meters to robot's right relative to faceHeading
    Translation2d fwdVec = new Translation2d(cos(desiredHeadingRad), sin(desiredHeadingRad));
    Translation2d rgtVec = new Translation2d(sin(desiredHeadingRad), -cos(desiredHeadingRad));
    Translation2d goalXY = tagField.plus(fwdVec.times(fwd)).plus(rgtVec.times(rgt));
    goalPose = new Pose2d(goalXY, faceHeading);

    InitLogger.logDouble("FaceP", "goal.x", goalPose.getX());
    InitLogger.logDouble("FaceP", "goal.y", goalPose.getY());
    InitLogger.logDouble("FaceP", "goal.headingDeg", faceHeading.getDegrees());
  }

  @Override
  public void execute() {
    if (goalPose == null || faceHeading == null) {
      // No target → stop and end
      drivetrain.setControl(
          drivetrain.m_pathApplyRobotSpeeds.withSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0))
      );
      return;
    }

    // Current pose
    Pose2d pose = robotPose.get();
    double curX = pose.getX();
    double curY = pose.getY();
    double curHdgRad = pose.getRotation().getRadians();

    // Field-frame errors (desired − current)
    double ex = goalPose.getX() - curX;                        // + if goal is east of robot
    double ey = goalPose.getY() - curY;                        // + if goal is north of robot
    double eh = wrapRad(faceHeading.getRadians() - curHdgRad); // wrapped heading error

    // P-control in FIELD frame
    double vxField = kVX * ex;
    double vyField = kVY * ey;
    double omega   = kOMEGA * eh;

    // Clamp speeds
    double mag = hypot(vxField, vyField);
    if (mag > MAX_V_MPS) {
      double s = MAX_V_MPS / (mag + 1e-9);
      vxField *= s; vyField *= s;
    }
    if (omega >  MAX_OMEGA_RAD) omega =  MAX_OMEGA_RAD;
    if (omega < -MAX_OMEGA_RAD) omega = -MAX_OMEGA_RAD;

    // Optional flips (use only if your chassis sign differs)
    if (INVERT_TRANSL) { vxField = -vxField; vyField = -vyField; }
    if (INVERT_OMEGA)  { omega   = -omega; }

    // Convert FIELD → ROBOT (safer than manual rotation)
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        vxField, vyField, omega, pose.getRotation()
    );

    // Drive
    drivetrain.setControl(
        drivetrain.m_pathApplyRobotSpeeds.withSpeeds(speeds)
    );

    // Finish conditions
    double posErr = hypot(ex, ey);
    double hdgErrDeg = Math.toDegrees(eh);
    boolean atPos = posErr <= POS_TOL_M;
    boolean atHdg = abs(hdgErrDeg) <= HDG_TOL_DEG;
    boolean timedOut = timer.hasElapsed(TIMEOUT_S);

    // Logs
    InitLogger.logDouble("FaceP", "drive.ex", ex);
    InitLogger.logDouble("FaceP", "drive.ey", ey);
    InitLogger.logDouble("FaceP", "drive.ehDeg", hdgErrDeg);
    InitLogger.logDouble("FaceP", "drive.vxField", vxField);
    InitLogger.logDouble("FaceP", "drive.vyField", vyField);
    InitLogger.logDouble("FaceP", "drive.omega", omega);
    InitLogger.logDouble("FaceP", "drive.posErr", posErr);
    InitLogger.logBoolean("FaceP", "drive.atPos", atPos);
    InitLogger.logBoolean("FaceP", "drive.atHdg", atHdg);
    InitLogger.logBoolean("FaceP", "drive.timeout", timedOut);

    if ((atPos && atHdg) || timedOut) {
      this.cancel(); // end() stops motion
    }
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(
        drivetrain.m_pathApplyRobotSpeeds.withSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0))
    );
    InitLogger.logMessage("FaceP", InitLogger.Level.INFO,
        interrupted ? "P-Drive CANCEL" : "P-Drive DONE");
  }

  @Override
  public boolean isFinished() {
    return goalPose == null || faceHeading == null;
  }

  // ----------------- helpers -----------------
  private static double wrapRad(double rad) {
    double r = rad % (2.0 * Math.PI);
    if (r <= -Math.PI) r += 2.0 * Math.PI;
    if (r >  Math.PI)  r -= 2.0 * Math.PI;
    return r;
  }
}
