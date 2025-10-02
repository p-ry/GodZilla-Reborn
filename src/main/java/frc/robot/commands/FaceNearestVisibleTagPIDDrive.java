package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
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
 * Single-phase PID drive (no PathPlanner, no rotate-first):
 *  - Compute nearest visible tag from Limelight robot-space data.
 *  - Build facing heading (snapped to {0, ±60, ±120, 180}).
 *  - Build goal pose with (forwardOffset, rightOffset) in the facing frame.
 *  - Run PIDs on field X, field Y, and heading simultaneously.
 *  - Convert to robot-relative with ChassisSpeeds.fromFieldRelativeSpeeds(...).
 */
public class FaceNearestVisibleTagPIDDrive extends Command {
  // Snap-to headings for your tags (deg)
  private static final double[] FIELD_TAG_HEADINGS_DEG = { 0, 60, 120, 180, -60, -120 };

  // Tolerances & timeouts
  private static final double POS_TOL_M   = 0.05;                // 5 cm
  private static final double HDG_TOL_DEG = 3.0;                 // 3 deg
  private static final double DRIVE_TIMEOUT_S = 6.0;             // safety timeout

  // PID gains (tune down if oscillation)
  private static final double kP_X   = 1.8;      // field X
  private static final double kP_Y   = 1.8;      // field Y
  private static final double kP_HDG = 2.2;      // heading (radians domain)

  // Speed clamps
  private static final double MAX_V_MPS   = 2.0;                   // linear speed cap
  private static final double MAX_OMEGA_R = Math.toRadians(180.0); // angular speed cap

  // If rotation goes the "wrong" direction with your drive, flip once here.
  private static final boolean INVERT_OMEGA = true;
  // If translation feels reversed due to unusual drive conventions, flip here (rare).
  private static final boolean INVERT_TRANSL = false;

  private final CommandSwerveDrivetrain drivetrain;
  private final Supplier<Pose2d> robotPose;
  private final String[] cams;

  // Latched targets
  private Pose2d goalPose = null;
  private Rotation2d faceHeading = null;

  // Controllers
  private final PIDController xPid = new PIDController(kP_X, 0.0, 0.0);
  private final PIDController yPid = new PIDController(kP_Y, 0.0, 0.0);
  private final PIDController hdgPid = new PIDController(kP_HDG, 0.0, 0.0);

  private final edu.wpi.first.wpilibj.Timer timer = new edu.wpi.first.wpilibj.Timer();

  public FaceNearestVisibleTagPIDDrive(
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
    setName("FaceNearestVisibleTagPIDDrive");

    // Heading PID is angular; enable wrapping
    hdgPid.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    timer.restart();
    goalPose = null;
    faceHeading = null;

    // --- Nearest visible tag from robot-space ---
    Pose2d pose = robotPose.get();
    Rotation2d robotHdg = pose.getRotation();

    Pose3d bestRS = null;
    String usedCam = null;
    double bestRange = Double.POSITIVE_INFINITY;

    for (String name : cams) {
      try {
        boolean tv = LimelightHelpers.getTV(name);
        InitLogger.logBoolean("FacePID." + name, "hasTarget", tv);
        if (!tv) continue;

        Pose3d rs = LimelightHelpers.getTargetPose3d_RobotSpace(name);
        if (rs == null) continue;

        double dx = rs.getX(), dy = rs.getY();      // +X fwd, +Y left (robot frame)
        double range = hypot(dx, dy);
        InitLogger.logDouble("FacePID." + name, "dxRobot", dx);
        InitLogger.logDouble("FacePID." + name, "dyRobot", dy);
        InitLogger.logDouble("FacePID." + name, "range", range);

        if (range < bestRange) {
          bestRange = range; bestRS = rs; usedCam = name;
        }
      } catch (Throwable t) {
        InitLogger.logMessage("FacePID." + name, InitLogger.Level.ERROR,
            "RobotSpace read exception: " + t.getMessage());
      }
    }

    if (bestRS == null) {
      InitLogger.logMessage("FacePID", InitLogger.Level.WARN, "No visible tag on any camera");
      return; // goalPose stays null → command will end immediately
    }

    // --- Tag field position = robotXY + rotate((dx,dy)_robot, robotHeading) ---
    Translation2d robotToTag_robot = new Translation2d(bestRS.getX(), bestRS.getY());
    Translation2d robotToTag_field = robotToTag_robot.rotateBy(robotHdg);
    Translation2d tagField = pose.getTranslation().plus(robotToTag_field);

    InitLogger.logMessage("FacePID", InitLogger.Level.INFO, "Using camera=" + usedCam);
    InitLogger.logDouble("FacePID", "robot.x", pose.getX());
    InitLogger.logDouble("FacePID", "robot.y", pose.getY());
    InitLogger.logDouble("FacePID", "tagField.x", tagField.getX());
    InitLogger.logDouble("FacePID", "tagField.y", tagField.getY());

    // --- Facing heading from ROBOT-space tag yaw, snapped to known set ---
    double tagYawRobotDeg = wrapDeg(Math.toDegrees(bestRS.getRotation().getZ()));
    double desiredRawDeg   = wrapDeg(robotHdg.getDegrees() + wrapDeg(tagYawRobotDeg + 180.0));
    double desiredSnapDeg  = snapToFieldHeadings(desiredRawDeg);
    faceHeading = Rotation2d.fromDegrees(desiredSnapDeg);

    InitLogger.logDouble("FacePID", "robotHeadingDeg", robotHdg.getDegrees());
    InitLogger.logDouble("FacePID", "tagYawRobotDeg", tagYawRobotDeg);
    InitLogger.logDouble("FacePID", "desiredRawDeg", desiredRawDeg);
    InitLogger.logDouble("FacePID", "desiredSnappedDeg", desiredSnapDeg);

    // --- Goal pose with offsets in the facing frame ---
    double fwd = Constants.forwardOffset;  // meters
    double rgt = Constants.rightOffset;    // meters

    Translation2d fwdVec = new Translation2d(cos(faceHeading.getRadians()), sin(faceHeading.getRadians()));
    Translation2d rgtVec = new Translation2d(sin(faceHeading.getRadians()), -cos(faceHeading.getRadians()));
    Translation2d goalXY = tagField.plus(fwdVec.times(fwd)).plus(rgtVec.times(rgt));
    goalPose = new Pose2d(goalXY, faceHeading);

    InitLogger.logDouble("FacePID", "goal.x", goalPose.getX());
    InitLogger.logDouble("FacePID", "goal.y", goalPose.getY());
    InitLogger.logDouble("FacePID", "goal.headingDeg", faceHeading.getDegrees());

    // --- Configure PIDs: setpoints are GOAL, input is CURRENT ---
    xPid.reset(); yPid.reset(); hdgPid.reset();
    xPid.setSetpoint(goalPose.getX());                     // field X setpoint
    yPid.setSetpoint(goalPose.getY());                     // field Y setpoint
    hdgPid.setSetpoint(faceHeading.getRadians());          // heading setpoint (radians)
  }

  @Override
  public void execute() {
    if (goalPose == null || faceHeading == null) {
      // No target; stop and end
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

    // PID in FIELD frame (standard: calculate(measurement) with setpoint already set)
    double vxField = xPid.calculate(curX);           // + when goalX > curX
    double vyField = yPid.calculate(curY);           // + when goalY > curY
    double omega   = hdgPid.calculate(curHdgRad);    // + when target heading > current (wrapped)

    // Clamp linear speed
    double mag = hypot(vxField, vyField);
    if (mag > MAX_V_MPS) {
      double s = MAX_V_MPS / (mag + 1e-9);
      vxField *= s; vyField *= s;
    }

    // Clamp omega
    if (omega >  MAX_OMEGA_R) omega =  MAX_OMEGA_R;
    if (omega < -MAX_OMEGA_R) omega = -MAX_OMEGA_R;

    // Optional polarity flips (rarely needed, set booleans above)
    if (INVERT_TRANSL) { vxField = -vxField; vyField = -vyField; }
    if (INVERT_OMEGA)   { omega   = -omega; }

    // Convert FIELD → ROBOT using WPILib helper (avoids rotation/sign mistakes)
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        vxField, vyField, omega, pose.getRotation()
    );

    // Drive
    drivetrain.setControl(
        drivetrain.m_pathApplyRobotSpeeds.withSpeeds(speeds)
    );

    // Finish conditions
    double ex = goalPose.getX() - curX;
    double ey = goalPose.getY() - curY;
    double posErr = hypot(ex, ey);
    double hdgErrDeg = wrapDeg(Math.toDegrees(hdgPid.getSetpoint() - curHdgRad));
    boolean atPos = posErr <= POS_TOL_M;
    boolean atHdg = abs(hdgErrDeg) <= HDG_TOL_DEG;
    boolean timedOut = timer.hasElapsed(DRIVE_TIMEOUT_S);

    // Logs
    InitLogger.logDouble("FacePID", "drive.ex", ex);
    InitLogger.logDouble("FacePID", "drive.ey", ey);
    InitLogger.logDouble("FacePID", "drive.vxField", vxField);
    InitLogger.logDouble("FacePID", "drive.vyField", vyField);
    InitLogger.logDouble("FacePID", "drive.omega", omega);
    InitLogger.logDouble("FacePID", "drive.posErr", posErr);
    InitLogger.logDouble("FacePID", "drive.hdgErrDeg", hdgErrDeg);
    InitLogger.logBoolean("FacePID", "drive.atPos", atPos);
    InitLogger.logBoolean("FacePID", "drive.atHdg", atHdg);
    InitLogger.logBoolean("FacePID", "drive.timeout", timedOut);

    if ((atPos && atHdg) || timedOut) {
      // stop in end()
      this.cancel();
    }
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(
        drivetrain.m_pathApplyRobotSpeeds.withSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0))
    );
    InitLogger.logMessage("FacePID", InitLogger.Level.INFO,
        interrupted ? "PID Drive CANCEL" : "PID Drive DONE");
  }

  @Override
  public boolean isFinished() {
    // We call cancel() on finish conditions inside execute(), so here we just check timer/goal presence.
    return goalPose == null || faceHeading == null;
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
    double bestAbs = abs(angDiffDeg(deg, best));
    for (int i = 1; i < FIELD_TAG_HEADINGS_DEG.length; i++) {
      double cand = FIELD_TAG_HEADINGS_DEG[i];
      double err = abs(angDiffDeg(deg, cand));
      if (err < bestAbs) { bestAbs = err; best = cand; }
    }
    return wrapDeg(best);
  }
}
