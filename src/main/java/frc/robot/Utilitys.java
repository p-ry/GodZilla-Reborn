// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.Utilitys.DriveToOptions;
import frc.robot.Utilitys.HeadingStrategy;
import frc.robot.Utilitys.TagMeasurement;
import frc.robot.Utilitys.VisibleTagMeasurementSupplier;
//import frc.robot.VisibleTagIdSupplier;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.InitLogger.Level;

import java.awt.geom.Point2D;
import static frc.robot.InitLogger.Level;
import static java.lang.Math.*;
import com.pathplanner.lib.util.PathPlannerLogging;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Commands;

import java.util.Comparator;
import edu.wpi.first.wpilibj.Timer;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import java.util.stream.IntStream;

/** Add your docs here. */
public class Utilitys {

    public Utilitys() {
    }

    /**
     * Supplier of currently visible AprilTag IDs (e.g., from Limelight or
     * PhotonVision).
     */

    public static LimelightHelpers.PoseEstimate mt2;
    public LimelightHelpers.PoseEstimate leftPose;
    public LimelightHelpers.PoseEstimate rightPose;
    public LimelightHelpers.PoseEstimate[] cameraPoses = new LimelightHelpers.PoseEstimate[2];
    public SwerveDrivePoseEstimator m_poseEstimator;


    private static final String LOG_NAME = "Drive/TagRaw"; // single log namespace


   

    /**
     * Strategy for selecting the robot's final heading when reaching the offset.
     */
    public enum HeadingStrategy {
        KEEP_CURRENT, MATCH_TAG_YAW, FACE_TAG, EXPLICIT
    }

    /** Options for drive-to-target behavior. */
    public record DriveToOptions(
            PathConstraints constraints,
            HeadingStrategy headingStrategy,
            Rotation2d explicitHeading,
            double positionToleranceMeters,
            Rotation2d headingTolerance,
            double replanPeriodSec,
            double replanPosDeltaMeters,
            Rotation2d replanHeadingDelta) {
        public static DriveToOptions defaults() {
            return new DriveToOptions(
                    new PathConstraints(2.0, 2.0, 3.0, 3.0),
                    HeadingStrategy.MATCH_TAG_YAW,
                    new Rotation2d(),
                    0.05,
                    Rotation2d.fromDegrees(3),
                    0.30,
                    0.10,
                    Rotation2d.fromDegrees(5));
        }
    }
  // ... keep your existing code ...

  // NEW: minimal raw-measurement container (robot-frame)
  public record TagMeasurement(int id, double dxRobot, double dyRobot) {}

  // NEW: supplier of current visible tag measurements (from Limelight adapter)
  @FunctionalInterface
  public interface VisibleTagMeasurementSupplier { List<TagMeasurement> get(); }

  // NEW: if you already have DriveToOptions, reuse it; otherwise keep as-is.
  // (No changes needed to your existing DriveToOptions)

  // NEW: raw-camera overload (does NOT use AprilTagFieldLayout at all)
  public static Command driveToDxDyFromNearestTagRaw(
    CommandSwerveDrivetrain drivetrain,
    java.util.function.Supplier<edu.wpi.first.math.geometry.Pose2d> robotPose,
    VisibleTagMeasurementSupplier visibleMeas,
    double dxMeters,
    double dyMeters,
    DriveToOptions options) {

  final DriveToOptions opts = (options == null) ? DriveToOptions.defaults() : options;

  // Computes the latest target pose from raw camera data
  java.util.function.Supplier<edu.wpi.first.math.geometry.Pose2d> computeTarget =
      () -> computeDxDyTargetFromRaw(
          robotPose.get(), visibleMeas.get(), dxMeters, dyMeters, opts);

  // Local state
  class State { edu.wpi.first.math.geometry.Pose2d lastTarget = null; Command active = null; double lastCheckTime = 0; }
  State state = new State();

  // === FINISH CONDITION (this runs every loop; we can log errors here) ===
  java.util.function.BooleanSupplier atGoal = () -> {
    var cur = robotPose.get();
    var tgt = computeTarget.get();

    double posErr = cur.getTranslation().getDistance(tgt.getTranslation());
    double degErr = Math.abs(cur.getRotation().minus(tgt.getRotation()).getDegrees());

    // LOG: finish condition errors
    InitLogger.logDouble(LOG_NAME, "error/pos", posErr);
    InitLogger.logDouble(LOG_NAME, "error/deg", degErr);

    boolean done = (posErr <= opts.positionToleranceMeters())
                && (degErr <= opts.headingTolerance().getDegrees());

    InitLogger.logBoolean(LOG_NAME, "atGoal", done);
    return done;
  };

  // === INIT (happens once when the command starts) ===
  Command initAndPlan = edu.wpi.first.wpilibj2.command.Commands.runOnce(() -> {
    var tgt = computeTarget.get();

    // LOG: initial plan target
    InitLogger.logMessage(LOG_NAME, Level.INFO, "Initial pathfind to computed target");
    InitLogger.logDouble(LOG_NAME, "init/target/x", tgt.getX());
    InitLogger.logDouble(LOG_NAME, "init/target/y", tgt.getY());
    InitLogger.logDouble(LOG_NAME, "init/target/headingDeg", tgt.getRotation().getDegrees());

    state.lastTarget = tgt;
    state.active = AutoBuilder.pathfindToPose(tgt, opts.constraints());
    state.active.schedule();
  }, drivetrain);

  // === PERIODIC REPLAN (runs while command is active) ===
  Command periodicReplan = edu.wpi.first.wpilibj2.command.Commands.run(() -> {
    double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    if (now - state.lastCheckTime < opts.replanPeriodSec()) return;
    state.lastCheckTime = now;

    var newTarget = computeTarget.get();
    if (state.lastTarget == null) { state.lastTarget = newTarget; return; }

    double dPos = state.lastTarget.getTranslation().getDistance(newTarget.getTranslation());
    double dDeg = Math.abs(state.lastTarget.getRotation().minus(newTarget.getRotation()).getDegrees());

    // LOG: replan deltas
    InitLogger.logDouble(LOG_NAME, "delta/pos", dPos);
    InitLogger.logDouble(LOG_NAME, "delta/deg", dDeg);

    boolean needsReplan = (dPos > opts.replanPosDeltaMeters())
                       || (dDeg > opts.replanHeadingDelta().getDegrees());

    InitLogger.logBoolean(LOG_NAME, "replan/needed", needsReplan);

    if (needsReplan) {
      InitLogger.logMessage(LOG_NAME, Level.INFO,
          "Replan: dPos=" + dPos + " dDeg=" + dDeg);

      if (state.active != null) state.active.cancel();
      state.active = AutoBuilder.pathfindToPose(newTarget, opts.constraints());
      state.active.schedule();
      state.lastTarget = newTarget;

      // LOG: replan target
      InitLogger.logDouble(LOG_NAME, "replan/target/x", newTarget.getX());
      InitLogger.logDouble(LOG_NAME, "replan/target/y", newTarget.getY());
      InitLogger.logDouble(LOG_NAME, "replan/target/headingDeg", newTarget.getRotation().getDegrees());
    }
  }, drivetrain);

  Command finishWhenAtGoal = edu.wpi.first.wpilibj2.command.Commands.waitUntil(atGoal);

  // Optional: one log when we finish
  Command onFinish = edu.wpi.first.wpilibj2.command.Commands.runOnce(() -> {
    InitLogger.logMessage(LOG_NAME, Level.INFO, "At goal; stopping command");
  });

  Command cleanup = edu.wpi.first.wpilibj2.command.Commands.runOnce(() -> {
    if (state.active != null) state.active.cancel();
  }, drivetrain);

  return initAndPlan.andThen(periodicReplan.until(atGoal)).andThen(onFinish).andThen(cleanup);
}

  // NEW: compute target from raw robot-frame measurements (no field layout)
  private static Pose2d computeDxDyTargetFromRaw(
    Pose2d robotFieldPose,
    List<TagMeasurement> meas,
    double dxMeters,
    double dyMeters,
    DriveToOptions opts) {

  // --- INPUT LOGGING ---
  InitLogger.logDouble(LOG_NAME, "visibleCount", (meas == null) ? 0 : meas.size());
  if (meas == null || meas.isEmpty()) {
    InitLogger.logMessage(LOG_NAME, Level.WARN, "No visible tags; returning robot pose");
    return robotFieldPose;
  }
  // Log each visible measurement (first few to avoid spam)
  int idx = 0;
  for (TagMeasurement m : meas) {
    if (idx++ >= 4) break; // keep it light
    double rng = hypot(m.dxRobot, m.dyRobot);
    InitLogger.logDouble(LOG_NAME, "meas" + idx + "/dxRobot", m.dxRobot);
    InitLogger.logDouble(LOG_NAME, "meas" + idx + "/dyRobot", m.dyRobot);
    InitLogger.logDouble(LOG_NAME, "meas" + idx + "/range", rng);
    InitLogger.logDouble(LOG_NAME, "meas" + idx + "/id", m.id);
  }

  // 1) Nearest visible tag in ROBOT frame
  TagMeasurement nearest = meas.stream()
      .min(java.util.Comparator.comparingDouble(m -> m.dxRobot * m.dxRobot + m.dyRobot * m.dyRobot))
      .orElse(meas.get(0));
  InitLogger.logDouble(LOG_NAME, "chosen/id", nearest.id);
  InitLogger.logDouble(LOG_NAME, "chosen/dxRobot", nearest.dxRobot);
  InitLogger.logDouble(LOG_NAME, "chosen/dyRobot", nearest.dyRobot);
  InitLogger.logDouble(LOG_NAME, "chosen/range", hypot(nearest.dxRobot, nearest.dyRobot));

  // Robot pose
  InitLogger.logDouble(LOG_NAME, "robot/x", robotFieldPose.getX());
  InitLogger.logDouble(LOG_NAME, "robot/y", robotFieldPose.getY());
  InitLogger.logDouble(LOG_NAME, "robot/headingDeg", robotFieldPose.getRotation().getDegrees());

  // 2) Convert tag ROBOT->FIELD
  Translation2d robotToTag_robot = new Translation2d(nearest.dxRobot, nearest.dyRobot);
  Translation2d robotToTag_field = robotToTag_robot.rotateBy(robotFieldPose.getRotation());
  Translation2d tagFieldTranslation = robotFieldPose.getTranslation().plus(robotToTag_field);
  InitLogger.logDouble(LOG_NAME, "tagField/x", tagFieldTranslation.getX());
  InitLogger.logDouble(LOG_NAME, "tagField/y", tagFieldTranslation.getY());

  // 3) APPROACH (+X tag->robot, +Y left)
  Translation2d tagToRobot_field = robotFieldPose.getTranslation().minus(tagFieldTranslation);
  double approachTheta = atan2(tagToRobot_field.getY(), tagToRobot_field.getX());
  Rotation2d approachRot = new Rotation2d(approachTheta);
  InitLogger.logDouble(LOG_NAME, "approach/deg", toDegrees(approachTheta));

  Translation2d offsetApproach = new Translation2d(dxMeters, dyMeters);
  Translation2d offsetField = offsetApproach.rotateBy(approachRot);
  Translation2d goalFieldTranslation = tagFieldTranslation.plus(offsetField);

  // 4) Final heading
  Rotation2d goalHeading = switch (opts.headingStrategy()) {
    case KEEP_CURRENT -> robotFieldPose.getRotation();
    case MATCH_TAG_YAW -> approachRot; // proxy without field layout yaw
    case FACE_TAG -> new Rotation2d(atan2(
        tagFieldTranslation.getY() - goalFieldTranslation.getY(),
        tagFieldTranslation.getX() - goalFieldTranslation.getX()));
    case EXPLICIT -> opts.explicitHeading();
  };

  // --- OUTPUT LOGGING ---
  InitLogger.logDouble(LOG_NAME, "goal/x", goalFieldTranslation.getX());
  InitLogger.logDouble(LOG_NAME, "goal/y", goalFieldTranslation.getY());
  InitLogger.logDouble(LOG_NAME, "goal/headingDeg", goalHeading.getDegrees());
  InitLogger.logMessage(LOG_NAME, Level.INFO, "Computed new target from RAW camera dx/dy");

  return new Pose2d(goalFieldTranslation, goalHeading);
}

  // ... keep your existing methods/overloads untouched ...


    public static Pose2d shiftPoseLeft(Pose2d originalPose, double forwardInches, double rightInches) {
        // Get current pose components
        double x = originalPose.getX();
        double y = originalPose.getY();
        Rotation2d theta = originalPose.getRotation();
        Rotation2d invTheta = theta.fromRadians(theta.getRadians() + Math.PI);

        // Compute new coordinates
        // Convert inches to meters (WPILib uses meters)
        double forwardMeters = Units.inchesToMeters(forwardInches);
        double rightMeters = Units.inchesToMeters(rightInches);

        // Calculate new position shift target to it's right
        double xNew = x + forwardMeters * Math.cos(theta.getRadians()) + rightMeters * Math.sin(theta.getRadians());
        double yNew = y + forwardMeters * Math.sin(theta.getRadians()) - rightMeters * Math.cos(theta.getRadians());

        // Return the new pose with the same orientation
        return new Pose2d(xNew, yNew, invTheta);
    }

    public static Pose2d shiftPoseRight(Pose2d originalPose, double forwardInches, double leftInches) {
        // Get current pose components

        double x = originalPose.getX();
        double y = originalPose.getY();
        Rotation2d theta = originalPose.getRotation(); // Rotation2d object
        double forwardMeters = Units.inchesToMeters(forwardInches);
        double rightMeters = Units.inchesToMeters(leftInches);
        Rotation2d invTheta = theta.fromRadians(theta.getRadians() + Math.PI);

        // Compute new coordinates (shift left) shift target to it's left
        double xNew = x + forwardMeters * Math.cos(theta.getRadians()) - rightMeters * Math.sin(theta.getRadians());
        double yNew = y + forwardMeters * Math.sin(theta.getRadians()) + rightMeters * Math.cos(theta.getRadians());

        // Return the new pose with the same orientation
        return new Pose2d(xNew, yNew, invTheta);
    }

    public static Command driveToIt(CommandSwerveDrivetrain drivetrain, boolean right) {

        StructPublisher<Pose2d> whereToPublisher = NetworkTableInstance.getDefault()
                .getStructTopic("WhereTo", Pose2d.struct)
                .publish();
        StructPublisher<Pose2d> tagRel2DPublisher = NetworkTableInstance.getDefault()
                .getStructTopic("TagRel2d", Pose2d.struct)
                .publish();
        StructPublisher<Transform2d> transPublisher = NetworkTableInstance.getDefault()
                .getStructTopic("Transform", Transform2d.struct)
                .publish();
        StructPublisher<Pose2d> ignorePublisher = NetworkTableInstance.getDefault()
                .getStructTopic("tagPose2d", Pose2d.struct)
                .publish();
        Translation2d targetTranslation;
        PathConstraints constraints = new PathConstraints(
                2.0, 3.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));
        double leftDist = 0;
        Pose2d where;
        double rightDist = 0;
        boolean validTarget = false;
        int[] tagIds = new int[3];
        int tagId;
        LimelightHelpers.LimelightResults resultsLeft = LimelightHelpers.getLatestResults("limelight-left");

        LimelightHelpers.LimelightResults resultsRight = LimelightHelpers.getLatestResults("limelight-right");
        // LimelightHelpers.LimelightResults results =
        // LimelightHelpers.getLatestResults("limelight-left");
        Pose3d targetPose3D;
        Pose2d robotPose = drivetrain.getPose();
        where = robotPose;
        double leftAmbiguity = 0;
        double rightAmbiguity = 0;
        double yawToTagRad, desiredRotationDeg;
        Rotation2d desiredHeading;
        // double[] targetPose =
        // LimelightHelpers.getTargetPose_RobotSpace("limelight-left");
        Rotation2d tagFieldYaw, robotRelYaw;
        Transform2d robotToTag;
        Pose2d tagRel2d;
        Pose2d tagPose2d;
        RawFiducial[] fiducialsLeft;
        RawFiducial[] fiducialsRight;
        boolean algae;

        if (Constants.cameraPoses[0] != null && Constants.cameraPoses[0].rawFiducials.length > 0) {
            // fiducialsLeft = LimelightHelpers.getRawFiducials("limelight-left");
            // leftAmbiguity = fiducialsLeft[0].ambiguity;
            leftAmbiguity = Constants.cameraPoses[0].rawFiducials[0].ambiguity;
            leftDist = Constants.cameraPoses[0].rawFiducials[0].distToRobot;
            // leftDist = resultsLeft.botpose_avgdist;
            validTarget = true;
            // leftAmbiguity = resultsLeft.targets_Fiducials[0].
            // getAmbiguity().getValueAsDouble; // Ensure getAmbiguity() is a valid method
            tagIds[0] = Constants.cameraPoses[0].rawFiducials[0].id; // (int)
                                                                     // resultsLeft.targets_Fiducials[0].fiducialID;
        } else {
            leftDist = 999999;
        }
        if (Constants.cameraPoses[1] != null && Constants.cameraPoses[1].rawFiducials.length > 0) {
            // fiducialsLeft = LimelightHelpers.getRawFiducials("limelight-left");
            // leftAmbiguity = fiducialsLeft[0].ambiguity;
            rightAmbiguity = Constants.cameraPoses[1].rawFiducials[0].ambiguity;
            rightDist = Constants.cameraPoses[1].rawFiducials[0].distToRobot;
            // leftDist = resultsLeft.botpose_avgdist;
            validTarget = true;
            // leftAmbiguity = resultsLeft.targets_Fiducials[0].
            // getAmbiguity().getValueAsDouble; // Ensure getAmbiguity() is a valid method
            tagIds[1] = Constants.cameraPoses[1].rawFiducials[0].id; // (int)
                                                                     // resultsLeft.targets_Fiducials[0].fiducialID;
        } else {
            rightDist = 999999;
        }

        // if (resultsRight.valid) {
        // fiducialsRight = LimelightHelpers.getRawFiducials("limelight-right");
        // rightAmbiguity = fiducialsRight[0].ambiguity;
        // rightDist = resultsRight.botpose_avgdist;

        // tagIds[1] = (int) resultsRight.targets_Fiducials[0].fiducialID;
        // validTarget = true;
        // } else {
        // rightDist = 999999;
        // }

        // SmartDashboard.putNumber("Left C Distance",leftDist);
        // SmartDashboard.putNumber("Right C Distance",rightDist);
        Pose3d tagPose3d = LimelightHelpers.getTargetPose3d_RobotSpace("limelight-left");
        // ******* may need to tchange to constants */

        // Pose3d robotPoseTargetSpacePose3d =
        // LimelightHelpers.getBotPose3d_TargetSpace("limelight-left");

        if (validTarget) {
            if (leftDist < rightDist) {
                tagId = tagIds[0];
                // targetPose3D = LimelightHelpers.getTargetPose3d_RobotSpace("limelight-left");
            } else {
                tagId = tagIds[1];
                // results = resultsRight;
                tagPose3d = LimelightHelpers.getTargetPose3d_RobotSpace("limelight-right");

                // tagPose3d = LimelightHelpers.getBotPose3d_TargetSpace("limelight-right");
            }
            // SmartDashboard.putNumber("tagID", tagId);

            Rotation2d yawOffset = new Rotation2d(tagPose3d.getRotation().getY());
            // Rotation2d yawOffset = new Rotation2d(targetPose3D.getRotation().getY());

            if (right) {

                // tagRel2d = new Pose2d(tagPose3d.getZ()-0.8, -tagPose3d.getX()
                // -Units.inchesToMeters(6.0),
                // new Rotation2d(tagPose3d.getRotation().getY()));

                // tagPose2d = Pose3Dto2D(tagPose3d);
                // robotToTag = new Transform2d(tagRel2d.getTranslation(),
                // tagRel2d.getRotation().unaryMinus());

                where = Utilitys.shiftPoseRight(Utilitys.getAprilTagPose(tagId),
                        Constants.forwardOffset, Constants.rightOffset);// 12//6.5); // 0.164285833);
            } else {
                where = Utilitys.shiftPoseLeft(Utilitys.getAprilTagPose(tagId),
                        Constants.forwardOffset, Constants.leftOffset);
                // tagRel2d = new Pose2d(tagPose3d.getZ()-0.4, -tagPose3d.getX()
                // -Units.inchesToMeters(6.0),
                // new Rotation2d(tagPose3d.getRotation().getY()));

                // tagRel2d = new Pose2d(-robotPoseTargetSpacePose3d.getX(),
                // robotPoseTargetSpacePose3d.getY(),
                // new Rotation2d(robotPoseTargetSpacePose3d.getRotation().getZ()));
                // tagPose2d = Pose3Dto2D(tagPose3d);
                // robotToTag = new Transform2d(tagRel2d.getTranslation(),
                // tagRel2d.getRotation().unaryMinus());

            }
            algae = Constants.algaeMode.get();
            if (algae) {
                where = Utilitys.shiftPoseRight(Utilitys.getAprilTagPose(tagId),
                        Constants.forwardOffset, 0.0);// 12//6.5); // 0.164285833);
            }
            // Pose2d whereTo = RobotContainer.drivetrain.botPose2d
            // .transformBy(robotToTag);
            // Pose2d whereTo = robotPose.plus( robotToTag);

            whereToPublisher.set(where);
            // tagRel2DPublisher.set(tagRel2d);
            // ignorePublisher.set(tagPose2d);
            // transPublisher.set(robotToTag);

            Command driveit = AutoBuilder.pathfindToPose(where, constraints, 0.0);
            return driveit;
        }
        return null;
        //
    }

    // PathPlannerPath path = PathPlannerPath.fromPathFile("Alpha",true);

    // Create the constraints to use while pathfinding. The constraints defined in
    // the path will only be used for the path.

    // NOT quite ready to drive it

    // Command driveit = AutoBuilder.pathfindToPose(where, constraints);

    public static Pose2d getAprilTagPose(int tagID) {
        try {
            // Load the official FRC AprilTag field layout (2024 example)
            // AprilTagFieldLayout fieldLayout =
            // AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();

            // Get the tag pose
            Optional<Pose2d> tagPose = Constants.fieldLayout.getTagPose(tagID).map(pose3d -> pose3d.toPose2d());

            return tagPose.orElse(null); // Return the pose if found, otherwise null
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }

    public static double distanceToTag(CommandSwerveDrivetrain drivetrain, int tagID) {
        Optional<Pose2d> tagPose = Constants.fieldLayout.getTagPose(tagID).map(pose3d -> pose3d.toPose2d());
        Pose2d botPose = drivetrain.botPose2d;
        Translation2d targetTranslation = tagPose.get().getTranslation();
        Translation2d botTranslation = botPose.getTranslation();
        return botTranslation.getDistance(targetTranslation);
    }

    public String bestCamera(PoseEstimate left, PoseEstimate right) {
        double leftAmbiguity = 0;
        double rightAmbiguity = 0;

        if (left == null && right == null) {
            return null;
        } else if (left == null) {
            return "limelight-right";
        } else if (right == null) {
            return "limelight-left";
        } else {
            if (left.tagCount > 0) {
                leftAmbiguity = left.rawFiducials[0].ambiguity;
            }
            if (right.tagCount > 0) {
                rightAmbiguity = right.rawFiducials[0].ambiguity;
            }
            if (leftAmbiguity < rightAmbiguity) {
                return "left";
            } else {
                return "right";
            }
        }
    }

    public PoseEstimate bestEstimate() {
        double leftAmbiguity = 42.0;
        double rightAmbiguity = 42.0;
        PoseEstimate left = null;
        PoseEstimate right = null;

        left = Constants.cameraPoses[0];
        right = Constants.cameraPoses[1];

        if (left == null && right == null) {
            return null;
        } else if (left == null) {
            return right;
        } else if (right == null) {
            return left;
        } else {
            if (left.tagCount > 0) {
                leftAmbiguity = left.rawFiducials[0].ambiguity;
            }
            if (right.tagCount > 0) {
                rightAmbiguity = right.rawFiducials[0].ambiguity;
            }
            if (leftAmbiguity < rightAmbiguity) {
                return left;
            } else {
                return right;
            }
        }
    }

    public static int grabTagID() {
        double leftDist, rightDist;
        double shiftDirection;
        int[] tagIds = new int[2];
        boolean validTarget = false;
        int tagId = 0;
        LimelightHelpers.LimelightResults resultsLeft = LimelightHelpers.getLatestResults("limelight-left");

        LimelightHelpers.LimelightResults resultsRight = LimelightHelpers.getLatestResults("limelight-right");
        // SmartDashboard.putNumber("right: ", resultsRight.botpose_avgdist);
        // SmartDashboard.putBoolean("valid", resultsRight.valid);

        if (resultsLeft.valid) {
            leftDist = resultsLeft.botpose_avgdist;
            validTarget = true;
            tagIds[0] = (int) resultsLeft.targets_Fiducials[0].fiducialID;
        } else {
            leftDist = 999999;
        }

        if (resultsRight.valid) {
            rightDist = resultsRight.botpose_avgdist;

            tagIds[1] = (int) resultsRight.targets_Fiducials[0].fiducialID;
            validTarget = true;
        } else {
            rightDist = 999999;
        }

        if (validTarget) {
            if (leftDist < rightDist) {
                tagId = tagIds[0];
                // SmartDashboard.putString("Camera", "left");

            } else {
                tagId = tagIds[1];
                // SmartDashboard.putString("Camera", "right");
            }
        }
        return tagId;

    }

    public static Pose2d Pose3Dto2D(Pose3d pose3d) {
        return new Pose2d(pose3d.getX(), pose3d.getY(), pose3d.getRotation().toRotation2d());
    }

    public class BezierCurve {

        public static List<Point2D.Double> generateCurve(Point2D p0, Point2D p1, Point2D p2, int numPoints) {

            List<Point2D.Double> curve = new ArrayList<>();

            for (int i = 0; i <= numPoints; i++) {

                double t = i / (double) numPoints;

                double x = Math.pow(1 - t, 2) * p0.getX() + 2 * (1 - t) * t * p1.getX() + Math.pow(t, 2) * p2.getX();

                double y = Math.pow(1 - t, 2) * p0.getY() + 2 * (1 - t) * t * p1.getY() + Math.pow(t, 2) * p2.getY();

                curve.add(new Point2D.Double(x, y));

            }

            return curve;

        }

    }

    public PoseEstimate grabPose(String camera, Pigeon2 gyro) {
        LimelightHelpers.SetRobotOrientation(camera, gyro.getYaw().getValueAsDouble(), 0, 0, 0, 0, 0);
        // LimelightHelpers.SetRobotOrientation("limelight-left",getGyroYaw().getDegrees(),
        // 0, 0, 0, 0, 0);

        mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(camera);
        return mt2;

    }

    // public void updateOdometry() {
    // boolean doRejectUpdate = false;
    // Pigeon2 gyro = RobotContainer.drivetrain.gyro;
    // cameraPoses[0] = grabPose("limelight-left");
    // cameraPoses[1] = grabPose("limelight-right");
    // for (int i = 0; i < 2; i++) {

    // doRejectUpdate = false;
    // if (cameraPoses[i] != null) {
    // if (cameraPoses[i].tagCount == 0) {
    // doRejectUpdate = true;
    // }
    // if (cameraPoses[i].pose.getX() < 0) {
    // doRejectUpdate = true;
    // }
    // if (cameraPoses[i].pose.getY() > 7.6) {
    // doRejectUpdate = true;
    // }
    // } else {
    // doRejectUpdate = true;
    // }
    // /// SmartDashboard.putNumber("estimated yaw",
    // // m_poseEstimator.getEstimatedPosition().getRotation().getDegrees();
    // if (gyro.getAngularVelocityZWorld().getValueAsDouble() > 360) // if our
    // angular velocity is greater
    // {
    // doRejectUpdate = true;
    // }
    // if (!doRejectUpdate) {
    // m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7,
    // 9999999));
    // m_poseEstimator.addVisionMeasurement(
    // cameraPoses[i].pose,
    // cameraPoses[i].timestampSeconds);
    // RobotContainer.drivetrain.swerveOdometry.resetPosition(getGyroYaw(gyro),
    // RobotContainer.drivetrain.getModulePositions(),
    // m_poseEstimator.getEstimatedPosition());
    // }

    // }

    // }

    public Rotation2d getGyroYaw(Pigeon2 gyro) {
        SmartDashboard.putNumber("yaw", gyro.getYaw().getValueAsDouble());
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    public PoseEstimate grabPose(CommandSwerveDrivetrain drivetrain, String camera) {
        LimelightHelpers.SetRobotOrientation(camera, drivetrain.gyro.getYaw().getValueAsDouble(), 0, 0,
                0, 0, 0);
        // LimelightHelpers.SetRobotOrientation("limelight-left",getGyroYaw().getDegrees(),
        // 0, 0, 0, 0, 0);

        mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(camera);
        return mt2;

    }

    // public static void addLimelightVisionMeasurements(String camera) {

    // //
    // LimelightHelpers.SetRobotOrientation("limelight-left",getGyroYaw().getDegrees(),
    // // 0, 0, 0, 0, 0);
    // var driveState = RobotContainer.drivetrain.getState();
    // double headingDeg = driveState.Pose.getRotation().getDegrees();
    // double omegaRps =
    // Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);
    // LimelightHelpers.SetRobotOrientation(camera, headingDeg, 0, 0, 0, 0, 0);
    // mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(camera);

    // if (mt2 != null && mt2.tagCount > 0 ) { // distance was 2
    // RobotContainer.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7,
    // 9999999));
    // RobotContainer.drivetrain.addVisionMeasurement(mt2.pose,
    // Utils.fpgaToCurrentTime(mt2.timestampSeconds));
    // RobotContainer.drivetrain.swerveOdometry.resetPosition(RobotContainer.drivetrain.getGyroYaw(),RobotContainer.drivetrain.getModulePositions(),
    // RobotContainer.drivetrain.m_poseEstimator.getEstimatedPosition());

    // }
    // }
}