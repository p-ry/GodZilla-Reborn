// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
//import frc.robot.Constants.POSES;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.math.geometry.Rotation3d;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

  public Pose3d tag1 = fieldLayout.getTagPose(1).get();
  public Pose3d tag2 = fieldLayout.getTagPose(2).get();
  public Pose3d tag3 = fieldLayout.getTagPose(3).get();
  public Pose3d tag4 = fieldLayout.getTagPose(4).get();
  public Pose3d tag5 = fieldLayout.getTagPose(5).get();
  public Pose3d tag6 = fieldLayout.getTagPose(6).get();
  public Pose3d tag7 = fieldLayout.getTagPose(7).get();
  public Pose3d tag8 = fieldLayout.getTagPose(8).get();
  public Pose3d tag9 = fieldLayout.getTagPose(9).get();
  public Pose3d tag10 = fieldLayout.getTagPose(10).get();
  public Pose3d tag11 = fieldLayout.getTagPose(11).get();
  public Pose3d tag12 = fieldLayout.getTagPose(12).get();
  public Pose3d tag13 = fieldLayout.getTagPose(13).get();
  public Pose3d tag14 = fieldLayout.getTagPose(14).get();
  public Pose3d tag15 = fieldLayout.getTagPose(15).get();
  public Pose3d tag16 = fieldLayout.getTagPose(16).get();
  public Pose3d tag17 = fieldLayout.getTagPose(17).get();

  public Pose3d tag18 = fieldLayout.getTagPose(18).get();
  public Pose3d tag19 = fieldLayout.getTagPose(19).get();
  public Pose3d tag20 = fieldLayout.getTagPose(20).get();
  public Pose3d tag21 = fieldLayout.getTagPose(21).get();
  public Pose3d tag22 = fieldLayout.getTagPose(22).get();
  
  public static final double trackWidth = Units.inchesToMeters(21.75); // TODO: This must be tuned to
  // specific
  // robot
public static final double wheelBase = Units.inchesToMeters(22); // TODO: This must be tuned to specific
// robot
public static final double wheelCircumference = Units.inchesToMeters(3.93*Math.PI); // TODO: This must be tuned to
  
public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));


                                public static final double maxSpeed = 3.0;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

public static final double kLowerArmPosL0 = 1.0;
public static final double kUpperArmPosL0 = 1.0;
public static final double kSliderPosL0 = 0;
public static final double kWristPosL0 = 0;

public static final double kLowerArmPosL1 = 10.0;
public static final double kUpperArmPosL1 = 10.0;
public static final double kSliderPosL1 = 0;
public static final double kWristPosL1 = 0;

public static final double kLowerArmPosL2 = 10;
public static final double kUpperArmPosL2 = 15;
public static final double kSliderPosL2 = 0;
public static final double kWristPosL2 = 5;

public static final double kLowerArmPosL3 = 19;
public static final double kUpperArmPosL3 = 15;
public static final double kSliderPosL3 = 0;
public static final double kWristPosL3 = 5;

public static final double kLowerArmPosL4 = 39;
public static final double kUpperArmPosL4 = 49;
public static final double kSliderPosL4 = 10;
public static final double kWristPosL4 = 10;

  public static class constField {
    public static Optional<Alliance> ALLIANCE = Optional.empty();
    public static final double FIELD_LENGTH = Units.feetToMeters(57) + Units.inchesToMeters(6 + 7 / 8.0);
    public static final double FIELD_WIDTH = Units.feetToMeters(26) + Units.inchesToMeters(5);

    /**
     * Boolean that controls when the path will be mirrored for the red
     * alliance. This will flip the path being followed to the red side of the
     * field.
     * The origin will remain on the Blue side.
     * 
     * @return If we are currently on Red alliance. Will return false if no alliance
     *         is found
     */
    public static boolean isRedAlliance() {
      var alliance = ALLIANCE;
      if (alliance.isPresent()) {
        return alliance.get() == DriverStation.Alliance.Red;
      }
      return false;
    };

public static class POSES {
  public static final Pose2d RESET_POSE = new Pose2d(0, 0, new Rotation2d());
  public static final Pose3d SCORING_ELEMENT_NOT_COLLECTED = new Pose3d(0, 0, -1, Rotation3d.kZero);

  // BRANCH POSES
  public static final Pose2d REEF_A = new Pose2d(3.171, 4.189, Rotation2d.fromDegrees(0));
  public static final Pose2d REEF_B = new Pose2d(3.171, 3.863, Rotation2d.fromDegrees(0));
  public static final Pose2d REEF_C = new Pose2d(3.688, 2.968, Rotation2d.fromDegrees(60));
  public static final Pose2d REEF_D = new Pose2d(3.975, 2.803, Rotation2d.fromDegrees(60));
  public static final Pose2d REEF_E = new Pose2d(5.001, 2.804, Rotation2d.fromDegrees(120));
  public static final Pose2d REEF_F = new Pose2d(5.285, 2.964, Rotation2d.fromDegrees(120));
  public static final Pose2d REEF_G = new Pose2d(5.805, 3.863, Rotation2d.fromDegrees(180));
  public static final Pose2d REEF_H = new Pose2d(5.805, 4.189, Rotation2d.fromDegrees(180));
  public static final Pose2d REEF_I = new Pose2d(5.288, 5.083, Rotation2d.fromDegrees(-120));
  public static final Pose2d REEF_J = new Pose2d(5.002, 5.248, Rotation2d.fromDegrees(-120));
  public static final Pose2d REEF_K = new Pose2d(3.972, 5.247, Rotation2d.fromDegrees(-60));
  public static final Pose2d REEF_L = new Pose2d(3.693, 5.079, Rotation2d.fromDegrees(-60));

  // CORAL STATION POSES
  public static final Pose2d LEFT_CORAL_STATION_FAR = new Pose2d(1.64, 7.33, Rotation2d.fromDegrees(-54.5));
  public static final Pose2d LEFT_CORAL_STATION_NEAR = new Pose2d(0.71, 6.68, Rotation2d.fromDegrees(-54.5));
  public static final Pose2d RIGHT_CORAL_STATION_FAR = new Pose2d(1.61, 0.70, Rotation2d.fromDegrees(55));
  public static final Pose2d RIGHT_CORAL_STATION_NEAR = new Pose2d(0.64, 1.37, Rotation2d.fromDegrees(55));

  // processor poses
  public static final Pose2d PROCESSOR = new Pose2d(6, .77, Rotation2d.fromDegrees(-90));

  private static final List<Pose2d> BLUE_REEF_POSES = List.of(REEF_A, REEF_B, REEF_C, REEF_D, REEF_E,
      REEF_F, REEF_G, REEF_H, REEF_I, REEF_J, REEF_K, REEF_L);
  private static final List<Pose2d> RED_REEF_POSES = getRedReefPoses();

  private static final Pose2d[] BLUE_POSES = new Pose2d[] { RESET_POSE, REEF_A, REEF_B, REEF_C, REEF_D, REEF_E,
      REEF_F, REEF_G, REEF_H, REEF_I, REEF_J, REEF_K, REEF_L };

  private static final Pose2d[] RED_POSES = getRedAlliancePoses();

  private static final List<Pose2d> BLUE_CORAL_STATION_POSES = List.of(LEFT_CORAL_STATION_FAR,
      LEFT_CORAL_STATION_NEAR, RIGHT_CORAL_STATION_FAR, RIGHT_CORAL_STATION_NEAR);
  private static final List<Pose2d> RED_CORAL_STATION_POSES = getRedCoralStationPoses();

  private static final Pose2d BLUE_PROCESSOR_POSE = PROCESSOR;
  private static final Pose2d RED_PROCESSOR_POSE = getRedProcessorPose();

  private static final List<Pose2d> PROCESSOR_POSES = List.of(BLUE_PROCESSOR_POSE, RED_PROCESSOR_POSE);

}

public static Pose2d getRedAlliancePose(Pose2d bluePose) {
      return new Pose2d(FIELD_LENGTH - (bluePose.getX()),
          FIELD_WIDTH - bluePose.getY(),
          bluePose.getRotation().plus(Rotation2d.fromDegrees(180)));
    }

    private static Pose2d[] getRedAlliancePoses() {
      Pose2d[] returnedPoses = new Pose2d[POSES.BLUE_POSES.length];

      for (int i = 0; i < POSES.BLUE_POSES.length; i++) {
        returnedPoses[i] = getRedAlliancePose(POSES.BLUE_POSES[i]);
      }
      return returnedPoses;
    }

    private static List<Pose2d> getRedReefPoses() {
      Pose2d[] returnedPoses = new Pose2d[POSES.BLUE_REEF_POSES.size()];

      for (int i = 0; i < POSES.BLUE_REEF_POSES.size(); i++) {
        returnedPoses[i] = getRedAlliancePose(POSES.BLUE_REEF_POSES.get(i));
      }

      return List.of(returnedPoses[0], returnedPoses[1], returnedPoses[2], returnedPoses[3], returnedPoses[4],
          returnedPoses[5], returnedPoses[6], returnedPoses[7], returnedPoses[8], returnedPoses[9], returnedPoses[10],
          returnedPoses[11]);
    }

    private static List<Pose2d> getRedCoralStationPoses() {
      Pose2d[] returnedPoses = new Pose2d[POSES.BLUE_CORAL_STATION_POSES.size()];

      for (int i = 0; i < POSES.BLUE_CORAL_STATION_POSES.size(); i++) {
        returnedPoses[i] = getRedAlliancePose(POSES.BLUE_CORAL_STATION_POSES.get(i));
      }

      return List.of(returnedPoses[0], returnedPoses[1], returnedPoses[2], returnedPoses[3]);
    }

    private static Pose2d getRedProcessorPose() {
      Pose2d returnedPose = POSES.BLUE_PROCESSOR_POSE;

      returnedPose = getRedAlliancePose(POSES.BLUE_PROCESSOR_POSE);

      return returnedPose;
    }

    /**
     * Gets the positions of all of the necessary field elements on the field. All
     * coordinates are in meters and are relative to the blue alliance.
     * 
     * @see <a href=
     *      https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#always-blue-origin">
     *      Robot Coordinate Systems</a>
     * @return An array of field element positions
     */
    public static Supplier<Pose2d[]> getFieldPositions() {
      if (ALLIANCE.isPresent() && ALLIANCE.get().equals(Alliance.Red)) {
        return () -> POSES.RED_POSES;

      }
      return () -> POSES.BLUE_POSES;
    }

    /**
     * Gets the positions of all of the necessary field elements on the field. All
     * coordinates are in meters and are relative to the blue alliance.
     * 
     * @see <a href=
     *      https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#always-blue-origin">
     *      Robot Coordinate Systems</a>
     * @return An array of the reef branches for your alliance
     */
    public static Supplier<List<Pose2d>> getReefPositions() {
      if (ALLIANCE.isPresent() && ALLIANCE.get().equals(Alliance.Red)) {
        return () -> POSES.RED_REEF_POSES;

      }
      return () -> POSES.BLUE_REEF_POSES;
    }

    public static Supplier<List<Pose2d>> getCoralStationPositions() {
      if (ALLIANCE.isPresent() && ALLIANCE.get().equals(Alliance.Red)) {
        return () -> POSES.RED_CORAL_STATION_POSES;
      }
      return () -> POSES.BLUE_CORAL_STATION_POSES;
    }

    public static Supplier<List<Pose2d>> getProcessorPositions() {
      return () -> POSES.PROCESSOR_POSES;
    }


}
}
