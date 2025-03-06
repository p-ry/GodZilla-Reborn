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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.math.util.Units;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

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
public static final double wheelCircumference = Units.inchesToMeters(4);
  
public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

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






}
