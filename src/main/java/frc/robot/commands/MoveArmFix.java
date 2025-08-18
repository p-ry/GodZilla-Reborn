package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.InitLogger;
import frc.robot.Utilitys;
import frc.robot.subsystems.Ace;
import frc.robot.subsystems.ArmAssembly;

import java.util.Collections;
import java.util.HashSet;
import java.util.Set;
import java.util.concurrent.atomic.AtomicBoolean;

public class MoveArmFix extends Command {
  public enum Level {
    HOME(0),
    LOAD(1),
    LEVEL2(2),
    LEVEL3(3),
    LEVEL4(4),
    LEVEL5(5),
    LEVEL6(6),
    OLD_MOVE(7),
    CLIMB(8),
    ADJUST_PLUS(12),
    RAISE_KICKSTAND(42),
    LOWER_FROM_4(44),
    CHOMP(50),
    LEVEL4_NO(400),
    UNKNOWN(-1);

    public final int id;

    Level(int id) {
      this.id = id;
    }

    public static Level fromInt(int v) {
      for (Level l : values()) {
        if (l.id == v) {
          return l;
        }
      }
      return UNKNOWN;
    }

    @Override
    public String toString() {
      return name() + "(" + id + ")";
    }
  }

  private final ArmAssembly arm;
  private final Ace ace;
  private final Level levelEnum;
  private final int shiftDirection;

  private double position;
  private int tagId;
  private double startTime;
  private Pose2d aprilTag = new Pose2d();
  private boolean applyDynamic = false;
  private boolean algae = false;

  private int prevLevel;
  private Level lastLevelEnum = null;
  private double levelStartTime = 0;
  private boolean reachedThisLevel = false;

  public static boolean retract;
  public static boolean slow, offset;

  private double levelReachedTime = 0;
  private static final double STABLE_DURATION = 0.1;

  private final Set<Subsystem> requirements = new HashSet<>();

  public MoveArmFix(ArmAssembly arm, Ace ace, int level, int shiftDirection) {
    this(arm, ace, Level.fromInt(level), shiftDirection);
  }

  public MoveArmFix(ArmAssembly arm, Ace ace, Level levelEnum, int shiftDirection) {
    this.arm = arm;
    this.ace = ace;
    this.levelEnum = levelEnum;
    this.shiftDirection = shiftDirection;
    //addRequirements(arm, ace);
  }

  @Override
  public Set<Subsystem> getRequirements() {
    return requirements;
  }

  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    position = arm.upperArm.getPos();
    prevLevel = arm.level;
    offset = false;

    tagId = Utilitys.grabTagID();
    SmartDashboard.putNumber("TagID", tagId);
    if (tagId > 0) {
      aprilTag = Utilitys.getAprilTagPose(tagId);
      SmartDashboard.putNumberArray(
          "AprilTag",
          new double[] {
              aprilTag.getX(), aprilTag.getY(), aprilTag.getRotation().getRadians()
          });
    }

    applyDynamic = false;
    AtomicBoolean mode = Constants.algaeMode;
    algae = mode != null ? mode.get() : false;
    slow = false;
    retract = false;

    lastLevelEnum = levelEnum;
    levelStartTime = Timer.getFPGATimestamp();
    reachedThisLevel = false;

    InitLogger.logMessage("MoveArmFix", "Initialized. Level=" + levelEnum);
    // SmartDashboard.putString("MoveArmFix/Level", levelEnum.toString());
  }

  @Override
  public void execute() {
    AtomicBoolean mode = Constants.algaeMode;
    algae = mode != null ? mode.get() : false;

    if (lastLevelEnum != levelEnum) {
      InitLogger.logMessage("MoveArmFix", "Level transition detected from " + lastLevelEnum + " to " + levelEnum);
      levelStartTime = Timer.getFPGATimestamp();
      reachedThisLevel = false;
      lastLevelEnum = levelEnum;
    }

    switch (levelEnum) {
      case HOME:
        arm.lowerArm.setPos(1.0);
        if (algae) {
          arm.upperArm.setPos(5, true);
          arm.wrist.setPos(3.0);
        } else {
          arm.upperArm.setPos(0, true);
          arm.wrist.setPos(0.7);
        }
        arm.slider.setPos(0.50);
        break;

      case LOAD:
        if (algae) {
          arm.lowerArm.setPos(18.0);
          arm.upperArm.setPos(1.0, true);
          arm.slider.setPos(1.5);
          arm.wrist.setPos(0.7);
        } else {
          arm.lowerArm.setPos(18.00);
          arm.upperArm.setPos(1.5, true);
          arm.wrist.setPos(0.1);
          arm.slider.setPos(0.5);
        }
        break;

      case LEVEL2:
        if (algae) {
          arm.lowerArm.setPos(12.0);
          arm.upperArm.setPos(5.0, true);
          arm.slider.setPos(0.50);
          arm.wrist.setPos(3.0);
        } else {
          arm.lowerArm.setPos(1.0);
          arm.upperArm.setPos(6.0, true);
          arm.slider.setPos(6.0);
          arm.wrist.setPos(5.0);
        }
        break;

      case LEVEL3:
        if (algae) {
          arm.lowerArm.setPos(28.0);
          arm.upperArm.setPos(22.0, false);
          arm.slider.setPos(1.5);
          arm.wrist.setPos(6.0);
        } else {
          arm.lowerArm.setPos(20);
          arm.upperArm.setPos(21, true);
          arm.slider.setPos(0.5);
          arm.wrist.setPos(6.5);
        }
        break;

      case LEVEL4:
        arm.lowerArm.setPos(26.5, true);
        arm.upperArm.setPos(34.0, true);
        arm.slider.setPos(30.5, false);
        arm.wrist.setPos(9.4);
        if (!offset) {
          ace.setPos(5.0);
          offset = true;
        }
        break;

      case LEVEL5:
        arm.lowerArm.setPos(30.0);
        arm.upperArm.setPos(4.0, applyDynamic);
        arm.slider.setPos(0.50);
        arm.wrist.setPos(3.0);
        break;

      case LEVEL6:
        arm.lowerArm.setPos(27.30);
        arm.upperArm.setPos(8.9, applyDynamic);
        arm.slider.setPos(0.50);
        arm.wrist.setPos(0.7);
        break;

      case OLD_MOVE:
        applyDynamic = true;
        arm.lowerArm.setPos(1.0);
        arm.upperArm.setPos(position, applyDynamic);
        arm.slider.setPos(0.50);
        arm.wrist.setPos(0.7);
        System.out.println("WARNING!!!!  OLD MOVE!!!!!!!");
        break;

      case CLIMB:
        // no-op
        break;

      case ADJUST_PLUS:
        arm.wrist.setPos(arm.wrist.getPos() + 1.0);
        break;

      case RAISE_KICKSTAND:
        arm.upperArm.setPos(16, applyDynamic);
        break;

      case LOWER_FROM_4:
        arm.lowerArm.setPos(18.00);
        arm.upperArm.setPos(1.5, false);
        arm.wrist.setPos(0.1);
        arm.slider.setPos(0.5, true);
        break;

      case CHOMP:
        InitLogger.logMessage("WARNING", "Chomp called with shiftDirection=" + shiftDirection);
        if (shiftDirection == 1) {
          arm.wrist.setSpeed(0.2);
        } else {
          arm.wrist.setSpeed(0);
        }
        break;

      case LEVEL4_NO:
        arm.lowerArm.setPos(26.5, true);
        arm.upperArm.setPos(34.0, true);
        arm.slider.setPos(30.5, false);
        arm.wrist.setPos(9.4);
        break;

      case UNKNOWN:
      default:
        InitLogger.logMessage("MoveArmFix", "Unknown level: " + levelEnum);
        if (algae) {
          arm.lowerArm.setPos(1.0);
          arm.upperArm.setPos(1.0, applyDynamic);
          arm.slider.setPos(1.50);
          arm.wrist.setPos(3.0);
        } else {
          arm.lowerArm.setPos(1.0);
          arm.upperArm.setPos(1.0, applyDynamic);
          arm.slider.setPos(0.50, false);
          arm.wrist.setPos(0.7);
        }
        break;
    }

    boolean atLevel = arm.isAtLevel();
    if (atLevel && !reachedThisLevel) {
      reachedThisLevel = true;

      double sinceLevelStart = Timer.getFPGATimestamp() - levelStartTime;
      InitLogger.logMessage("MoveArmFix",
          "Reached level " + levelEnum + " in " + String.format("%.3f", sinceLevelStart) + "s");
    }

    double timeSinceLevel = Timer.getFPGATimestamp() - levelStartTime;
    if (!reachedThisLevel && timeSinceLevel > 1.0) {
      InitLogger.logMessage("MoveArmFix",
          "WARNING: Level " + levelEnum + " not achieved after " + String.format("%.2f", timeSinceLevel) + "s");
      reachedThisLevel = true; // avoid repeat spam
    }

    SmartDashboard.putString("MoveArmFix/Level", levelEnum.toString());
  }

  @Override
  public void end(boolean interrupted) {
    InitLogger.logMessage("MoveArmFix", "Ended. Level=" + levelEnum + " interrupted=" + interrupted);
  }

  @Override
  public boolean isFinished() {
    boolean atLevel = arm.isAtLevel();
    if (atLevel) {
      if (levelReachedTime == 0) {
        levelReachedTime = Timer.getFPGATimestamp();
      }
      if (Timer.getFPGATimestamp() - levelReachedTime >= STABLE_DURATION) {
        return true;
      }
    } else {
      levelReachedTime = 0;
    }
    return false; // no longer using elapsed-time cutoff or combine both if needed
  }

  @Override
  public boolean runsWhenDisabled() {
    return false; // or true if you want it to run while disabled
  }
}
