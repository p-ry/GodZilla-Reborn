// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/* import Miracle.java;
*/
package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.io.Console;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix.led.CANdle;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.DataLogManager;

import edu.wpi.first.wpilibj.Timer;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.auto.CommandUtil;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DriveItCommand;
import frc.robot.commands.Extend;
import frc.robot.commands.MoveArm;
import frc.robot.commands.MoveArmFix;
//import frc.robot.commands.MoveArmFix;
import frc.robot.commands.Retract;
import frc.robot.commands.RobotCentricDriveCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.Constants;
import frc.robot.subsystems.Ace;
import frc.robot.subsystems.ArmAssembly;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LowerArm;
import frc.robot.subsystems.Wrist;
import frc.robot.commands.SetArmBrakeMode;
import frc.robot.commands.WaitForCoral;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.awt.geom.Point2D;
import frc.robot.commands.FollowCurve;
import frc.robot.Utilitys.DriveToOptions;
import frc.robot.Utilitys.HeadingStrategy;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utilitys.VisibleTagMeasurementSupplier;

import java.util.function.Supplier;

public class RobotContainer {

  // public static Pigeon2 gyro;
  public static double MaxSpeed = 4.73;// TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts
                                       // desired
                                       // top
                                       // speed

  public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                          // second
                                                                                          // max angular velocity
  public static double driveDeadband = 0.473;
  public static double turnDeadband = 0.47;
  public static double garbage = 0;
  public static final AtomicBoolean pathWarmupComplete = new AtomicBoolean(false);
  public static final AtomicBoolean pathFindingWarmupComplete = new AtomicBoolean(false);

  private double prevHeading = 0;
  private double slowFactor = 3;
  // public static CANdle candle = new CANdle(37);

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  // private final Telemetry logger = new Telemetry(MaxSpeed);

  // ace.setBrakeMode(true);

  public CommandSwerveDrivetrain drivetrain;
  // = TunerConstants.createDrivetrain();
  public ArmAssembly mArm;
  // = new ArmAssembly(false, 99);
  public Ace ace;
  // = new Ace(0);
  public static int prevLevel = 0;

  // private final CommandXboxController driver = new CommandXboxController(0);
  public final CommandXboxController controller = new CommandXboxController(0);
  private final Joystick copilot = new Joystick(1);
  private final Joystick copilot2 = new Joystick(2);
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(driveDeadband).withRotationalDeadband(turnDeadband)
      .withDriveRequestType(DriveRequestType.Velocity);

  private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
      .withDeadband(0.0)
      .withRotationalDeadband(turnDeadband)
      .withDriveRequestType(DriveRequestType.Velocity);

  final JoystickButton Dump = new JoystickButton(copilot, 1);
  final JoystickButton Lv2L = new JoystickButton(copilot, 2);
  final JoystickButton Lv2R = new JoystickButton(copilot, 3);
  final JoystickButton Lv3L = new JoystickButton(copilot, 4);
  final JoystickButton Lv3R = new JoystickButton(copilot, 5);
  final JoystickButton Lv4L = new JoystickButton(copilot, 6);
  final JoystickButton Lv4R = new JoystickButton(copilot, 7);
  // final JoystickButton Climb = new JoystickButton(copilot, 8);
  // final JoystickButton Pull = new JoystickButton(copilot, 9);
  final JoystickButton Intake = new JoystickButton(copilot, 10);
  final JoystickButton Outtake = new JoystickButton(copilot, 11);
  final JoystickButton Algae = new JoystickButton(copilot, 8);
  final JoystickButton Process = new JoystickButton(copilot2, 1);
  final JoystickButton Load = new JoystickButton(copilot, 12);
  final JoystickButton Barge = new JoystickButton(copilot2, 2);
  // final JoystickButton Chomp = new JoystickButton(copilot, 9);
  final JoystickButton CoveredSwitch = new JoystickButton(copilot, 9);

  final Trigger lTrigger = controller.leftTrigger();
  final Trigger rTrigger = controller.rightTrigger();
  final Trigger leftBumper = controller.leftBumper();
  final Trigger rightBumper = controller.rightBumper();
  public static boolean loading = false;
  public static int BlueAlliance = 1;
  public static Command driveIt;
  public static boolean rightTree = true;
  public static double maxSpeedConstant = 4.73;
  public static double maxAngularRateConstant = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  // public static Point2D.Double base = new
  // Point2D.Double(85.393,509.0);//.17645,.50898890);
  // public static Point2D.Double startPoint = new
  // Point2D.Double(-90.00,780.0);//-.02777, .5207621);

  // public static Point2D.Double controlPoint1 = new
  // Point2D.Double(100.0,800.0);//-0.4,.5);
  // public static Point2D.Double controlPoint2 = new
  // Point2D.Double(250.0,1684.7);//158.0,1684.7);//.2,0.6);
  // public static Point2D.Double endPoint = new
  // Point2D.Double(0.00,2020.0);//80.0,2040.0-20.1,1160.95);//0.0041,1.85795);

  private final Supplier<AprilTagFieldLayout> fieldLayoutSupplier = () -> Constants.fieldLayout;
  private final VisibleTagMeasurementSupplier visibleMeas = () ->
  Utilitys.collectVisibleTagMeasurementsByAPI(Constants.LIMELIGHT_NAMES);

  // === Robot pose supplier ===
  private final Supplier<Pose2d> robotPoseSupplier;// = drivetrain::getPose; // adapt if your API differs

  // === Offsets (meters) ===
  // Positive X is forward from tag; Positive Y is left from tag.
  private static final double APPROACH_X = 0.150; // stand 0.80m in front of tag
  private static final double LATERAL_Y = 0.40; // 0.40m left/right of tag centerline

// DriveToOptions(
//   PathConstraints constraints,
//   HeadingStrategy headingStrategy,
//   Rotation2d explicitHeading,
//   double positionToleranceMeters,
//   Rotation2d headingTolerance,
//   double replanPeriodSec,
//   double replanPosDeltaMeters,
//   Rotation2d replanHeadingDelta
// )

private static final DriveToOptions DRIVE_OPTS = new DriveToOptions(
    new PathConstraints(2.0, 2.0, 0.5, 1.0), // m/s, m/s^2, rad/s, rad/s^2
    HeadingStrategy.FACE_TAG,                // Turn to face the tag
    new Rotation2d(),                        // Only used if EXPLICIT
    0.05,                                    // 5 cm position window
    Rotation2d.fromDegrees(3.0),             // 3° heading window
    0.25,                                    // Re-evaluate 4 Hz
    0.10,                                    // Replan if target shifts > 10 cm
    Rotation2d.fromDegrees(5.0)              // Or heading shifts > 5°
);


  /* Path follower */
  private final SendableChooser<Command> AutoChooser;

  public RobotContainer() {

    InitLogger.time("ArmAssemblyInit", () -> {
      mArm = new ArmAssembly(false, 99);
    });

    InitLogger.time("AceInit", () -> {
      ace = new Ace(0);
    });

    InitLogger.time("DriveTrainInit", () -> {
      drivetrain = TunerConstants.createDrivetrain();
    });

    // gyro = new Pigeon2(0, "Canivore");
    // SmartDashboard.putNumber("prevHeading", prevHeading);
    System.out.println("Left Y: " + controller.getLeftY());

    System.out.println("Left X: " + controller.getLeftX());
    System.out.println("Right X: " + controller.getRightX());
    System.out.println("Right Y: " + controller.getRightY());
    robotPoseSupplier = drivetrain::getPose; // adapt if your API differs


    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(
            -(controller.getLeftY())
                * MaxSpeed * BlueAlliance) // Drive

            .withVelocityY(-(controller.getLeftX()) * MaxSpeed * BlueAlliance) // Drive

            .withRotationalRate(-controller.getRightX() * MaxAngularRate) // Drive
                                                                          // counterclockwise
                                                                          // with
                                                                          // negative
                                                                          // X
                                                                          // (left)
        ));
    ;

    NamedCommands.registerCommand("raiseArm", new MoveArmFix(mArm, ace, 42, -1));
    NamedCommands.registerCommand("level3", new MoveArmFix(mArm, ace, 3, 1));
    NamedCommands.registerCommand("Load", new MoveArmFix(mArm, ace, 1, 0)
        .alongWith(new InstantCommand(() -> {
          // ace.setSpeed(0.9);
          ace.resetStateMachine();
          Constants.AutonomousMode = true;
          // ace.resetStateMachine();
          loading = true;

        })));
    NamedCommands.registerCommand("WaitForCoral", new WaitForCoral(ace));
    NamedCommands.registerCommand("L1", new MoveArmFix(mArm, ace, 6, 0));
    NamedCommands.registerCommand("L2", new MoveArmFix(mArm, ace, 2, 0));
    NamedCommands.registerCommand("L3", new MoveArmFix(mArm, ace, 3, 0));
    NamedCommands.registerCommand("L4",
        new FollowCurve(mArm, ace, Constants.startPoint, Constants.controlPoint1, Constants.controlPoint2,
            Constants.endPoint, Constants.base, () -> mArm.lowerArm.getDegs(), () -> mArm.upperArm.getDegs(),
            () -> mArm.slider.getMM(), true));
    // MoveArmFix(mArm, ace, 4, 0));
    NamedCommands.registerCommand("L4No",
        new FollowCurve(mArm, ace, Constants.startPoint, Constants.controlPoint1, Constants.controlPoint2,
            Constants.endPoint, Constants.base, () -> mArm.lowerArm.getDegs(), () -> mArm.upperArm.getDegs(),
            () -> mArm.slider.getMM(), true));
    // new MoveArmFix(mArm, ace, 400, 0));
    NamedCommands.registerCommand("Intake",
        new InstantCommand(() -> ace.setSpeed(1))
            .alongWith(new InstantCommand(() -> ace.gotIt = false))
            .alongWith(new InstantCommand(() -> ace.coralPresent = false)));

    drivetrain.configureAutoBuilder();
    Pathfinding.setPathfinder(new LocalADStar());
    CommandScheduler.getInstance().schedule(
        new WaitCommand(0.04).andThen(this::scheduleWarmups));

    AutoChooser = AutoBuilder.buildAutoChooser("none");
    SmartDashboard.putData("AutoChooser", AutoChooser);
    configureBindings();

    
  }
  public CommandSwerveDrivetrain getDrivetrain() {
    return drivetrain;
}

  public void scheduleWarmups() {
    scheduleFollowPathWarmup();
    schedulePathfindingWarmup();
  }
  private Command makeDriveToNearestVisibleTag(double dxMeters, double dyMeters) {
    return Utilitys.driveToDxDyFromNearestTagRaw(
        drivetrain,
        robotPoseSupplier,   // your field Pose2d (odom/estimator)
        visibleMeas,         // <-- now uses the correct API
        dxMeters,
        dyMeters,
        DRIVE_OPTS
    );
  }
  

  private void scheduleFollowPathWarmup() {
    System.out.println("[Init] Scheduling FollowPathCommand warmup...");

    Command warmup = FollowPathCommand.warmupCommand()
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf)
        .ignoringDisable(true)
        .andThen(() -> {
          System.out.println("[Warmup] FollowPathCommand warmup complete.");
          pathWarmupComplete.set(true);
        });

    Command wrapped = new ProxyCommand(() -> warmup)
        .finallyDo(interrupted -> {
          System.out.println("[Warmup] FollowPathCommand finished. Interrupted? "
              + interrupted);
        });

    wrapped.schedule();
  }

  private void schedulePathfindingWarmup() {
    System.out.println("[Init] Scheduling PathfindingCommand warmup...");

    Command warmup = PathfindingCommand.warmupCommand()
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf)
        .ignoringDisable(true)
        .andThen(() -> {
          System.out.println("[Warmup] PathfindingCommand warmup complete.");
          pathFindingWarmupComplete.set(true);
        });

    Command wrapped = new ProxyCommand(() -> warmup)
        .finallyDo(interrupted -> {
          System.out.println("[Warmup] PathfindingCommand finished. Interrupted? "
              + interrupted);
        });

    wrapped.schedule();
  }

  public boolean isFollowPathWarmupComplete() {
    return pathWarmupComplete.get();
  }

  public boolean isPathFindingWarmupComplete() {
    return pathFindingWarmupComplete.get();

  }

  private void configureBindings() {

    Algae.onTrue(new InstantCommand(() -> {
      ace.setSpeed(0.8);
      Constants.algaeMode.set(true);
    }));
    Algae.onFalse(new InstantCommand(() -> {
      ace.setSpeed(0);
      Constants.algaeMode.set(false);
    }));

    Load.onTrue(new MoveArmFix(mArm, ace, 1, 0)
        .alongWith(new InstantCommand(() -> {
          // ace.setSpeed(0.9);
          loading = true;
          ace.resetStateMachine();

        })));

    Load
        .onFalse(new MoveArmFix(mArm, ace, 0, 0)
            .alongWith(new InstantCommand(() -> loading = false)));
    Process
        .onTrue(new InstantCommand(() -> {
          Constants.endX -= 25.0;
          Constants.endPoint.setLocation(Constants.endX, Constants.endY);

          // SmartDashboard.putNumber("endX", Constants.endX);
          // SmartDashboard.putNumber("endpoinX", Constants.endPoint.getX());
          // SmartDashboard.putNumber("endpoinY", Constants.endPoint.getY());
        }));

    Barge
        .onTrue(new InstantCommand(() -> {
          Constants.endX += 25.0;
          Constants.endPoint.setLocation(Constants.endX, Constants.endY);
          // SmartDashboard.putNumber("endX", Constants.endX);
          // SmartDashboard.putNumber("endpoinX", Constants.endPoint.getX());
          // SmartDashboard.putNumber("endpoinY", Constants.endPoint.getY());
        }));

    Dump
        .whileTrue(new MoveArmFix(mArm, ace, 6, 0));
    Dump
        .onFalse(new MoveArmFix(mArm, ace, 0, 0));

    lTrigger.whileTrue(
        new RunCommand(() -> {
          double axis = controller.getLeftTriggerAxis(); // 0 → 1

          double vLeft = axis * 0.5;

          drivetrain.setControl(
              robotCentricDrive
                  .withVelocityX(0.0) // no fwd/back
                  .withVelocityY(vLeft) // +Y = left
                  .withRotationalRate(0.0)); // no spin
        }, drivetrain));
    lTrigger.onFalse(new InstantCommand(() -> {
      drivetrain.setControl(
          robotCentricDrive
              .withVelocityX(0.0) // no fwd/back
              .withVelocityY(0.0) // +Y = left
              .withRotationalRate(0.0)); // no spin
    }, drivetrain));
    rTrigger.whileTrue(
        new RunCommand(() -> {
          double axis = controller.getRightTriggerAxis(); // 0 → 1
          double vRight = axis * 0.5;

          drivetrain.setControl(
              robotCentricDrive
                  .withVelocityX(0.0) // no fwd/back
                  .withVelocityY(-vRight) // +Y = right
                  .withRotationalRate(0.0)); // no spin
        }, drivetrain));

    rTrigger.onFalse(new InstantCommand(() -> {
      drivetrain.setControl(
          robotCentricDrive
              .withVelocityX(0.0) // no fwd/back
              .withVelocityY(0.0) // +Y = right
              .withRotationalRate(0.0)); // no spin
    }, drivetrain));

    Lv2L.onTrue(new InstantCommand(() -> {
      MaxSpeed = maxSpeedConstant;
      MaxAngularRate = maxAngularRateConstant / 2;
      rightTree = false;
    }).alongWith(new MoveArmFix(mArm, ace, 2, -1)));
    Lv2L.onFalse(new InstantCommand(() -> {
      MaxSpeed = maxSpeedConstant;
      MaxAngularRate = maxAngularRateConstant;
    }).alongWith(new MoveArmFix(mArm, ace, 44, 0)));

    Lv3L.onTrue(new InstantCommand(() -> {
      MaxSpeed = maxSpeedConstant;
      MaxAngularRate = maxAngularRateConstant / 3;
      rightTree = false;
    }).alongWith(new MoveArmFix(mArm, ace, 3, -1)));
    Lv3L.onFalse(new InstantCommand(() -> {
      MaxSpeed = maxSpeedConstant;
      MaxAngularRate = maxAngularRateConstant;
    }).alongWith(new MoveArmFix(mArm, ace, 44, 0)));

    // *********TRUE *************************************** */
    Lv4L.onTrue(new FollowCurve(mArm, ace, Constants.startPoint, Constants.controlPoint1, Constants.controlPoint2,
        Constants.endPoint, Constants.base, () -> mArm.lowerArm.getDegs(), () -> mArm.upperArm.getDegs(),
        () -> mArm.slider.getMM(), false)
        .alongWith(new InstantCommand(() -> {
          MaxSpeed = maxSpeedConstant / 4;
          MaxAngularRate = maxAngularRateConstant / 2.5;
          rightTree = false;
        })));
    Lv4L.onFalse(new MoveArmFix(mArm, ace, 0, 0)
        .alongWith(new InstantCommand(() -> {

          MaxSpeed = maxSpeedConstant;

          MaxAngularRate = maxAngularRateConstant;
          // mArm.wrist.setPos(0.7);
        })));

    Intake
        .whileTrue(new InstantCommand(() -> ace.setSpeed(1)));
    Intake
        .onFalse(new InstantCommand(() -> ace.setSpeed(0)));
    Outtake
        .whileTrue(new InstantCommand(() -> ace.setSpeed(-0.5)));
    Outtake
        .onFalse(new InstantCommand(() -> ace.setSpeed(0)));

    leftBumper.onTrue(new InstantCommand(() -> {
      driveIt = makeDriveToNearestVisibleTag(APPROACH_X, +LATERAL_Y);
      if (driveIt != null) {
        driveIt.schedule();
      }
    }));

    leftBumper
        .onFalse(new InstantCommand(() -> {

          if (driveIt != null) {
            driveIt.cancel();
          }
        }));

        rightBumper.onTrue(new InstantCommand(() -> {
          driveIt = makeDriveToNearestVisibleTag(APPROACH_X, -LATERAL_Y);
          if (driveIt != null) {
            driveIt.schedule();
          }
        }));
    
        rightBumper
            .onFalse(new InstantCommand(() -> {
    
              if (driveIt != null) {
                driveIt.cancel();
              }
            }));

    
    // rightBumper

    // .onTrue(new InstantCommand(() -> {

    // driveIt = Utilitys.driveToIt(drivetrain, true);// rightTree

    // if (driveIt != null) {
    // driveIt.schedule();

    // }

    // }));
    // rightBumper
    // .onFalse(new InstantCommand(() -> {

    // if (driveIt != null) {
    // driveIt.cancel();
    // }
    // }));

    // leftBumper.onTrue(

    // new InstantCommand(() -> {

    // driveIt = Utilitys.driveToIt(drivetrain, false);
    // if (driveIt != null) {
    // driveIt.schedule();
    // }
    // }));
    // leftBumper
    // .onFalse(new InstantCommand(() -> {

    // if (driveIt != null) {
    // driveIt.cancel();
    // }
    // }));

    controller
        .start()
        .onTrue(new InstantCommand(() -> drivetrain.gyro.reset()));
    controller.start()
        .onTrue(new InstantCommand(() -> drivetrain.setHeading(new Rotation2d(0))));
controller.b().whileTrue(
  Utilitys.faceNearestVisibleTagCmd(drivetrain,drivetrain::getPose,"limelight-left","limelight-right")

);


    controller.y().whileTrue(
        new InstantCommand(() -> mArm.wrist.moveIt(-0.5)));
    controller.x().whileTrue(new InstantCommand(() -> mArm.wrist.moveIt(0.5)));

    // controller.a()
    // .whileTrue(new SetArmBrakeMode(mArm, ace,NeutralModeValue.Coast))
    // .onFalse(new SetArmBrakeMode(mArm, ace,NeutralModeValue.Brake));

  }

  public void resetGyro() {
    drivetrain.resetGyroToAlliance();
  }

  public Command getAutonomousCommand() {
    /* Run the path selected from the auto chooser */
    return AutoChooser.getSelected();
  }
}
