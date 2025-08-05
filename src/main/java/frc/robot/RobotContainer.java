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
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import java.util.concurrent.atomic.AtomicBoolean;

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
  final JoystickButton Chomp = new JoystickButton(copilot, 9);
  final JoystickButton CoveredSwitch = new JoystickButton(copilot, 8);

  final Trigger lTrigger = controller.leftTrigger();
  final Trigger rTrigger = controller.rightTrigger();
  public static boolean loading = false;
  public static int BlueAlliance = 1;
  public static Command driveIt;
  public static boolean rightTree = true;
  public static double maxSpeedConstant = 4.73;
  public static double maxAngularRateConstant = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
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
    SmartDashboard.putNumber("prevHeading", prevHeading);
    System.out.println("Left Y: " + controller.getLeftY());

    System.out.println("Left X: " + controller.getLeftX());
    System.out.println("Right X: " + controller.getRightX());
    System.out.println("Right Y: " + controller.getRightY());

    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(
            -(controller.getLeftY())
                * MaxSpeed * BlueAlliance) // Drive

            // -(controller.getLeftY() * controller.getLeftY()
            // * Math.signum(controller.getLeftY()))
            // * MaxSpeed) // Drive
            // // forward
            // with
            // negative
            // Y
            // (forward)
            .withVelocityY(-(controller.getLeftX()) * MaxSpeed * BlueAlliance) // Drive

            // .withVelocityY(-(controller.getLeftX() * controller.getLeftX()
            // * Math.signum(controller.getLeftX()) * MaxSpeed)) // Drive
            // // left
            // with
            // negative X (left)
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
    NamedCommands.registerCommand("Load", new WaitCommand(0.7).andThen(
        new MoveArmFix(mArm, ace, 1, 0).alongWith(new InstantCommand(() -> ace.setSpeed(1)))));
    NamedCommands.registerCommand("L1", new MoveArmFix(mArm, ace, 6, 0));
    NamedCommands.registerCommand("L2", new MoveArmFix(mArm, ace, 2, 0));
    NamedCommands.registerCommand("L3", new MoveArmFix(mArm, ace, 3, 0));
    NamedCommands.registerCommand("L4", new MoveArmFix(mArm, ace, 4, 0));
    NamedCommands.registerCommand("L4No", new MoveArmFix(mArm, ace, 400, 0));
    NamedCommands.registerCommand("Intake",
        new InstantCommand(() -> ace.setSpeed(1))
            .alongWith(new InstantCommand(() -> ace.gotIt = false))
            .alongWith(new InstantCommand(() -> ace.coralPresent = false)));
    new EventTrigger("L400").onTrue(new MoveArmFix(mArm, ace, 4, 0));
    new EventTrigger("LoadIt").onTrue(new MoveArmFix(mArm, ace, 1, 0)
        .alongWith(new InstantCommand(() -> System.out.println("loadit"))
            .alongWith(new InstantCommand(() -> ace.setSpeed(1)))));

    drivetrain.configureAutoBuilder();
    Pathfinding.setPathfinder(new LocalADStar());
    CommandScheduler.getInstance().schedule(
        new WaitCommand(0.04).andThen(this::scheduleWarmups));

    AutoChooser = AutoBuilder.buildAutoChooser("none");
    SmartDashboard.putData("AutoChooser", AutoChooser);
    configureBindings();
  }

  public void scheduleWarmups() {
    scheduleFollowPathWarmup();
    schedulePathfindingWarmup();
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

    // controller
    // .rightBumper()
    // .onTrue(new InstantCommand())

    // Process
    // .onTrue(new MoveArmFix(mArm, 12));
    // Algae
    // .onTrue(new InstantCommand(() -> ace.setSpeed(.8)));
    // Algae
    // .onFalse(new InstantCommand(() -> ace.setSpeed(0)));
    Algae.onTrue(new InstantCommand(() -> {
      ace.setSpeed(0.8);
      Constants.algaeMode.set(true);
    }));
    Algae.onFalse(new InstantCommand(() -> {
      ace.setSpeed(0);
      Constants.algaeMode.set(false);
    }));

    Load
        .onTrue(

            new MoveArmFix(mArm, ace, 1, 0)
                .alongWith(
                    new InstantCommand(() -> {
                      ace.setSpeed(0.9);
                      ace.gotIt = false;
                      ace.coralPresent = false;
                      loading = true;
                    })));

    Load
        .onFalse(new MoveArmFix(mArm, ace, 0, 0));
    Load.onFalse(new InstantCommand(() -> loading = false));
    Process
        .onTrue(new MoveArmFix(mArm, ace, 5, 0));
    // Process.whileTrue(new InstantCommand(() -> ace.setSpeed(0.1)));

    Process
        .onFalse(new MoveArmFix(mArm, ace, 0, 0));
    // Process
    // .onFalse(new InstantCommand(() -> ace.setSpeed(0)));

    Barge
        .onTrue(new MoveArmFix(mArm, ace, 42, -1));
    Barge
        .onFalse(new MoveArmFix(mArm, ace, 0, 0));

    Dump
        .whileTrue(new MoveArmFix(mArm, ace, 6, 0));
    Dump
        .onFalse(new MoveArmFix(mArm, ace, 0, 0));

    // Chomp.onTrue(new InstantCommand(() -> {
    // // ace.setSpeed(1);
    // mArm.wrist.setSpeed(.3);
    // System.out.println("Chomp is on");
    // })); // Chomp is on
    // Chomp.onFalse(new InstantCommand(() -> {
    // // ace.setSpeed(0);
    // mArm.wrist.setSpeed(0);
    // System.out.println("Chomp is off");
    // })); // Chomp is off

    CoveredSwitch.whileTrue(
        new MoveArmFix(mArm, ace, 8, 0)// Need to add isfinished command

    );

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

    // /* Stop the moment the trigger is released --------------------------- */
    // .onFalse(new InstantCommand(swerve::stop, swerve));
    // }
    // **************TRUE ******** */
    Lv2L.whileTrue(new MoveArmFix(mArm, ace, 2, -1));
    Lv2L.onTrue(new InstantCommand(() -> MaxSpeed = maxSpeedConstant));
    Lv2L.onTrue(new InstantCommand(() -> MaxAngularRate = maxAngularRateConstant / 2));
    Lv2L.onTrue(new InstantCommand(() -> rightTree = false));
    Lv2R.whileTrue(new MoveArmFix(mArm, ace, 2, 1));
    Lv2R.onTrue(new InstantCommand(() -> MaxSpeed = maxSpeedConstant));
    Lv2R.onTrue(new InstantCommand(() -> MaxAngularRate = maxAngularRateConstant / 2));
    Lv2R.onTrue(new InstantCommand(() -> rightTree = true));

    // ********FALSE ******** */
    Lv2L.onFalse(new MoveArmFix(mArm, ace, 44, 0));
    // .andThen(new InstantCommand(() -> ace.setSpeed(1))));

    Lv2L.onFalse(new InstantCommand(() -> MaxSpeed = maxSpeedConstant));
    Lv2L.onFalse(new InstantCommand(() -> MaxAngularRate = maxAngularRateConstant));
    Lv2R.onFalse(new MoveArmFix(mArm, ace, 44, 0));
    // .andThen(new InstantCommand(() -> ace.setSpeed(1))));
    Lv2R.onFalse(new InstantCommand(() -> MaxSpeed = maxSpeedConstant));
    Lv2R.onFalse(new InstantCommand(() -> MaxAngularRate = maxAngularRateConstant));

    // ******** True ****** */
    Lv3L.whileTrue(new MoveArmFix(mArm, ace, 3, -1));
    Lv3L.onTrue(new InstantCommand(() -> MaxSpeed = maxSpeedConstant / 3));
    Lv3L.onTrue(new InstantCommand(() -> MaxAngularRate = maxAngularRateConstant / 2));
    Lv3L.onTrue(new InstantCommand(() -> rightTree = false));

    Lv3R.whileTrue(new MoveArmFix(mArm, ace, 3, 1));
    Lv3R.onTrue(new InstantCommand(() -> MaxSpeed = maxSpeedConstant / 3));
    Lv3R.onTrue(new InstantCommand(() -> MaxAngularRate = maxAngularRateConstant / 2));
    Lv3R.onTrue(new InstantCommand(() -> rightTree = true));
    // ******** FALSE *** *****************************************/
    // Lv3L.onFalse(new Retract(mArm, 3).andThen(new MoveArmFix(mArm, 1, 0)));
    Lv3L.onFalse(new MoveArmFix(mArm, ace, 44, 0));
    // .andThen(new InstantCommand(() -> ace.setSpeed(1))));
    Lv3L.onFalse(new InstantCommand(() -> MaxSpeed = maxSpeedConstant));
    Lv3L.onFalse(new InstantCommand(() -> MaxAngularRate = maxAngularRateConstant));

    // Lv3R.onFalse(new Retract(mArm, 3).andThen(new MoveArmFix(mArm, 1, 0)));
    Lv3R.onFalse(new MoveArmFix(mArm, ace, 44, 0));
    // .andThen(new InstantCommand(() -> ace.setSpeed(1))));
    Lv3R.onFalse(new InstantCommand(() -> MaxSpeed = maxSpeedConstant));
    Lv3R.onFalse(new InstantCommand(() -> MaxAngularRate = maxAngularRateConstant));
    // *********TRUE *************************************** */
    Lv4L.onTrue(new MoveArmFix(mArm, ace, 4, -1));
    Lv4L.onTrue(new InstantCommand(() -> MaxSpeed = maxSpeedConstant / 4));
    Lv4L.onTrue(new InstantCommand(() -> MaxAngularRate = maxAngularRateConstant / 2.5));
    Lv4L.onTrue(new InstantCommand(() -> rightTree = false));

    Lv4R.onTrue(new MoveArmFix(mArm, ace, 4, 1));
    Lv4R.onTrue(new InstantCommand(() -> MaxSpeed = maxSpeedConstant / 4));
    Lv4R.onTrue(new InstantCommand(() -> MaxAngularRate = maxAngularRateConstant / 2.5));
    Lv4R.onTrue(new InstantCommand(() -> rightTree = true));
    // *********FALSE **************************************************/
    Lv4L.onFalse(new MoveArmFix(mArm, ace, 44, 0).andThen(new WaitCommand(0.1))
        .andThen(new InstantCommand(() -> {

          MaxSpeed = maxSpeedConstant;

          MaxAngularRate = maxAngularRateConstant;

        })));

    // .andThen(new InstantCommand(() -> ace.setSpeed(1))));
    // Lv4L.onFalse(new InstantCommand(() -> MaxSpeed = MaxSpeed * 4));
    // Lv4L.onFalse(new InstantCommand(() -> MaxAngularRate = MaxAngularRate * 2));
    Lv4R.onFalse(new MoveArmFix(mArm, ace, 44, 0).andThen(new WaitCommand(0.1))
        .andThen(new InstantCommand(() -> {
          MaxSpeed = maxSpeedConstant;
          MaxAngularRate = maxAngularRateConstant;

        })));

    Intake
        .whileTrue(new InstantCommand(() -> ace.setSpeed(1)));
    Intake
        .onFalse(new InstantCommand(() -> ace.setSpeed(0)));
    Outtake
        .whileTrue(new InstantCommand(() -> ace.setSpeed(-0.5)));
    Outtake
        .onFalse(new InstantCommand(() -> ace.setSpeed(0)));

    controller.rightBumper()

        .onTrue(new InstantCommand(() -> {

          driveIt = Utilitys.driveToIt(drivetrain, true);// rightTree

          if (driveIt != null) {
            driveIt.schedule();

          }

        }));
    controller.rightBumper()
        .onFalse(new InstantCommand(() -> {

          if (driveIt != null) {
            driveIt.cancel();
          }
        }));

    controller.leftBumper().onTrue(

        new InstantCommand(() -> {

          driveIt = Utilitys.driveToIt(drivetrain, false);
          if (driveIt != null) {
            driveIt.schedule();
          }
        }));
    controller.leftBumper()
        .onFalse(new InstantCommand(() -> {

          if (driveIt != null) {
            driveIt.cancel();
          }
        }));

    controller
        .start()
        .onTrue(new InstantCommand(() -> drivetrain.gyro.reset()));
    controller.start()
        .onTrue(new InstantCommand(() -> drivetrain.setHeading(new Rotation2d(0))));

    controller.b().whileTrue(drivetrain.applyRequest(
        () -> point.withModuleDirection(
            new Rotation2d(-controller.getLeftY(), -controller.getLeftX()))));

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
