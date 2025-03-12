// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/* import Miracle.java;
*/
package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.io.Console;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DriveToAmpPath;
import frc.robot.commands.Extend;
import frc.robot.commands.MoveArmFix;
//import frc.robot.commands.MoveArmFix;
import frc.robot.commands.Retract;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Ace;
import frc.robot.subsystems.ArmAssembly;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Wrist;

public class RobotContainer {
          //public static Pigeon2 gyro;
        private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                      // speed
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                          // second
                                                                                          // max angular velocity

        /* Setting up bindings for necessary control of the swerve drive platform */
        /* private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors

            */
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
        private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        // private final Telemetry logger = new Telemetry(MaxSpeed);

        // ace.setBrakeMode(true);

        // private final CommandXboxController driver = new CommandXboxController(0);
        private static final Joystick copilot = new Joystick(1);
        private final Joystick copilot2 = new Joystick(2);

        public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        public static final ArmAssembly mArm = new ArmAssembly(false, 99);
        public final Ace ace = new Ace(0);
        public static int prevLevel = 0;
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
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
        public static final JoystickButton Algae = new JoystickButton(copilot, 8);
        final JoystickButton Process = new JoystickButton(copilot2, 1);
        final JoystickButton Load = new JoystickButton(copilot, 12);
        // final JoystickButton Barge = new JoystickButton(copilot2, 2);
        // public final Wrist Wrist = new Wrist();
        private final CommandXboxController controller = new CommandXboxController(0);
        



        /* Path follower */
        private final SendableChooser<Command> AutoChooser;

        public RobotContainer() {
                //gyro = new Pigeon2(0, "Canivore");

                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> drive.withVelocityX(
                                                -(controller.getLeftY() * controller.getLeftY()
                                                                * Math.signum(controller.getLeftY()))
                                                                * MaxSpeed) // Drive
                                                // forward
                                                // with
                                                // negative
                                                // Y
                                                // (forward)
                                                .withVelocityY(-(controller.getLeftX() * controller.getLeftX()
                                                                * Math.signum(controller.getLeftX()) * MaxSpeed)) // Drive
                                                                                                                  // left
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
                AutoChooser = AutoBuilder.buildAutoChooser("none");
                SmartDashboard.putData("AutoChooser", AutoChooser);
                

                configureBindings();
        }

        private void configureBindings() {

                // controller
                // .rightBumper()
                // .onTrue(new InstantCommand())

                // Process
                // .onTrue(new MoveArmFix(mArm, 12));
                Algae
                                .onTrue(new InstantCommand(() -> ace.setSpeed(-.2)));
                Algae
                                .onFalse(new InstantCommand(() -> ace.setSpeed(0)));

                Load
                                .whileTrue(new MoveArmFix(mArm, 1, 0));
                Load
                                .onTrue(new InstantCommand(() -> ace.gotIt=false));
                                

                Load
                                .onFalse(new MoveArmFix(mArm, 0, 0));
                Process
                                .onTrue(new MoveArmFix(mArm, 5, 0));
                // Process.whileTrue(new InstantCommand(() -> ace.setSpeed(0.1)));

                Process
                                .onFalse(new MoveArmFix(mArm, 0, 0));
                // Process
                // .onFalse(new InstantCommand(() -> ace.setSpeed(0)));

                Dump
                                .whileTrue(new MoveArmFix(mArm, 6, 0));
                Dump
                                .onFalse(new MoveArmFix(mArm, 0, 0));
                Lv2L
                                .whileTrue(new MoveArmFix(mArm, 2, -1));
                Lv2L
                                .onFalse(new MoveArmFix(mArm, 0, 0));
                Lv2R
                                .whileTrue(new MoveArmFix(mArm, 2, 1));
                Lv2R
                                .onFalse(new MoveArmFix(mArm, 0, 0));
                Lv3L
                                .onTrue(new MoveArmFix(mArm, 3, -1));
                Lv3L
                        .onFalse(new Retract(mArm,3).andThen(new MoveArmFix(mArm, 0, 0)));
                Lv3R
                                .onTrue(new MoveArmFix(mArm, 3, 1));
                Lv3R
                .onFalse(new Retract(mArm,3).andThen(new MoveArmFix(mArm, 0, 0)));
                Lv4L
                                .onTrue(new MoveArmFix(mArm, 4, -1).andThen(new Extend(mArm,4)));
                Lv4L
                .onFalse(new Retract(mArm,4).andThen(new Extend(mArm, 99). andThen(new MoveArmFix(mArm, 0, 0))));
                Lv4R
                                .onTrue(new MoveArmFix(mArm, 4, 1).andThen(new Extend(mArm,4)));
                Lv4R
                .onFalse(new Retract(mArm,4).andThen(new Extend(mArm, 99). andThen(new MoveArmFix(mArm, 0, 0))));
                /*
                 * Climb need to find correct Position
                 * .onTrue(new MoveArmFix(mArm, 2));
                 * Pull
                 * .onTrue(new MoveArmFix(mArm, 2));
                 */
                Intake
                                .whileTrue(new InstantCommand(() -> ace.setSpeed(1)));
                Intake
                                .onFalse(new InstantCommand(() -> ace.setSpeed(0)));
                Outtake
                                .whileTrue(new InstantCommand(() -> ace.setSpeed(-1.0)));
                Outtake
                                .onFalse(new InstantCommand(() -> ace.setSpeed(0)));
                /*
                 * Process need to find corect Position
                 * .onTrue(new MoveArmFix(mArm, 2));
                 * Load
                 * .onTrue(new MoveArmFix(mArm, 2));
                 * Barge
                 * .onTrue(new MoveArm(mArm, 2));
                 */

                controller.rightBumper()
                                .onTrue(new DriveToAmpPath(1));
                controller.leftBumper().onTrue(new InstantCommand(() -> 
                        drivetrain.resetGyro()));

controller.leftBumper().onTrue(new InstantCommand(() -> drivetrain.setHeading(new Rotation2d(0))));

                

                controller
                                .start()
                                .onTrue(new InstantCommand(() -> drivetrain.gyro.reset()));

            
                controller.b().whileTrue(drivetrain.applyRequest(
                                () -> point.withModuleDirection(
                                                new Rotation2d(-controller.getLeftY(), -controller.getLeftX()))));

                controller.pov(0).whileTrue(
                                drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
                controller.pov(180)
                                .whileTrue(drivetrain.applyRequest(
                                                () -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

                // Run SysId routines when holding back/start and X/Y.
                // Note that each routine should be run exactly once in a single log.

                /*
                 * driver.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction
                 * .kForward));
                 * joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction
                 * .kReverse));
                 * joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(
                 * Direction.kForward));
                 * joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(
                 * Direction.kReverse));
                 * 
                 * joystick.a().whileTrue(new InstantCommand(() ->
                 * System.out.println(drivetrain.gyro.getYaw().getValueAsDouble())));
                 * 
                 * 
                 * // reset the field-centric heading on left bumper press
                 * joystick.leftBumper().onTrue(drivetrain.runOnce(() ->
                 * drivetrain.seedFieldCentric()));
                 */
                // drivetrain.registerTelemetry(logger::telemeterize);
        }

        public Command getAutonomousCommand() {
                /* Run the path selected from the auto chooser */
                return AutoChooser.getSelected();
        }
}
