// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

//import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.RobotCentric;
//import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RobotCentricDriveCommand extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final CommandXboxController controller;
  private final SwerveRequest.RobotCentric driveRequest;

  public RobotCentricDriveCommand(CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric robotCentric, CommandXboxController controller) {
      this.drivetrain = drivetrain;
      this.controller = controller;
      this.driveRequest =  robotCentric;
      //.withDeadband(RobotContainer.MaxSpeed*0.1)
      //.withRotationalDeadband(RobotContainer.MaxAngularRate*0.1)
      //.withDriveRequestType(DriveRequestType.Velocity);
      //addRequirements(drivetrain);//robotCentric;
      
  }

  @Override
  public void execute() {
    System.out.println("Robot");
      drivetrain.applyRequest(() -> driveRequest
          .withVelocityX(-(controller.getLeftY() * controller.getLeftY() * Math.signum(controller.getLeftY())) * RobotContainer.MaxSpeed/2)
          .withVelocityY(-(controller.getLeftX() * controller.getLeftX() * Math.signum(controller.getLeftX())) * RobotContainer.MaxSpeed/2)
          .withRotationalRate(-controller.getRightX() * RobotContainer.MaxAngularRate/2)
      );
      SmartDashboard.putNumber("leftY",controller.getLeftY());
  }

  @Override
  public boolean isFinished() {
      return false; // Runs while held
  }
}

