// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Utilitys;
import frc.robot.subsystems.Ace;
import frc.robot.subsystems.ArmAssembly;
//import frc.robot.commands.Retract;
//import frc.robot.subsystems.LowerArm;
//import frc.robot.subsystems.Slider;
//import frc.robot.subsystems.UpperArm;
//import frc.robot.subsystems.Wrist;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveArmFix extends Command {
  /** Creates a new Retract. */

  double position;
  int level;
  int tagId;

  ArmAssembly myArm;
  int shiftDirection;

  boolean algae;
  public static boolean retract;
  public int prevLevel;
  double startTime;
  Pose2d aprilTag;
  public static boolean applyDynamic;
  public static boolean slow;

  public MoveArmFix(ArmAssembly myArm, int level, int direction) {

    this.level = level;
    this.shiftDirection = direction;

    this.myArm = myArm;
    aprilTag = new Pose2d();
    //slow = true;
  }

  /** Creates a new MoveArm. */

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    position = myArm.upperArm.getPos();
    prevLevel = myArm.level;
    tagId = Utilitys.grabTagID();
    // this.direction = direction;
    SmartDashboard.putNumber("TagID", tagId);
    if (tagId > 0) {
      aprilTag = Utilitys.getAprilTagPose(tagId);
      SmartDashboard.putNumberArray("AprilTag",
          new double[] { aprilTag.getX(), aprilTag.getY(), aprilTag.getRotation().getRadians() });

    }

    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // SmartDashboard.putNumber("Wristpos", wrist.getPos());
    algae = RobotContainer.Algae.getAsBoolean();
    applyDynamic = false;
    RobotContainer.loading = false;

    switch (level) {
      case 0:

        myArm.lowerArm.setPos(1.0); // 16.10 load
        if (algae) { // upperArm.dynamic.Velocity = 1;
                     // upperArm.dynamic.Acceleration = 1;
                     // upperArm.dynamic.Jerk = 50;
          myArm.upperArm.setPos(5, true);
          myArm.wrist.setPos(3.0);
        } else {
          myArm.upperArm.setPos(0, true);
          myArm.wrist.setPos(0.0);
        } // upperArm.UpperArmRight.setControl(upperArm.dynamic.withPosition(1.0));
        // upperArm.setPos(0.5);//0.0 load
        myArm.slider.setPos(0.50);

        // ace.setSpeed(0);

        // System.out.println("home");
        break;
      case 1:  //Load
        if (algae) {
          myArm.lowerArm.setPos(18.0);
          myArm.upperArm.setPos(1.0, true);
          myArm.slider.setPos(1.5);
          myArm.wrist.setPos(0.0);
        } else {
          RobotContainer.loading = true;
          myArm.lowerArm.setPos(18.00); //19
          myArm.upperArm.setPos(3.5, true);// 0.0 load  //3.7
          myArm.wrist.setPos(0 );
          myArm.slider.setPos(0.5);
        }
        // System.out.println("Level 1");
        break;
      case 2:
        if (algae) {
          myArm.lowerArm.setPos(1.0); // 16.10 load
          myArm.upperArm.setPos(5, true);
          myArm.slider.setPos(0.50);
          myArm.wrist.setPos(3.0);

          // myArm.lowerArm.setPos(22.0);
          // myArm.upperArm.setPos(13.0, applyDynamic);
          // myArm.slider.setPos(-2.0);
          // myArm.wrist.setPos(3.0);
        } else {

          myArm.lowerArm.setPos(1.0);
          myArm.upperArm.setPos(6.0, true);
          myArm.slider.setPos(6.0);
          myArm.wrist.setPos(5.0);
          break;
          /*
           * myArm.lowerArm.setPos(13.5);
           * upperArm.setPos(11.5);
           * myArm.slider.setPos(-0.5);
           * myArm.wrist.setPos(6.5);
           */
          // System.out.println("Level 2");
          // m_rob .setSpeed(1);
        }

        break;
      case 3:
        if (algae) {
          myArm.lowerArm.setPos(25.0);
          myArm.upperArm.setPos(20.0, false);
          myArm.slider.setPos(1.5);
          myArm.wrist.setPos(3.0);
        } else {
          myArm.lowerArm.setPos(20);
          myArm.upperArm.setPos(21, true);
          myArm.slider.setPos(0.5);
          myArm.wrist.setPos(6.5);
        }

        // System.out.println("Level 3");
        break;

      case 4:
        myArm.lowerArm.setPos(26.6, true);// myArm.lowerArm.setPos(27.2);// 28.2
        myArm.upperArm.setPos(33.0, true);
        myArm.slider.setPos(31.8,false);//30

         myArm.wrist.setPos(10.4);//9.4

        // System.out.println("Level 4");
        break;

      case 5:

        myArm.lowerArm.setPos(30.0);
        myArm.upperArm.setPos(4.0, applyDynamic);
        myArm.slider.setPos(0.50);
        myArm.wrist.setPos(3.0);

        // System.out.println("Level 2");
        // m_rob .setSpeed(1);
        break;
      case 6: // Level 1

        myArm.lowerArm.setPos(27.30);
        myArm.upperArm.setPos(8.9, applyDynamic);
        myArm.slider.setPos(0.50);
        myArm.wrist.setPos(0.0);
        break;

      case 7: // going from L4 to L0

        applyDynamic = true;
        myArm.lowerArm.setPos(1.0); // 16.10 load
        myArm.upperArm.setPos(position, applyDynamic);

        // upperArm.setPos(0.5);//0.0 load
        myArm.slider.setPos(0.50);
        myArm.wrist.setPos(0.0);

        // ace.setSpeed(0);

        System.out.println("WARNING!!!!  OLD MOVE!!!!!!!");
        break;
      case 12:
        myArm.wrist.setPos(myArm.wrist.getPos() + 1.0);
        break;
      case 42: // raise upperArm to release kickstand
        myArm.upperArm.setPos(16, applyDynamic);
        break;

      case 44: // lower from position 4 MOVE THAT SLIDER FAST
        myArm.lowerArm.setPos(18.00);  //19
        myArm.upperArm.setPos(3.5, false);// 0.0 load  //3.7
        myArm.wrist.setPos(0);
        myArm.slider.setPos(0.5, true);
        break;

      case 50: // chomp
        if (shiftDirection == 1) {
          myArm.wrist.setSpeed(0.2);
        } else {
          myArm.wrist.setSpeed(0);
        }
        break;

      default:
        if (algae) {
          myArm.lowerArm.setPos(1.0);
          myArm.upperArm.setPos(1.0, applyDynamic);
          myArm.slider.setPos(1.50);
          myArm.wrist.setPos(3.0);
        } else {

          myArm.lowerArm.setPos(1.0); // 16.10 load
          myArm.upperArm.setPos(1.0, applyDynamic);

          // myArm.upperArm.dynamic.Velocity = 1;
          // myArm.upperArm.dynamic.Acceleration = 1;
          // myArm.upperArm.dynamic.Jerk = 50;
          // myArm.upperArm.myArm.upperArmLeft.setControl(myArm.upperArm.dynamic.withPosition(1.0));
          // myArm.upperArm.myArm.upperArmRight.setControl(myArm.upperArm.dynamic.withPosition(1.0));
          // myArm.upperArm.setPos(0.5);//0.0 load
          myArm.slider.setPos(0.50, false);
          myArm.wrist.setPos(0.0);
        }

        // myArm.shiftDirection=shiftDirection;

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double elaspedTime = Timer.getTimestamp() - startTime;
    SmartDashboard.putBoolean("AtLevel", myArm.isAtLevel());
    return (myArm.isAtLevel()
        || (elaspedTime > 3.0));

  }
}
