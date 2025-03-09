// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.imageio.plugins.tiff.ExifTIFFTagSet;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

/** Add your docs here. */
public class SwerveMods {
    //public SwerveModule

public static void setDesiredState(SwerveModule module, SwerveModuleState desiredState) {
    double speed = desiredState.speedMetersPerSecond;
    double angle = desiredState.angle.getDegrees();

    // Set drive motor velocity (assumes proper velocity conversion)
    module.getDriveMotor().setControl(new VelocityVoltage(speed));

    // Set steering motor position (angle in degrees)
    module.getSteerMotor().setControl(new PositionVoltage(angle));
}


public static double getDriveMotorSpeed(double speed) {
    // Get the encoder counts per 100ms (this will depend on your motor's setup)
    double velocity= speed/6.75*0.319185;
  
    return velocity;
    
}


public static SwerveModuleState getState(SwerveModule module) {
double velocity= getDriveMotorSpeed(module.getDriveMotor().getVelocity().getValueAsDouble());
Rotation2d angle = Rotation2d.fromRotations(module.getSteerMotor().getPosition().getValueAsDouble()/TunerConstants.kSteerGearRatio);




        return new SwerveModuleState(velocity,angle);

            


    }

}
