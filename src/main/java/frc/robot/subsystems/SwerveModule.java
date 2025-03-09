// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

public class SwerveModule {
    private final TalonFX driveMotor;
    private final TalonFX turnMotor;

    // Kraken 60 gear ratios & wheel circumference
    private static final double DRIVE_GEAR_RATIO = 6.75; 
    private static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(3.93);
    private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_METERS * Math.PI;

    private final VelocityVoltage driveControl = new VelocityVoltage(0);
    private final MotionMagicVoltage turnControl = new MotionMagicVoltage(0);

    public SwerveModule(int driveMotorID, int turnMotorID) {
        driveMotor = new TalonFX(driveMotorID);
        turnMotor = new TalonFX(turnMotorID);

        // Configure drive motor
        driveMotor.getConfigurator().apply(new com.ctre.phoenix6.configs.TalonFXConfiguration());
        driveMotor.setPosition(0); // Reset encoder
        
        // Configure turn motor
        turnMotor.getConfigurator().apply(new com.ctre.phoenix6.configs.TalonFXConfiguration());
        turnMotor.setPosition(0); // Reset encoder
    }

    public SwerveModulePosition getPosition() {
        double driveMeters = driveMotor.getPosition().getValueAsDouble() * (WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIO);
        double turnRadians = Math.toRadians(turnMotor.getPosition().getValueAsDouble() % 360);
        
        return new SwerveModulePosition(driveMeters, new Rotation2d(turnRadians));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize state to prevent unnecessary movement
        
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getPosition().angle);

        // Convert speed (m/s) to motor velocity (rotations per second)
        double driveVelocity = optimizedState.speedMetersPerSecond / (WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIO);
        double turnPosition = Math.toDegrees(optimizedState.angle.getRadians());

        // Apply motor outputs
        driveMotor.setControl(driveControl.withVelocity(driveVelocity));
        turnMotor.setControl(turnControl.withPosition(turnPosition));
    }
}

/** Add your docs here. */
//public class SwerveModule {}
