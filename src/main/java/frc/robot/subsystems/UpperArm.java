// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;

public class UpperArm extends SubsystemBase {

  TalonFX UpperArmLeft;
  TalonFX UpperArmRight;
  Follower UpperArmRightFollower;
  PositionDutyCycle motorRequest;
  double requestedPosition;
  boolean atPosition;
  MotionMagicVoltage controlUpper;
  TalonFXConfiguration talonFXConfigs;
  // MotionMagicVoltage controlUpperRight;
  DynamicMotionMagicVoltage dynamic = new DynamicMotionMagicVoltage(0, 10, 60, 200);
  // PID coefficients
  double kP = 10.0;
  double kI = 0.0;
  double kD = 0.000;
  double maxVel = 200;
  double maxAcc = 300;
  double minVel = 0;
  double kJerk = 800;
  boolean change = false;
  boolean updatePID = false;
  // TalonFXConfigurator leftConfigurator;

  /** Creates a new UpperArm. */
  public UpperArm() {

    talonFXConfigs = new TalonFXConfiguration();
    controlUpper = new MotionMagicVoltage(0);
    UpperArmLeft = new TalonFX(33);
    UpperArmRight = new TalonFX(34);
    var slot0Configs = talonFXConfigs.Slot0;
    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    
    slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = kP; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = kI; // no output for integrated error
    slot0Configs.kD = kD; // A velocity error of 1 rps results in 0.1 V output

    // set Motion Magic settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = maxVel; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration = maxAcc; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = kJerk; // Target jerk of 1600 rps/s/s (0.1 seconds)

    UpperArmLeft.getConfigurator().apply(talonFXConfigs);
    UpperArmRight.getConfigurator().apply(talonFXConfigs);

var motorConfigs = new MotorOutputConfigs();
    motorConfigs.Inverted = InvertedValue.Clockwise_Positive;
    
    // LowerArmRightFollower = new Follower(31, true);
    // LowerArmRight.setControl(LowerArmRightFollower);
    
    UpperArmRight.getConfigurator().apply(motorConfigs); 
    
    UpperArmLeft.setNeutralMode(NeutralModeValue.Brake);
    UpperArmRight.setNeutralMode(NeutralModeValue.Brake);




//    UpperArmRightFollower = new Follower(33, true);
//    UpperArmRight.setControl(UpperArmRightFollower);
    requestedPosition = getPos();
  }

  public void setPos(double position) {
    UpperArmLeft.setControl(controlUpper.withPosition(position));
    UpperArmRight.setControl(controlUpper.withPosition(position));
    // motorRequest = new PositionDutyCycle(position);
    // UpperArmLeft.setControl(motorRequest);
    requestedPosition = position;

  }
  public void setSpeed(double speed) {
   // UpperArmLeft.set(speed);
  }

  public double getPos() {
    return UpperArmLeft.getRotorPosition().getValueAsDouble();
  }
  public double getRightPos(){
    return UpperArmRight.getRotorPosition().getValueAsDouble();
  }
  public boolean atPos(TalonFX talon) {
    return Math.abs(talon.getRotorPosition().getValueAsDouble() - requestedPosition) < 0.8;
  }
  public boolean atPos() {
    return atPosition;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("upArm", getPos());
   
    if(atPos(UpperArmLeft) && atPos(UpperArmRight)){
      atPosition = true;  
     } else {
      atPosition = false;
    }
    // This method will be called once per scheduler run
  }

  // This method will be called once per scheduler run
}