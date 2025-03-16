// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;

public class UpperArm extends SubsystemBase implements Sendable{

  public static TalonFX UpperArmLeft;
  public static  TalonFX UpperArmRight;
  Follower UpperArmRightFollower;
  PositionDutyCycle motorRequest;
  double requestedPosition;
  boolean atPosition;
  MotionMagicVoltage controlUpper;
  TalonFXConfiguration talonFXConfigs;
  // MotionMagicVoltage controlUpperRight;
  // PID coefficients
  double kP = 10.0;
  double kI = 0.0;
  double kD = 0.000;
  public static double maxVel = 300;
  public static double maxAcc = 300;
  public static double minVel = 0;
  public static double kJerk = 800;
  boolean change = false;
  boolean updatePID = false;
  boolean applyDynamic;
  public static DynamicMotionMagicVoltage dynamic1 = new DynamicMotionMagicVoltage(0, maxVel,maxAcc,kJerk);
  
  public static DynamicMotionMagicVoltage dynamic2 = new DynamicMotionMagicVoltage(0, 10, 60, 200);
  private Slot0Configs pidConfigs = new Slot0Configs();
  private MotionMagicConfigs mmConfigs = new MotionMagicConfigs();
  

  // TalonFXConfigurator leftConfigurator;

  /** Creates a new UpperArm. */
  public UpperArm() {

    this.applyDynamic = false;
    
    talonFXConfigs = new TalonFXConfiguration();
    controlUpper = new MotionMagicVoltage(0);
    UpperArmLeft = new TalonFX(33);
    UpperArmRight = new TalonFX(34);
    pidConfigs = talonFXConfigs.Slot0;
    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    
    
    
    pidConfigs.kS = 0.25; // Add 0.25 V output to overcome static friction
    pidConfigs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    pidConfigs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    pidConfigs.kP = kP; // A position error of 2.5 rotations results in 12 V output
    pidConfigs.kI = kI; // no output for integrated error
    pidConfigs.kD = kD; // A velocity error of 1 rps results in 0.1 V output

    // set Motion Magic settings
    mmConfigs= talonFXConfigs.MotionMagic;
    mmConfigs.MotionMagicCruiseVelocity = maxVel; // Target cruise velocity of 80 rps
    mmConfigs.MotionMagicAcceleration = maxAcc; // Target acceleration of 160 rps/s (0.5 seconds)
    mmConfigs.MotionMagicJerk = kJerk; // Target jerk of 1600 rps/s/s (0.1 seconds)
    

    UpperArmLeft.getConfigurator().apply(talonFXConfigs);
    UpperArmRight.getConfigurator().apply(talonFXConfigs);

var rightMotorConfigs = new MotorOutputConfigs();
    rightMotorConfigs.Inverted = InvertedValue.Clockwise_Positive;
    
    // LowerArmRightFollower = new Follower(31, true);
    // LowerArmRight.setControl(LowerArmRightFollower);
    
    UpperArmRight.getConfigurator().apply(rightMotorConfigs);
    
    UpperArmLeft.setNeutralMode(NeutralModeValue.Brake);
    UpperArmRight.setNeutralMode(NeutralModeValue.Brake);

 ShuffleboardTab tab = Shuffleboard.getTab("Arms");
    tab.add("UpperArm", this);
    
    


//    UpperArmRightFollower = new Follower(33, true);
//    UpperArmRight.setControl(UpperArmRightFollower);
    //requestedPosition = getPos();
  }

  public void setPos(double position) {
    setPos(position, true);
  }

  public void setPos(double position, boolean applyDynamic) {
   
    this.applyDynamic = applyDynamic;
    requestedPosition = position;
    if (applyDynamic){
      UpperArmRight.setControl(dynamic1.withPosition(position));
      UpperArmLeft.setControl(dynamic1.withPosition(position));
    }else{
      
    UpperArmLeft.setControl(controlUpper.withPosition(position));
    UpperArmRight.setControl(controlUpper.withPosition(position));
    }
    // motorRequest = new PositionDutyCycle(position);
    // UpperArmLeft.setControl(motorRequest);
    

   

  }
  public void setSpeed(double speed) {
   // UpperArmLeft.set(speed);
  }

  public double getPos() {
    return UpperArmLeft.getPosition().getValueAsDouble();
  }
  public double getRightPos(){
    return UpperArmRight.getPosition().getValueAsDouble();
  }
  public boolean atPos(TalonFX talon) {
    return Math.abs(talon.getPosition().getValueAsDouble() - requestedPosition) < 0.8;
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

   @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("UpperArm");
    // builder.addDoubleProperty("Velocity RPM", () ->
    // wrist.getVelocity().getValueAsDouble() * 60, null);
    builder.addDoubleProperty("Position - Left", () -> UpperArmLeft.getPosition().getValueAsDouble(), null);
    builder.addDoubleProperty("Position - Right",()-> UpperArmRight.getPosition().getValueAsDouble(), null);
    builder.addDoubleProperty("Setpoint", () -> requestedPosition, this::setPos);
    // builder.addDoubleProperty("Output Voltage", () ->
    // wrist.getMotorVoltage().getValueAsDouble(), null);
    // PID Tuning
    builder.addDoubleProperty("kP", () -> pidConfigs.kP, (val) -> {
      pidConfigs.kP = val;
      UpperArmLeft.getConfigurator().apply(pidConfigs);
      UpperArmRight.getConfigurator().apply(pidConfigs);
    });
    builder.addDoubleProperty("kI", () -> pidConfigs.kI, (val) -> {
      pidConfigs.kI = val;
      UpperArmLeft.getConfigurator().apply(pidConfigs);
      UpperArmRight.getConfigurator().apply(pidConfigs);
    });
    builder.addDoubleProperty("kD", () -> pidConfigs.kD, (val) -> {
      pidConfigs.kD = val;
      UpperArmLeft.getConfigurator().apply(pidConfigs);
      UpperArmRight.getConfigurator().apply(pidConfigs);
    });
    builder.addDoubleProperty("kF", () -> pidConfigs.kV, (val) -> {
      pidConfigs.kV = val;
      UpperArmLeft.getConfigurator().apply(pidConfigs);
      UpperArmRight.getConfigurator().apply(pidConfigs);
    });
    builder.addDoubleProperty("MMVel", () -> mmConfigs.MotionMagicCruiseVelocity, (val) -> {
      mmConfigs.MotionMagicCruiseVelocity = val;
      UpperArmLeft.getConfigurator().apply(mmConfigs );
      UpperArmRight.getConfigurator().apply(mmConfigs);
    });
    builder.addDoubleProperty("MMAccel", () -> mmConfigs.MotionMagicAcceleration, (val) -> {
      mmConfigs.MotionMagicAcceleration = val;
      UpperArmLeft.getConfigurator().apply(mmConfigs );
      UpperArmRight.getConfigurator().apply(mmConfigs);
    });
    builder.addDoubleProperty("MMJerk", () -> mmConfigs.MotionMagicJerk, (val) -> {
      mmConfigs.MotionMagicJerk = val;
      UpperArmLeft.getConfigurator().apply(mmConfigs );
      UpperArmRight.getConfigurator().apply(mmConfigs);
    });
  }
  
}
