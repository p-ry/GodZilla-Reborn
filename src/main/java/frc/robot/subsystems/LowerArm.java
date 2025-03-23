// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
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
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;

public class LowerArm extends SubsystemBase implements Sendable{

  TalonFX LowerArmLeft;
  TalonFX LowerArmRight;
  Follower LowerArmRightFollower;
  PositionDutyCycle motorRequest;
  double requestedPosition;
  boolean atPosition;

  MotionMagicVoltage controlLower;
  TalonFXConfiguration talonFXConfigs;
  TalonFXConfigurator leftConfigurator;
  MotionMagicVoltage leftRequest, rightRequest;
  
  // PID coefficients
  double kP = 10.0;
  double kI = 0.0;
  double kD = 0.000;
  double kS = .25;
  static double maxVel = 200;
  static double maxAcc = 600;
  double minVel = 0;
  static double kJerk = 1000;
  boolean change = false;
  boolean updatePID = false;
  public static double fastVel = 200;
  public static double fastAcc = 600;
  public static double fastJerk = 1000;
  public static double slowVel = 150;
  public static double slowAcc = 600;
  public static double slowJerk = 600;
  
  boolean fast;
  
  public static DynamicMotionMagicVoltage dynamic = new DynamicMotionMagicVoltage(0, fastVel,fastAcc,fastJerk);
  
  private Slot0Configs pidConfigs = new Slot0Configs();
  private MotionMagicConfigs mmConfigs = new MotionMagicConfigs();

  /** Creates a new LowerArm. */
  public LowerArm() {
    // leftConfigurator
    LowerArmLeft = new TalonFX(31,"Canivore2");
    LowerArmRight = new TalonFX(32,"Canivore2");
    // talonFXConfigs = new TalonFXConfiguration();
 
    leftRequest = new MotionMagicVoltage(0);
    rightRequest = new MotionMagicVoltage(0);

    talonFXConfigs = new TalonFXConfiguration();
    talonFXConfigs.MotorOutput.NeutralMode=NeutralModeValue.Brake;

  
   //talonFXConfigsRight = new TalonFXConfiguration();

    LowerArmLeft.setNeutralMode(NeutralModeValue.Brake);
    LowerArmRight.setNeutralMode(NeutralModeValue.Brake);

    // set slot 0 gains
    pidConfigs = talonFXConfigs.Slot0;
    pidConfigs.kS = kS; // Add 0.25 V output to overcome static friction
    pidConfigs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    pidConfigs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    pidConfigs.kP = kP; // A position error of 2.5 rotations results in 12 V output
    pidConfigs.kI = kI; // no output for integrated error
    pidConfigs.kD = kD; // A velocity error of 1 rps results in 0.1 V output
    // set Motion Magic settings
    mmConfigs = talonFXConfigs.MotionMagic;
    mmConfigs.MotionMagicCruiseVelocity = maxVel; // Target cruise velocity of 80 rps
    mmConfigs.MotionMagicAcceleration = maxAcc; // Target acceleration of 160 rps/s (0.5 seconds)
    mmConfigs.MotionMagicJerk = kJerk; // Target jerk of 1600 rps/s/s (0.1 seconds)
   
     
    var rightMotorConfigs = new MotorOutputConfigs();
    rightMotorConfigs.Inverted = InvertedValue.Clockwise_Positive;

    
    // LowerArmRightFollower = new Follower(31, true);
    // LowerArmRight.setControl(LowerArmRightFollower);
    LowerArmLeft.getConfigurator().apply(talonFXConfigs);
    LowerArmRight.getConfigurator().apply(talonFXConfigs);
    LowerArmRight.getConfigurator().apply(rightMotorConfigs);    

    // leftConfigurator = LowerArmLeft.getConfigurator();
    // leftConfigurator.apply(talonFXConfigs);

    
 ShuffleboardTab tab = Shuffleboard.getTab("Arms");
    tab.add("LowerArm", this);
   

  }

  public void setPos(double position) {
    requestedPosition = position;
    LowerArmLeft.setControl(leftRequest.withPosition(position));
    LowerArmRight.setControl(rightRequest.withPosition(position));
    
  }
public void setPos(double position, boolean fast){
  SmartDashboard.putBoolean("Fast", fast);
  

  this.fast = fast;
  requestedPosition = position;
  if (fast){
    
 LowerArmLeft.setControl(dynamic.withVelocity(fastVel).withAcceleration(fastAcc).withJerk(fastJerk).withPosition(position));
 LowerArmRight.setControl(dynamic.withVelocity(fastVel).withAcceleration(fastAcc).withJerk(fastJerk).withPosition(position));
   
  }else {
    LowerArmLeft.setControl(dynamic.withVelocity(slowVel).withAcceleration(slowAcc).withJerk(slowJerk).withPosition(position));
    LowerArmRight.setControl(dynamic.withVelocity(slowVel).withAcceleration(slowAcc).withJerk(slowJerk).withPosition(position));
  }

 
 }


  public void setSpeed(double speed) {
    
   // LowerArmLeft.set(speed);
    //LowerArmRight.set(speed);
  }

  public double getPos() {
    return LowerArmLeft.getPosition().getValueAsDouble();
  }
  public double getRightPos() {
    return LowerArmRight.getPosition().getValueAsDouble();
  }
 public boolean atPos(TalonFX talon) {
    return Math.abs(talon.getPosition().getValueAsDouble() - requestedPosition) < 0.8;
  }
  public boolean atPos() {
    return atPosition;
  }

public void updatePID(){
  LowerArmLeft.getConfigurator().apply(pidConfigs);
  LowerArmRight.getConfigurator().apply(pidConfigs);
  
 
  
}

  @Override
  public void periodic() {
    //updatePID = SmartDashboard.getBoolean("update", updatePID);
    
     
    if (atPos(LowerArmLeft) && atPos(LowerArmRight)) {
      atPosition = true;
    } else {
      atPosition = false;
    }
   
    

  }
@Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("LowerArm");
    // builder.addDoubleProperty("Velocity RPM", () ->
    // wrist.getVelocity().getValueAsDouble() * 60, null);
    builder.addDoubleProperty("Position - Left", () -> LowerArmLeft.getPosition().getValueAsDouble(), null);
    builder.addDoubleProperty("Position - Right",()-> LowerArmRight.getPosition().getValueAsDouble(), null);
    builder.addDoubleProperty("Setpoint", () -> requestedPosition, this::setPos);
    // builder.addDoubleProperty("Output Voltage", () ->
    // wrist.getMotorVoltage().getValueAsDouble(), null);
    // PID Tuning
    builder.publishConstBoolean("Fast",fast);
    builder.publishConstBoolean("AtPosition",atPosition);
  

    builder.addDoubleProperty("kP", () -> kP, (value) -> { kP = value; });
    builder.addDoubleProperty("kI", () -> kI, (value) -> { kI = value; });
    builder.addDoubleProperty("kD", () -> kD, (value) -> { kD = value; });
    builder.addDoubleProperty("kS", () -> kS, (value) -> { kS= value;  });
    builder.addDoubleProperty("MMVel", () -> slowVel, (value) -> {
      slowVel= value;});
      
    builder.addDoubleProperty("MMAccel", () -> slowAcc, (val) -> {
     slowAcc = val;});
      
    builder.addDoubleProperty("MMJerk", () -> slowJerk, (val) -> {
      slowJerk = val;});
       // Update PID dynamically
       builder.addBooleanProperty("Update", () -> false, (pressed) -> updatePID());
  }
}
