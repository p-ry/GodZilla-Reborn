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
  double kS = .25;
  public static double maxVel = 30;
  public static double maxAcc = 30;
  public static double minVel = 0;
  public static double kJerk = 400;
  boolean change = false;
  boolean updatePID = false;
  boolean fast;
  public static double fastVel = 300;
  public static double fastAcc = 300;
  public static double fastJerk = 800;
  public static double slowVel = 150;
  public static double slowAcc = 300;
  public static double slowJerk = 300;
  
  
  public static DynamicMotionMagicVoltage dynamic = new DynamicMotionMagicVoltage(0, 80, 300, 800);
  private Slot0Configs pidConfigs = new Slot0Configs();
  private MotionMagicConfigs mmConfigs = new MotionMagicConfigs();
  

  // TalonFXConfigurator leftConfigurator;

  /** Creates a new UpperArm. */
  public UpperArm() {

    this.fast = false;
    
    talonFXConfigs = new TalonFXConfiguration();
    controlUpper = new MotionMagicVoltage(0);
    UpperArmLeft = new TalonFX(33,"Canivore2");
    UpperArmRight = new TalonFX(34,"Canivore2");
    pidConfigs = talonFXConfigs.Slot0;
    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    
    
    
    pidConfigs.kS = kS; // Add 0.25 V output to overcome static friction
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

  }

  public void setPos(double position) {
    setPos(position,fast);
  }

  public void setPos(double position, boolean fast) {
   
   
    this.fast = fast;
    requestedPosition = position;
    if (fast){
      UpperArmLeft.setControl(dynamic.withVelocity(fastVel).withAcceleration(fastAcc).withJerk(fastJerk).withPosition(position));
      UpperArmRight.setControl(dynamic.withVelocity(fastVel).withAcceleration(fastAcc).withJerk(fastJerk).withPosition(position));
    }else {
    UpperArmLeft.setControl(dynamic.withVelocity(slowVel).withAcceleration(slowAcc).withJerk(slowJerk).withPosition(position));
      UpperArmRight.setControl(dynamic.withVelocity(slowVel).withAcceleration(slowAcc).withJerk(slowJerk).withPosition(position));
    }

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
    return Math.abs(talon.getPosition().getValueAsDouble() - requestedPosition) < 1.0;
  }
  public boolean atPos() {
    return atPosition;
  }
public void updatePID(){
 // UpperArmLeft.getConfigurator().apply(pidConfigs);
 // UpperArmRight.getConfigurator().apply(pidConfigs);
  
  
}
  @Override
  public void periodic() {
    
    
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
    builder.publishConstDouble("Velocity", slowVel);
    
    // builder.addDoubleProperty("Output Voltage", () ->
    // wrist.getMotorVoltage().getValueAsDouble(), null);
    // PID Tuning
    builder.publishConstBoolean("Fast", fast);
    builder.publishConstBoolean("AtPosition", atPosition);
    
    
    builder.addDoubleProperty("kP", () -> kP, (value) -> { kP = value; });
    builder.addDoubleProperty("kI", () -> kI, (value) -> { kI = value; });
    builder.addDoubleProperty("kD", () -> kD, (value) -> { kD = value; });
    builder.addDoubleProperty("kS", () -> kS, (value) -> { kS= value;  });
    builder.addDoubleProperty("MMVel", () -> slowVel, (value) -> {
      slowVel = value;});
      
    builder.addDoubleProperty("MMAccel", () -> slowAcc, (val) -> {
      slowAcc= val;});

    builder.addDoubleProperty("MMJerk", () -> slowJerk, (val) -> {
      slowJerk = val;});
       // Update PID dynamically
       builder.addBooleanProperty("Update", () -> false, (pressed) -> updatePID());
    
  }
  
}
