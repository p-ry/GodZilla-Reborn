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
import frc.robot.Constants;
import au.grapplerobotics.LaserCan;

import static edu.wpi.first.units.Units.Newton;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfigurator;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

public class Slider extends SubsystemBase implements Sendable{
  TalonFXS slider;

  TalonFXSConfigurator sliderConfigurator;
  PositionDutyCycle sliderController;
  MotionMagicVoltage mmController;

  TalonFXSConfiguration sliderConfigs;
  double requestedPosition;
  boolean atPosition;
 
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM,  allowedErr;
  public double i, d, ff, aFF;
  private Slot0Configs pidConfigs = new Slot0Configs();
  private Slot1Configs pidConfigs2 = new Slot1Configs();
   private MotionMagicConfigs mmConfigs = new MotionMagicConfigs();
 
  public static double kJerk = 400;
// public static DynamicMotionMagicVoltage dynamic1 = new DynamicMotionMagicVoltage(0, maxVel,maxAcc,kJerk);
// public static DynamicMotionMagicVoltage dynamic2 = new DynamicMotionMagicVoltage(0, 800,1600, 600);
  
 public static DynamicMotionMagicVoltage dynamic = new DynamicMotionMagicVoltage(0, 300, 300, 800);
 
 public static  MotionMagicVoltage  mmControllerMagicVoltage  = new MotionMagicVoltage(-1.0);
 public static PositionVoltage sController;
 public static boolean fast = true;
 boolean updatePID = false;

 public static double fastVel = 300;
 public static double fastAcc = 900;
 public static double fastJerk = 4000;
 public static double slowVel = 60;
 public static double slowAcc = 900;
 public static double slowJerk = 1800;//1800
  

  /** Creates a new Slider. */
  public Slider() {

    slider = new TalonFXS(35,"Canivore2");
    //sliderController = new PositionDutyCycle(0);
    sController = new PositionVoltage(0);
    mmController  = new MotionMagicVoltage(0);
    sliderConfigs =   new TalonFXSConfiguration();

    sliderConfigs.Commutation.MotorArrangement=MotorArrangementValue.Minion_JST;
    sliderConfigs.MotorOutput.Inverted =InvertedValue.Clockwise_Positive;
    //sliderConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = ReverseLimitValue
    pidConfigs = sliderConfigs.Slot0;
    pidConfigs2 = sliderConfigs.Slot1;
    pidConfigs.kP = 3.0;
    pidConfigs2.kP = 0.02;
    pidConfigs.kS=0.5;
    pidConfigs.kV=0.15;

    mmConfigs= sliderConfigs.MotionMagic;
    mmConfigs.MotionMagicCruiseVelocity = fastVel; // Target cruise velocity of 80 rps
    mmConfigs.MotionMagicAcceleration = fastAcc; // Target acceleration of 160 rps/s (0.5 seconds)
    mmConfigs.MotionMagicJerk = fastJerk; // Target jerk of 1600 rps/s/s (0.1 seconds)
    slider.getConfigurator().apply(sliderConfigs);
    slider.setNeutralMode(NeutralModeValue.Brake);
    ShuffleboardTab tab = Shuffleboard.getTab("Arms");
    tab.add("Slider", this);
    SmartDashboard.putData(slider);
    
    

  }

  public void setPos(double position) {
    setPos(position, true);
  }


  public void setPos(double position,boolean fast) {
    
      this.fast = fast;
      requestedPosition = position;
      if (fast){
        slider.setControl(dynamic.withVelocity(fastVel).withAcceleration(fastAcc).withJerk(fastJerk).withPosition(position));
     }else {
      slider.setControl(dynamic.withVelocity(slowVel).withAcceleration(slowAcc).withJerk(slowJerk).withPosition(position));
      }
  
    }
   
   
  public double getPos() {
    return slider.getPosition().getValueAsDouble();

  }

  public boolean atPos() {
    return atPosition;
  }
  public void updatePID(){
    pidConfigs.kP = kP;
    slider.getConfigurator().apply(pidConfigs);
    
  }

  @Override
  public void periodic() {
   
    if (Math.abs(getPos() - requestedPosition) < 1.0) {
      atPosition = true;
    } else {
      atPosition = false;
    }
    
    
    // This method will be called once per scheduler run
  }
  
@Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Slider");
    // builder.addDoubleProperty("Velocity RPM", () ->
    // wrist.getVelocity().getValueAsDouble() * 60, null);
    builder.addDoubleProperty("Position", () -> slider.getPosition().getValueAsDouble(), null);


    builder.addDoubleProperty("Setpoint", () -> requestedPosition, this::setPos);
    builder.publishConstBoolean("AtPosition",atPosition);
    builder.publishConstBoolean("Fast", fast);
    builder.publishConstDouble("Velocity", slowVel);
    builder.publishConstDouble("RPS", slider.getVelocity().getValueAsDouble());
    
    builder.addDoubleProperty("LeftOffset", () -> Constants.leftOffset,(value) ->Constants.leftOffset = value);// builder.addDoubleProperty("Output Voltage", () ->
    builder.addDoubleProperty("RightOffset", () -> Constants.rightOffset,(value) ->Constants.rightOffset = value);// builder.addDoubleProperty("Output Voltage", () ->
    builder.addDoubleProperty("ForwardOffset", () -> Constants.forwardOffset,(value) ->Constants.forwardOffset = value);// builder.addDoubleProperty("Output Voltage", () ->
    // wrist.getMotorVoltage().getValueAsDouble(), null);
    // PID Tuning
    builder.addDoubleProperty("kP", () -> kP, (value) ->kP = value);
    //  slider.getConfigurator().apply(pidConfigs);
    
    
    builder.addDoubleProperty("kI", () -> pidConfigs.kI, (val) -> {
      pidConfigs.kI = val;
      slider.getConfigurator().apply(pidConfigs);
     
    });
    builder.addDoubleProperty("kD", () -> pidConfigs.kD, (val) -> {
      pidConfigs.kD = val;
      slider.getConfigurator().apply(pidConfigs);
      
    });
    builder.addDoubleProperty("kF", () -> pidConfigs.kV, (val) -> {
      pidConfigs.kV = val;
      //slider.getConfigurator().apply(pidConfigs);
     
    });
    builder.addDoubleProperty("MMVel", () -> slowVel, (val) -> {
      slowVel = val;
      //slider.getConfigurator().apply(mmConfigs );
     
    });
    builder.addDoubleProperty("MMAccel", () -> slowAcc, (val) -> {
      slowAcc= val;
      //slider.getConfigurator().apply(mmConfigs );
      
    });
    builder.addDoubleProperty("MMJerk", () -> slowJerk, (val) -> {
      slowJerk = val;
     // slider.getConfigurator().apply(mmConfigs );
     
    });
    builder.addBooleanProperty("ApplyPID",()-> false,(pressed) ->updatePID());
    
  }
}
