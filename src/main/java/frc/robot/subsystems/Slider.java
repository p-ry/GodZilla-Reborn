package frc.robot.subsystems;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.*;

public class Slider extends SubsystemBase implements Sendable {

  private final TalonFXS slider;
  private final TalonFXSConfigurator sliderConfigurator;
  private final TalonFXSConfiguration sliderConfigs;

  private final Slot0Configs pidConfigs;
  private final MotionMagicConfigs mmConfigs;

  private static final DynamicMotionMagicVoltage dynamic = new DynamicMotionMagicVoltage(0, 300, 300, 800);
  private static final PositionVoltage sController = new PositionVoltage(0);

  
  private boolean fast = true;
  private double requestedPosition = 0;
  private double cachedPosition = 0;
  private boolean atPosition = false;
  private boolean updatePending = false;
  private final VelocityDutyCycle velocityRequest = new VelocityDutyCycle(0).withSlot(0);
  
 private static double velocitySetpoint = 0;

  // Tunable PID constants
  public double kP = 2.5, kI = 0.0, kD = 0.0, kV = 0.25, kS = 0.6;

  // Motion Magic profiles
  public static double fastVel = 300, fastAcc = 600, fastJerk = 2000;
  public static double slowVel = 60, slowAcc = 900, slowJerk = 1800;

  public Slider() {
    slider = new TalonFXS(35, "Canivore2");
    sliderConfigurator = slider.getConfigurator();
    sliderConfigs = new TalonFXSConfiguration();

    // Motor config
    sliderConfigs.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    sliderConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    sliderConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // PID
    pidConfigs = sliderConfigs.Slot0;
    pidConfigs.kP = kP;
    pidConfigs.kI = kI;
    pidConfigs.kD = kD;
    pidConfigs.kV = kV;
    pidConfigs.kS = kS;

    // Motion Magic
    mmConfigs = sliderConfigs.MotionMagic;
    mmConfigs.MotionMagicCruiseVelocity = fastVel;
    mmConfigs.MotionMagicAcceleration = fastAcc;
    mmConfigs.MotionMagicJerk = fastJerk;

    sliderConfigurator.apply(sliderConfigs);
    if (Constants.enableShuffleboard) {
      ShuffleboardTab tab = Shuffleboard.getTab("Arms");
      tab.add("Wrist", this);
  }
  
  }

  public void setBrakeMode(NeutralModeValue mode) {
    slider.getConfigurator().refresh(sliderConfigs);
    sliderConfigs.MotorOutput.NeutralMode = mode;
    slider.getConfigurator().apply(sliderConfigs);
  }

  public void setTargetVelocityRPS(double velocityRPS) {
    velocitySetpoint = velocityRPS;
    velocityRequest.Velocity = velocitySetpoint;
    slider.setControl(velocityRequest);
    SmartDashboard.putNumber("SliderRPSCmd", velocitySetpoint);
    
  }

  public void stop(TalonFXS motor) {
    motor.stopMotor();
  }

  public double getCurrentVelocity(TalonFXS motor) {
    return motor.getVelocity().getValueAsDouble();
  }

  public boolean atTargetVelocity(TalonFXS motor, double targetRPS, double tolerance) {
    return Math.abs(getCurrentVelocity(motor) - targetRPS) < tolerance;
  }



  public void setPos(double position) {
    setPos(position, true);
  }

  public void setPos(double position, boolean fast) {
    this.fast = fast;
    this.requestedPosition = position;
    slider.setControl(
      dynamic
        .withVelocity(fast ? fastVel : slowVel)
        .withAcceleration(fast ? fastAcc : slowAcc)
        .withJerk(fast ? fastJerk : slowJerk)
        .withPosition(position)
    );
  }

  public double getPos() {
    return cachedPosition;
  }

  public boolean atPos() {
    return atPosition;
  }

  public void updatePID() {
    pidConfigs.kP = kP;
    pidConfigs.kI = kI;
    pidConfigs.kD = kD;
    pidConfigs.kV = kV;
    pidConfigs.kS = kS;
    sliderConfigurator.apply(pidConfigs);
  }

  @Override
  public void periodic() {
    cachedPosition = slider.getPosition().getValueAsDouble();
    atPosition = Math.abs(cachedPosition - requestedPosition) < 1.0;

    if (updatePending) {
      updatePID();
      updatePending = false;
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Slider");

    builder.addDoubleProperty("Position", this::getPos, null);
    builder.addDoubleProperty("Setpoint", () -> requestedPosition, this::setPos);
    builder.addBooleanProperty("AtPosition", () -> atPosition, null);
    builder.addBooleanProperty("Fast", () -> fast, null);
    builder.addDoubleProperty("VelocityRPS", () -> slider.getVelocity().getValueAsDouble(), null);

    // Motion Magic profile tuning
    builder.addDoubleProperty("MMVel", () -> slowVel, (val) -> {
      if (slowVel != val) {
        slowVel = val;
      }
    });
    builder.addDoubleProperty("MMAccel", () -> slowAcc, (val) -> {
      if (slowAcc != val) {
        slowAcc = val;
      }
    });
    builder.addDoubleProperty("MMJerk", () -> slowJerk, (val) -> {
      if (slowJerk != val) {
        slowJerk = val;
      }
    });

    // PID tuning
    builder.addDoubleProperty("kP", () -> kP, (val) -> {
      if (kP != val) {
        kP = val;
        updatePending = true;
      }
    });
    builder.addDoubleProperty("kI", () -> kI, (val) -> {
      if (kI != val) {
        kI = val;
        updatePending = true;
      }
    });
    builder.addDoubleProperty("kD", () -> kD, (val) -> {
      if (kD != val) {
        kD = val;
        updatePending = true;
      }
    });
    builder.addDoubleProperty("kF", () -> kV, (val) -> {
      if (kV != val) {
        kV = val;
        updatePending = true;
      }
    });

    // Field offsets
    builder.addDoubleProperty("LeftOffset", () -> Constants.leftOffset, (val) -> {
      if (Constants.leftOffset != val) Constants.leftOffset = val;
    });
    builder.addDoubleProperty("RightOffset", () -> Constants.rightOffset, (val) -> {
      if (Constants.rightOffset != val) Constants.rightOffset = val;
    });
    builder.addDoubleProperty("ForwardOffset", () -> Constants.forwardOffset, (val) -> {
      if (Constants.forwardOffset != val) Constants.forwardOffset = val;
    });

    // Trigger PID apply manually
    builder.addBooleanProperty("ApplyPID", () -> false, (val) -> {
      if (val) updatePending = true;
    });
  }
}
