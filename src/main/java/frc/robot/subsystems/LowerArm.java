package frc.robot.subsystems;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;

public class LowerArm extends SubsystemBase implements Sendable {

  private final TalonFX lowerLeft = new TalonFX(31, "Canivore2");
  private final TalonFX lowerRight = new TalonFX(32, "Canivore2");

  private final TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
  private final Slot0Configs pidConfigs;
  private final MotionMagicConfigs mmConfigs;

  private static final DynamicMotionMagicVoltage dynamic = new DynamicMotionMagicVoltage(0, 200, 600, 1000);
  private final MotionMagicVoltage leftRequest = new MotionMagicVoltage(0);
  private final MotionMagicVoltage rightRequest = new MotionMagicVoltage(0);

  private double cachedLeftPos = 0;
  private double cachedRightPos = 0;
  private double requestedPosition = 0;
  private boolean atPosition = false;
  private boolean updatePending = false;
  private boolean fast = true;

  // Tunable constants
  public double kP = 10.0, kI = 0.0, kD = 0.0, kS = 0.25;
  public static double fastVel = 200, fastAcc = 600, fastJerk = 1000;
  public static double slowVel = 150, slowAcc = 600, slowJerk = 600;

  public LowerArm() {
    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    talonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    pidConfigs = talonFXConfigs.Slot0;
    pidConfigs.kP = kP;
    pidConfigs.kI = kI;
    pidConfigs.kD = kD;
    pidConfigs.kS = kS;
    pidConfigs.kV = 0.12;
    pidConfigs.kA = 0.01;

    mmConfigs = talonFXConfigs.MotionMagic;
    mmConfigs.MotionMagicCruiseVelocity = fastVel;
    mmConfigs.MotionMagicAcceleration = fastAcc;
    mmConfigs.MotionMagicJerk = fastJerk;

    lowerLeft.getConfigurator().apply(talonFXConfigs);
    lowerRight.getConfigurator().apply(talonFXConfigs);

    ShuffleboardTab tab = Shuffleboard.getTab("Arms");
    tab.add("LowerArm", this);
  }

  public void setPos(double position) {
    setPos(position, true);
  }

  public void setPos(double position, boolean fastMode) {
    this.fast = fastMode;
    this.requestedPosition = position;

    double vel = fast ? fastVel : slowVel;
    double acc = fast ? fastAcc : slowAcc;
    double jerk = fast ? fastJerk : slowJerk;

    lowerLeft.setControl(dynamic.withVelocity(vel).withAcceleration(acc).withJerk(jerk).withPosition(position));
    lowerRight.setControl(dynamic.withVelocity(vel).withAcceleration(acc).withJerk(jerk).withPosition(position));
  }

  public double getPosLeft() {
    return cachedLeftPos;
  }

  public double getPosRight() {
    return cachedRightPos;
  }

  public boolean atPos() {
    return atPosition;
  }

  public void updatePID() {
    pidConfigs.kP = kP;
    pidConfigs.kI = kI;
    pidConfigs.kD = kD;
    pidConfigs.kS = kS;
    lowerLeft.getConfigurator().apply(pidConfigs);
    lowerRight.getConfigurator().apply(pidConfigs);
  }

  public void setBrakeMode(NeutralModeValue mode) {
    MotorOutputConfigs config = new MotorOutputConfigs();
    lowerLeft.getConfigurator().refresh(config);
    config.NeutralMode = mode;
    lowerLeft.getConfigurator().apply(config);
    lowerRight.getConfigurator().apply(config);
  }

  @Override
  public void periodic() {
    cachedLeftPos = lowerLeft.getPosition().getValueAsDouble();
    cachedRightPos = lowerRight.getPosition().getValueAsDouble();

    atPosition =
        Math.abs(cachedLeftPos - requestedPosition) < 1.0 &&
        Math.abs(cachedRightPos - requestedPosition) < 1.0;

    if (updatePending) {
      updatePID();
      updatePending = false;
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("LowerArm");

    builder.addDoubleProperty("Position - Left", this::getPosLeft, null);
    builder.addDoubleProperty("Position - Right", this::getPosRight, null);
    builder.addDoubleProperty("Setpoint", () -> requestedPosition, this::setPos);
    builder.addBooleanProperty("AtPosition", this::atPos, null);
    builder.addBooleanProperty("Fast", () -> fast, null);

    // Motion Magic tuning
    builder.addDoubleProperty("MMVel", () -> slowVel, (val) -> {
      if (slowVel != val) slowVel = val;
    });
    builder.addDoubleProperty("MMAccel", () -> slowAcc, (val) -> {
      if (slowAcc != val) slowAcc = val;
    });
    builder.addDoubleProperty("MMJerk", () -> slowJerk, (val) -> {
      if (slowJerk != val) slowJerk = val;
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
    builder.addDoubleProperty("kS", () -> kS, (val) -> {
      if (kS != val) {
        kS = val;
        updatePending = true;
      }
    });

    builder.addBooleanProperty("Update", () -> false, (pressed) -> {
      if (pressed) updatePending = true;
    });
  }
}
