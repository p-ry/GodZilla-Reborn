package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.InitLogger;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DataLogManager;

import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.InvertedValue;

public abstract class DualArmSegmentBase extends SubsystemBase implements edu.wpi.first.util.sendable.Sendable {
  protected final TalonFX left;
  protected final TalonFX right;

  protected final TalonFXConfiguration leftConfig = new TalonFXConfiguration();
  protected final TalonFXConfiguration rightConfig = new TalonFXConfiguration();

  protected final Slot0Configs leftPID;
  protected final Slot0Configs rightPID;
  protected final MotionMagicConfigs leftMM;
  protected final MotionMagicConfigs rightMM;

  protected final DynamicMotionMagicVoltage dynamic;

  protected double cachedLeftPos = 0;
  protected double cachedRightPos = 0;
  protected double requestedPosition = 0;
  protected boolean atPosition = false;
  protected boolean fast = false;

  // Tuning parameters (mutable)
  protected double kP = 10.0;
  protected double kI = 0.0;
  protected double kD = 0.0;
  protected double kS = 0.25;

  protected double fastVel;
  protected double fastAcc;
  protected double fastJerk;
  protected double slowVel;
  protected double slowAcc;
  protected double slowJerk;

  protected double switchToFastThreshold = 12.0;
  protected double switchToSlowThreshold = 8.0;
  private final PositionDutyCycle motorPosRequest = new PositionDutyCycle(0).withSlot(2);
  private final VelocityDutyCycle velocityRequest = new VelocityDutyCycle(0).withSlot(1);
  private static double velocitySetpoint = 0;

  // Logging rate-limiter
  private double lastLogTime = 0;

  public DualArmSegmentBase(
      int leftId,
      int rightId,
      String canBusName,
      DynamicMotionMagicVoltage dynamic,
      double fastVel,
      double fastAcc,
      double fastJerk,
      double slowVel,
      double slowAcc,
      double slowJerk,
      boolean invertLeft,
      boolean invertRight) {
    this.left = new TalonFX(leftId, canBusName);
    this.right = new TalonFX(rightId, canBusName);
    this.dynamic = dynamic;

    this.fastVel = fastVel;
    this.fastAcc = fastAcc;
    this.fastJerk = fastJerk;
    this.slowVel = slowVel;
    this.slowAcc = slowAcc;
    this.slowJerk = slowJerk;

    // Refresh to get existing (current) configuration so unspecified fields are
    // preserved
    left.getConfigurator().refresh(leftConfig);
    right.getConfigurator().refresh(rightConfig);

    leftPID = leftConfig.Slot0;
    rightPID = rightConfig.Slot0;

    leftMM = leftConfig.MotionMagic;
    rightMM = rightConfig.MotionMagic;

    // Motor output defaults
    leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    leftConfig.MotorOutput.Inverted = invertLeft ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;
    rightConfig.MotorOutput.Inverted = invertRight ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;

    // PID initial values from mutable fields
    leftPID.kP = kP;
    leftPID.kI = kI;
    leftPID.kD = kD;
    leftPID.kS = kS;
    leftPID.kV = 0.12;
    leftPID.kA = 0.01;

    rightPID.kP = kP;
    rightPID.kI = kI;
    rightPID.kD = kD;
    rightPID.kS = kS;
    rightPID.kV = 0.12;
    rightPID.kA = 0.01;
    // Velocity slot (for velocity control mode)
    Slot1Configs leftPID1 = leftConfig.Slot1;
    Slot1Configs rightPID1 = rightConfig.Slot1;
    leftPID1.kP = 0.02;
    leftPID1.kI = 0.0;
    leftPID1.kD = 0.0;
    leftPID1.kS = 0.3;
    leftPID1.kV = 0.0;
    leftPID1.kA = 0.00;
    rightPID1.kP = 0.02;
    rightPID1.kI = 0.0;
    rightPID1.kD = 0.0;
    rightPID1.kS = 0.3;
    rightPID1.kV = 0.0;
    rightPID1.kA = 0.0;

    // Fast motion magic profile
    leftMM.MotionMagicCruiseVelocity = fastVel;
    leftMM.MotionMagicAcceleration = fastAcc;
    leftMM.MotionMagicJerk = fastJerk;

    rightMM.MotionMagicCruiseVelocity = fastVel;
    rightMM.MotionMagicAcceleration = fastAcc;
    rightMM.MotionMagicJerk = fastJerk;

    // Apply initial config
    left.getConfigurator().apply(leftConfig);
    right.getConfigurator().apply(rightConfig);

  }

  public void setBrakeMode(NeutralModeValue mode) {
    MotorOutputConfigs leftOut = new MotorOutputConfigs();
    left.getConfigurator().refresh(leftOut);
    leftOut.NeutralMode = mode;
    left.getConfigurator().apply(leftOut);

    MotorOutputConfigs rightOut = new MotorOutputConfigs();
    right.getConfigurator().refresh(rightOut);
    rightOut.NeutralMode = mode;
    right.getConfigurator().apply(rightOut);
  }

  public void setDeg(double degrees) {
    double position;
    if (this.getClass().getSimpleName().equals("LowerArm")) {
      position = degrees * (128.0 / 360.0);
    } else if (this.getClass().getSimpleName().equals("UpperArm")) {
      position = degrees * (100.0 / 360.0);
    } else {
      position = degrees;
    }
    left.setControl(motorPosRequest.withPosition(position));
    right.setControl(motorPosRequest.withPosition(position));
  }

  public void setPos(double position) {
    setPos(position, fast);
  }

  public void setPos(double position, boolean fast) {
    this.fast = fast;
    this.requestedPosition = position;

    double vel = fast ? fastVel : slowVel;
    double acc = fast ? fastAcc : slowAcc;
    double jerk = fast ? fastJerk : slowJerk;

    left.setControl(dynamic.withVelocity(vel).withAcceleration(acc).withJerk(jerk).withPosition(position));
    right.setControl(dynamic.withVelocity(vel).withAcceleration(acc).withJerk(jerk).withPosition(position));

    if (this.getClass().getSimpleName().equals("LowerArm")) {
      SmartDashboard.putNumber(this.getClass().getSimpleName(),
          (0.5 * (cachedLeftPos + cachedRightPos) * (360.0 / 128.0)));
    }
    if (this.getClass().getSimpleName().equals("UpperArm")) {
      SmartDashboard.putNumber(this.getClass().getSimpleName(),
          (0.5 * (cachedLeftPos + cachedRightPos) * (360.0 / 100.0)));
    }

  }

  public void setPosAutoSpeed(double position) {
    double avgPos = 0.5 * (cachedLeftPos + cachedRightPos);
    double distance = Math.abs(position - avgPos);

    if (!fast && distance > switchToFastThreshold) {
      fast = true;
    } else if (fast && distance < switchToSlowThreshold) {
      fast = false;
    }
    setPos(position, fast);
  }

  public double getPos() {

    return 0.5 * (cachedLeftPos + cachedRightPos);
  }

  public double getDegs() {
    // Special case for lowerArm and upperArm to convert to degrees
    // based on the encoder resolution.
    // This is a workaround for the fact that the encoder resolution
    // is not the same for different arm segments.

    if (this.getClass().getSimpleName().equals("LowerArm")) {

      return (0.5 * (cachedLeftPos + cachedRightPos) * (360.0 / 128.0));
    }
    if (this.getClass().getSimpleName().equals("UpperArm")) {
      return (0.5 * (cachedLeftPos + cachedRightPos) * (360.0 / 100.0));
    } else {
      System.out.println("DualArmSegmentBase.getDegs() called on unknown segment: " + this.getClass().getSimpleName());
      return 0.5 * (cachedLeftPos + cachedRightPos);
    }
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

  /** Apply current PID fields to the hardware. */
  public void updatePID() {
    leftPID.kP = kP;
    leftPID.kI = kI;
    leftPID.kD = kD;
    leftPID.kS = kS;

    rightPID.kP = kP;
    rightPID.kI = kI;
    rightPID.kD = kD;
    rightPID.kS = kS;

    left.getConfigurator().apply(leftConfig);
    right.getConfigurator().apply(rightConfig);
  }

  /** Apply an explicit motion magic profile (overrides fast/slow values). */
  public void updateMotionMagic(double cruiseVel, double accel, double jerk) {
    leftMM.MotionMagicCruiseVelocity = cruiseVel;
    leftMM.MotionMagicAcceleration = accel;
    leftMM.MotionMagicJerk = jerk;

    rightMM.MotionMagicCruiseVelocity = cruiseVel;
    rightMM.MotionMagicAcceleration = accel;
    rightMM.MotionMagicJerk = jerk;

    left.getConfigurator().apply(leftConfig);
    right.getConfigurator().apply(rightConfig);
  }

  // --- New velocity-control helpers ---
  /**
   * Command both motors to a target velocity (rotations/sec).
   */

  public void setTargetVelocityRPS(double velocityRPS) {
    // SmartDashboard.putNumber("LowerArm Velocity", velocityRPS);
    velocitySetpoint = velocityRPS;
    velocityRequest.Velocity = velocitySetpoint;
    left.setControl(velocityRequest);
    right.setControl(velocityRequest);
    String msg = this.getClass().getSimpleName() + " Vel ";
    SmartDashboard.putNumber(msg, velocityRPS);
    InitLogger.logDouble(this.getClass().getSimpleName(), "Velocity", velocityRPS);
  }

  /**
   * Stop a given motor immediately.
   */
  public void stop(TalonFX motor) {
    motor.stopMotor();
  }

  /**
   * Read the current velocity (RPS) of a motor.
   */
  public double getCurrentVelocity(TalonFX motor) {
    return motor.getVelocity().refresh().getValueAsDouble();
  }

  /**
   * Return true if the motor is within tolerance of the target RPS.
   */
  public boolean atTargetVelocity(TalonFX motor, double targetRPS, double tolerance) {
    return Math.abs(getCurrentVelocity(motor) - targetRPS) < tolerance;
  }

  @Override
  public void periodic() {
    cachedLeftPos = left.getPosition().getValueAsDouble();
    cachedRightPos = right.getPosition().getValueAsDouble();

    atPosition = Math.abs(cachedLeftPos - requestedPosition) < 1.0 &&
        Math.abs(cachedRightPos - requestedPosition) < 1.0;
        SmartDashboard.putNumber(this.getClass().getSimpleName(),getDegs());
        

    double now = Timer.getFPGATimestamp();
    if (now - lastLogTime >= 0.5) { // log up to twice a second
      String periodicMsg = String.format(
          "LeftPos=%.2f RightPos=%.2f Setpoint=%.2f Fast=%b AtPosition=%b",
          cachedLeftPos, cachedRightPos, requestedPosition, fast, atPosition);
      InitLogger.logMessage(this.getClass().getSimpleName(), periodicMsg);
      InitLogger.logDouble(this.getClass().getSimpleName(), "LeftPos", cachedLeftPos);
      InitLogger.logDouble(this.getClass().getSimpleName(), "RightPos", cachedRightPos);
      InitLogger.logDouble(this.getClass().getSimpleName(), "AveragePos", 0.5 * (cachedLeftPos + cachedRightPos));
      lastLogTime = now;
    }
  }

  protected void configureSendable(SendableBuilder builder) {
    builder.setSmartDashboardType(this.getClass().getSimpleName());

    builder.addDoubleProperty("Position - Left", this::getPosLeft, null);
    builder.addDoubleProperty("Position - Right", this::getPosRight, null);
    builder.addDoubleProperty("Setpoint", () -> requestedPosition, this::setPos);
    builder.addBooleanProperty("Fast", () -> fast, null);
    builder.addBooleanProperty("AtPosition", this::atPos, null);

    // PID tuning
    builder.addDoubleProperty("kP", () -> kP, (val) -> {
      if (kP != val) {
        kP = val;
        updatePID();
      }
    });
    builder.addDoubleProperty("kI", () -> kI, (val) -> {
      if (kI != val) {
        kI = val;
        updatePID();
      }
    });
    builder.addDoubleProperty("kD", () -> kD, (val) -> {
      if (kD != val) {
        kD = val;
        updatePID();
      }
    });
    builder.addDoubleProperty("kS", () -> kS, (val) -> {
      if (kS != val) {
        kS = val;
        updatePID();
      }
    });

    // Motion Magic tuning (fast)
    builder.addDoubleProperty("FastVel", () -> fastVel, (val) -> {
      if (fastVel != val) {
        this.fastVel = val;
        if (fast) {
          updateMotionMagic(fastVel, fastAcc, fastJerk);
        }
      }
    });
    builder.addDoubleProperty("FastAcc", () -> fastAcc, (val) -> {
      if (fastAcc != val) {
        this.fastAcc = val;
        if (fast) {
          updateMotionMagic(fastVel, fastAcc, fastJerk);
        }
      }
    });
    builder.addDoubleProperty("FastJerk", () -> fastJerk, (val) -> {
      if (fastJerk != val) {
        this.fastJerk = val;
        if (fast) {
          updateMotionMagic(fastVel, fastAcc, fastJerk);
        }
      }
    });

    // Motion Magic tuning (slow)
    builder.addDoubleProperty("SlowVel", () -> slowVel, (val) -> {
      if (slowVel != val) {
        this.slowVel = val;
        if (!fast) {
          updateMotionMagic(slowVel, slowAcc, slowJerk);
        }
      }
    });
    builder.addDoubleProperty("SlowAcc", () -> slowAcc, (val) -> {
      if (slowAcc != val) {
        this.slowAcc = val;
        if (!fast) {
          updateMotionMagic(slowVel, slowAcc, slowJerk);
        }
      }
    });
    builder.addDoubleProperty("SlowJerk", () -> slowJerk, (val) -> {
      if (slowJerk != val) {
        this.slowJerk = val;
        if (!fast) {
          updateMotionMagic(slowVel, slowAcc, slowJerk);
        }
      }
    });

    // Manual apply buttons if needed
    builder.addBooleanProperty("Apply PID", () -> false, pressed -> {
      if (pressed) {
        updatePID();
      }
    });
    builder.addBooleanProperty("Apply Fast MM", () -> false, pressed -> {
      if (pressed) {
        updateMotionMagic(fastVel, fastAcc, fastJerk);
      }
    });
    builder.addBooleanProperty("Apply Slow MM", () -> false, pressed -> {
      if (pressed) {
        updateMotionMagic(slowVel, slowAcc, slowJerk);
      }
    });
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    configureSendable(builder);
  }
}
