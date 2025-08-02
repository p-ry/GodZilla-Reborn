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

public class UpperArm extends SubsystemBase {

  private final TalonFX upperLeft = new TalonFX(33, "Canivore2");
  private final TalonFX upperRight = new TalonFX(34, "Canivore2");

  private final TalonFXConfiguration talonFXConfigsLeft = new TalonFXConfiguration();
  private final TalonFXConfiguration talonFXConfigsRight = new TalonFXConfiguration();


  private final Slot0Configs pidConfigsLeft;
  private final Slot0Configs pidConfigsRight;
  private final MotionMagicConfigs mmConfigsLeft;
  private final MotionMagicConfigs mmConfigsRight;

  private static final DynamicMotionMagicVoltage dynamic = new DynamicMotionMagicVoltage(0, 80, 300, 800);
  private final MotionMagicVoltage leftRequest = new MotionMagicVoltage(0);
  private final MotionMagicVoltage rightRequest = new MotionMagicVoltage(0);

  private double cachedLeftPos = 0;
  private double cachedRightPos = 0;
  private double requestedPosition = 0;
  private boolean atPosition = false;
  private boolean updatePending = false;
  private boolean fast = false;

  private static final double SWITCH_TO_FAST_THRESHOLD = 12.0;
  private static final double SWITCH_TO_SLOW_THRESHOLD = 8.0;


  // PID Constants

  public double kP = 10.0, kI = 0.0, kD = 0.0, kS = 0.25;

  // Motion Magic Profiles
  public static double fastVel = 300, fastAcc = 300, fastJerk = 800;
  public static double slowVel = 150, slowAcc = 300, slowJerk = 300;

  public UpperArm() {

    upperLeft.getConfigurator().refresh(talonFXConfigsLeft);
    upperRight.getConfigurator().refresh(talonFXConfigsRight);

   talonFXConfigsLeft.MotorOutput.Inverted=InvertedValue.CounterClockwise_Positive;
   talonFXConfigsRight.MotorOutput.Inverted=InvertedValue.Clockwise_Positive;

    pidConfigsLeft = talonFXConfigsLeft.Slot0;
    pidConfigsRight = talonFXConfigsRight.Slot0;

    mmConfigsLeft = talonFXConfigsLeft.MotionMagic;
    mmConfigsRight = talonFXConfigsRight.MotionMagic;


    // Motor and PID configuration
    talonFXConfigsLeft.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    talonFXConfigsRight.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pidConfigsLeft.kP = kP;
    pidConfigsLeft.kI = kI;
    pidConfigsLeft.kD = kD;
    pidConfigsLeft.kS = kS;
    pidConfigsLeft.kV = 0.12;
    pidConfigsLeft.kA = 0.01;
    pidConfigsRight.kP = kP;
    pidConfigsRight.kI = kI;
    pidConfigsRight.kD = kD;
    pidConfigsRight.kS = kS;
    pidConfigsRight.kV = 0.12;
    pidConfigsRight.kA = 0.01;
    // Motion Magic configuration
    mmConfigsLeft.MotionMagicCruiseVelocity = fastVel;
    mmConfigsLeft.MotionMagicAcceleration = fastAcc;
    mmConfigsLeft.MotionMagicJerk = fastJerk;
    mmConfigsRight.MotionMagicCruiseVelocity = fastVel;
    mmConfigsRight.MotionMagicAcceleration = fastAcc;
    mmConfigsRight.MotionMagicJerk = fastJerk;
    // Slow Motion Magic configuration
    talonFXConfigsLeft.MotionMagic.MotionMagicCruiseVelocity = slowVel;
    talonFXConfigsLeft.MotionMagic.MotionMagicAcceleration = slowAcc;
    talonFXConfigsLeft.MotionMagic.MotionMagicJerk = slowJerk;
    talonFXConfigsRight.MotionMagic.MotionMagicCruiseVelocity = slowVel;
    talonFXConfigsRight.MotionMagic.MotionMagicAcceleration = slowAcc;
    talonFXConfigsRight.MotionMagic.MotionMagicJerk = slowJerk;






    mmConfigsLeft.MotionMagicCruiseVelocity = slowVel;
    mmConfigsLeft.MotionMagicAcceleration = slowAcc;
    mmConfigsLeft.MotionMagicJerk = slowJerk;
    talonFXConfigsLeft.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    talonFXConfigsRight.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    upperLeft.getConfigurator().apply(talonFXConfigsLeft);
    upperRight.getConfigurator().apply(talonFXConfigsRight);
   

    // upperLeft.setNeutralMode(NeutralModeValue.Brake);
    // upperRight.setNeutralMode(NeutralModeValue.Brake);

    ShuffleboardTab tab = Shuffleboard.getTab("Arms");
    tab.add("UpperArm", this);
  }

  // public void setBrakeMode(NeutralModeValue mode) {
  //   MotorOutputConfigs config = new MotorOutputConfigs();
  //   upperLeft.getConfigurator().refresh(config);
  //   config.NeutralMode = mode;
  //   upperLeft.getConfigurator().apply(config);
  //   upperRight.getConfigurator().apply(config);
  // }

  public void setPos(double position) {
    setPos(position, fast);
  }

  public void setPos(double position, boolean fast) {
    this.fast = fast;
    requestedPosition = position;

    double vel = fast ? fastVel : slowVel;
    double acc = fast ? fastAcc : slowAcc;
    double jerk = fast ? fastJerk : slowJerk;

    upperLeft.setControl(dynamic.withVelocity(vel).withAcceleration(acc).withJerk(jerk).withPosition(position));
    upperRight.setControl(dynamic.withVelocity(vel).withAcceleration(acc).withJerk(jerk).withPosition(position));
  }
  public void setPosAutoSpeed(double position) {
    double avgPos = 0.5 * (cachedLeftPos + cachedRightPos);
    double distance = Math.abs(position - avgPos);

    // Only switch if distance crosses outside the hysteresis band
    if (!fast && distance > SWITCH_TO_FAST_THRESHOLD) {
      fast = true;
    } else if (fast && distance < SWITCH_TO_SLOW_THRESHOLD) {
      fast = false;
    }

    setPos(position, fast);
  }


public double getPos() {
    return 0.5*(cachedLeftPos + cachedRightPos);
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
    upperLeft.getConfigurator().apply(pidConfigsLeft);
    upperRight.getConfigurator().apply(pidConfigsRight);
  }

  public void updateMotionMagic() {
    mmConfigsLeft.MotionMagicCruiseVelocity = slowVel;
    mmConfigsLeft.MotionMagicAcceleration = slowAcc;
    mmConfigsLeft.MotionMagicJerk = slowJerk;

    mmConfigsRight.MotionMagicCruiseVelocity = slowVel;
    mmConfigsRight.MotionMagicAcceleration = slowAcc;
    mmConfigsRight.MotionMagicJerk = slowJerk;

    upperLeft.getConfigurator().apply(mmConfigsLeft);
    upperRight.getConfigurator().apply(mmConfigsRight);
  }


  @Override
  public void periodic() {
    cachedLeftPos = upperLeft.getPosition().getValueAsDouble();
    cachedRightPos = upperRight.getPosition().getValueAsDouble();

    atPosition = Math.abs(cachedLeftPos - requestedPosition) < 1.0 &&
                 Math.abs(cachedRightPos - requestedPosition) < 1.0;

    if (updatePending) {
      updatePID();
      updatePending = false;
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("UpperArm");

    builder.addDoubleProperty("Position - Left", this::getPosLeft, null);
    builder.addDoubleProperty("Position - Right", this::getPosRight, null);
    builder.addDoubleProperty("Setpoint", () -> requestedPosition, this::setPos);
    builder.addBooleanProperty("Fast", () -> fast, null);
    builder.addBooleanProperty("AtPosition", this::atPos, null);

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

    // MM tuning
    builder.addDoubleProperty("MMVel", () -> slowVel, (val) -> {
      if (slowVel != val) slowVel = val;
    });
    builder.addDoubleProperty("MMAccel", () -> slowAcc, (val) -> {
      if (slowAcc != val) slowAcc = val;
    });
    builder.addDoubleProperty("MMJerk", () -> slowJerk, (val) -> {
      if (slowJerk != val) slowJerk = val;
    });

    builder.addBooleanProperty("Update", () -> false, (pressed) -> {
      if (pressed) {
        updatePending = true;
        updateMotionMagic();  // Apply MM tuning
      }
    });

 }}


