package frc.robot.subsystems;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class LowerArm extends SubsystemBase implements Sendable {

  private final TalonFX lowerLeft = new TalonFX(31, "Canivore2");
  private final TalonFX lowerRight = new TalonFX(32, "Canivore2");

  private final TalonFXConfiguration leftConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration rightConfig = new TalonFXConfiguration();

  private static final DynamicMotionMagicVoltage dynamic = new DynamicMotionMagicVoltage(0, 200, 600, 1000);

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
    lowerLeft.getConfigurator().refresh(leftConfig);
    lowerRight.getConfigurator().refresh(rightConfig);

    // Apply neutral mode safely
    leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
   leftConfig.MotorOutput.Inverted=InvertedValue.CounterClockwise_Positive;
    rightConfig.MotorOutput.Inverted=InvertedValue.Clockwise_Positive;

    // PID config
    configurePID(leftConfig.Slot0);
    configurePID(rightConfig.Slot0);

    // Motion Magic config
    configureMM(leftConfig.MotionMagic, fastVel, fastAcc, fastJerk);
    configureMM(rightConfig.MotionMagic, fastVel, fastAcc, fastJerk);

    // Apply full configs
    lowerLeft.getConfigurator().apply(leftConfig);
    lowerRight.getConfigurator().apply(rightConfig);

    ShuffleboardTab tab = Shuffleboard.getTab("Arms");
    tab.add("LowerArm", this);
  }

  private void configurePID(Slot0Configs cfg) {
    cfg.kP = kP;
    cfg.kI = kI;
    cfg.kD = kD;
    cfg.kS = kS;
    cfg.kV = 0.12;
    cfg.kA = 0.01;
  }

  private void configureMM(MotionMagicConfigs mm, double vel, double acc, double jerk) {
    mm.MotionMagicCruiseVelocity = vel;
    mm.MotionMagicAcceleration = acc;
    mm.MotionMagicJerk = jerk;
  }

  public void setPos(double position) {
    setPos(position, fast);
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

  public double getPos() {
    return 0.5 * (cachedLeftPos + cachedRightPos);
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
    configurePID(leftConfig.Slot0);
    configurePID(rightConfig.Slot0);
    lowerLeft.getConfigurator().apply(leftConfig.Slot0);
    lowerRight.getConfigurator().apply(rightConfig.Slot0);
  }

  public void updateMotionMagic() {
    configureMM(leftConfig.MotionMagic, slowVel, slowAcc, slowJerk);
    configureMM(rightConfig.MotionMagic, slowVel, slowAcc, slowJerk);
    lowerLeft.getConfigurator().apply(leftConfig.MotionMagic);
    lowerRight.getConfigurator().apply(rightConfig.MotionMagic);
  }

  // public void setBrakeMode(NeutralModeValue mode) {
  //   MotorOutputConfigs cfg = new MotorOutputConfigs();
  //   lowerLeft.getConfigurator().refresh(cfg);
  //   cfg.NeutralMode = mode;
  //   lowerLeft.getConfigurator().apply(cfg);
  //   lowerRight.getConfigurator().apply(cfg);
  // }

  @Override
  public void periodic() {
    cachedLeftPos = lowerLeft.getPosition().getValueAsDouble();
    cachedRightPos = lowerRight.getPosition().getValueAsDouble();

    atPosition = Math.abs(cachedLeftPos - requestedPosition) < 1.0 &&
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

    builder.addDoubleProperty("MMVel", () -> slowVel, (val) -> { if (slowVel != val) slowVel = val; });
    builder.addDoubleProperty("MMAccel", () -> slowAcc, (val) -> { if (slowAcc != val) slowAcc = val; });
    builder.addDoubleProperty("MMJerk", () -> slowJerk, (val) -> { if (slowJerk != val) slowJerk = val; });

    builder.addDoubleProperty("kP", () -> kP, (val) -> { if (kP != val) { kP = val; updatePending = true; } });
    builder.addDoubleProperty("kI", () -> kI, (val) -> { if (kI != val) { kI = val; updatePending = true; } });
    builder.addDoubleProperty("kD", () -> kD, (val) -> { if (kD != val) { kD = val; updatePending = true; } });
    builder.addDoubleProperty("kS", () -> kS, (val) -> { if (kS != val) { kS = val; updatePending = true; } });

    builder.addBooleanProperty("Update", () -> false, (pressed) -> {
      if (pressed) {
        updatePending = true;
        updateMotionMagic();
      }
    });
  }
}
