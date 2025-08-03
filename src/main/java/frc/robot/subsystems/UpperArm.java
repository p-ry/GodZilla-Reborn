package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class UpperArm extends DualArmSegmentBase {
  // Motion Magic Profiles
  private static final double FAST_VEL = 300;
  private static final double FAST_ACC = 300;
  private static final double FAST_JERK = 800;
  private static final double SLOW_VEL = 150;
  private static final double SLOW_ACC = 300;
  private static final double SLOW_JERK = 300;

  public UpperArm() {
    super(
        /* leftId */ 33,
        /* rightId */ 34,
        /* canBusName */ "Canivore2",
        /* dynamic */ new DynamicMotionMagicVoltage(0, 80, 300, 800),
        /* fastVel */ FAST_VEL,
        /* fastAcc */ FAST_ACC,
        /* fastJerk */ FAST_JERK,
        /* slowVel */ SLOW_VEL,
        /* slowAcc */ SLOW_ACC,
        /* slowJerk */ SLOW_JERK,
        /* invertLeft */ false,
        /* invertRight */ true
    );

    // Optional: set initial brake mode (already covered by base but kept for clarity)
    setBrakeMode(NeutralModeValue.Brake);

    // Put on Shuffleboard
    ShuffleboardTab tab = Shuffleboard.getTab("Arms");
    tab.add("UpperArm", this);
  }
}
