package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;

import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class LowerArm extends DualArmSegmentBase {
  // Motion Magic Profiles
  private static final double FAST_VEL = 200;
  private static final double FAST_ACC = 600;
  private static final double FAST_JERK = 1000;
  private static final double SLOW_VEL = 150;
  private static final double SLOW_ACC = 600;
  private static final double SLOW_JERK = 600;

  public LowerArm() {
    super(
        /* leftId */ 31,
        /* rightId */ 32,
        /* canBusName */ "Canivore2",
        /* dynamic */ new DynamicMotionMagicVoltage(0, 200, 600, 1000),
        /* fastVel */ FAST_VEL,
        /* fastAcc */ FAST_ACC,
        /* fastJerk */ FAST_JERK,
        /* slowVel */ SLOW_VEL,
        /* slowAcc */ SLOW_ACC,
        /* slowJerk */ SLOW_JERK,
        /* invertLeft */ false,
        /* invertRight */ true
    );

    setBrakeMode(NeutralModeValue.Brake);
if (Constants.enableShuffleboard) {
    ShuffleboardTab tab = Shuffleboard.getTab("Arms");
    tab.add("Wrist", this);
}

  }
}
