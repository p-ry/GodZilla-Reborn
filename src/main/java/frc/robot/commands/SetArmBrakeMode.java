// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmAssembly;
public class SetArmBrakeMode extends InstantCommand {
    public SetArmBrakeMode(ArmAssembly arm, NeutralModeValue mode) {
        super(() -> 
        arm.upperArm.setBrakeMode(mode));
        arm.lowerArm.setBrakeMode(mode);
        arm.slider.setBrakeMode(mode);
        arm.ace.setBrakeMode(mode);
        arm.wrist.setBrakeMode(mode);
    }
}
