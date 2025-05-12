package frc.robot.commands;

import java.util.List;

import com.ctre.phoenix6.configs.Slot2Configs;

import java.awt.geom.Point2D;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
//import java.awt.geom.GeneralPath;
//import java.awt.geom.Path2D;
//import java.awt.geom.*;

import frc.robot.subsystems.ArmAssembly;
import frc.robot.BezierLogger;
import frc.robot.RobotContainer;
import frc.robot.Utilitys.BezierCurve;
//import frc.robot.subsystems.ArmController;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmAssembly;

public class FollowCurve extends Command {
    private final ArmAssembly arm;
    private final BezierCurve curve;
    private final double totalTime;
    private final double dt;
    private double time;
    private double lastShoulderDeg;
    private double lastSliderMeters;

    private static final double MAX_SHOULDER_DEG = 90.0;
    private static final double MAX_DELTA_SHOULDER_DEG = 2.0;
    private static final double L1 = 0.4962; // m
    private static final double L2 = 0.6969; // m

    public FollowCurve(ArmAssembly arm, BezierCurve curve, double totalTime, double dt) {
        this.arm = arm;
        this.curve = curve;
        this.totalTime = totalTime;
        this.dt = dt;
        this.time = 0;
        this.lastShoulderDeg = Double.NaN;
        this.lastSliderMeters = 0;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        time = 0;
        lastShoulderDeg = Double.NaN;
        lastSliderMeters = 0;
    }

    @Override
    public void execute() {
        if (time > totalTime) return;

        var pos = curve.getPositionAtTime(time / totalTime);
        var vel = curve.getVelocityAtTime(time / totalTime);
        double dx = pos.x;
        double dy = pos.y;
        double dxdT = vel.x / totalTime;
        double dydT = vel.y / totalTime;

        double dist = Math.hypot(dx, dy);
        if (dist > L1 + L2 || dist < Math.abs(L1 - L2)) return;

        double cosTheta2 = (dx * dx + dy * dy - L1 * L1 - L2 * L2) / (2 * L1 * L2);
        double theta2 = Math.acos(cosTheta2);
        double k1 = L1 + L2 * Math.cos(theta2);
        double k2 = L2 * Math.sin(theta2);
        double theta1 = Math.atan2(dy, dx) - Math.atan2(k2, k1);

        double shoulderDeg = Math.toDegrees(theta1);
        double elbowDeg = Math.toDegrees(theta2);

        // Enforce shoulder limits
        shoulderDeg = Math.min(shoulderDeg, MAX_SHOULDER_DEG);

        if (!Double.isNaN(lastShoulderDeg)) {
            double delta = shoulderDeg - lastShoulderDeg;
            if (Math.abs(delta) > MAX_DELTA_SHOULDER_DEG) {
                shoulderDeg = lastShoulderDeg + Math.copySign(MAX_DELTA_SHOULDER_DEG, delta);
            }
        }

        // Recalculate theta1 in radians after limiting
        double limitedTheta1Rad = Math.toRadians(shoulderDeg);
        double limitedTheta2Rad = Math.toRadians(elbowDeg);

        // Reconstruct (x, y) from joint angles to track true tip position
        double trueX = L1 * Math.cos(limitedTheta1Rad) + L2 * Math.cos(limitedTheta1Rad + limitedTheta2Rad);
        double trueY = L1 * Math.sin(limitedTheta1Rad) + L2 * Math.sin(limitedTheta1Rad + limitedTheta2Rad);

        // Calculate slider value as Euclidean distance from base
        double currentSlider = Math.hypot(trueX, trueY);
        double sliderVelocity = (currentSlider - lastSliderMeters) / dt;
        lastSliderMeters = currentSlider;

        // Estimate arm angular velocities (rough Jacobian)
        double dShoulderDeg = (shoulderDeg - lastShoulderDeg) / dt;
        double dElbowDeg = dxdT * Math.cos(theta2) + dydT * Math.sin(theta2); // rough estimate

        // Send joint velocities
        arm.setJointVelocities(dShoulderDeg, dElbowDeg, sliderVelocity);
        lastShoulderDeg = shoulderDeg;

        time += dt;
    }

    @Override
    public boolean isFinished() {
        return time >= totalTime;
    }

    @Override
    public void end(boolean interrupted) {
        arm.setJointVelocities(0.0, 0.0, 0.0);
    }
}
