package frc.robot.commands;

import java.util.List;
import java.awt.geom.Point2D;
import edu.wpi.first.wpilibj2.command.Command;
//import java.awt.geom.GeneralPath;
//import java.awt.geom.Path2D;
//import java.awt.geom.*;

import frc.robot.subsystems.ArmAssembly;
import frc.robot.BezierLogger;
import frc.robot.RobotContainer;
import frc.robot.Utilitys.BezierCurve;
//import frc.robot.subsystems.ArmController;

public class FollowCurve extends Command {

    private final ArmAssembly arm;
    private static List<Point2D.Double> path;
    private int index = 0;
    double t = 0.0; // Parameter for the Bezier curve
    private double L1 = 0.4962; // meters
    private double L2 = 0.6969;
    private Point2D p0;
    private Point2D p1;
    private Point2D p2;
    private Point2D p3;
    private int numberOfPoints = 10;
    private double baseX = 0.0; // Base X coordinate
    private double baseY = 0.0; // Base Y coordinate
    private Point2D base;
    private static double sliderLength = 0.0; // Slider length

    public FollowCurve(ArmAssembly arm, Point2D p0, Point2D p1, Point2D p2, Point2D p3, Point2D base) {
        this.arm = arm;
        this.path = BezierCurve.generateCurve(p0, p1, p2, numberOfPoints);
        this.p0 = p0;
        this.p1 = p1;
        this.p2 = p2;
        this.p3 = p3;
        this.base = base;

        // addRequirements(arm);
        System.out.println("Path size: " + path.size());
    }

    @Override
    public void initialize() {
        path = BezierCurve.generateCurve(p0, p1, p2, numberOfPoints);
        t = 0;
        BezierLogger logger = new BezierLogger();
        Point2D p0 = new Point2D.Double(this.p0.getX(), this.p0.getY());
        Point2D p1 = new Point2D.Double(this.p1.getX(), this.p1.getY());
        Point2D p2 = new Point2D.Double(this.p2.getX(), this.p2.getY());
        Point2D p3 = new Point2D.Double(this.p3.getX(), this.p3.getY());

        List<Point2D.Double> curvePoints = BezierCurve.generateCurve(p0, p1, p2, numberOfPoints);
        // logger.logCurve(curvePoints, p0, p1, p2, p3);
        sliderLength = 0.0; // Reset slider length

    }

    @Override
    public void execute() {

        if (t > 1) {
            return;
        }
        // Point2D p0 = path.get(0);
        // Point2D p1 = path.get(1);
        // Point2D p2 = path.get(2);
        // Point2D p3 = path.get(3);
        // Point2D point = getPoint(t, p0, p1, p2, p3);
        // // arm.moveToXY(point.getX(),point.getY());
        Point2D.Double point = getPoint(t, p0, p1, p2, p3);

        double[] angles = solve(point.getX(), point.getY(), L1, L2, base.getX(), base.getY());
        System.out.println("X: " + point.getX() + " Y: " + point.getY());
        System.out.print(" " + angles[0] + " " + angles[1]);

        arm.setJointAngles(angles[0], angles[1]);
        t += 1.0 / numberOfPoints;

        // if (index < path.size()) {
        // Point2D target = path.get(index);
        // arm.moveToXY(target.getX(), target.getY());
        // index++;
        // }

    }

    public static Point2D.Double getPoint(double t, Point2D p0, Point2D p1, Point2D p2, Point2D p3) {
        double oneMinusT = 1.0 - t;
        double y = Math.pow(oneMinusT, 3) * p0.getX() +
                3.0 * Math.pow(oneMinusT, 2) * t * p1.getX() +
                3.0 * oneMinusT * Math.pow(t, 2) * p2.getX() +
                Math.pow(t, 3) * p3.getX();

        double z = Math.pow(oneMinusT, 3) * p0.getY() +
                3.0 * Math.pow(oneMinusT, 2) * t * p1.getY() +
                3.0 * oneMinusT * Math.pow(t, 2) * p2.getY() +
                Math.pow(t, 3) * p3.getY();

        return new Point2D.Double(y, z);
    }

    public static double[] solve(double targetX, double targetY, double L1, double L2, double baseX, double baseY) {
        double theta1 = 0.0; // shoulder angle
        double theta2 = 0.0; // elbow angle
        double dx = targetX - baseX;
        double dy = targetY - baseY;
        double dist = Math.hypot(dx, dy);
        System.out.println("Distance: " + dist);

        sliderLength = dist - L1 - L2; // Calculate the slider length
        if (sliderLength < 0) {
            sliderLength = 0; // Ensure the slider length is non-negative
        }

        double sL2 = L2 + sliderLength; // Calculate the length of the second arm segment

        double angle1 = Math.acos((L1 * L1 + dist * dist - sL2 * sL2) / (2 * L1 * dist));

        double baseAngle = Math.atan2(dy, dx);
        double shoulderAngle = baseAngle - angle1;
        shoulderAngle = Math.min(shoulderAngle, Math.toRadians(70));// Math.PI/2); // Limit shoulder angle to [-90°,
                                                                    // 90°]
        double jointX = baseX + L1 * Math.cos(shoulderAngle);
        double jointY = baseY + L1 * Math.sin(shoulderAngle);
        dx = targetX - jointX;
        dy = targetY - jointY;

        // True angle between joint and target
        double targetAngle = Math.atan2(dy, dx);

        // Elbow angle is angle between L1 and L2 segments
        double elbowAngle = targetAngle - shoulderAngle;

        dist = Math.hypot(dx, dy); //distance from joint to target
        sliderLength = dist - L2; // Calculate the slider length
        if (sliderLength < 0) {
            sliderLength = 0; // Ensure the slider length is non-negative
        }

        double angle2 = Math.acos((L1 * L1 + sL2 * sL2 - dist * dist) / (2 * L1 * sL2));

        // elbowAngle = Math.PI - angle2;
        System.out.println("Shoulder Angle: " + Math.toDegrees(shoulderAngle));
        System.out.println("Angle2: " + Math.toDegrees(angle2));
        System.out.println("Elbow Angle: " + Math.toDegrees(elbowAngle));//(180.0 - Math.toDegrees(elbowAngle)));
        System.out.println("Target Angle: " + Math.toDegrees(targetAngle));
        System.out.println("Slider: " + sliderLength);
        System.out.println("Distance from joint to target: " + dist);

        // double baseAngle = Math.atan2(dy, dx); // angle from base to target
        // double angle1 = Math.acos((L1 * L1 + dist * dist - L2 * L2) / (2 * L1 * dist)); // internal triangle angle
        // double shoulderAngle = baseAngle - angle1;
        // System.out.println("Shoulder Angle: " + Math.toDegrees(shoulderAngle));
        // // System.out.println("baseAngle: " + Math.toDegrees(baseAngle));
        // // System.out.println("angle1: " + Math.toDegrees(angle1));

        // double angle2 = Math.acos((L1 * L1 + L2 * L2 - dist * dist) / (2 * L1 * L2));
        // // System.out.println("angle2" +angle2);
        // System.out.println("Elbow: " + Math.toDegrees(angle2));
        // // double elbowAngle = Math.PI-(Math.PI - angle2);
        // double elbowAngle = angle2;

        // double jointX = baseX + L1 * Math.cos(shoulderAngle);
        // double jointY = baseY + L1 * Math.sin(shoulderAngle);
        // double endX = jointX + L2 * Math.cos(shoulderAngle + elbowAngle);
        // double endY = jointY + L2 * Math.sin(shoulderAngle + elbowAngle);

        // Math.sqrt(y * y + z * z);
        // distance = Math.min(distance, L1 + L2); // Clamp to max reach
        // Law of Cosines for elbow (internal angle)
        // double cosAngle = (distance - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);
        // cosAngle = Math.max(-1.0, Math.min(1.0, cosAngle)); // clamp
        // double internalAngle = Math.acos(cosAngle); // always positive

        // // ✅ Define positive elbow angle as CW, so negate it
        // // double theta2 = -internalAngle+ Math.PI+Math.toRadians(45); // Elbow angle
        // (internal angle)
        // double theta2 = Math.PI-internalAngle;//+ Math.PI; // Elbow angle (internal
        // angle)
        // // Shoulder angle from horizontal
        // double k1 = L1 + L2 * Math.cos(theta2);
        // double k2 = L2 * Math.sin(theta2);
        // double theta1 = Math.atan2(z, y) - Math.atan2(k2, k1);
        theta1 = Math.toDegrees(shoulderAngle);
        theta2 = Math.toDegrees(elbowAngle);

        // Convert to degrees for output
        return new double[] { theta1, theta2 };
    }

    @Override
    public boolean isFinished() {

        return index >= path.size();
    }
}
