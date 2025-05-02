package frc.robot.commands;

import java.util.List;
import java.awt.geom.Point2D;
import edu.wpi.first.wpilibj2.command.Command;
//import java.awt.geom.GeneralPath;
//import java.awt.geom.Path2D;
//import java.awt.geom.*;

import frc.robot.subsystems.ArmAssembly;
import frc.robot.Utilitys.BezierCurve;
//import frc.robot.subsystems.ArmController;

public class FollowCurve extends Command {

    private final ArmAssembly arm;
    private static List<Point2D> path;
    private int index = 0;
    double t = 0.0; // Parameter for the Bezier curve
    private double L1 = 0.4962; // meters
    private double L2 = 0.6969;
    private Point2D p0;
    private Point2D p1;
    private Point2D p2;
    private int numberOfPoints = 200;

    public FollowCurve(ArmAssembly arm, Point2D p0, Point2D p1, Point2D p2) {
        this.arm = arm;
        this.path = BezierCurve.generateCurve(p0, p1, p2, numberOfPoints);
        this.p0 = p0;
        this.p1 = p1;
        this.p2 = p2;

        // addRequirements(arm);
        System.out.println("Path size: " + path.size());
    }

    @Override
    public void initialize() {
        path = BezierCurve.generateCurve(p0, p1, p2, numberOfPoints);
        t=0;
    }

    @Override
    public void execute() {

        if (t > 1) {
            return;
        }
        Point2D p0 = path.get(0);
        Point2D p1 = path.get(1);
        Point2D p2 = path.get(2);
        Point2D p3 = path.get(3);
        Point2D point = getPoint(t, p0, p1, p2, p3);
        // arm.moveToXY(point.getX(),point.getY());
        double[] angles = solve(point.getX(), point.getY(), L1, L2);
         System.out.println("X: " + point.getX() + " Y: " + point.getY());
     System.out.print(" "+angles[0] + " " + angles[1]);
     
        arm.setJointAngles(angles[0], angles[1]);
        t += 1.0/numberOfPoints;

        // if (index < path.size()) {
        // Point2D target = path.get(index);
        // arm.moveToXY(target.getX(), target.getY());
        // index++;
        // }

    }

    public static Point2D getPoint(double t, Point2D p0, Point2D p1, Point2D p2, Point2D p3) {
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

    public static double[] solve(double y, double z, double L1, double L2) {
        double distance = Math.sqrt(y * y + z * z);
       // distance = Math.min(distance, L1 + L2); // Clamp to max reach
         // Law of Cosines for elbow (internal angle)
         double cosAngle = (distance - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);
        cosAngle = Math.max(-1.0, Math.min(1.0, cosAngle)); // clamp
         double internalAngle = Math.acos(cosAngle); // always positive
 
         // ✅ Define positive elbow angle as CW, so negate it
       //  double theta2 = -internalAngle+ Math.PI+Math.toRadians(45); // Elbow angle (internal angle)
       double theta2 = Math.PI-internalAngle;//+ Math.PI; // Elbow angle (internal angle)
         // Shoulder angle from horizontal
         double k1 = L1 + L2 * Math.cos(theta2);
         double k2 = L2 * Math.sin(theta2);
         double theta1 = Math.atan2(z, y) - Math.atan2(k2, k1);
         theta1 = Math.toDegrees(theta1);
         theta2 = Math.toDegrees(theta2);

        // Convert to degrees for output
        return new double[] { theta1, theta2 };
    }

    @Override
    public boolean isFinished() {

        return index >= path.size();
    }
}
