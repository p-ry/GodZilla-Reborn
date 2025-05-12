package frc.robot;

import java.awt.geom.Point2D;

public class BezierCurveJava {
    // Assume control points (P0, P1, P2, P3) are defined, for example:
    private Point2D P0, P1, P2, P3;

    // Constructor
    public BezierCurveJava(Point2D p0, Point2D p1, Point2D p2, Point2D p3) {
        this.P0 = P0;
        this.P1 = P1;
        this.P2 = P2;
        this.P3 = P3;
    }

    // Calculate position at a given time t (0 <= t <= 1)
    public Point2D getPositionAtTime(double t) {
        // Cubic Bézier curve formula
        double x = Math.pow(1 - t, 3) * P0.getX() +
                3 * Math.pow(1 - t, 2) * t * P1.getX() +
                3 * (1 - t) * Math.pow(t, 2) * P2.getX() +
                Math.pow(t, 3) * P3.getX();

        double y = Math.pow(1 - t, 3) * P0.getY() +
                3 * Math.pow(1 - t, 2) * t * P1.getY() +
                3 * (1 - t) * Math.pow(t, 2) * P2.getY() +
                Math.pow(t, 3) * P3.getY();
        return new Point2D.Double(x, y); // Math.hypot(x, y); // Euclidean distance between the points (or simply the
                                         // x-coordinate if it's 1D)
    }

    // Calculate velocity at a given time t (derivative of position with respect to
    // t)
    public Point2D getVelocityAtTime(double t) {
        // Derivative of cubic Bézier curve formula (velocity)
        double xVel = -3 * Math.pow(1 - t, 2) * P0.getX() +
                3 * Math.pow(1 - t, 2) * P1.getX() +
                6 * (1 - t) * t * P2.getX() +
                3 * Math.pow(t, 2) * P3.getX();

        double yVel = -3 * Math.pow(1 - t, 2) * P0.getY() +
                3 * Math.pow(1 - t, 2) * P1.getY() +
                6 * (1 - t) * t * P2.getY() +
                3 * Math.pow(t, 2) * P3.getY();

        return new Point2D.Double(xVel, yVel);
    }
}