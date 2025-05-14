package frc.robot;

import java.awt.geom.Point2D;
public class BezierCurveJava {
    public final Point2D.Double P0, P1, P2, P3;
    private final int resolution = 1000;
    private final double[] arcLengths;
    private final double[] tValues;
    private final double totalArcLength;

    public BezierCurveJava(Point2D.Double p0, Point2D.Double p1, Point2D.Double p2, Point2D.Double p3) {
        this.P0 = p0;
        this.P1 = p1;
        this.P2 = p2;
        this.P3 = p3;

        arcLengths = new double[resolution + 1];
        tValues = new double[resolution + 1];

        double length = 0;
        Point2D.Double prev = getRawPositionAtTime(0);
        for (int i = 1; i <= resolution; i++) {
            double t = i / (double) resolution;
            Point2D.Double pos = getRawPositionAtTime(t);
            double dx = pos.getX() - prev.getX();
            double dy = pos.getY() - prev.getY();
            length += Math.hypot(dx, dy);

            arcLengths[i] = length;
            tValues[i] = t;
            prev = pos;
        }

        totalArcLength = length;
    }

    public double getTotalArcLength() {
        return totalArcLength;
    }

    /** Returns (x,y) point on the curve at t (0 ≤ t ≤ 1) using arc-length parameterization */
    public Point2D.Double getPositionAtArcLengthTime(double s01) {
        double s = s01 * totalArcLength;
        double t = findTForArcLength(s);
        return getRawPositionAtTime(t);
    }

    /** Returns (dx,dy) on the curve at t (0 ≤ t ≤ 1) using arc-length parameterization */
    public Point2D.Double getVelocityAtArcLengthTime(double s01) {
        double s = s01 * totalArcLength;
        double t = findTForArcLength(s);
        return getRawVelocityAtTime(t);
    }

    public Point2D.Double getRawPositionAtTime(double t) {
        double oneMinusT = 1 - t;
        double x = oneMinusT * oneMinusT * oneMinusT * P0.getX()
                + 3 * oneMinusT * oneMinusT * t * P1.getX()
                + 3 * oneMinusT * t * t * P2.getX()
                + t * t * t * P3.getX();

        double y = oneMinusT * oneMinusT * oneMinusT * P0.getY()
                + 3 * oneMinusT * oneMinusT * t * P1.getY()
                + 3 * oneMinusT * t * t * P2.getY()
                + t * t * t * P3.getY();

        return new Point2D.Double(x, y);
    }

    public Point2D.Double getRawVelocityAtTime(double t) {
        double oneMinusT = 1 - t;
        double xVel = -3 * oneMinusT * oneMinusT * P0.getX()
                + 3 * oneMinusT * oneMinusT * P1.getX()
                - 6 * oneMinusT * t * P1.getX()
                + 6 * oneMinusT * t * P2.getX()
                - 3 * t * t * P2.getX()
                + 3 * t * t * P3.getX();

        double yVel = -3 * oneMinusT * oneMinusT * P0.getY()
                + 3 * oneMinusT * oneMinusT * P1.getY()
                - 6 * oneMinusT * t * P1.getY()
                + 6 * oneMinusT * t * P2.getY()
                - 3 * t * t * P2.getY()
                + 3 * t * t * P3.getY();

        return new Point2D.Double(xVel, yVel);
    }

    /** Binary search for t at given arc length */
    public double findTForArcLength(double s) {
        if (s <= 0) return 0;
        if (s >= totalArcLength) return 1;

        int low = 0;
        int high = resolution;

        while (low <= high) {
            int mid = (low + high) / 2;
            if (arcLengths[mid] < s) {
                low = mid + 1;
            } else {
                high = mid - 1;
            }
        }

        // Linear interpolate between arcLengths[low - 1] and arcLengths[low]
        double s0 = arcLengths[low - 1];
        double s1 = arcLengths[low];
        double t0 = tValues[low - 1];
        double t1 = tValues[low];

        double factor = (s - s0) / (s1 - s0);
        return t0 + factor * (t1 - t0);
    }
}
