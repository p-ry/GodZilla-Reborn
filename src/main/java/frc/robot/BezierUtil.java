package frc.robot;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;

public class BezierUtil {
    public static Point2D evaluate(Point2D p0, Point2D p1, Point2D p2, Point2D p3, double t) {
        double u = 1 - t;
        double x = u*u*u*p0.getX() + 3*u*u*t*p1.getX() + 3*u*t*t*p2.getX() + t*t*t*p3.getX();
        double y = u*u*u*p0.getY() + 3*u*u*t*p1.getY() + 3*u*t*t*p2.getY() + t*t*t*p3.getY();
        return new Point2D.Double(x, y);
    }

    public static List<Point2D> generateCurve(Point2D p0, Point2D p1, Point2D p2, Point2D p3) {
        List<Point2D> points = new ArrayList<>();
        for (double t = 0; t <= 1.0; t += 0.01) {
            points.add(evaluate(p0, p1, p2, p3, t));
        }
        return points;
    }
}
