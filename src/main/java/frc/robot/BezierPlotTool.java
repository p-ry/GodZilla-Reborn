package frc.robot;
import javax.swing.*;
import java.awt.*;
import java.awt.geom.Point2D;
import java.util.ArrayList;

public class BezierPlotTool extends JPanel {
    private final BezierCurveJava bezier;
    private final java.util.List<Point2D.Double> rawPoints = new ArrayList<>();
    private final java.util.List<Point2D.Double> arcLengthPoints = new ArrayList<>();

    public BezierPlotTool(BezierCurveJava bezier) {
        this.bezier = bezier;

        for (int i = 0; i <= 1000; i++) {
            double t = i / 1000.0;
            rawPoints.add(bezier.getRawPositionAtTime(t));
            arcLengthPoints.add(bezier.getPositionAtArcLengthTime(t));
        }
    }

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        // Draw raw curve (in red)
        g2d.setColor(Color.RED);
        drawPath(g2d, rawPoints, getWidth(), getHeight());

        // Draw arc-length corrected curve (in green)
        g2d.setColor(Color.GREEN);
        drawPath(g2d, arcLengthPoints, getWidth(), getHeight());
    }

    private void drawPath(Graphics2D g2d, java.util.List<Point2D.Double> points, int width, int height) {
        if (points.isEmpty()) return;

        double minX = points.stream().mapToDouble(Point2D::getX).min().orElse(0);
        double maxX = points.stream().mapToDouble(Point2D::getX).max().orElse(1);
        double minY = points.stream().mapToDouble(Point2D::getY).min().orElse(0);
        double maxY = points.stream().mapToDouble(Point2D::getY).max().orElse(1);

        double scaleX = width / (maxX - minX);
        double scaleY = height / (maxY - minY);

        Point2D.Double prev = points.get(0);
        for (int i = 1; i < points.size(); i++) {
            Point2D.Double curr = points.get(i);
            int x1 = (int) ((prev.getX() - minX) * scaleX);
            int y1 = height - (int) ((prev.getY() - minY) * scaleY);
            int x2 = (int) ((curr.getX() - minX) * scaleX);
            int y2 = height - (int) ((curr.getY() - minY) * scaleY);
            g2d.drawLine(x1, y1, x2, y2);
            prev = curr;
        }
    }

    public static void main(Point2D.Double p0, Point2D.Double p1, Point2D.Double p2, Point2D.Double p3) {
        // Point2D.Double p0 = new Point2D.Double(-27.77, 580.7621);
        // Point2D.Double p1 = new Point2D.Double(-50.0, 1000.0);
        // Point2D.Double p2 = new Point2D.Double(-10.0, 1200.0);
        // Point2D.Double p3 = new Point2D.Double(-20.1, 1960.95);

        BezierCurveJava curve = new BezierCurveJava(p0, p1, p2, p3);

        JFrame frame = new JFrame("Bezier Curve Plotter");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setSize(1200, 1200);
        frame.add(new BezierPlotTool(curve));
        frame.setVisible(true);
    }
}
