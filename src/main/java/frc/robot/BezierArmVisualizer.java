package frc.robot;


import javax.swing.*;
import java.awt.*;
import java.awt.geom.Point2D;
import java.util.List;
import java.util.Arrays;

public class BezierArmVisualizer extends JPanel {
    private final List<Point2D.Double> controlPoints;
    private final double L1 = 496; // Lower arm length
    private final double L2 = 696; // Upper arm length
    private double sliderLength = 0.0; // Slider length
    
    
    private double t = 0.0;
    //private final Timer timer;

    public BezierArmVisualizer(List<Point2D.Double> controlPoints) {
        this.controlPoints = controlPoints;
        setPreferredSize(new Dimension(1400, 1000));
        setBackground(Color.WHITE);

        // Timer to animate the curve
        final Timer timer = new Timer(50, e -> {
            t += 0.02;
            if (t > 1) {
                ((Timer) e.getSource()).stop();
                System.out.printf("Final point: (%.2f, %.2f)%n", bezierPoint(1).x, bezierPoint(1).y);
                return;
            }
            repaint();
        });
        timer.start();
    }

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2 = (Graphics2D) g.create();

        // Transform origin to bottom-center and flip Y-axis
        g2.translate(getWidth() / 2, getHeight());
        g2.scale(1, -.5);

        drawControlPolygon(g2);
        drawBezierCurve(g2);
        drawArm(g2, 0, 0, bezierPoint(t)); // Base at (0, 0)

        g2.dispose();
    }

    private void drawControlPolygon(Graphics2D g2) {
        g2.setColor(Color.GRAY);
        for (int i = 0; i < controlPoints.size() - 1; i++) {
            Point2D.Double p1 = controlPoints.get(i);
            Point2D.Double p2 = controlPoints.get(i + 1);
            g2.drawLine((int) p1.x, (int) p1.y, (int) p2.x, (int) p2.y);
        }
    }

    private void drawBezierCurve(Graphics2D g2) {
        g2.setColor(Color.RED);
        Point2D.Double prev = bezierPoint(0);
        for (double step = 0.01; step <= 1.0; step += 0.01) {
            Point2D.Double point = bezierPoint(step);
            g2.drawLine((int) prev.x, (int) prev.y, (int) point.x, (int) point.y);
            prev = point;
        }
    }

    private Point2D.Double bezierPoint(double t) {
        Point2D.Double p0 = controlPoints.get(0);
        Point2D.Double p1 = controlPoints.get(1);
        Point2D.Double p2 = controlPoints.get(2);
        Point2D.Double p3 = controlPoints.get(3);

        double x = Math.pow(1 - t, 3) * p0.x +
                   3 * Math.pow(1 - t, 2) * t * p1.x +
                   3 * (1 - t) * t * t * p2.x +
                   Math.pow(t, 3) * p3.x;

        double y = Math.pow(1 - t, 3) * p0.y +
                   3 * Math.pow(1 - t, 2) * t * p1.y +
                   3 * (1 - t) * t * t * p2.y +
                   Math.pow(t, 3) * p3.y;

        return new Point2D.Double(x, y);
    }

    private void drawArm(Graphics2D g2, double baseX, double baseY, Point2D.Double target) {
        double dx = target.x - baseX;
        double dy = target.y - baseY;
        double dist = Math.hypot(dx, dy);
        
        sliderLength = dist-L1-L2; // Calculate the slider length
        if (sliderLength < 0) {
            sliderLength = 0; // Ensure the slider length is non-negative
        }


        double sL2 = L2+sliderLength; // Calculate the length of the second arm segment
        
        //dist = Math.min(dist, L1 + L2);
        //dist = Math.max(dist, Math.abs(L1 - L2));

        double angle1 = Math.acos((L1*L1 + dist*dist - sL2*sL2) / (2 * L1 * dist));
        
        double baseAngle = Math.atan2(dy, dx);
        double shoulderAngle = baseAngle - angle1;
        shoulderAngle= Math.min(shoulderAngle, Math.toRadians(80));//Math.PI/2); // Limit shoulder angle to [-90°, 90°]
        double jointX = baseX + L1 * Math.cos(shoulderAngle);
        double jointY = baseY + L1 * Math.sin(shoulderAngle);
        dx = target.x - jointX;
        dy = target.y - jointY;

// True angle between joint and target
double targetAngle = Math.atan2(dy, dx);

// Elbow angle is angle between L1 and L2 segments
double elbowAngle = targetAngle - shoulderAngle;

        dist = Math.hypot(dx, dy);
        sliderLength = dist - L2; // Calculate the slider length
        if (sliderLength < 0) {
            sliderLength = 0; // Ensure the slider length is non-negative
        }
        


        double angle2 = Math.acos((L1*L1 + sL2*sL2 - dist*dist) / (2 * L1 * sL2));
       // double elbowAngle = Math.PI - angle2;
        System.out.println("Shoulder Angle: " + Math.toDegrees(shoulderAngle));
        System.out.println("Angle2: " + Math.toDegrees(angle2));
        System.out.println("Elbow Angle: " + (180.0-Math.toDegrees(elbowAngle)));
        System.out.println("Slider: " + sliderLength);
        double endX = jointX + sL2 * Math.cos(shoulderAngle + elbowAngle);
        double endY = jointY + sL2 * Math.sin(shoulderAngle + elbowAngle);
        double sEndx = jointX + sliderLength * Math.cos(shoulderAngle + elbowAngle);
        double sEndy = jointY + sliderLength * Math.sin(shoulderAngle + elbowAngle);
       
        g2.setColor(Color.BLUE);
        g2.setStroke(new BasicStroke(4));
        g2.drawLine((int) baseX, (int) baseY, (int) jointX, (int) jointY);
        g2.drawLine((int) jointX, (int) jointY, (int) endX, (int) endY);
        g2.setColor(Color.GREEN);
        g2.drawLine((int) jointX, (int) jointY, (int) sEndx, (int) sEndy);

        g2.setColor(Color.BLACK);
        g2.fillOval((int) baseX - 5, (int) baseY - 5, 10, 10);
        g2.fillOval((int) jointX - 5, (int) jointY - 5, 10, 10);

        g2.setColor(Color.MAGENTA);
        g2.fillOval((int) target.x - 4, (int) target.y - 4, 8, 8);
        
        g2.setColor(Color.BLACK);
        g2.setFont(new Font("SansSerif", Font.PLAIN, 12));
        g2.drawString(String.format("Shoulder: %.1f°", Math.toDegrees(shoulderAngle)), 10, 20);
        g2.drawString(String.format("Elbow: %.1f°",(180- Math.toDegrees(elbowAngle))), 10, 35);
       // g2.drawString(String.format("L2 Angle: %.1f°", Math.toDegrees(l2Angle)), 10, 50);

    }

    public static Point2D.Double scaleIt(Point2D.Double point) {
        return new Point2D.Double(point.x * 1000, point.y * 1000);
    }
    /** Call this from Robot.java in simulation only */
    public static void showVisualizer() {
        if (GraphicsEnvironment.isHeadless()) {
            System.out.println("GUI not available (headless environment).");
            return;
        }

        List<Point2D.Double> controlPoints = Arrays.asList(
            scaleIt(RobotContainer.startPoint),
            scaleIt(RobotContainer.controlPoint1),
            scaleIt(RobotContainer.controlPoint2),
            scaleIt(RobotContainer.endPoint));
        // Uncomment the following lines to use hardcoded control points instead
        //     , // Start point
        //     new Point2D.Double(0, 300),
        //     new Point2D.Double(-100, 500),
        //     new Point2D.Double(-200, 600),
        //     new Point2D.Double(-100, 900)
        // );

        SwingUtilities.invokeLater(() -> {
            JFrame frame = new JFrame("Bezier Curve Arm Visualizer");
            frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            frame.getContentPane().add(new BezierArmVisualizer(controlPoints));
            frame.pack();
            frame.setLocationRelativeTo(null);
            frame.setVisible(true);
        });
    }
}
