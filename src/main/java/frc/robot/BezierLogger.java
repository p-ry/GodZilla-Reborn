package frc.robot;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StructLogEntry;
import java.util.List;
import java.awt.geom.Point2D;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public class BezierLogger {
    private final DoubleArrayLogEntry curveLog;
    private final DoubleArrayLogEntry controlLog;

    public BezierLogger() {
        DataLog log = DataLogManager.getLog();
       curveLog = new DoubleArrayLogEntry(log, "/Arm/BezierPoint");
       controlLog = new DoubleArrayLogEntry(log, "/Arm/ControlPoint");
    }

    public void logCurve(List<Point2D> curvePoints, Point2D... controlPoints) {
        for (Point2D pt : curvePoints) {
            curveLog.append(new double[] {pt.getX(), pt.getY()});
        }
        for (Point2D cp : controlPoints) {
            controlLog.append(new double[] {cp.getX(), cp.getY()});
        }
    }
}
