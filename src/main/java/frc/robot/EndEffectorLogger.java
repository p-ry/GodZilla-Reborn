package frc.robot;
import org.littletonrobotics.advantagekit.log.struct.StructLogEntry; 

import edu.wpi.first.wpilibj.DataLogManager;
import java.awt.geom.Point2D;

public class EndEffectorLogger {
    private final StructLogEntry<Point2D.Double> effectorLog;

    public EndEffectorLogger() {
        var log = DataLogManager.getLog();
        StructLogEntry<Point2D.Double> effectorLog =   new StructLogEntry<Point2D.Double>(Point2dStruct.instance, DataLogManager.getLog(), "/Arm/EndEffector");
    
    }

    public void log(Point2D point) {
        effectorLog.append(new Point2D.Double(point.getX(), point.getY()));
    }
}
