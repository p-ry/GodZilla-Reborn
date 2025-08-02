package frc.robot;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
public class InitLogger {
    private static final String INIT_PREFIX = "Init/";

    /** Start logging early (call once in robotInit) */
    public static void startLogging() {
        DataLogManager.start();
    }

    /** Log a named duration in seconds */
    public static void logDuration(String name, double durationSeconds) {
        new DoubleLogEntry(DataLogManager.getLog(), INIT_PREFIX + name + "/Duration")
            .append(durationSeconds);
    }

    /** Log the current timestamp under a given name */
    public static void logNow(String name) {
        new DoubleLogEntry(DataLogManager.getLog(), INIT_PREFIX + name + "/Time")
            .append(Timer.getFPGATimestamp());
    }

    /** Log a string message */
    public static void logMessage(String name, String message) {
        new StringLogEntry(DataLogManager.getLog(), INIT_PREFIX + name + "/Message")
            .append(message);
    }

    /** Measure and log how long a block of code takes */
    public static void time(String name, Runnable block) {
        double start = Timer.getFPGATimestamp();
        block.run();
        double end = Timer.getFPGATimestamp();
        logDuration(name, end - start);
    }
}
