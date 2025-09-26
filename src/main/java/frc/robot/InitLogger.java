package frc.robot;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import java.util.concurrent.ConcurrentHashMap;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Map;

public class InitLogger {
  private static final String INIT_PREFIX = "Init/";

  public enum Level {
    INFO, WARN, ERROR
  }
  

  // Caches to avoid recreating entries
  private static final Map<String, StringLogEntry> stringEntries = new ConcurrentHashMap<>();
  private static final Map<String, DoubleLogEntry> doubleEntries = new ConcurrentHashMap<>();
  private static final Map<String, BooleanLogEntry> booleanEntries = new ConcurrentHashMap<>();

  /** Start logging early (call once in robotInit) */
  public static void startLogging() {
    if(Constants.logging){
    String dir = Files.exists(Path.of("/U")) ? "/U/logs" : "/home/lvuser/logs";
    try {
      Files.createDirectories(Path.of(dir));
    } catch (Exception ignored) {
    }
    DataLogManager.start(dir,"",0.5);//.start(dir, ""); // ~250ms flush = low I/O contention
    DataLogManager.logConsoleOutput(false); // avoid spam/overhead
    DriverStation.reportWarning("Logging to " + dir, false);
   
  }
  }

  public static void stopLogging() {
   if(Constants.logging){
     DataLogManager.stop();
   }
  }

  /** Log a double value under a given name/field (e.g., position). */
  public static void logDouble(String name, String field, double value) {
    if(Constants.logging){
      String key = name + "/" + field;
    DoubleLogEntry entry = doubleEntries.computeIfAbsent(key,
        k -> new DoubleLogEntry(DataLogManager.getLog(), k));
    entry.append(value);
    }
  }

  /** Log a named duration in seconds */
  public static void logDuration(String name, double durationSeconds) {
   if(Constants.logging){
    
    String key = INIT_PREFIX + name + "/Duration";
    DoubleLogEntry entry = doubleEntries.computeIfAbsent(key,
        k -> new DoubleLogEntry(DataLogManager.getLog(), k));
    entry.append(durationSeconds);
   }
  }

  /** Log the current timestamp under a given name */
  public static void logNow(String name) {
    if(Constants.logging){
       String key = INIT_PREFIX + name + "/Time";
    DoubleLogEntry entry = doubleEntries.computeIfAbsent(key,
        k -> new DoubleLogEntry(DataLogManager.getLog(), k));
    entry.append(Timer.getFPGATimestamp());
    }
  }

  /** Log a string message at INFO level */
  public static void logMessage(String name, String message) {
    if(Constants.logging){
       logMessage(name, Level.INFO, message);
    }
  }

  /** Log a string message with explicit level */
  public static void logMessage(String name, Level level, String message) {
    
    if(!Constants.logging){
    String key = INIT_PREFIX + name + "/" + level.name() + "/Message";
    StringLogEntry entry = stringEntries.computeIfAbsent(key,
        k -> new StringLogEntry(DataLogManager.getLog(), k));
    entry.append(message);
    }
  }

  /** Log a boolean value */
  public static void logBoolean(String name, String field, boolean value) {
   if(!Constants.logging){
    
    String key = INIT_PREFIX + name + "/" + field;
    BooleanLogEntry entry = booleanEntries.computeIfAbsent(key,
        k -> new BooleanLogEntry(DataLogManager.getLog(), k));
    entry.append(value);
   }
  }

  /** Measure and log how long a block of code takes */
  public static void time(String name, Runnable block) {
    if(!Constants.logging){
    double start = Timer.getFPGATimestamp();
    block.run();
    double end = Timer.getFPGATimestamp();
    logDuration(name, end - start);
    }
  }
}
