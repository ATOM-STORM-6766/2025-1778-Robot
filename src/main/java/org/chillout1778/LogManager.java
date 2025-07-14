package org.chillout1778;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LogManager {

  public enum LogLevel {
    ALL,
    SUBSYSTEMS_ONLY,
    COMMANDS_ONLY,
    NONE
  }

  private static boolean loggingEnabled = true;
  private static LogLevel logLevel = LogLevel.ALL;

  // 日志控制方法
  public static void setLoggingEnabled(boolean enabled) {
    loggingEnabled = enabled;
  }

  public static boolean isLoggingEnabled() {
    return loggingEnabled;
  }

  public static void setLogLevel(LogLevel level) {
    logLevel = level;
  }

  public static LogLevel getLogLevel() {
    return logLevel;
  }

  // 快速禁用所有日志的方法
  public static void disableAllLogging() {
    setLoggingEnabled(false);
  }

  // 快速启用所有日志的方法
  public static void enableAllLogging() {
    setLoggingEnabled(true);
    setLogLevel(LogLevel.ALL);
  }

  // 注册Subsystem
  public static void registerSubsystem(SubsystemBase subsystem) {
    if (!loggingEnabled || logLevel == LogLevel.COMMANDS_ONLY || logLevel == LogLevel.NONE) {
      return;
    }

    ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");
    tab.add(subsystem);
  }

  // 注册Command
  public static void registerCommand(String name, Command command) {
    if (!loggingEnabled || logLevel == LogLevel.SUBSYSTEMS_ONLY || logLevel == LogLevel.NONE) {
      return;
    }

    ShuffleboardTab tab = Shuffleboard.getTab("Commands");
    tab.add(name, command);
  }

  // 注册普通Sendable
  public static void registerSendable(String tabName, String name, Sendable sendable) {
    if (!loggingEnabled || logLevel == LogLevel.NONE) {
      return;
    }

    ShuffleboardTab tab = Shuffleboard.getTab(tabName);
    tab.add(name, sendable);
  }

  // 注册到Robot tab的便捷方法
  public static void registerToRobotTab(String name, Sendable sendable) {
    registerSendable("Robot", name, sendable);
  }

  // 注册到PID Controllers tab的便捷方法
  public static void registerPIDController(String name, Sendable pidController) {
    registerSendable("PID Controllers", name, pidController);
  }
}
