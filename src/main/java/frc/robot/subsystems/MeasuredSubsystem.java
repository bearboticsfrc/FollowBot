// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class MeasuredSubsystem extends SubsystemBase {
  static final double k_uS_to_mS = 1000.0; // microseconds
  
  static Map<String, MeasuredSubsystem> s_map = new ConcurrentHashMap<String, MeasuredSubsystem>();
  static boolean enabled = true;
  static long resetTime;
  static long lastEndTime;

  long min;
  long max;
  long total;
  long count;

   public MeasuredSubsystem() {
    super();
    s_map.put(getName(), this);
    resetStats();
  }

  public abstract void monitored_periodic();

  @Override
  public void periodic() {
    if (!enabled) {
      monitored_periodic();
      return;
    }
    long startTime = RobotController.getFPGATime();
    count++;
    monitored_periodic();
    lastEndTime = RobotController.getFPGATime();
    long time = lastEndTime - startTime;
    
    //save the stats
    total += time;
    min = Math.min(min, time);
    max = Math.max(max, time);
  }

  void resetStats() {
    count = max = total = 0L;
    min = 999999999L;
  }

  public double getMinTime() {
    return this.min / k_uS_to_mS;
  } 

  public double getMaxTime() {
    return this.max / k_uS_to_mS;
  } 

  public double getAvgTime() {
    return (count > 0) ? (this.total /this.count) / k_uS_to_mS  : -1.0;
  } 

  public String toString() {
   return String.format("Timing for %-25.20s: min = %05.3fms  max = %05.3fms avg = %05.3fms (%d) \n", getName(),
    getMinTime(), getMaxTime(), getAvgTime(), count );
  }

  public static String getStats() {
    StringBuilder s = new StringBuilder();
    long fcount = s_map.values().stream().findFirst().get().count;
    
    s.append("\n***********************************************************\n");
    s.append(String.format("Total run time: %05.5f seconds.\n", (lastEndTime - resetTime) / 1e6));
    s.append(String.format("Estimated frame period: %05.5f milli-seconds.\n\n", (lastEndTime - resetTime) / k_uS_to_mS / fcount));
    for (String k : s_map.keySet()) { 
      s.append(s_map.get(k).toString());
    }
    s.append("***********************************************************\n");
    return s.toString();
  }

  public static void resetAllStats() {
    for (String k : s_map.keySet()) { 
     s_map.get(k).resetStats();
    }
    resetTime = RobotController.getFPGATime();
  }

  public static void setEnabled(boolean enabled_param) {
    enabled = enabled_param;
  }

}