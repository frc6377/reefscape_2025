// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.Set;

/** Add your docs here. */
public class HowdyTempWarning {
  private final NetworkTable MotorTempsTable =
      NetworkTableInstance.getDefault().getTable("Motor Temps");

  public Set<String> MotorTemps = MotorTempsTable.getKeys();

  public void checkTemps() {
    MotorTemps = MotorTempsTable.getKeys();
    for (String key : MotorTemps) {
      if (100 < MotorTempsTable.getEntry(key).getDouble(Double.NaN)) {
        System.out.println("Motor Temp at " + key + " is too high stop and wait");
      } else if (120 < MotorTempsTable.getEntry(key).getDouble(Double.NaN)) {
      }
    }
  }
}
