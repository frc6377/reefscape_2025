// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;

import java.util.Set;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class HowdyTempWarning {
  private final NetworkTable MotorTempsTable =
      NetworkTableInstance.getDefault().getTable("AdvantageKit/RealOutputs/Motor Temps");

  public Set<String> MotorTemps = MotorTempsTable.getKeys();

  public void checkTemps() {
    MotorTemps = MotorTempsTable.getKeys();
    for (String key : MotorTemps) {
      boolean motorTooHot = 120 < MotorTempsTable.getEntry(key).getDouble(Double.NaN);
      //TODO: set value to actual maximum safe operating temp
      boolean tempDangerous = 140 < MotorTempsTable.getEntry(key).getDouble(Double.NaN);
      Logger.recordOutput("Motor Temp Bool/" + key, motorTooHot);
      if (motorTooHot) {
        DriverStation.reportWarning("Motor Temp at " + key + " is too high stop and wait", false);
      }
      if (!Robot.isCompetition && tempDangerous) {
        DriverStation.reportError("Motor Temp at " + key + " is exceeding safe operating temperature forcing stop", null);
      }
    }
  }
}
