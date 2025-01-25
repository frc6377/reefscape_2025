// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Climber;

public class RobotContainer {

  private final Climber climber = new Climber();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    OI.getButton(OI.Operator.RTrigger).onTrue(climber.climb());
    OI.getButton(OI.Operator.LTrigger).onTrue(climber.retract());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
