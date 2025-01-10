// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Elevator;

public class RobotContainer {

  private final Elevator elevator = new Elevator();

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_driverController.rightBumper().and(m_driverController.leftBumper()).onTrue(elevator.L0());
    m_driverController.rightBumper().and(m_driverController.a()).onTrue(elevator.L1());
    m_driverController.rightBumper().and(m_driverController.b()).onTrue(elevator.L2());
    m_driverController.rightBumper().and(m_driverController.x()).onTrue(elevator.L3());
    m_driverController.rightBumper().and(m_driverController.y()).onTrue(elevator.L4());
    m_driverController
        .b()
        .and(m_driverController.rightBumper().negate())
        .whileTrue(elevator.goUp());
    m_driverController
        .a()
        .and(m_driverController.rightBumper().negate())
        .whileTrue(elevator.goDown());
    m_driverController.start().onTrue(elevator.zeroMotorEncoder());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
