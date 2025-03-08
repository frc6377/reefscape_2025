// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CoralScorer;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.signaling.RGB;
import frc.robot.subsystems.signaling.Signaling;
import java.util.function.Supplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReefSignaling {
  private final CoralScorer coralScorerSubsystem;
  private final Elevator elavatorSubsystem;
  private final Signaling signalingSubsystem;

  /** Creates a new ReefSignaling. */
  public ReefSignaling(
      CoralScorer coralScorerSubsystem, Elevator elavatorSubsystem, Signaling signalingSubsystem) {
    this.coralScorerSubsystem = coralScorerSubsystem;
    this.elavatorSubsystem = elavatorSubsystem;
    this.signalingSubsystem = signalingSubsystem;
  }

  public Supplier<RGB> getRGB() {
    if (coralScorerSubsystem.hasCoralBool()
        && elavatorSubsystem
            .getElevatorHeight()
            .gt(Constants.ElevatorConstants.kL0Height.plus(Inches.of(5)))) {
      if (coralScorerSubsystem.getReefSensorBool()) {
        return () -> RGB.GREEN;
      } else {
        return () -> RGB.YELLOW;
      }
    } else {
      return () -> signalingSubsystem.getColorFromAlliance(Constants.kAllianceColor);
    }
  }

  public Command getCommand() {
    return signalingSubsystem.setColor(getRGB().get());
  }
}
