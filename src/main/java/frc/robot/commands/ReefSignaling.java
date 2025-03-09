// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.signaling.RGB;
import frc.robot.subsystems.signaling.Signaling;
import java.util.function.Supplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReefSignaling {
  private final Signaling signalingSubsystem;

  /** Creates a new ReefSignaling. */
  public ReefSignaling(Signaling signalingSubsystem) {
    this.signalingSubsystem = signalingSubsystem;
  }

  public Supplier<RGB> getRGB() {
    if (true) {
      if (true) {
        return () -> RGB.GREEN;
      } else {
        return () -> RGB.YELLOW;
      }
    } else {
      return () -> RGB.BLUE;
    }
  }

  public Command getCommand() {
    return signalingSubsystem.setColor(getRGB().get());
  }
}
