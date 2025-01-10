// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

public class MapleSimArenaSubsystem extends SubsystemBase {
  private SwerveDriveSimulation driveSimulation;

  /** Creates a new MapleSimArenaSubsystem. */
  public MapleSimArenaSubsystem() {}

  public SwerveDriveSimulation getDriveSimulation() {
    return driveSimulation;
  }

  public Command clearSimFeild() {
    return Commands.runOnce(() -> SimulatedArena.getInstance().clearGamePieces());
  }

  public Command resetSimFeild() {
    return Commands.runOnce(() -> SimulatedArena.getInstance().placeGamePiecesOnField());
  }

  public Command resetSimFeildAuto() {
    return Commands.runOnce(() -> SimulatedArena.getInstance().resetFieldForAuto());
  }

  private StructArrayPublisher<Pose3d> coralPoses =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("CoralPieceArray", Pose3d.struct)
          .publish();
  private StructArrayPublisher<Pose3d> algeaPoses =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("AlgeaPieceArray", Pose3d.struct)
          .publish();

  @Override
  public void periodic() {
    coralPoses.accept(
        SimulatedArena.getInstance().getGamePiecesByType("Coral").toArray(Pose3d[]::new));
    algeaPoses.accept(
        SimulatedArena.getInstance().getGamePiecesByType("Algae").toArray(Pose3d[]::new));
  }
}
