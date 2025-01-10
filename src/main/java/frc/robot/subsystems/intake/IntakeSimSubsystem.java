// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;

public class IntakeSimSubsystem extends SubsystemBase {
  private IntakeSimulation intakeSim;
  private SwerveDriveSimulation driveSimulation;

  /** Creates a new IntakeSimSubsystem. */
  public IntakeSimSubsystem(SwerveDriveSimulation driveSimulation) {
    this.driveSimulation = driveSimulation;
    intakeSim =
        IntakeSimulation.OverTheBumperIntake(
            "Coral",
            driveSimulation,
            IntakeConstants.kIntakeWidth,
            IntakeConstants.kIntakeExtension,
            IntakeSimulation.IntakeSide.FRONT,
            IntakeConstants.kIntakeCapacity);
  }

  public Command intake() {
    return Commands.runEnd(
        () -> {
          intakeSim.startIntake();
        },
        () -> {
          intakeSim.stopIntake();
        });
  }

  public Command outtake() {
    return Commands.runOnce(
        () -> {
          ReefscapeAlgaeOnFly newAlgae =
              new ReefscapeAlgaeOnFly(
                  driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                  new Translation2d(),
                  driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                  new Rotation2d(),
                  0,
                  10,
                  0);
          newAlgae.addGamePieceAfterTouchGround(SimulatedArena.getInstance());
          newAlgae.launch();
        });
  }

  public Command clearIntake() {
    return Commands.runOnce(
        () -> {
          intakeSim.removeObtainedGamePieces(SimulatedArena.getInstance());
        });
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("IntakeSimSubsystem/IntakeCurrent", intakeSim.getGamePiecesAmount());
  }
}
