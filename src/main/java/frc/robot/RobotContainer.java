// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.SubsystemToggles;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.vision.*;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
@SuppressWarnings("unused")
public class RobotContainer {
  // Subsystems
  private ElevatorSubsystem m_ElevatorSubsystem;
  private IntakeSubsystem m_IntakeSimSubsystem;
  private Drive drive;
  private MapleSimArenaSubsystem m_MapleSimArenaSubsystem;
  private Vision vision;
  public SwerveDriveSimulation driveSimulation = null;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        if (SubsystemToggles.kUseDrive) {
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(TunerConstants.FrontLeft),
                  new ModuleIOTalonFX(TunerConstants.FrontRight),
                  new ModuleIOTalonFX(TunerConstants.BackLeft),
                  new ModuleIOTalonFX(TunerConstants.BackRight));
        }
        if (SubsystemToggles.kUseVision) {
          vision =
              new Vision(
                  drive,
                  new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation),
                  new VisionIOLimelight(VisionConstants.camera1Name, drive::getRotation));
        }

        if (SubsystemToggles.kUseIntake) {
          m_IntakeSimSubsystem = new IntakeSubsystem();
        }
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        if (SubsystemToggles.kUseDrive) {
          driveSimulation =
              new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
          SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
          drive =
              new Drive(
                  new GyroIOSim(driveSimulation.getGyroSimulation()),
                  new ModuleIOSim(driveSimulation.getModules()[0]),
                  new ModuleIOSim(driveSimulation.getModules()[1]),
                  new ModuleIOSim(driveSimulation.getModules()[2]),
                  new ModuleIOSim(driveSimulation.getModules()[3]));
          m_MapleSimArenaSubsystem = new MapleSimArenaSubsystem(driveSimulation);
        }

        if (SubsystemToggles.kUseVision) {
          vision =
              new Vision(
                  drive,
                  new VisionIOPhotonVisionSim(
                      camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
                  new VisionIOPhotonVisionSim(
                      camera1Name, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose));
        }

        if (SubsystemToggles.kUseIntake) {
          m_IntakeSimSubsystem = new IntakeSubsystem(driveSimulation);
        }

        if (SubsystemToggles.kUseElevator) {
          m_ElevatorSubsystem = new ElevatorSubsystem();
        }
        break;

      default:
        // Replayed robot, disable IO implementations
        if (SubsystemToggles.kUseDrive) {
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {});
        }

        if (SubsystemToggles.kUseVision) {
          vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});
        }
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    if (Robot.isReal()) {
      drive.setDefaultCommand(
          DriveCommands.joystickDrive(
              drive,
              () -> -OI.getAxisSupplier(OI.Driver.LeftX).get(),
              () -> -OI.getAxisSupplier(OI.Driver.LeftY).get(),
              () -> -OI.getAxisSupplier(OI.Driver.RightX).get()));
    } else {
      drive.setDefaultCommand(
          DriveCommands.joystickDrive(
              drive,
              () -> -OI.getAxisSupplier(OI.Keyboard.AD).get(),
              () -> -OI.getAxisSupplier(OI.Keyboard.WS).get(),
              () -> OI.getAxisSupplier(OI.Keyboard.ArrowLeftRight).get()));
    }

    if (Robot.isReal()) {
      // Reset gyro / odometry
      final Runnable resetOdometry =
          Constants.currentMode == Constants.Mode.SIM
              ? () -> drive.resetOdometry(driveSimulation.getSimulatedDriveTrainPose())
              : () ->
                  drive.resetOdometry(
                      new Pose2d(drive.getPose().getTranslation(), new Rotation2d()));
      OI.getButton(OI.Driver.Start).onTrue(Commands.runOnce(resetOdometry).ignoringDisable(true));
    } else if (Robot.isSimulation()) {
      OI.getButton(OI.Keyboard.M)
          .and(() -> !m_ElevatorSubsystem.getHasGamePiece())
          .whileTrue(m_IntakeSimSubsystem.IntakeCommand());

      OI.getButton(OI.Keyboard.Z).onTrue(m_ElevatorSubsystem.L1());
      OI.getButton(OI.Keyboard.X).onTrue(m_ElevatorSubsystem.L2());
      OI.getButton(OI.Keyboard.C).onTrue(m_ElevatorSubsystem.L3());
      OI.getButton(OI.Keyboard.V).onTrue(m_ElevatorSubsystem.L4());
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void resetSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void displaySimFieldToAdvantageScope() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));

    if (m_IntakeSimSubsystem.GetPieceFromIntake()) {
      m_ElevatorSubsystem.setHasGamePiece(true);
    }

    if (m_ElevatorSubsystem.getHasGamePiece()) {
      Pose2d drivePose = driveSimulation.getSimulatedDriveTrainPose();
      Logger.recordOutput(
          "FieldSimulation/Robot Game Piece Pose",
          new Pose3d(
              drivePose.getX(),
              drivePose.getY(),
              m_ElevatorSubsystem.getElevatorHeight().in(Meters),
              new Rotation3d(0, 0, drivePose.getRotation().getRadians())));
    } else {
      Logger.recordOutput("FieldSimulation/Robot Game Piece Pose", new Pose3d());
    }
  }
}
