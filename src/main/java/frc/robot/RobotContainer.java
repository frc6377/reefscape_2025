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

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CoralScorer;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.MapleSimArenaSubsystem;
import frc.robot.subsystems.TempIntake;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.vision.*;
import java.util.Set;
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
  private final Drive drive;
  private final Vision vision;
  private final Elevator elevator = new Elevator();
  private final CoralScorer coralScorer = new CoralScorer();
  private TempIntake tempIntake;
  private MapleSimArenaSubsystem m_MapleSimArenaSubsystem;

  private SwerveDriveSimulation driveSimulation;
  private Pose2d driveSimDefualtPose = new Pose2d(2, 2, new Rotation2d());
  private Pose3d robotCoralPose = new Pose3d();

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
                new ModuleIOTalonFXReal(TunerConstants.FrontRight),
                new ModuleIOTalonFXReal(TunerConstants.BackLeft),
                new ModuleIOTalonFXReal(TunerConstants.BackRight));
        this.vision =
            new Vision(
                drive, new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation));

        tempIntake = new TempIntake();

        break;
      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        driveSimDefualtPose =
            DriverStation.getAlliance().orElse(Alliance.Red).equals(Alliance.Blue)
                ? new Pose2d(
                    Meters.of(2), Constants.kFieldWidth.minus(Meters.of(2)), new Rotation2d())
                : new Pose2d(
                    Constants.kFieldLength.minus(Meters.of(2)),
                    Meters.of(2),
                    new Rotation2d(Degrees.of(180)));

        driveSimulation = new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d());
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        m_MapleSimArenaSubsystem = new MapleSimArenaSubsystem(driveSimulation);
        tempIntake = new TempIntake(driveSimulation);

        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOTalonFXSim(TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
                new ModuleIOTalonFXSim(TunerConstants.FrontRight, driveSimulation.getModules()[1]),
                new ModuleIOTalonFXSim(TunerConstants.BackLeft, driveSimulation.getModules()[2]),
                new ModuleIOTalonFXSim(TunerConstants.BackRight, driveSimulation.getModules()[3]));
        vision =
            new Vision(
                drive,
                new VisionIOPhotonVisionSim(
                    camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose));

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});

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

  private void configureButtonBindings() {
    // Change the raw boolean to true to pick keyboard durring simulation
    boolean usingKeyboard = false && Robot.isSimulation();

    OI.getButton(usingKeyboard ? OI.Keyboard.Z : OI.Driver.X).onTrue(elevator.L0());
    OI.getButton(OI.Driver.Back).onTrue(elevator.L1());
    OI.getButton(usingKeyboard ? OI.Keyboard.X : OI.Driver.A).onTrue(elevator.L2());
    OI.getButton(usingKeyboard ? OI.Keyboard.C : OI.Driver.B).onTrue(elevator.L3());
    OI.getButton(usingKeyboard ? OI.Keyboard.V : OI.Driver.Y).onTrue(elevator.L4());
    OI.getPOVButton(OI.Driver.DPAD_UP)
        .whileTrue(elevator.goUp(OI.getAxisSupplier(OI.Driver.RightY)));
    OI.getPOVButton(OI.Driver.DPAD_DOWN)
        .whileTrue(elevator.goUp(OI.getAxisSupplier(OI.Driver.RightY)));

    SmartDashboard.putData(elevator.limitHit());

    // Score Commpands
    OI.getTrigger(usingKeyboard ? OI.Keyboard.ForwardSlash : OI.Driver.LTrigger)
        .whileTrue(
            Robot.isSimulation()
                ? m_MapleSimArenaSubsystem.scoreCoral(() -> robotCoralPose)
                : coralScorer.scoreClockWise());
    OI.getButton(usingKeyboard ? OI.Keyboard.ArrowUpDown : OI.Driver.LBumper)
        .whileTrue(coralScorer.scoreCounterClockWise());

    // Temp Intake
    OI.getTrigger(OI.Driver.RTrigger).whileTrue(tempIntake.IntakeCommand());
    OI.getButton(OI.Driver.RBumper).whileTrue(tempIntake.OuttakeCommand());

    // Reset gyro / odometry, Runnable
    final Runnable resetGyro =
        Constants.currentMode == Constants.Mode.SIM
            ? () ->
                drive.setPose(
                    driveSimulation
                        .getSimulatedDriveTrainPose()) // reset odometry to actual robot pose
            // during simulation
            : () ->
                drive.setPose(
                    new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -OI.getAxisSupplier(usingKeyboard ? OI.Keyboard.AD : OI.Driver.LeftY).get(),
            () -> OI.getAxisSupplier(usingKeyboard ? OI.Keyboard.WS : OI.Driver.LeftX).get(),
            () ->
                OI.getAxisSupplier(usingKeyboard ? OI.Keyboard.ArrowLeftRight : OI.Driver.RightX)
                    .get()));
    OI.getButton(OI.Driver.Start).onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

    OI.getButton(usingKeyboard ? OI.Keyboard.M : OI.Driver.RSB)
        .whileTrue(DriveCommands.GoToPose(() -> drive.getClosestScorePose(), Set.of(drive)));

    for (int i = 0; i < Constants.kPoleLetters.length; i++) {
      for (int j = 0; j < 3; j++) {
        OI.getButton(OI.StreamDeck.streamDeckButtons[i])
            .onTrue(drive.setPoseScored(Constants.kPoleLetters[i], j));
      }
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

  public void startAuto() {
    if (Constants.currentMode != Constants.Mode.SIM) return;
    driveSimulation.setSimulationWorldPose(drive.getPose());
  }

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    driveSimulation.setSimulationWorldPose(driveSimDefualtPose);
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

    Pose3d closestReef = m_MapleSimArenaSubsystem.getClosestScorePose(robotCoralPose);

    if (closestReef != null) {
      Logger.recordOutput(
          "FieldSimulation/Closest Score Pose", new Pose3d[] {robotCoralPose, closestReef});
    } else {
      Logger.recordOutput(
          "FieldSimulation/Closest Score Pose", new Pose3d[] {new Pose3d(), new Pose3d()});
    }

    if (tempIntake.GetPieceFromIntake()) {
      m_MapleSimArenaSubsystem.setRobotHasCoral(true);
    }

    if (m_MapleSimArenaSubsystem.getRobotHasCoral()) {
      Pose2d drivePose = driveSimulation.getSimulatedDriveTrainPose();
      robotCoralPose =
          new Pose3d(
              drivePose.getX(),
              drivePose.getY(),
              elevator.getElevatorMechHeight().in(Meters),
              new Rotation3d(0, 0, drivePose.getRotation().getRadians()));

      Logger.recordOutput("FieldSimulation/Robot Game Piece Pose", robotCoralPose);
    } else {
      Logger.recordOutput("FieldSimulation/Robot Game Piece Pose", new Pose3d());
    }
  }
}
