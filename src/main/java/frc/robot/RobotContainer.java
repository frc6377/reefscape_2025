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

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.SimulationFeildConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CoralScorer;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MapleSimArenaSubsystem;
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

  private EventLoop testEventLoop = new EventLoop();

  // Change the raw boolean to true to pick keyboard during simulation
  private final boolean usingKeyboard = true && Robot.isSimulation();

  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final IntakeSubsystem intake;
  private final Elevator elevator = new Elevator();
  private final CoralScorer coralScorer = new CoralScorer();
  private MapleSimArenaSubsystem mapleSimArenaSubsystem;

  private SwerveDriveSimulation driveSimulation;
  private Pose2d driveSimDefualtPose = new Pose2d(2, 2, new Rotation2d());

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
        intake = IntakeSubsystem.create();

        break;
      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        driveSimDefualtPose =
            DriverStation.getAlliance().orElse(Alliance.Red).equals(Alliance.Blue)
                ? new Pose2d(
                    Meters.of(2),
                    SimulationFeildConstants.kFieldWidth.minus(Meters.of(2)),
                    new Rotation2d())
                : new Pose2d(
                    SimulationFeildConstants.kFieldLength.minus(Meters.of(2)),
                    Meters.of(2),
                    new Rotation2d(Degrees.of(180)));

        driveSimulation = new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d());
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        mapleSimArenaSubsystem = new MapleSimArenaSubsystem(driveSimulation);

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
        intake = IntakeSubsystem.create(driveSimulation);

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
        intake = IntakeSubsystem.create();

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
    autoChooser.addOption(
        "Drive SysId Turning (Quasistatic Forward)",
        drive.sysIdQuasistaticTurning(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId Turning (Quasistatic Reverse)",
        drive.sysIdQuasistaticTurning(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId Turning (Dynamic Forward)",
        drive.sysIdDynamicTurning(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId Turning (Dynamic Reverse)",
        drive.sysIdDynamicTurning(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Elevator SysID (Quasistatic Forward)", elevator.sysIdQuasistatic(Direction.kForward));
    autoChooser.addOption(
        "Elevator SysID (Quasistatic Reverse)", elevator.sysIdQuasistatic(Direction.kReverse));
    autoChooser.addOption(
        "Elevator SysID (Dynamic Forward)", elevator.sysIdDynamic(Direction.kForward));
    autoChooser.addOption(
        "Elevator SysID (Dynamic Reverse)", elevator.sysIdDynamic(Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
    configureTestButtonBindsing();
  }

  public EventLoop getTestEventLoop() {
    return testEventLoop;
  }

  private Trigger testTrig(Trigger t) {
    return new Trigger(testEventLoop, t);
  }

  private void configureTestButtonBindsing() {
    testTrig(usingKeyboard ? OI.getButton(OI.Keyboard.Period) : OI.getPOVButton(OI.Driver.DPAD_UP))
        .whileTrue(
            usingKeyboard
                ? elevator.goUp(() -> 1.0)
                : elevator.goUp(OI.getAxisSupplier(OI.Driver.RightY)));
    testTrig(usingKeyboard ? OI.getButton(OI.Keyboard.Comma) : OI.getPOVButton(OI.Driver.DPAD_DOWN))
        .whileTrue(
            usingKeyboard
                ? elevator.goDown(() -> 1.0)
                : elevator.goDown(OI.getAxisSupplier(OI.Driver.RightY)));
    testTrig(OI.getPOVButton(OI.Driver.DPAD_RIGHT)).whileTrue(intake.intakeCommand());
    testTrig(OI.getPOVButton(OI.Driver.DPAD_LEFT)).whileTrue(intake.outtakeCommand());
    testTrig(OI.getButton(OI.Driver.RBumper)).whileTrue(intake.conveyorEject());
    testTrig(OI.getButton(OI.Driver.LBumper)).whileTrue(intake.conveyorFeed());
    testTrig(OI.getPOVButton(OI.Operator.DPAD_UP)).whileTrue(intake.intakeAndConveyorCommandSafe());
    testTrig(OI.getPOVButton(OI.Operator.DPAD_DOWN))
        .whileTrue(intake.intakeAndConveyorCommandScoreL1());
    testTrig(OI.getButton(OI.Driver.A)).onTrue(intake.seedEncoder());
    testTrig(OI.getButton(OI.Driver.X)).whileTrue(intake.extendPivotCommand());
    testTrig(OI.getButton(OI.Driver.Y)).whileTrue(intake.retractPivotCommand());
  }

  private void configureButtonBindings() {
    // Change the raw boolean to true to pick keyboard durring simulation
    boolean usingKeyboard = true;
    Logger.recordOutput("USING KEYBOARD", usingKeyboard);

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

    if (usingKeyboard && Robot.isSimulation()) {
      OI.getButton(OI.Keyboard.Z).onTrue(elevator.L0());
      OI.getButton(OI.Keyboard.X).onTrue(elevator.L2());
      OI.getButton(OI.Keyboard.C).onTrue(elevator.L3());
      OI.getButton(OI.Keyboard.V).onTrue(elevator.L4());
      SmartDashboard.putData(elevator.limitHit());

      OI.getPOVButton(OI.Driver.DPAD_DOWN).onTrue(Commands.runOnce(() -> SignalLogger.stop()));

      // Score Commpands
      OI.getButton(OI.Keyboard.ForwardSlash)
          .whileTrue(
              Robot.isSimulation()
                  ? mapleSimArenaSubsystem.scoreCoral()
                  : coralScorer.scoreClockWise());

      // Temp Intake
      OI.getButton(OI.Keyboard.Period).whileTrue(intake.intakePivotCommand());
      OI.getButton(OI.Keyboard.Comma).whileTrue(intake.pivotDownCommand());

      // Default command, normal field-relative drive
      drive.setDefaultCommand(
          DriveCommands.joystickDrive(
              drive,
              OI.getAxisSupplier(OI.Keyboard.AD),
              OI.getAxisSupplier(OI.Keyboard.WS),
              OI.getAxisSupplier(OI.Keyboard.ArrowLR)));

      OI.getButton(OI.Keyboard.M)
          .whileTrue(DriveCommands.GoToPose(() -> drive.getClosestScorePose(), Set.of(drive)));

      mapleSimArenaSubsystem
          .canScore()
          .whileTrue(Commands.runEnd(() -> OI.Driver.setRumble(1), () -> OI.Driver.setRumble(0)));
    } else {
      OI.getButton(OI.Driver.X).onTrue(elevator.L0());
      OI.getButton(OI.Driver.Back).onTrue(elevator.L1());
      OI.getButton(OI.Driver.A).onTrue(elevator.L2());
      OI.getButton(OI.Driver.B).onTrue(elevator.L3());
      OI.getButton(OI.Driver.Y).onTrue(elevator.L4());
      OI.getPOVButton(OI.Driver.DPAD_UP)
          .whileTrue(elevator.goUp(OI.getAxisSupplier(OI.Driver.RightY)));
      OI.getPOVButton(OI.Driver.DPAD_DOWN)
          .whileTrue(elevator.goUp(OI.getAxisSupplier(OI.Driver.RightY)));

      SmartDashboard.putData(elevator.limitHit());

      // Score Commpands
      OI.getTrigger(OI.Driver.LTrigger)
          .whileTrue(
              Robot.isSimulation()
                  ? mapleSimArenaSubsystem.scoreCoral()
                  : coralScorer.scoreClockWise());
      OI.getButton(OI.Driver.LBumper).whileTrue(coralScorer.scoreCounterClockWise());

      // Temp Intake
      OI.getTrigger(OI.Driver.RTrigger).whileTrue(intake.intakeCommand());
      OI.getTrigger(OI.Driver.RBumper).whileTrue(intake.outtakeCommand());

      // Default command, normal field-relative drive
      drive.setDefaultCommand(
          DriveCommands.joystickDrive(
              drive,
              OI.getAxisSupplier(OI.Driver.LeftY),
              OI.getAxisSupplier(OI.Driver.LeftX),
              OI.getAxisSupplier(OI.Driver.RightX)));
      OI.getButton(OI.Driver.Start)
          .onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

      OI.getButton(OI.Driver.RSB)
          .whileTrue(DriveCommands.GoToPose(() -> drive.getClosestScorePose(), Set.of(drive)));
    }

    /* This is for creating the button mappings for logging what coral have been scored
     * The Driverstation has a hard limit of 32 buttons so we use 2 different vjoy controllers
     * to get the effective 64 buttons that we need for logging. this first 16 buttons of every controller are
     * used for the front and back coral scored poses. */
    int rows = 3;
    for (int i = 0; i < Constants.kPoleLetters.length / 2; i++) {
      for (int j = 0; j < rows; j++) {
        OI.getButton(OI.StreamDeck.streamDeckButtons[0][i * rows + j])
            .onTrue(drive.setPoseScored(Constants.kPoleLetters[i], j))
            .onFalse(drive.setPoseScored(Constants.kPoleLetters[i], j));
        OI.getButton(OI.StreamDeck.streamDeckButtons[1][i * rows + j])
            .onTrue(drive.setPoseScored(Constants.kPoleLetters[i + 6], j))
            .onFalse(drive.setPoseScored(Constants.kPoleLetters[i + 6], j));
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
    mapleSimArenaSubsystem.resetSimFeild();
  }

  public void displaySimFieldToAdvantageScope() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    if (!mapleSimArenaSubsystem.getRobotHasCoral() && intake.GetPieceFromIntake())
      mapleSimArenaSubsystem.setRobotHasCoral(true);
    mapleSimArenaSubsystem.updateRobotCoralPose(elevator.getElevatorHeight());
  }
}
