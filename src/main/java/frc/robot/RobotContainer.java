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
import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
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
import frc.robot.Constants.FeildConstants;
import frc.robot.Constants.IntakeConstants.CoralEnum;
import frc.robot.OI.Driver;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
// import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralScorer;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.MapleSimArenaSubsystem;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.LocateCoral;
import frc.robot.subsystems.vision.*;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
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
  // Change the raw boolean to true to pick keyboard during simulation
  private final boolean usingKeyboard = true && Robot.isSimulation();

  private EventLoop testEventLoop = new EventLoop();

  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private MapleSimArenaSubsystem mapleSimArenaSubsystem;
  private final Elevator elevator = new Elevator();
  private final CoralScorer coralScorer = new CoralScorer();
  private static final Sensors sensors = new Sensors();
  private final IntakeSubsystem intake;
  //   private final Climber climber = new Climber();

  private boolean elevatorNotL1 = true;
  private boolean intakeAlgeaMode = false;

  private SwerveDriveSimulation driveSimulation;
  private Pose2d driveSimDefualtPose = new Pose2d(2, 2, new Rotation2d());

  // Trigger Variable
  private final Trigger coralOuttakeButton = OI.getButton(OI.Driver.RBumper);

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
        intake = new IntakeSubsystem(sensors, null);

        DriverStation.getAlliance();
        break;
      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        driveSimDefualtPose =
            Constants.kAllianceColor.equals(Alliance.Blue)
                ? new Pose2d(
                    Meters.of(2), FeildConstants.kFieldWidth.minus(Meters.of(2)), new Rotation2d())
                : new Pose2d(
                    FeildConstants.kFieldLength.minus(Meters.of(2)),
                    Meters.of(2),
                    new Rotation2d(Degrees.of(180)));

        driveSimulation = new SwerveDriveSimulation(Drive.mapleSimConfig, driveSimDefualtPose);
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
        intake = new IntakeSubsystem(sensors, driveSimulation);
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
        intake = new IntakeSubsystem(sensors, null);
        break;
    }

    // Register Named Commands
    NamedCommands.registerCommand(
        "ElvL0", elevator.L0().until(elevator.elevatorAtSetpoint().debounce(0.5)));
    NamedCommands.registerCommand(
        "ElvL2", elevator.L2().until(elevator.elevatorAtSetpoint().debounce(0.5)));
    NamedCommands.registerCommand(
        "ElvL3", elevator.L3().until(elevator.elevatorAtSetpoint().debounce(0.5)));
    NamedCommands.registerCommand(
        "ElvL4", elevator.L4().until(elevator.elevatorAtSetpoint().debounce(0.5)));
    NamedCommands.registerCommand(
        "Intake",
        elevator
            .L0()
            .until(elevator.elevatorAtSetpoint().debounce(0.5))
            .andThen(intakeAutoCommand()));
    NamedCommands.registerCommand("Score", scorerAutoCommand());

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
        "Drive Velocity Test", DriveCommands.RunVelocity(drive, MetersPerSecond.of(2), 5));

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
    testTrig(OI.getPOVButton(OI.Driver.DPAD_UP))
        .whileTrue(elevator.setElvPercent(OI.getAxisSupplier(OI.Driver.RightY).get()));
    // testTrig(OI.getPOVButton(OI.Driver.DPAD_RIGHT)).whileTrue(intake.intakeCommand());
    // testTrig(OI.getPOVButton(OI.Driver.DPAD_LEFT)).whileTrue(intake.outtakeCommand());
    // testTrig(OI.getButton(OI.Driver.RBumper)).whileTrue(intake.conveyorEject());
    // testTrig(OI.getButton(OI.Driver.LBumper)).whileTrue(intake.conveyorFeed());
    // testTrig(usingKeyboard ? OI.getButton(OI.Keyboard.M) : OI.getTrigger(OI.Driver.RTrigger))
    //     .whileTrue(climber.runRaw(Volts.of(3)));
    // testTrig(usingKeyboard ? OI.getButton(OI.Keyboard.Comma) : OI.getTrigger(OI.Driver.LTrigger))
    //     .whileTrue(climber.runRaw(Volts.of(-3)));
    // testTrig(usingKeyboard ? OI.getButton(OI.Keyboard.Period) : OI.getButton(OI.Driver.Start))
    //     .onTrue(climber.toggleJeopardy());
  }

  private void configureButtonBindings() {
    SmartDashboard.putData(
        Commands.runOnce(
            () -> {
              SignalLogger.stop();
            }));

    // Climber Buttons
    // OI.getTrigger(OI.Operator.RTrigger)
    //     .and(OI.getButton(OI.Operator.LBumper))
    //     .whileTrue(climber.climb());
    // OI.getTrigger(OI.Operator.LTrigger)
    //     .and(OI.getButton(OI.Operator.LBumper))
    //     .whileTrue(climber.retract());
    // OI.getButton(OI.Operator.Start).onTrue(climber.zero());

    // Elevator Buttons
    OI.getPOVButton(OI.Driver.DPAD_UP).onTrue(elevator.L0());
    OI.getPOVButton(OI.Driver.DPAD_LEFT).onTrue(elevator.L2());
    OI.getPOVButton(OI.Driver.DPAD_RIGHT).onTrue(elevator.L3());
    OI.getPOVButton(OI.Driver.DPAD_DOWN).onTrue(elevator.L4());
    OI.getButton(OI.Driver.Start).onTrue(elevator.limitHit());

    // Intake Buttons
    OI.getTrigger(OI.Driver.RTrigger).and(() -> !intakeAlgeaMode).whileTrue(intake.floorIntake());
    OI.getTrigger(OI.Driver.RTrigger).and(() -> intakeAlgeaMode).whileTrue(intake.algaeIntake());
    OI.getTrigger(OI.Driver.RTrigger).and(() -> intakeAlgeaMode).whileFalse(intake.algaeHold());

    intake
        .intakeHasUnalignedCoralTrigger()
        .onTrue(new LocateCoral(sensors::getSensorState, intake, () -> elevatorNotL1).asProxy());

    intake
        .intakeHasCoralTrigger()
        .onTrue(
            Robot.isReal()
                ? intake
                    .conveyerInCommand()
                    .alongWith(coralScorer.intakeCommand())
                    .until(
                        () ->
                            sensors.getSensorState() == CoralEnum.NO_CORAL
                                || coralScorer.hasCoral().getAsBoolean())
                : Commands.runOnce(
                    () -> {
                      if (!mapleSimArenaSubsystem.getRobotHasCoral()
                          && intake.intakeHasCoralTrigger().getAsBoolean())
                        mapleSimArenaSubsystem.setRobotHasCoral(true);
                    }));

    OI.getButton(OI.Driver.RBumper).whileTrue(intake.floorOuttake());
    OI.getButton(OI.Operator.Y)
        .onTrue(
            Commands.runOnce(
                () -> {
                  elevatorNotL1 = !elevatorNotL1;
                  Logger.recordOutput("Intake/Mode", elevatorNotL1);
                }));
    OI.getButton(OI.Operator.X)
        .onTrue(
            Commands.runOnce(
                () -> {
                  intakeAlgeaMode = !intakeAlgeaMode;
                  Logger.recordOutput("Intake/Algea Mode", intakeAlgeaMode);
                }));

    OI.getButton(OI.Driver.X).whileTrue(intake.l1ScoreModeB()); // Temporary
    intake.setDefaultCommand(intake.Idle());

    // Scorer Buttons
    OI.getTrigger(OI.Driver.LTrigger)
        .and(() -> !intakeAlgeaMode)
        .whileTrue(coralScorer.scoreCommand());
    OI.getTrigger(OI.Driver.LTrigger).and(() -> intakeAlgeaMode).whileTrue(intake.algaeOuttake());
    OI.getButton(OI.Driver.LBumper).whileTrue(coralScorer.reverseCommand());

    // Reset gyro / odometry, Runnable
    final Runnable resetGyro =
        Constants.currentMode == Constants.Mode.SIM
            ? () -> drive.setPose(driveSimulation.getSimulatedDriveTrainPose())
            : () ->
                drive.setPose(
                    new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            OI.getAxisSupplier(OI.Driver.LeftY),
            OI.getAxisSupplier(OI.Driver.LeftX),
            OI.getButton(Driver.RSB).getAsBoolean()
                ? () -> 0.0
                : OI.getAxisSupplier(OI.Driver.RightX)));
    OI.getButton(OI.Driver.Back).onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));
    OI.getButton(OI.Driver.RSB)
        .whileTrue(
            DriveCommands.AlignToReef(
                drive,
                OI.getAxisSupplier(Driver.LeftY),
                OI.getAxisSupplier(Driver.LeftX),
                drive.getAlignRotation()));

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

    // Button to update Setpoints of the elevator based on the Stream Deck nobs
    OI.getButton(OI.StreamDeck.streamDeckButtons[1][16])
        .onTrue(
            elevator.tuneSetpoints(
                OI.getAxisSupplier(OI.StreamDeck.Nob1),
                OI.getAxisSupplier(OI.StreamDeck.Nob2),
                OI.getAxisSupplier(OI.StreamDeck.Nob3),
                OI.getAxisSupplier(OI.StreamDeck.Nob4)));

    // Temp Keyboard Buttons for sim with no controller
    if (usingKeyboard) {
      // Driving
      drive.setDefaultCommand(
          DriveCommands.joystickDrive(
              drive,
              () -> OI.getAxisSupplier(OI.Keyboard.AD).get(),
              () -> OI.getAxisSupplier(OI.Keyboard.WS).get(),
              () -> OI.getAxisSupplier(OI.Keyboard.ArrowLR).get()));
      OI.getButton(OI.Keyboard.M)
          .whileTrue(
              DriveCommands.AlignToReef(
                  drive,
                  OI.getAxisSupplier(OI.Keyboard.AD),
                  OI.getAxisSupplier(OI.Keyboard.WS),
                  drive.getAlignRotation()));

      // Intake
      OI.getButton(OI.Keyboard.ForwardSlash).whileTrue(intake.floorIntake());

      // Elevator Buttons
      OI.getButton(OI.Keyboard.Z).onTrue(elevator.L0());
      OI.getButton(OI.Keyboard.X).onTrue(elevator.L2());
      OI.getButton(OI.Keyboard.C).onTrue(elevator.L3());
      OI.getButton(OI.Keyboard.V).onTrue(elevator.L4());

      // Scorer
      OI.getButton(OI.Keyboard.Period).whileTrue(coralScorer.scoreCommand());
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

  public Command intakeAutoCommand() {
    if (Robot.isSimulation()) {
      return intake.floorIntake().onlyWhile(() -> !mapleSimArenaSubsystem.getRobotHasCoral());
    } else {
      return intake.floorIntake().until(intake.intakeHasCoralTrigger());
    }
  }

  public Command scorerAutoCommand() {
    if (Robot.isSimulation()) {
      return Commands.runOnce(() -> intake.removePieceFromIntakeSim())
          .andThen(mapleSimArenaSubsystem.scoreCoral())
          .until(() -> !mapleSimArenaSubsystem.getRobotHasCoral());
    } else {
      return coralScorer.scoreCommand().until(coralScorer.hasCoral().negate());
    }
  }

  public void givePreLoad() {
    intake.addGamePieceToIntakeSim();
  }

  public void startAuto() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    driveSimulation.setSimulationWorldPose(drive.getPose());
  }

  public void seedIntakeEncoder() {
    intake.seedEncoder();
  }

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    driveSimulation.setSimulationWorldPose(driveSimDefualtPose);
    mapleSimArenaSubsystem.resetSimFeild().initialize();
    intake.resetSim();
  }

  public void displaySimFieldToAdvantageScope() {
    if (Constants.currentMode != Constants.Mode.SIM) return;
    mapleSimArenaSubsystem.updateRobotCoralPose(elevator.getElevatorHeight());
  }
}
