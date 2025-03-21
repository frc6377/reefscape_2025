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
import static frc.robot.Constants.IntakeConstants.kPivotCoralStationAngle;
import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera1;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IntakeConstants.CoralEnum;
import frc.robot.Constants.SubsystemEnabled;
import frc.robot.OI.Driver;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgeaRemover;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralScorer;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.MapleSimArenaSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFXReal;
import frc.robot.subsystems.drive.ModuleIOTalonFXSim;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.LocateCoral;
import frc.robot.subsystems.vision.*;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import java.util.Set;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.jspecify.annotations.Nullable;
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
  private final boolean usingKeyboard = false && Robot.isSimulation();

  private EventLoop testEventLoop = new EventLoop();

  // Subsystems
  @Nullable private final Drive drive;
  @Nullable private final Vision vision;
  @Nullable private final MapleSimArenaSubsystem mapleSimArenaSubsystem;

  @Nullable private final IntakeSubsystem intake;
  @Nullable private final Sensors sensors;

  @Nullable private final Elevator elevator;
  @Nullable private final CoralScorer coralScorer;

  @Nullable private final AlgeaRemover algeaRemover;
  @Nullable private final Climber climber;

  private boolean elevatorNotL1 = true;
  private boolean intakeAlgeaMode = false;
  private boolean coralStationMode = false;
  private Command scoreL1;

  private Command locateCoral;

  private final Trigger coralOuttakeButton;
  private final Trigger coralHandoffCompleteTrigger;
  private final Trigger UpButtonTrigger;
  private final Trigger DownButtonTrigger;
  private final Trigger RightButtonTrigger;
  private final Trigger LeftButtonTrigger;

  @Nullable private SwerveDriveSimulation driveSimulation;
  private Pose2d driveSimDefaultPose = new Pose2d(2, 2, new Rotation2d());

  // Dashboard inputs
  @Nullable private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        mapleSimArenaSubsystem = null;

        // Real robot, instantiate hardware IO implementations
        drive =
            SubsystemEnabled.kDrivebase
                ? new Drive(
                    new GyroIOPigeon2(),
                    new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
                    new ModuleIOTalonFXReal(TunerConstants.FrontRight),
                    new ModuleIOTalonFXReal(TunerConstants.BackLeft),
                    new ModuleIOTalonFXReal(TunerConstants.BackRight))
                : null;

        this.vision =
            SubsystemEnabled.kVision && drive != null
                ? new Vision(
                    drive,
                    // new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation),
                    new VisionIOLimelight(VisionConstants.camera1Name, drive::getRotation))
                : null;

        sensors = SubsystemEnabled.kIntake ? new Sensors() : null;
        intake =
            SubsystemEnabled.kIntake && sensors != null ? new IntakeSubsystem(sensors, null) : null;
        break;
      case SIM:
        if (SubsystemEnabled.kDrivebase) {
          // Sim robot, instantiate physics sim IO implementations
          driveSimDefaultPose =
              Constants.kAllianceColor.equals(Alliance.Blue)
                  ? new Pose2d(
                      Meters.of(2),
                      FieldConstants.kFieldWidth.minus(Meters.of(2)),
                      new Rotation2d())
                  : new Pose2d(
                      FieldConstants.kFieldLength.minus(Meters.of(2)),
                      Meters.of(2),
                      new Rotation2d(Degrees.of(180)));

          driveSimulation = new SwerveDriveSimulation(Drive.mapleSimConfig, driveSimDefaultPose);
          SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
          mapleSimArenaSubsystem = new MapleSimArenaSubsystem(driveSimulation);

          drive =
              new Drive(
                  new GyroIOSim(driveSimulation.getGyroSimulation()),
                  new ModuleIOTalonFXSim(TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
                  new ModuleIOTalonFXSim(
                      TunerConstants.FrontRight, driveSimulation.getModules()[1]),
                  new ModuleIOTalonFXSim(TunerConstants.BackLeft, driveSimulation.getModules()[2]),
                  new ModuleIOTalonFXSim(
                      TunerConstants.BackRight, driveSimulation.getModules()[3]));
        } else {
          mapleSimArenaSubsystem = null;
          drive = null;
        }

        vision =
            SubsystemEnabled.kVision && drive != null && driveSimulation != null
                ? new Vision(
                    drive,
                    new VisionIOPhotonVisionSim(
                        camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
                    new VisionIOPhotonVisionSim(
                        camera1Name, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose))
                : null;

        sensors = SubsystemEnabled.kIntake ? new Sensors() : null;
        intake =
            SubsystemEnabled.kIntake && sensors != null
                ? new IntakeSubsystem(sensors, driveSimulation)
                : null;
        break;

      default:
        // Replayed robot, disable IO implementations
        mapleSimArenaSubsystem = null;
        drive =
            SubsystemEnabled.kDrivebase
                ? new Drive(
                    new GyroIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {})
                : null;
        vision =
            SubsystemEnabled.kVision && drive != null
                ? new Vision(drive, new VisionIO() {}, new VisionIO() {})
                : null;
        sensors = SubsystemEnabled.kIntake ? new Sensors() : null;
        intake =
            SubsystemEnabled.kIntake && sensors != null ? new IntakeSubsystem(sensors, null) : null;
        break;
    }

    coralScorer = SubsystemEnabled.kCoralScorer ? new CoralScorer() : null;
    elevator = SubsystemEnabled.kCoralScorer ? new Elevator() : null;
    algeaRemover = SubsystemEnabled.kCoralScorer ? new AlgeaRemover() : null;
    climber = SubsystemEnabled.kCoralScorer ? new Climber() : null;

    // Trigger Variables
    coralOuttakeButton = OI.getButton(OI.Driver.RBumper);
    UpButtonTrigger = OI.getButton(OI.Driver.POV0);
    DownButtonTrigger = OI.getButton(OI.Driver.POV90);
    RightButtonTrigger = OI.getButton(OI.Driver.POV180);
    LeftButtonTrigger = OI.getButton(OI.Driver.POV270);
    coralHandoffCompleteTrigger =
        SubsystemEnabled.kIntake && sensors != null && coralScorer != null
            ? new Trigger(
                () ->
                    sensors.getSensorState() == CoralEnum.NO_CORAL
                        || coralScorer.hasCoralTrigger().getAsBoolean())
            : new Trigger(() -> true);
    scoreL1 = intake != null ? intake.l1ScoreModeA() : Commands.none();

    Trigger isDoneScoring =
        sensors != null
            ? new Trigger(() -> (sensors.getSensorState() == CoralEnum.NO_CORAL))
            : new Trigger(() -> true);

    // Register Named Commands
    if (elevator != null) {
      NamedCommands.registerCommand("ElvL0", elv0Command());
      NamedCommands.registerCommand(
          "ElvL2 DeadLine",
          elevator.L2().andThen(waitForElevator()).withDeadline(Commands.waitSeconds(1.75)));
      NamedCommands.registerCommand(
          "ElvL3 DeadLine",
          elevator.L3().andThen(waitForElevator()).withDeadline(Commands.waitSeconds(1.75)));
      NamedCommands.registerCommand(
          "ElvL4 DeadLine",
          elevator.L4().andThen(waitForElevator()).withDeadline(Commands.waitSeconds(1.75)));
      NamedCommands.registerCommand("ElvL2", elevator.L2().andThen(waitForElevator()));
      NamedCommands.registerCommand("ElvL3", elevator.L3().andThen(waitForElevator()));
      NamedCommands.registerCommand("ElvL4", elevator.L4().andThen(waitForElevator()));
      NamedCommands.registerCommand("Zero Elv", elevator.limitHit());
    }
    if (elevator != null && intake != null) {
      NamedCommands.registerCommand(
          "Intake",
          new SequentialCommandGroup(
              elv0Command(), intakeAutoCommand(), Commands.waitUntil(coralHandoffCompleteTrigger)));
      NamedCommands.registerCommand(
          "Intake Floor",
          new SequentialCommandGroup(
              elv0Command(),
              intakeFloorAutoCommand(),
              Commands.waitUntil(coralHandoffCompleteTrigger)));
    }
    if (coralScorer != null) NamedCommands.registerCommand("Score", scorerAutoCommand());
    NamedCommands.registerCommand("Set L1 Mode", Commands.runOnce(() -> elevatorNotL1 = false));
    if (intake != null)
      NamedCommands.registerCommand(
          "L1 Score",
          new SequentialCommandGroup(
              Commands.runOnce(() -> elevatorNotL1 = false),
              Commands.waitUntil(intake.intakeHasCoralTrigger()),
              scoreL1.asProxy().until(isDoneScoring.debounce(3))));
    if (drive != null && coralScorer != null) {
      NamedCommands.registerCommand(
          "Strafe", drive.strafe().until(coralScorer.scorerAlignedTrigger()));
      NamedCommands.registerCommand(
          "Start R - E",
          DriveCommands.GoToPose(
              Constants.DrivetrainConstants.SCORE_POSES.get("E"), Set.of(drive)));
      NamedCommands.registerCommand(
          "Start L - I",
          DriveCommands.GoToPose(
              Constants.DrivetrainConstants.SCORE_POSES.get("I"), Set.of(drive)));
    }

    // Set up auto routines
    autoChooser =
        drive != null
            ? new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser())
            : new LoggedDashboardChooser<>("Auto Choices", new SendableChooser<Command>());

    if (drive != null) {
      // Set up SysId routines
      autoChooser.addOption(
          "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
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
          "Drive SysID (All)",
          drive
              .sysIdQuasistaticTurning(SysIdRoutine.Direction.kForward)
              .andThen(Commands.waitSeconds(0.5))
              .andThen(drive.sysIdQuasistaticTurning(SysIdRoutine.Direction.kReverse))
              .andThen(Commands.waitSeconds(0.5))
              .andThen(drive.sysIdDynamicTurning(SysIdRoutine.Direction.kForward))
              .andThen(Commands.waitSeconds(0.5))
              .andThen(drive.sysIdDynamicTurning(SysIdRoutine.Direction.kReverse)));
    } else {
      autoChooser.addOption("No Autos Avaiable", Commands.none());
    }

    // Configure the button bindings
    configureButtonBindings();
    configureTestButtonBindings();
  }

  public EventLoop getTestEventLoop() {
    return testEventLoop;
  }

  private Trigger testTrig(Trigger t) {
    return new Trigger(testEventLoop, t);
  }

  private void configureTestButtonBindings() {
    // Elevator Test Buttons
    // testTrig(OI.getPOVButton(OI.Driver.DPAD_UP))
    //     .whileTrue(elevator.setElvPercent(OI.getAxisSupplier(OI.Driver.RightY).get()));

    // Algae Remover Test Buttons
    // testTrig(OI.getPOVButton(OI.Driver.DPAD_RIGHT)).whileTrue(intake.intakeCommand());
    // testTrig(OI.getPOVButton(OI.Driver.DPAD_LEFT)).whileTrue(intake.outtakeCommand());
    // testTrig(OI.getButton(OI.Driver.RBumper)).whileTrue(intake.conveyorEject());
    // testTrig(OI.getButton(OI.Driver.LBumper)).whileTrue(intake.conveyorFeed());
    // testTrig(OI.getButton(OI.Driver.X)).whileTrue(intake.extendPivotCommand());
    // testTrig(OI.getButton(OI.Driver.Y)).whileTrue(intake.retractPivotCommand());

    // Intake Test Buttons
    // testTrig(usingKeyboard ? OI.getButton(OI.Keyboard.Period) : OI.getButton(OI.Driver.A))
    //     .toggleOnTrue(intake.movePivot(IntakeConstants.kClimbingAngle));

    // Climber Test Buttons
    // testTrig(OI.getPOVButton(OI.Operator.DPAD_RIGHT)).onTrue(climber.servoToZero());
    // testTrig(OI.getButton(OI.Driver.LBumper)).onTrue(climber.engageServo());
    // testTrig(OI.getButton(OI.Driver.RBumper)).onTrue(climber.disengageServo());
    // testTrig(OI.getTrigger(OI.Driver.RTrigger)).whileTrue(climber.runRaw(Volts.of(3)));
    // testTrig(OI.getTrigger(OI.Driver.LTrigger)).whileTrue(climber.runRaw(Volts.of(-3)));
    // // testTrig(OI.getButton(OI.Driver.B)).onTrue(climber.extendToCage());
    // testTrig(usingKeyboard ? OI.getButton(OI.Keyboard.M) : OI.getTrigger(OI.Driver.RTrigger))
    //     .whileTrue(climber.runRaw(Volts.of(3)));
    // testTrig(usingKeyboard ? OI.getButton(OI.Keyboard.Comma) : OI.getTrigger(OI.Driver.LTrigger))
    //     .whileTrue(climber.runRaw(Volts.of(-3)));
    // testTrig(OI.getButton(OI.Driver.X)).onTrue(climber.climberToZero());
    // testTrig(OI.getButton(OI.Driver.A)).onTrue(climber.retract());
    // testTrig(OI.getButton(OI.Driver.Y)).onTrue(climber.extendToCage());
    // testTrig(OI.getButton(OI.Driver.B)).onTrue(climber.extendFully());
  }

  private void configureButtonBindings() {

    SmartDashboard.putData(
        Commands.runOnce(
            () -> {
              SignalLogger.stop();
            }));
    if (coralScorer != null && elevator != null) {
      coralScorer
          .scorerAlignedTrigger()
          .and(coralScorer.hasCoralTrigger())
          .and(elevator.elevatorAtSetpoint(ElevatorConstants.kL0Height).negate())
          .and(elevator.elevatorAtCurrentSetpoint())
          .whileTrue(
              Commands.runEnd(
                  () -> {
                    OI.Driver.setRumble(0.5);
                    OI.Operator.setRumble(0.5);
                  },
                  () -> {
                    OI.Driver.setRumble(0);
                    OI.Operator.setRumble(0);
                  }));
    }

    // Elevator Buttons
    if (elevator != null) {
      OI.getButton(OI.Driver.A).onTrue(elevator.L0());
      OI.getButton(OI.Driver.B).onTrue(elevator.L2());
      OI.getButton(OI.Driver.X).onTrue(elevator.L3());
      OI.getButton(OI.Driver.Y).onTrue(elevator.L4());
      OI.getButton(OI.Driver.Start).onTrue(elevator.limitHit());
    }

    Logger.recordOutput("Intake/Modes/L1 Score Mode", !elevatorNotL1);
    Logger.recordOutput("Intake/Modes/Algae Mode", intakeAlgeaMode);
    Logger.recordOutput("Intake/Modes/Coral Station Mode", coralStationMode);

    OI.getButton(OI.Operator.Y)
        .onTrue(
            Commands.runOnce(
                () -> {
                  elevatorNotL1 = !elevatorNotL1;
                  Logger.recordOutput("Intake/Modes/L1 Score Mode", !elevatorNotL1);
                }));
    OI.getButton(OI.Operator.X)
        .onTrue(
            Commands.runOnce(
                () -> {
                  intakeAlgeaMode = !intakeAlgeaMode;
                  Logger.recordOutput("Intake/Modes/Algae Mode", intakeAlgeaMode);
                }));
    OI.getButton(OI.Operator.B)
        .onTrue(
            Commands.runOnce(
                () -> {
                  Logger.recordOutput("Intake/Modes/Coral Station Mode", coralStationMode);
                }));

    if (intake != null) {
      OI.getButton(OI.Driver.RTrigger)
          .and(() -> !intakeAlgeaMode && !coralStationMode)
          .whileTrue(intake.floorIntake());
      OI.getButton(OI.Driver.RTrigger)
          .and(() -> !intakeAlgeaMode && coralStationMode)
          .whileTrue(intake.humanPlayerIntake());
      OI.getButton(OI.Driver.RTrigger).and(() -> intakeAlgeaMode).whileTrue(intake.algaeIntake());
      OI.getButton(OI.Driver.RTrigger).and(() -> intakeAlgeaMode).whileFalse(intake.algaeHold());

      if (sensors != null && intake != null)
        locateCoral =
            new LocateCoral(
                sensors::getSensorState,
                intake,
                coralOuttakeButton.or(OI.getButton(OI.Driver.RTrigger)));

      intake
          .intakeHasUnalignedCoralTrigger()
          .and(coralOuttakeButton.negate())
          .and(OI.getButton(OI.Driver.RTrigger).negate())
          .and(() -> !CommandScheduler.getInstance().isScheduled(scoreL1))
          .onTrue(locateCoral);

      OI.getButton(OI.Driver.LTrigger)
          .and(() -> !elevatorNotL1 && !intakeAlgeaMode)
          .whileTrue(scoreL1); // Temporary

      intake.setDefaultCommand(intake.Idle());

      if (sensors != null) {
        if (coralScorer != null) {
          // Command locateCoral =
          //     new LocateCoral(
          //         sensors::getSensorState,
          //         intake,
          //         coralOuttakeButton.or(OI.getTrigger(OI.Driver.LTrigger)));

          OI.getButton(OI.Driver.A)
              .whileTrue(intake.conveyerInCommand().alongWith(coralScorer.intakeCommand()));
          // .until(coralHandoffCompleteTrigger));

          if (elevator != null) {
            Trigger intakeButton =
                intake
                    .intakeHasCoralTrigger()
                    .and(() -> elevatorNotL1)
                    .and(coralOuttakeButton.negate())
                    .and(
                        () ->
                            !CommandScheduler.getInstance()
                                .isScheduled(
                                    new LocateCoral(
                                        sensors::getSensorState,
                                        intake,
                                        coralOuttakeButton.or(OI.getButton(OI.Driver.LTrigger)))))
                    .and(elevator.elevatorAtSetpoint(ElevatorConstants.kL0Height));

            intakeButton.onTrue(
                intake
                    .conveyerInCommand()
                    .alongWith(coralScorer.intakeCommand())
                    .until(coralHandoffCompleteTrigger));
            coralOuttakeButton.whileTrue(intake.floorOuttake());

            if (mapleSimArenaSubsystem != null) {
              intakeButton.onTrue(
                  Commands.runOnce(() -> mapleSimArenaSubsystem.setRobotHasCoral(true)));
              coralOuttakeButton.whileTrue(intake.floorOuttake());
            }
          }
        }
      }
    }

    // Scorer Buttons
    if (coralScorer != null && intake != null && mapleSimArenaSubsystem != null) {
      OI.getButton(OI.Driver.LScoreTrigger)
          .and(() -> !intakeAlgeaMode && elevatorNotL1)
          .whileTrue(
              Robot.isReal()
                  ? coralScorer.runScorer(OI.getAxisSupplier(OI.Driver.LeftTriggerAxis))
                  : mapleSimArenaSubsystem.scoreCoral());
      OI.getButton(OI.Driver.LTrigger).and(() -> intakeAlgeaMode).whileTrue(intake.algaeOuttake());
      OI.getButton(OI.Driver.LBumper).whileTrue(coralScorer.reverseCommand());
    }

    // Algae Remover
    if (algeaRemover != null) {
      OI.getButton(OI.Operator.LBumper).toggleOnTrue(algeaRemover.removeUpCommand());
      OI.getButton(OI.Operator.RBumper).toggleOnTrue(algeaRemover.removeDownCommand());
      OI.getButton(OI.Operator.LTrigger).whileTrue(algeaRemover.upCommand());
      OI.getButton(OI.Operator.RTrigger).whileTrue(algeaRemover.downCommand());
    }

    // Climber Buttons
    // if (climber != null && intake != null) {
    // OI.getPOVButton(OI.Operator.DPAD_UP)
    //     .onTrue(climber.retract())
    //     .toggleOnTrue(intake.movePivot(kClimbingAngle));
    // OI.getPOVButton(OI.Operator.DPAD_LEFT).onTrue(climber.extendToCage());
    // OI.getPOVButton(OI.Operator.DPAD_DOWN).onTrue(climber.extendFully());

    if (drive != null && driveSimulation != null) {
      final Runnable resetGyro =
          Constants.currentMode == Constants.Mode.SIM
              ? () -> drive.setPose(driveSimulation.getSimulatedDriveTrainPose())
              : () ->
                  drive.setPose(
                      new Pose2d(
                          drive.getPose().getTranslation(),
                          new Rotation2d(
                              DriverStation.getAlliance().get() == Alliance.Blue
                                  ? Degrees.zero()
                                  : Degrees.of(180)))); // zero gyro

      // Default command, normal field-relative drive
      drive.setDefaultCommand(
          DriveCommands.joystickDrive(
              drive,
              OI.getAxisSupplier(OI.Driver.LeftY),
              OI.getAxisSupplier(OI.Driver.LeftX),
              OI.getButton(Driver.RSB).getAsBoolean()
                  ? () -> 0.0
                  : OI.getAxisSupplier(OI.Driver.RightX)));
      OI.getButton(OI.Driver.Back)
          .onTrue(
              Robot.isReal()
                  ? Commands.runOnce(resetGyro, drive).ignoringDisable(true)
                  : Commands.runOnce(() -> resetSimulationField()));
      OI.getButton(OI.Driver.RSB)
          .whileTrue(
              DriveCommands.AlignToReef(
                  drive,
                  OI.getAxisSupplier(Driver.LeftY),
                  OI.getAxisSupplier(Driver.LeftX),
                  drive.getAlignRotation()));

      UpButtonTrigger.or(DownButtonTrigger)
          .or(RightButtonTrigger)
          .or(LeftButtonTrigger)
          .whileTrue(
              DriveCommands.POVDrive(
                  drive,
                  () ->
                      (UpButtonTrigger.getAsBoolean() ? 1.0 : 0.0)
                          + (DownButtonTrigger.getAsBoolean() ? -1.0 : 0.0),
                  () ->
                      (LeftButtonTrigger.getAsBoolean() ? 1.0 : 0.0)
                          + (RightButtonTrigger.getAsBoolean() ? -1.0 : 0.0)));

      /* This is for creating the button mappings for logging what coral have been scored
       * The Driverstation has a hard limit of 32 buttons so we use 2 different vjoy controllers
       * to get the effective 64 buttons that we need for logging. this first 16 buttons of every controller are
       * used for the front and back coral scored poses. */
      // int rows = 3;
      // for (int i = 0; i < Constants.kPoleLetters.length / 2; i++) {
      //   for (int j = 0; j < rows; j++) {
      //     OI.getButton(OI.StreamDeck.streamDeckButtons[0][i * rows + j])
      //         .onChange(drive.setPoseScored(Constants.kPoleLetters[i], j));
      //     OI.getButton(OI.StreamDeck.streamDeckButtons[1][i * rows + j])
      //         .onChange(
      //             drive.setPoseScored(
      //                 Constants.kPoleLetters[i + Constants.kPoleLetters.length / 2], j));
      //   }
      // }

    }
    // Button to update Setpoints of the elevator based on the Stream Deck nobs
    // TODO: Fix axis input
    if (elevator != null) {
      OI.getButton(OI.StreamDeck.streamDeckButtons[1][31])
          .onTrue(
              elevator
                  .tuneSetpoints(
                      () -> OI.getAxisSupplier(OI.StreamDeck.Nob1).get(),
                      () -> OI.getAxisSupplier(OI.StreamDeck.Nob2).get(),
                      () -> OI.getAxisSupplier(OI.StreamDeck.Nob3).get(),
                      () -> OI.getAxisSupplier(OI.StreamDeck.Nob4).get())
                  .ignoringDisable(true));
    }

    if (Robot.isSimulation() && intake != null && drive != null && mapleSimArenaSubsystem != null) {
      new Trigger(() -> mapleSimArenaSubsystem.getRobotHasCoral())
          .onFalse(Commands.runOnce(() -> intake.removePieceFromIntakeSim()));
    }

    // Temp Keyboard Buttons for sim with no controller
    if (usingKeyboard) {
      // Driving
      if (drive != null) {
        drive.setDefaultCommand(
            DriveCommands.joystickDrive(
                drive,
                OI.getAxisSupplier(OI.Keyboard.AD),
                OI.getAxisSupplier(OI.Keyboard.WS),
                OI.getAxisSupplier(OI.Keyboard.ArrowLR)));
        // OI.getButton(OI.Keyboard.M)
        //     .whileTrue(
        //         DriveCommands.AlignToReef(
        //             drive,
        //             OI.getAxisSupplier(OI.Keyboard.AD),
        //             OI.getAxisSupplier(OI.Keyboard.WS),
        //             drive.getAlignRotation()));
      }

      // Intake
      if (intake != null) {
        OI.getButton(OI.Keyboard.ForwardSlash).whileTrue(intake.floorIntake());
      }

      // Elevator Buttons
      if (elevator != null) {
        OI.getButton(OI.Keyboard.Z).onTrue(elevator.L0());
        OI.getButton(OI.Keyboard.X).onTrue(elevator.L2());
        OI.getButton(OI.Keyboard.C).onTrue(elevator.L3());
        OI.getButton(OI.Keyboard.V).onTrue(elevator.L4());
      }

      // Scorer
      if (coralScorer != null && mapleSimArenaSubsystem != null) {
        OI.getButton(OI.Keyboard.Period).whileTrue(mapleSimArenaSubsystem.scoreCoral());

        // Reset Button
        OI.getButton(OI.Keyboard.Comma).onTrue(Commands.runOnce(() -> resetSimulationField()));
      }

      // POV Drive
      if (drive != null)
        OI.getButton(OI.Keyboard.M)
            .whileTrue(
                DriveCommands.POVDrive(
                    drive,
                    () -> OI.getAxisSupplier(OI.Keyboard.JL).get(),
                    () -> OI.getAxisSupplier(OI.Keyboard.IK).get()));
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if (autoChooser != null) {
      return autoChooser.get();
    } else {
      return Commands.none();
    }
  }

  public Command elv0Command() {
    if (elevator != null) {
      return elevator.L0().andThen(waitForElevator().withTimeout(0.5).andThen(elevator.limitHit()));
    } else {
      return Commands.none();
    }
  }

  public Command waitForElevator() {
    if (elevator != null) {
      return Commands.waitUntil(elevator.elevatorAtCurrentSetpoint());
    } else {
      return Commands.none();
    }
  }

  public Command intakeAutoCommand() {
    if (intake != null) {
      if (Robot.isSimulation()) {
        return intake
            .humanPlayerIntake()
            .until(intake.pivotAtSetpoint(kPivotCoralStationAngle))
            .andThen(() -> intake.addGamePieceToIntakeSim())
            .asProxy();
      } else {
        return intake.humanPlayerIntake().until(intake.intakeHasCoralTrigger()).asProxy();
      }
    } else {
      return Commands.none();
    }
  }

  public Command intakeFloorAutoCommand() {
    if (intake != null) {
      if (Robot.isSimulation() && mapleSimArenaSubsystem != null) {
        return intake
            .floorIntake()
            .onlyWhile(() -> !mapleSimArenaSubsystem.getRobotHasCoral())
            .asProxy();
      } else {
        return intake.floorIntake().until(intake.intakeHasCoralTrigger()).asProxy();
      }
    } else {
      return Commands.none();
    }
  }

  public Command scorerAutoCommand() {
    if (coralScorer != null) {
      if (Robot.isSimulation() && mapleSimArenaSubsystem != null && intake != null) {
        return Commands.runOnce(() -> intake.removePieceFromIntakeSim())
            .andThen(mapleSimArenaSubsystem.scoreCoral())
            .until(() -> !mapleSimArenaSubsystem.getRobotHasCoral())
            .asProxy();
      } else {
        return coralScorer.scoreCommand().until(coralScorer.hasCoralTrigger().negate()).asProxy();
      }
    } else {
      return Commands.none();
    }
  }

  public Command algeaRemoverAutoCommand() {
    if (algeaRemover != null) {
      return algeaRemover
          .removeUpCommand()
          .until(algeaRemover.algeaArmAtSetpoint())
          .andThen(algeaRemover.upCommand())
          .asProxy();
    } else {
      return Commands.none();
    }
  }

  public void givePreLoad() {
    if (intake != null) {
      intake.addGamePieceToIntakeSim();
    }
  }

  public void startSimAuto() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    if (drive != null && driveSimulation != null && intake != null) {
      intake.addGamePieceToIntakeSim();
      driveSimulation.setSimulationWorldPose(drive.getPose());
    }
  }

  public void seedEncoders() {
    if (intake != null) {
      intake.seedEncoder();
    }
    if (algeaRemover != null) {
      algeaRemover.seedEncoder();
    }
    if (climber != null) {
      climber.seedEncoder();
    }
  }

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    if (mapleSimArenaSubsystem != null) {
      mapleSimArenaSubsystem.resetSimField().initialize();
    }
    if (intake != null) {
      intake.resetSim();
    }
    if (driveSimulation != null) {
      driveSimulation.setSimulationWorldPose(driveSimDefaultPose);
    }
  }

  public void displaySimFieldToAdvantageScope() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    if (mapleSimArenaSubsystem != null && elevator != null) {
      mapleSimArenaSubsystem.updateRobotCoralPose(elevator.getElevatorHeight());
    }
  }
}
