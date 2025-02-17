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
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.IntakeConstants.kPivotRetractAngle;
import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;

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
import frc.robot.Constants.IntakeConstants.CoralEnum;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralScorer;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.LocateCoral;
import frc.robot.subsystems.intake.PassToScorer;
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

  private EventLoop testEventLoop = new EventLoop();

  // Change the raw boolean to true to pick keyboard during simulation
  private final boolean usingKeyboard = false && Robot.isSimulation();

  // Subsystems
  private final Climber climber = new Climber();

  private final Drive drive;
  private final Vision vision;
  private final Elevator elevator = new Elevator();
  private final CoralScorer coralScorer = new CoralScorer();
  private static final Sensors sensors = new Sensors();
  private final IntakeSubsystem intake = new IntakeSubsystem(sensors);

  private boolean elevatorOrL1Mode = false;

  private SwerveDriveSimulation driveSimulation;
  private Pose2d driveSimDefualtPose;

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

        break;
      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        driveSimDefualtPose =
            DriverStation.getAlliance().equals(Alliance.Red)
                ? new Pose2d(
                    Meters.of(2), Constants.kFieldWidth.minus(Meters.of(2)), new Rotation2d())
                : new Pose2d(
                    Constants.kFieldLength.minus(Meters.of(2)),
                    Meters.of(2),
                    new Rotation2d(Degrees.of(180)));

        driveSimulation =
            new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(2, 2, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
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
    testTrig(OI.getPOVButton(OI.Driver.DPAD_UP))
        .whileTrue(elevator.setElvPercent(OI.getAxisSupplier(OI.Driver.RightY).get()));
    testTrig(OI.getPOVButton(OI.Driver.DPAD_RIGHT)).whileTrue(intake.intakeCommand());
    testTrig(OI.getPOVButton(OI.Driver.DPAD_LEFT)).whileTrue(intake.outtakeCommand());
    testTrig(OI.getButton(OI.Driver.RBumper)).whileTrue(intake.conveyorEject());
    testTrig(OI.getButton(OI.Driver.LBumper)).whileTrue(intake.conveyorFeed());
    testTrig(OI.getButton(OI.Driver.X)).whileTrue(intake.extendPivotCommand());
    testTrig(OI.getButton(OI.Driver.Y)).whileTrue(intake.retractPivotCommand());
    testTrig(usingKeyboard ? OI.getButton(OI.Keyboard.M) : OI.getTrigger(OI.Driver.RTrigger))
        .whileTrue(climber.runRaw(Volts.of(3)));
    testTrig(usingKeyboard ? OI.getButton(OI.Keyboard.Comma) : OI.getTrigger(OI.Driver.LTrigger))
        .whileTrue(climber.runRaw(Volts.of(-3)));
    testTrig(usingKeyboard ? OI.getButton(OI.Keyboard.Period) : OI.getButton(OI.Driver.Start))
        .onTrue(climber.toggleJeopardy());
  }

  private void configureButtonBindings() {
    // Set the intake rollers to the left and right triggers
    // OI.getPOVButton(OI.Driver.DPAD_UP)
    //     .and(OI.getButton(OI.Driver.RBumper).negate())
    //     .whileTrue(intake.intakeToBirdhouse());
    OI.getPOVButton(OI.Driver.DPAD_UP).whileTrue(coralScorer.scoreCommand());
    // OI.getButton(OI.Driver.RBumper).whileTrue(intake.floorIntake());
    OI.getButton(OI.Driver.RBumper).whileTrue(intake.floorIntake());
    new Trigger(
            () ->
                sensors.getSensorState() != CoralEnum.NO_CORAL
                    && !intake.atSetpoint(kPivotRetractAngle))
        .onTrue(
            new LocateCoral(sensors::getSensorState, intake, () -> elevatorOrL1Mode)
                .andThen(
                    new PassToScorer(
                        intake, () -> elevatorOrL1Mode, coralScorer, sensors::getSensorState)));
    OI.getButton(OI.Driver.LBumper).whileTrue(intake.floorOuttake());
    OI.getPOVButton(OI.Driver.DPAD_DOWN)
        .onTrue(
            Commands.runOnce(
                () -> {
                  elevatorOrL1Mode = !elevatorOrL1Mode;
                  SmartDashboard.putBoolean("Intake/Mode", elevatorOrL1Mode);
                }));
    intake.setDefaultCommand(intake.Idle());

    OI.getTrigger(OI.Operator.RTrigger).onTrue(climber.climb());
    OI.getTrigger(OI.Operator.LTrigger).onTrue(climber.retract());
    OI.getButton(OI.Operator.Start).onTrue(climber.zero());
    OI.getButton(usingKeyboard ? OI.Keyboard.Z : OI.Driver.X).onTrue(elevator.L0());
    OI.getButton(usingKeyboard ? OI.Keyboard.M : OI.Driver.Back).whileTrue(intake.l1ScoreModeB());
    OI.getButton(usingKeyboard ? OI.Keyboard.X : OI.Driver.A).onTrue(elevator.L2());
    OI.getButton(usingKeyboard ? OI.Keyboard.C : OI.Driver.B).onTrue(elevator.L3());
    OI.getButton(usingKeyboard ? OI.Keyboard.V : OI.Driver.Y).onTrue(elevator.L4());
    SmartDashboard.putData(elevator.limitHit());

    // Intake Buttons
    // OI.getTrigger(OI.Driver.RTrigger).whileTrue(intake.floorIntake());
    // OI.getButton(OI.Driver.RBumper).whileTrue(intake.floorOuttake());

    // Handoff Buttons
    OI.getPOVButton(OI.Driver.DPAD_LEFT)
        .whileTrue(intake.conveyerOutCommand().alongWith(coralScorer.reverseCommand()));

    // Scorer Buttons
    OI.getTrigger(usingKeyboard ? OI.Keyboard.ForwardSlash : OI.Driver.LTrigger)
        .whileTrue(coralScorer.scoreCommand());
    OI.getTrigger(usingKeyboard ? OI.Keyboard.ArrowUpDown : OI.Driver.RTrigger)
        .whileTrue(coralScorer.reverseCommand());

    // Reset gyro / odometry, Runnable
    final Runnable resetGyro =
        Constants.currentMode == Constants.Mode.SIM
            ? () -> drive.setPose(driveSimulation.getSimulatedDriveTrainPose())
            : () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()));

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -OI.getAxisSupplier(usingKeyboard ? OI.Keyboard.AD : OI.Driver.LeftY).get(),
            () -> OI.getAxisSupplier(usingKeyboard ? OI.Keyboard.WS : OI.Driver.LeftX).get(),
            () ->
                OI.getAxisSupplier(usingKeyboard ? OI.Keyboard.ArrowLeftRight : OI.Driver.RightX)
                    .get()));
    OI.getButton(usingKeyboard ? OI.Keyboard.ForwardSlash : OI.Driver.Start)
        .onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void seedIntakeEncoder() {
    intake.seedEncoder();
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
  }
}
