// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.MechPoses.kIntakeStartPose;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CANIDs;
import frc.robot.Constants.DIOConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.MechPoses;
import frc.robot.Robot;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import utilities.DebugEntry;
import utilities.TOFSensorSimple;
import utilities.TOFSensorSimple.TOFType;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private TalonFX intakeMotor;

  private TalonFXSimState simPivotMotor;

  private TalonFX pivotMotor;
  private TalonFX conveyorMotor;
  private Angle pivotSetpoint = kPivotRetractAngle;

  private DutyCycleEncoder throughBoreEncoder;

  // Sensor closest to birdhouse
  private TOFSensorSimple sensor;
  private SimDeviceSim simSensor;
  private SimBoolean simbeam;

  private Mechanism2d mech = new Mechanism2d(2, 2);
  private MechanismLigament2d pivotArmMech;

  private enum IntakeState {
    Idle,
    atSensor,
    coralLoaded
  }

  private IntakeState state = IntakeState.Idle;
  private Timer timer = new Timer();

  private SingleJointedArmSim pivotSim;

  private DebugEntry<Double> pivotOutput;
  private DebugEntry<String> currentCommand;

  private IntakeSimulation intakeSim;
  private Pose3d intakeSimPose = Robot.isSimulation() ? MechPoses.kIntakeStartPose : null;

  public IntakeSubsystem(SwerveDriveSimulation driveSim) {
    intakeMotor = new TalonFX(CANIDs.kIntakeMotor);
    pivotMotor = new TalonFX(CANIDs.kPivotMotor);
    conveyorMotor = new TalonFX(CANIDs.kConveyorMotor);
    sensor = new TOFSensorSimple(CANIDs.kConveyorSensor, Inches.of(1), TOFType.LASER_CAN);
    throughBoreEncoder = new DutyCycleEncoder(DIOConstants.kthroughBoreEncoderID, 1, armZero);

    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = kPivotP;
    slot0Configs.kI = kPivotI;
    slot0Configs.kD = kPivotD;
    slot0Configs.kG = kPivotG;
    slot0Configs.kA = kPivotA;
    slot0Configs.kV = kPivotV;
    slot0Configs.GravityType = kPivotGravityType;

    var feedbackConfigs = new FeedbackConfigs();
    feedbackConfigs.RotorToSensorRatio = 1;
    feedbackConfigs.SensorToMechanismRatio = kSensorToMechanism;

    var pivotMotionMagic =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(kMotionMagicCruiseVelocity)
            .withMotionMagicAcceleration(kMotionMagicAcceleration)
            .withMotionMagicJerk(kMotionMagicJerk);

    pivotMotor.getConfigurator().apply(slot0Configs);
    pivotMotor.getConfigurator().apply(feedbackConfigs);
    pivotMotor.getConfigurator().apply(pivotMotionMagic);
    pivotMotor.setControl(new CoastOut()); // Temporary

    pivotOutput = new DebugEntry<Double>(0.0, "Pivot Output", this);
    currentCommand = new DebugEntry<String>("none", "Pivot Command", this);

    if (Robot.isSimulation()) {
      simPivotMotor = pivotMotor.getSimState();
      simPivotMotor.Orientation =
          ChassisReference
              .CounterClockwise_Positive; // FIXME: Change orientation is it doesn't work
      pivotSim =
          new SingleJointedArmSim(
              DCMotor.getFalcon500(1),
              kGearing,
              kMOI.in(KilogramSquareMeters),
              kLength.in(Meters),
              kPivotExtendAngle.minus(Degrees.of(90)).in(Radians),
              kPivotRetractAngle.plus(Degrees.of(90)).in(Radians),
              false,
              kPivotRetractAngle.in(Radians));
      pivotArmMech =
          mech.getRoot("Root", 1, 0)
              .append(
                  new MechanismLigament2d("Pivot Mech", 1, 90, 10, new Color8Bit(Color.kPurple)));
      SmartDashboard.putData("Intake/Pivot Arm Mech", mech);

      intakeSim =
          IntakeSimulation.OverTheBumperIntake(
              "Coral",
              driveSim,
              IntakeConstants.kIntakeWidth,
              IntakeConstants.kIntakeExtension,
              IntakeSimulation.IntakeSide.FRONT,
              IntakeConstants.kIntakeCapacity);

      simSensor = new SimDeviceSim("TOF", CANIDs.kConveyorSensor);
      simbeam = simSensor.getBoolean("BeamBroken");
    }
  }

  public static IntakeSubsystem create(SwerveDriveSimulation driveSim) {
    return new IntakeSubsystem(driveSim);
  }

  public static IntakeSubsystem create() {
    return new IntakeSubsystem(null);
  }

  private void setPivotMotor(double speed) {
    pivotMotor.set(speed);
  }

  private void setConveyerMotor(double speed) {
    conveyorMotor.set(speed);
  }

  private void setIntakeMotor(double speed) {
    intakeMotor.set(speed);
  }

  private void setPivotPose(Angle setPoint) {
    pivotMotor.setControl(new MotionMagicVoltage(setPoint));
    pivotSetpoint = setPoint;
  }

  public IntakeState getIntakeState() {
    return state;
  }

  public Trigger getBeamBroken() {
    return sensor.beamBroken();
  }

  private boolean atSetpoint() {
    return pivotMotor.getPosition().getValue().isNear(pivotSetpoint, kPivotTolerance);
  }

  public Angle getPivotPosition() {
    return pivotMotor.getPosition().getValue();
  }

  public boolean GetPieceFromIntake() {
    if (intakeSim != null) {
      return intakeSim.obtainGamePieceFromIntake();
    } else {
      return false;
    }
  }

  /** Pivot down */
  public Command extendPivotCommand() {
    return runOnce(() -> setPivotPose(kPivotExtendAngle));
  }

  /** Pivot up */
  public Command retractPivotCommand() {
    return runOnce(() -> setPivotPose(kPivotRetractAngle));
  }

  // Made a command to spin clockwise
  public Command intakeCommand() {
    return startEnd(() -> setIntakeMotor(kIntakeSpeed), () -> setIntakeMotor(0));
  }

  // Made a command to spin counter clockwise
  public Command outtakeCommand() {
    return startEnd(() -> setIntakeMotor(-kIntakeSpeed), () -> setIntakeMotor(0));
  }

  /** Pivots down when user presses button */
  public Command pivotDownCommand() {
    return startEnd(() -> setPivotMotor(-kPivotSpeed), () -> setPivotMotor(0));
  }

  /** Pivots up when user presses button */
  public Command pivotUpCommand() {
    return startEnd(() -> setPivotMotor(kPivotSpeed), () -> setPivotMotor(0));
  }

  /** Pushes game piece from conveyor into birdhouse */
  public Command conveyorFeed() {
    return startEnd(() -> setConveyerMotor(-kConveyorSpeed), () -> setConveyerMotor(0));
  }

  public Command conveyorEject() {
    return startEnd(() -> setConveyerMotor(kConveyorSpeed), () -> setConveyerMotor(0));
  }

  public Command intakePivotCommand() {
    return runEnd(
        () -> {
          setIntakeMotor(kIntakeSpeed);
          setPivotPose(kPivotExtendAngle);
          if (Robot.isSimulation()) intakeSim.startIntake();
        },
        () -> {
          setIntakeMotor(0);
          setPivotPose(kPivotRetractAngle);
          if (Robot.isSimulation()) intakeSim.stopIntake();
        });
  }

  public Command intakeAndConveyorCommandSafe() {
    return runEnd(
        () -> {
          setConveyerMotor(-kConveyorSpeed);
          setIntakeMotor(kIntakeSpeed);
        },
        () -> {
          setConveyerMotor(0);
          setIntakeMotor(0);
        });
  }

  public Command intakeAndConveyorCommandScoreL1() {
    return runEnd(
        () -> {
          setConveyerMotor(kConveyorSpeed);
          setIntakeMotor(kIntakeSpeed);
        },
        () -> {
          setConveyerMotor(0);
          setIntakeMotor(0);
        });
  }

  public Command intakeToBirdhousePhase1() {
    return startEnd(
            () -> {
              extendPivotCommand().initialize();
              intakeCommand().initialize();
              conveyorFeed().initialize();
            },
            () -> {
              extendPivotCommand().end(false);
              intakeCommand().end(false);
              conveyorFeed().end(false);
            })
        .until(sensor.beamBroken());
  }

  public Command intakeToBirdhousePhase2() {
    return retractPivotCommand()
        .andThen(Commands.waitUntil(this::atSetpoint)) // FIXME: Fix debouncing if neccesary
        .andThen(conveyorFeed().until(sensor.beamBroken().negate()));
  }

  public Command intakeToBirdhouse() {
    return intakeToBirdhousePhase1()
        .andThen(intakeToBirdhousePhase2())
        .withName("Intake Phase 1 and 2");
  }

  public Command ejectFromBirdhouse() {
    return conveyorEject();
  }

  public Command seedEncoder() {
    return runOnce(
        () -> {
          pivotMotor.setPosition(throughBoreEncoder.get());
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake/Intake Motor Output", intakeMotor.get());
    SmartDashboard.putNumber("Intake/Pivot Motor Output", pivotMotor.get());
    SmartDashboard.putNumber("Intake/Conveyor Motor Output", conveyorMotor.get());
    SmartDashboard.putNumber("Intake/Pivot Setpoint (Degrees)", pivotSetpoint.in(Degrees));
    SmartDashboard.putNumber(
        "Intake/Pivot Position (Degrees)", pivotMotor.getPosition().getValue().in(Degrees));
    SmartDashboard.putNumber("Intake/Absolute Encoder Position", throughBoreEncoder.get());
    pivotOutput.log(pivotMotor.get());
    if (this.getCurrentCommand() != null) {
      currentCommand.log(this.getCurrentCommand().getName());
    } else {
      currentCommand.log("none");
    }
  }

  public void simulationPeriodic() {
    pivotSim.setInputVoltage(pivotMotor.getMotorVoltage().getValue().in(Volts));
    pivotSim.update(Robot.defaultPeriodSecs);
    simPivotMotor.setRawRotorPosition(Radians.of(pivotSim.getAngleRads()).times(kGearing));
    simPivotMotor.setRotorVelocity(
        RadiansPerSecond.of(pivotSim.getVelocityRadPerSec()).times(kGearing));
    simPivotMotor.setSupplyVoltage(RobotController.getBatteryVoltage());
    pivotArmMech.setAngle(Radians.of(pivotSim.getAngleRads()).in(Degrees));

    switch (state) {
      case Idle:
        if (MathUtil.isNear(
                pivotSim.getAngleRads(), kPivotExtendAngle.in(Radians), kPivotTolerance.in(Radians))
            && Math.abs(intakeMotor.get()) > 0
            && Math.abs(conveyorMotor.get()) > 0) {
          timer.start();
        } else {
          timer.stop();
        }

        if (timer.hasElapsed(2)) {
          simbeam.set(true);
          state = IntakeState.atSensor;
          timer.reset();
        }
        break;
      case atSensor:
        if (Math.abs(conveyorMotor.get()) > 0) {
          if (!timer.isRunning()) {
            timer.start();
          }
        } else {
          if (timer.isRunning()) {
            timer.stop();
          }
        }

        if (timer.hasElapsed(2)) {
          simbeam.set(false);
          state = IntakeState.coralLoaded;
          timer.reset();
        }
        break;
      case coralLoaded:
        state = IntakeState.Idle;
        break;
    }

    intakeSimPose =
        new Pose3d(
            kIntakeStartPose.getTranslation(),
            kIntakeStartPose.getRotation().plus(new Rotation3d(0, -pivotSim.getAngleRads(), 0)));

    Logger.recordOutput("Odometry/Mech Poses/Intake Pose", intakeSimPose);
    Logger.recordOutput("Intake/State", state);
  }
}
