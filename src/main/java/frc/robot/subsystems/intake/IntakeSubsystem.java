// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.SensorIDs.kSensor2ID;
import static frc.robot.Constants.SensorIDs.kSensor3ID;
import static frc.robot.Constants.SensorIDs.kSensor4ID;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
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
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.CoralEnum;
import frc.robot.Robot;
import frc.robot.Sensors;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

@SuppressWarnings("unused")
public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private TalonFX intakeMotor;

  private TalonFX conveyorMotor;
  private TalonFX pivotMotor;
  private TalonFXSimState simPivotMotor;

  private TalonFXConfiguration intakeMotorConfig;
  private TalonFXConfiguration conveyorMotorConfig;
  private TalonFXConfiguration pivotMotorConfig;

  private Angle pivotSetpoint = kPivotRetractAngle;

  private DutyCycleEncoder throughBoreEncoder;

  private Mechanism2d mech = new Mechanism2d(2, 2);
  private MechanismLigament2d pivotArmMech;

  private enum IntakeState {
    FLOOR_INTAKE,
    FLOOR_OUTTAKE,
    HP_CORAL_INTAKE,
    L1_SCORE_MODE_A,
    L1_SCORE_MODE_B,
    L1_SCORE,
    ALGAE_INTAKE,
    ALGAE_HOLD,
    ALGAE_OUTTAKE,
    IDLE,
    LOCATE_CORAL,
    PASS_CORAL_TO_SCORER,
    HOLD_CORAL
  }

  private IntakeState intakeState = IntakeState.IDLE;
  private CoralEnum coralState = CoralEnum.NO_CORAL;

  private Sensors sensors;

  private boolean elevatorNotL1 = false;

  // Simulation
  private Timer t1 = new Timer();
  private Timer t2 = new Timer();
  private Timer t3 = new Timer();
  private Timer t4 = new Timer();
  private Timer t5 = new Timer();

  private SingleJointedArmSim pivotSim;
  private IntakeSimulation intakeSim;

  public IntakeSubsystem(Sensors sensors, SwerveDriveSimulation driveSim) {
    intakeMotor = new TalonFX(CANIDs.kIntakeMotor);
    pivotMotor = new TalonFX(CANIDs.kPivotMotor);
    conveyorMotor = new TalonFX(CANIDs.kConveyorMotor);
    throughBoreEncoder =
        new DutyCycleEncoder(DIOConstants.kthroughBoreEncoderID, 1, armZero.in(Rotations));
    this.sensors = sensors;

    intakeMotorConfig = new TalonFXConfiguration();
    intakeMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;
    intakeMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    intakeMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;
    intakeMotor.getConfigurator().apply(intakeMotorConfig);

    conveyorMotorConfig = new TalonFXConfiguration();
    conveyorMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;
    conveyorMotor.getConfigurator().apply(conveyorMotorConfig);

    pivotMotorConfig = new TalonFXConfiguration();
    pivotMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;
    pivotMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    pivotMotorConfig.Slot0 = IntakeConstants.kPivotArmPID.getSlot0Configs();
    pivotMotorConfig.MotionMagic = IntakeConstants.kPivotArmMM.getMotionMagicConfigs();
    pivotMotorConfig.Feedback =
        new FeedbackConfigs()
            .withRotorToSensorRatio(1)
            .withSensorToMechanismRatio(kSensorToMechanism);

    pivotMotor.getConfigurator().apply(pivotMotorConfig);

    pivotMotor.setPosition(throughBoreEncoder.get());

    if (Robot.isSimulation()) {
      simPivotMotor = pivotMotor.getSimState();
      simPivotMotor.Orientation = ChassisReference.CounterClockwise_Positive;
      pivotSim =
          new SingleJointedArmSim(
              DCMotor.getFalcon500(1),
              kGearing,
              kMOI.in(KilogramSquareMeters),
              kLength.in(Meters),
              kPivotExtendAngle.minus(Degrees.of(150)).in(Radians),
              kPivotRetractAngle.plus(Degrees.of(150)).in(Radians),
              false,
              kPivotRetractAngle.in(Radians));
      pivotArmMech =
          mech.getRoot("Root", 1, 0)
              .append(
                  new MechanismLigament2d("Pivot Mech", 1, 90, 10, new Color8Bit(Color.kPurple)));
      SmartDashboard.putData("Intake/Pivot Arm", mech);

      intakeSim =
          IntakeSimulation.OverTheBumperIntake(
              "Coral",
              driveSim,
              IntakeConstants.kIntakeWidth,
              IntakeConstants.kIntakeExtension,
              IntakeSimulation.IntakeSide.FRONT,
              IntakeConstants.kIntakeCapacity);
    }
  }

  public void elevatorMode() {}

  public boolean GetPieceFromIntake() {
    return intakeSim.obtainGamePieceFromIntake();
  }

  public Sensors getSensors() {
    return sensors;
  }

  public boolean atSetpoint(Angle setpoint) {
    return pivotMotor.getPosition().getValue().isNear(setpoint, kPivotTolerance);
  }

  protected void setConveyerMotor(double speed) {
    DutyCycleOut dutyCycleOut = new DutyCycleOut(speed);
    dutyCycleOut.EnableFOC = true;
    conveyorMotor.setControl(dutyCycleOut);
  }

  protected void setIntakeMotor(double speed) {
    DutyCycleOut dutyCycleOut = new DutyCycleOut(speed);
    dutyCycleOut.EnableFOC = true;
    intakeMotor.setControl(dutyCycleOut);
  }

  protected void goToPivotPosition(Angle setpoint) {
    MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(setpoint);
    motionMagicVoltage.EnableFOC = true;
    pivotMotor.setControl(motionMagicVoltage);
    pivotSetpoint = setpoint;
  }

  public void seedEncoder() {
    if (throughBoreEncoder.get() < 0.5) {
      pivotMotor.setPosition(throughBoreEncoder.get());
    } else {
      pivotMotor.setPosition(throughBoreEncoder.get() - 1);
    }
  }

  public void removePieceFromIntakeSim() {
    intakeSim.obtainGamePieceFromIntake();
  }

  public void addGamePieceToIntakeSim() {
    intakeSim.addGamePieceToIntake();
  }

  public void resetSim() {
    pivotSim.setState(kPivotRetractAngle.in(Radians), RadiansPerSecond.zero().in(RadiansPerSecond));
  }

  public Angle getPivotAngle() {
    if (Robot.isSimulation()) return Radians.of(pivotSim.getAngleRads());
    return pivotMotor.getPosition().getValue();
  }

  public Trigger pivotAtSetpoint(Angle pivotSetpoint) {
    return new Trigger(() -> atSetpoint(pivotSetpoint));
  }

  public Trigger intakeHasUnalignedCoralTrigger() {
    return new Trigger(
        () -> sensors.getSensorState() != CoralEnum.NO_CORAL && !atSetpoint(kPivotRetractAngle));
  }

  public Trigger intakeHasCoralTrigger() {
    if (Robot.isSimulation()) return new Trigger(() -> intakeSim.getGamePiecesAmount() > 0);
    return new Trigger(
        () -> sensors.getSensorState() != CoralEnum.NO_CORAL && atSetpoint(kPivotRetractAngle));
  }

  // Belt Commands
  public Command conveyerInCommand() {
    return runEnd(
            () -> {
              setConveyerMotor(-kConveyorSpeed);
              setIntakeMotor(kIntakeHandoffSpeed);
            },
            () -> {
              setConveyerMotor(0);
              setIntakeMotor(0);
            })
        .withName("conveyerInCommand");
  }

  public Command conveyerOutCommand() {
    return runEnd(
            () -> {
              setConveyerMotor(kConveyorSpeed);
              setIntakeMotor(kIntakeHandoffSpeed);
            },
            () -> {
              setConveyerMotor(0);
              setIntakeMotor(0);
            })
        .withName("conveyerOutCommand");
  }

  public Command movePivot(Angle angle) {
    return startEnd(() -> goToPivotPosition(angle), () -> {});
  }

  public Command floorIntake() {
    return startEnd(
            () -> {
              goToPivotPosition(kPivotExtendAngle);
              setIntakeMotor(kIntakeSpeed);
              if (Robot.isSimulation()) intakeSim.startIntake();
            },
            () -> {
              goToPivotPosition(kPivotRetractAngle);
            })
        .withName("floorIntake");
  }

  public Command floorOuttake() {
    return startEnd(() -> goToPivotPosition(kPivotExtendAngle), () -> {})
        .until(pivotAtSetpoint(kPivotExtendAngle))
        .andThen(startEnd(() -> intakeMotor.set(kOuttakeSpeed), () -> {}))
        .withName("floorOuttake");
  }

  public Command humanPlayerIntake() {
    return startEnd(
            () -> {
              goToPivotPosition(kPivotCoralStationAngle);
              setIntakeMotor(kIntakeSpeed);
              if (Robot.isSimulation()) intakeSim.startIntake();
            },
            () -> {})
        .withName("humanPlayerIntake");
  }

  public Command algaeIntake() {
    return startEnd(
            () -> {
              goToPivotPosition(kPivotAlgaeIntakeAngle);
              setIntakeMotor(-kIntakeSpeed);
            },
            () -> {})
        .withName("algaeIntake");
  }

  public Command algaeHold() {
    return startEnd(
            () -> {
              goToPivotPosition(kPivotAlgaeIntakeAngle);
              intakeMotor.setControl(new TorqueCurrentFOC(kHoldPower));
            },
            () -> {})
        .withName("algeaHold");
  }

  public Command algaeOuttake() {
    return startEnd(
            () -> {
              goToPivotPosition(kPivotAlgaeIntakeAngle);
              setIntakeMotor(kIntakeSpeed);
            },
            () -> {})
        .withName("algaeOuttake");
  }

  public Command l1ScoreModeA() {
    return startEnd(() -> goToPivotPosition(kPivotL1Score), () -> {})
        .until(pivotAtSetpoint(kPivotL1Score))
        .andThen(Commands.startEnd(() -> setIntakeMotor(kOuttakeSpeed), () -> {}))
        .withName("L1ScoreModeA");
  }

  public Command l1ScoreModeB() {
    return startEnd(
            () -> {
              goToPivotPosition(kPivotRetractAngle);
              setIntakeMotor(kHoldSpeed);
              setConveyerMotor(-kConveyorSpeed);
            },
            () -> {})
        .withName("L1ScoreModeB");
  }

  public Command Idle() {
    return startEnd(
            () -> {
              goToPivotPosition(kPivotRetractAngle);
              setIntakeMotor(0);
              setConveyerMotor(0);
              if (Robot.isSimulation()) intakeSim.stopIntake();
            },
            () -> {})
        .withName("IDLE");
  }

  private boolean checkSimIntake(double expectedSpeed) {
    return Math.signum(expectedSpeed) == Math.signum(intakeMotor.get());
  }

  private boolean checkSimConveyor(double expectedSpeed) {
    return Math.signum(expectedSpeed) == Math.signum(conveyorMotor.get());
  }

  @Override
  public void periodic() {
    // Intake Rollers
    Logger.recordOutput("Intake/Rollers/Motor Output", intakeMotor.get());
    Logger.recordOutput(
        "Intake/Rollers/Motor Voltage (Volts)", intakeMotor.getMotorVoltage().getValue().in(Volts));

    // Convayor
    Logger.recordOutput("Intake/Conveyor/Motor Output", conveyorMotor.get());
    Logger.recordOutput(
        "Intake/Conveyor/Motor Voltage (Volts)",
        conveyorMotor.getMotorVoltage().getValue().in(Volts));
    Logger.recordOutput(
        "Intake/Conveyor/Velocity (RPS)",
        conveyorMotor.getVelocity().getValue().in(RotationsPerSecond));

    // Pivot
    Logger.recordOutput("Intake/Pivot/Motor Output", pivotMotor.get());
    Logger.recordOutput("Intake/Pivot/Setpoint (Degrees)", pivotSetpoint.in(Degrees));
    Logger.recordOutput(
        "Intake/Pivot/Position (Degrees)", pivotMotor.getPosition().getValue().in(Degrees));
    Logger.recordOutput(
        "Intake/Pivot/Absolute Encoder (Degrees)",
        Rotations.of(throughBoreEncoder.get()).in(Degrees));
    Logger.recordOutput("Intake/Pivot/At Setpoint", atSetpoint(pivotSetpoint));

    // States
    Logger.recordOutput("Intake/States/Intake State", intakeState.toString());

    Logger.recordOutput("Intake/Intake Has Coral Trigger", intakeHasCoralTrigger());
    Logger.recordOutput(
        "Intake/Intake Has Unaligned Coral Trigger", intakeHasUnalignedCoralTrigger());

    // Pose 3D of Intake
    Logger.recordOutput(
        "Odometry/Mech Poses/Intake Pose",
        new Pose3d(
            DrivetrainConstants.kIntakeStartPose.getTranslation(),
            new Rotation3d(
                0,
                getPivotAngle()
                    .times(-1)
                    .plus(DrivetrainConstants.kIntakeStartPose.getRotation().getMeasureY())
                    .in(Radians),
                0)));

    // Log TOF Sensors
    for (int i : new int[] {kSensor2ID, kSensor3ID, kSensor4ID}) {
      Logger.recordOutput(
          "Intake/TOFSensors/" + i + " Dist (Inches)", sensors.getSensorDist(i).in(Inches));
      Logger.recordOutput("Intake/TOFSensors/" + i + " bool", sensors.getSensorBool(i));
    }

    String currentCommand =
        this.getCurrentCommand() != null ? this.getCurrentCommand().getName() : "None";
    Logger.recordOutput("Intake/Current Command", currentCommand);
  }

  public void simulationPeriodic() {
    pivotSim.setInputVoltage(pivotMotor.getMotorVoltage().getValue().in(Volts));
    pivotSim.update(Robot.defaultPeriodSecs);
    simPivotMotor.setRawRotorPosition(Radians.of(pivotSim.getAngleRads()).times(kGearing));
    simPivotMotor.setRotorVelocity(
        RadiansPerSecond.of(pivotSim.getVelocityRadPerSec()).times(kGearing));
    simPivotMotor.setSupplyVoltage(RobotController.getBatteryVoltage());
    pivotArmMech.setAngle(Radians.of(pivotSim.getAngleRads()).in(Degrees));

    coralState = sensors.getSensorState();

    Logger.recordOutput(
        "Intake/Pivot/Sim/Pivot Sim Angle", Radians.of(pivotSim.getAngleRads()).in(Degrees));

    if (IntakeConstants.kEnableStateMachineSim) {
      switch (intakeState) {
        case IDLE:
          sensors.setSimState(CoralEnum.NO_CORAL);

          if (atSetpoint(kPivotExtendAngle) && checkSimIntake(kIntakeSpeed)) {
            t1.start();
          } else {
            t1.stop();
          }

          if (t1.hasElapsed(0.5)) {
            intakeState = IntakeState.FLOOR_INTAKE;
            t1.stop();
            t1.reset();
          }

          if (atSetpoint(kPivotCoralStationAngle) && checkSimIntake(kIntakeSpeed)) {
            t2.start();
          } else {
            t2.stop();
          }

          if (t2.hasElapsed(0.5)) {
            intakeState = IntakeState.HP_CORAL_INTAKE;
            t2.stop();
            t2.reset();
          }

          if (atSetpoint(kPivotAlgaeIntakeAngle) && checkSimIntake(-kIntakeSpeed)) {
            t3.start();
          } else {
            t3.stop();
          }

          if (t3.hasElapsed(0.5)) {
            intakeState = IntakeState.ALGAE_INTAKE;
            t3.stop();
            t3.reset();
          }
          break;
        case FLOOR_INTAKE:
          if (Math.random() <= 0.33) {
            sensors.setSimState(CoralEnum.CORAL_TOO_CLOSE);
          } else if (Math.random() >= 0.33 && Math.random() <= 0.66) {
            sensors.setSimState(CoralEnum.CORAL_TOO_FAR);
          } else {
            sensors.setSimState(CoralEnum.CORAL_ALIGNED);
          }
          intakeState = IntakeState.LOCATE_CORAL;
          break;
        case FLOOR_OUTTAKE:
          break;
        case HP_CORAL_INTAKE:
          if (Math.random() <= 0.33) {
            sensors.setSimState(CoralEnum.CORAL_TOO_CLOSE);
          } else if (Math.random() >= 0.33 && Math.random() <= 0.66) {
            sensors.setSimState(CoralEnum.CORAL_TOO_FAR);
          } else {
            sensors.setSimState(CoralEnum.CORAL_ALIGNED);
          }
          intakeState = IntakeState.LOCATE_CORAL;
          break;
        case ALGAE_INTAKE:
          Commands.waitSeconds(0.5);
          intakeState = IntakeState.ALGAE_HOLD;
          break;
        case ALGAE_HOLD:
          Commands.waitSeconds(0.5);
          intakeState = IntakeState.ALGAE_OUTTAKE;
          break;
        case ALGAE_OUTTAKE:
          Commands.waitSeconds(0.5);
          if (intakeMotor.get() > 0) {
            intakeState = IntakeState.IDLE;
          }
          break;
        case LOCATE_CORAL:
          if (coralState == CoralEnum.CORAL_TOO_CLOSE) {
            if (atSetpoint(kPivotCoralStationAngle) // coral station
                && checkSimIntake(kIntakeSpeed)
                && checkSimConveyor(kConveyorSpeed)) {
              t4.start();
            }
          } else if (coralState == CoralEnum.CORAL_TOO_FAR) {
            if (atSetpoint(kPivotRetractAngle)
                && checkSimIntake(kIntakeSpeed)
                && checkSimConveyor(-kConveyorSpeed)) {
              t4.start();
            }
          } else if (coralState == CoralEnum.CORAL_ALIGNED) {
            t4.start();
          } else if (coralState == CoralEnum.NO_CORAL) {
            t4.stop();
            System.out.println("Compbot is haunted");
          }

          if (t4.hasElapsed(1)) {
            sensors.setSimState(CoralEnum.CORAL_ALIGNED);
            intakeState = IntakeState.PASS_CORAL_TO_SCORER;
            t4.stop();
            t4.reset();
          }

          if (atSetpoint(kPivotRetractAngle)
              && checkSimIntake(0)
              && checkSimConveyor(-kConveyorSpeed)) {
            sensors.setSimState(CoralEnum.CORAL_ALIGNED);
            intakeState = IntakeState.HOLD_CORAL;
          }
          break;
        case HOLD_CORAL:
          intakeState = IntakeState.PASS_CORAL_TO_SCORER;
          break;
        case PASS_CORAL_TO_SCORER:
          if (elevatorNotL1) {
            intakeState =
                IntakeState.L1_SCORE; // FIXME: Change to elevator score state if we have one
          } else {
            intakeState = IntakeState.L1_SCORE;
          }
          break;
        case L1_SCORE:
          if (atSetpoint(kPivotRetractAngle)
              && checkSimIntake(kIntakeSpeed)
              && checkSimConveyor(-kConveyorSpeed)) {
            t5.start();
          } else {
            t5.stop();
          }

          if (t5.hasElapsed(0.5)) {
            intakeState = IntakeState.IDLE;
            sensors.setSimState(CoralEnum.NO_CORAL);
            t5.stop();
            t5.reset();
          }

          break;
        case L1_SCORE_MODE_A:
          break;
        case L1_SCORE_MODE_B:
          break;
      }
    }
  }
}
