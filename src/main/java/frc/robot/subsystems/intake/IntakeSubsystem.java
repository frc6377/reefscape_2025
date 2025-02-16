// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.IntakeConstants.armZero;
import static frc.robot.Constants.IntakeConstants.kConveyorSpeed;
import static frc.robot.Constants.IntakeConstants.kGearing;
import static frc.robot.Constants.IntakeConstants.kIntakeHandoffSpeed;
import static frc.robot.Constants.IntakeConstants.kIntakeSpeed;
import static frc.robot.Constants.IntakeConstants.kLength;
import static frc.robot.Constants.IntakeConstants.kMOI;
import static frc.robot.Constants.IntakeConstants.kMotionMagicAcceleration;
import static frc.robot.Constants.IntakeConstants.kMotionMagicCruiseVelocity;
import static frc.robot.Constants.IntakeConstants.kMotionMagicJerk;
import static frc.robot.Constants.IntakeConstants.kOuttakeSpeed;
import static frc.robot.Constants.IntakeConstants.kPivotExtendAngle;
import static frc.robot.Constants.IntakeConstants.kPivotRetractAngle;
import static frc.robot.Constants.IntakeConstants.kPivotSpeed;
import static frc.robot.Constants.IntakeConstants.kPivotTolerance;
import static frc.robot.Constants.IntakeConstants.kSensorToMechanism;
import static frc.robot.Constants.IntakeConstants.kalgae;
import static frc.robot.Constants.IntakeConstants.kcoralStation;
import static frc.robot.Constants.IntakeConstants.kl1;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
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
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.CoralEnum;
import frc.robot.Robot;
import frc.robot.Sensors;
import org.littletonrobotics.junction.Logger;
import utilities.DebugEntry;

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

  private DebugEntry<Double> pivotOutput;
  private DebugEntry<String> currentCommand;

  private Sensors sensors;

  private boolean elevatorNotL1 = false;

  // Simulation
  private Timer t1 = new Timer();
  private Timer t2 = new Timer();
  private Timer t3 = new Timer();
  private Timer t4 = new Timer();
  private Timer t5 = new Timer();

  private SingleJointedArmSim pivotSim;

  public IntakeSubsystem(Sensors sensors) {
    intakeMotor = new TalonFX(CANIDs.kIntakeMotor);
    pivotMotor = new TalonFX(CANIDs.kPivotMotor);
    conveyorMotor = new TalonFX(CANIDs.kConveyorMotor);
    throughBoreEncoder =
        new DutyCycleEncoder(DIOConstants.kthroughBoreEncoderID, 1, armZero.in(Rotations));
    this.sensors = sensors;

    intakeMotorConfig = new TalonFXConfiguration();
    intakeMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;
    intakeMotor.getConfigurator().apply(intakeMotorConfig);

    conveyorMotorConfig = new TalonFXConfiguration();
    conveyorMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;
    conveyorMotor.getConfigurator().apply(conveyorMotorConfig);

    pivotMotorConfig = new TalonFXConfiguration();
    pivotMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;
    pivotMotorConfig.Slot0 = IntakeConstants.kPivotArmPID.getSlotConfigs();
    pivotMotorConfig.Feedback =
        new FeedbackConfigs()
            .withRotorToSensorRatio(1)
            .withSensorToMechanismRatio(kSensorToMechanism);
    pivotMotorConfig.MotionMagic =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(kMotionMagicCruiseVelocity)
            .withMotionMagicAcceleration(kMotionMagicAcceleration)
            .withMotionMagicJerk(kMotionMagicJerk);

    pivotMotor.getConfigurator().apply(pivotMotorConfig);
    pivotMotor.setControl(new CoastOut()); // Temporary (TODO: Is this still needed?)

    pivotOutput = new DebugEntry<Double>(0.0, "Pivot Output", this);
    currentCommand = new DebugEntry<String>("none", "Pivot Command", this);

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
              kPivotExtendAngle.minus(Degrees.of(90)).in(Radians),
              kPivotRetractAngle.plus(Degrees.of(90)).in(Radians),
              false,
              kPivotRetractAngle.in(Radians));
      pivotArmMech =
          mech.getRoot("Root", 1, 0)
              .append(
                  new MechanismLigament2d("Pivot Mech", 1, 90, 10, new Color8Bit(Color.kPurple)));
      SmartDashboard.putData("Intake/Pivot Arm", mech);
    }
  }

  public void elevatorMode() {}

  public Sensors getSensors() {
    return sensors;
  }

  public boolean atSetpoint(Angle setpoint) {
    return pivotMotor.getPosition().getValue().isNear(setpoint, kPivotTolerance);
  }

  protected void setPivotMotor(double speed) {
    pivotMotor.set(speed);
  }

  protected void setConveyerMotor(double speed) {
    conveyorMotor.set(speed);
  }

  protected void setIntakeMotor(double speed) {
    intakeMotor.set(speed);
  }

  protected void goToPivotPosition(Angle setpoint) {
    pivotMotor.setControl(new MotionMagicVoltage(setpoint));
    pivotSetpoint = setpoint;
  }

  public void seedEncoder() {
    pivotMotor.setPosition(throughBoreEncoder.get());
  }

  public Angle getPivotPosition() {
    return pivotMotor.getPosition().getValue();
  }

  public Trigger pivotAtSetpoint(Angle pivotSetpoint) {
    return new Trigger(() -> atSetpoint(pivotSetpoint));
  }

  public Trigger intakeHasCoralTrigger() {
    return new Trigger(
        () -> sensors.getSensorState() != CoralEnum.NO_CORAL && !atSetpoint(kPivotRetractAngle));
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
        });
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
        });
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

  public Command floorIntake() {
    return startEnd(
        () -> {
          goToPivotPosition(kPivotExtendAngle);
          intakeMotor.set(kIntakeSpeed);
        },
        () -> {});
  }

  public Command floorOuttake() {
    return runEnd(
            () -> goToPivotPosition(kPivotExtendAngle), () -> goToPivotPosition(kPivotRetractAngle))
        .until(pivotAtSetpoint(pivotSetpoint))
        .andThen(() -> intakeMotor.set(kOuttakeSpeed));
  }

  public Command humanPlayerIntake() {
    return startEnd(
        () -> {
          goToPivotPosition(kcoralStation);
          intakeMotor.set(kIntakeSpeed);
        },
        () -> {});
  }

  public Command algaeIntake() {
    return startEnd(
        () -> {
          goToPivotPosition(kalgae);
          intakeMotor.set(-kIntakeSpeed);
        },
        () -> {});
  }

  public Command algaeHold() {
    return startEnd(
        () -> {
          goToPivotPosition(kalgae);
          intakeMotor.set(-kIntakeSpeed);
        },
        () -> {});
  }

  public Command algaeOuttake() {
    return startEnd(
        () -> {
          goToPivotPosition(kalgae);
          intakeMotor.set(kIntakeSpeed);
        },
        () -> {});
  }

  public Command l1ScoreModeA() {
    return startEnd(
        () -> {
          goToPivotPosition(kl1);
          intakeMotor.set(-kIntakeSpeed);
        },
        () -> {});
  }

  public Command l1ScoreModeB() {
    return startEnd(
        () -> {
          goToPivotPosition(kPivotRetractAngle);
          intakeMotor.set(kIntakeSpeed / 5);
          conveyorMotor.set(-kConveyorSpeed);
        },
        () -> {});
  }

  public Command Idle() {
    return startEnd(
        () -> {
          goToPivotPosition(kPivotRetractAngle);
          setIntakeMotor(0);
          setConveyerMotor(0);
        },
        () -> {});
  }

  private boolean checkSimIntake(double expectedSpeed) {
    return Math.signum(expectedSpeed) == Math.signum(intakeMotor.get());
  }

  private boolean checkSimConveyor(double expectedSpeed) {
    return Math.signum(expectedSpeed) == Math.signum(conveyorMotor.get());
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
    SmartDashboard.putNumber(
        "Intake/Absolute Encoder Position (Degrees)",
        Rotations.of(throughBoreEncoder.get()).in(Degrees));
    SmartDashboard.putString("Intake/Intake State", intakeState.toString());
    SmartDashboard.putString("Intake/Coral State", coralState.toString());
    SmartDashboard.putNumber(
        "Intake/Belt Velocity (RPM)",
        RotationsPerSecond.of(conveyorMotor.getVelocity().getValueAsDouble()).in(RPM));

    // Log TOF Sensors
    for (int i = 2; i < 5; i++) {
      Logger.recordOutput(
          "Intake/TOF/Sensor" + i + " Dist (Inches)", sensors.getSensorDist(i).in(Inches));
      Logger.recordOutput("Intake/TOF/Sensor" + i + " bool", sensors.getSensorBool(i));
    }

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

    coralState = sensors.getSensorState();

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

        if (atSetpoint(kcoralStation) && checkSimIntake(kIntakeSpeed)) {
          t2.start();
        } else {
          t2.stop();
        }

        if (t2.hasElapsed(0.5)) {
          intakeState = IntakeState.HP_CORAL_INTAKE;
          t2.stop();
          t2.reset();
        }

        if (atSetpoint(kalgae) && checkSimIntake(-kIntakeSpeed)) {
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
          if (atSetpoint(kcoralStation) // coral station
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
    }
  }
}
