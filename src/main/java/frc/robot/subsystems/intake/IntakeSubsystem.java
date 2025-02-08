// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.IntakeConstants.*;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
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
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDs;
import frc.robot.Constants.DIOConstants;
import frc.robot.Robot;
import utilities.DebugEntry;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private TalonFX intakeMotor;

  private TalonFXSimState simPivotMotor;

  private TalonFX pivotMotor;
  private TalonFX conveyorMotor;
  private Angle pivotSetpoint = kPivotRetractAngle;

  private DutyCycleEncoder throughBoreEncoder;

  private Mechanism2d mech = new Mechanism2d(2, 2);
  private ComplexWidget widget;
  private MechanismLigament2d pivotArmMech;

  private enum IntakeState {
    FLOOR_INTAKE,
    FLOOR_OUTTAKE,
    HP_CORAL_INTAKE,
    L1_SCORE_MODE_A,
    L1_SCORE_MODE_B,
    ALGAE_INTAKE,
    ALGAE_HOLD,
    ALGAE_OUT,
    IDLE,
    LOCATE_CORAL,
    PASS_CORAL_TO_SCORER,
    HOLD_CORAL
  }

  private IntakeState state = IntakeState.IDLE;
  private Timer t = new Timer();

  private SingleJointedArmSim pivotSim;

  private DebugEntry<Double> pivotOutput;
  private DebugEntry<String> currentCommand;

  public IntakeSubsystem() {
    intakeMotor = new TalonFX(CANIDs.kIntakeMotor);
    pivotMotor = new TalonFX(CANIDs.kPivotMotor);
    conveyorMotor = new TalonFX(CANIDs.kConveyorMotor);
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

    pivotMotor.setPosition(throughBoreEncoder.get());

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
      if (widget == null) {
        widget = Shuffleboard.getTab(getName()).add("Pivot Arm", mech);
      }
    }
  }

  public void elevatorMode() {}

  private boolean atSetpoint() {
    return pivotMotor.getPosition().getValue().isNear(pivotSetpoint, kPivotTolerance);
  }

  protected boolean atSetpoint(Angle setpoint) {
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

  /** Pivot down */
  public Command extendPivotCommand() {
    return runOnce(
        () -> {
          pivotMotor.setControl(new MotionMagicVoltage(kPivotExtendAngle));
          pivotSetpoint = kPivotExtendAngle;
        });
  }

  /** Pivot up */
  public Command retractPivotCommand() {
    return runOnce(
        () -> {
          pivotMotor.setControl(new MotionMagicVoltage(kPivotRetractAngle));
          pivotSetpoint = kPivotRetractAngle;
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
        () -> {}); // FIXME: runENd might need to be a start end
  }

  public Command floorOuttake() {
    return startEnd(
        () -> {
          goToPivotPosition(kPivotExtendAngle);
          intakeMotor.set(-kIntakeSpeed);
        },
        () -> {});
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
          intakeMotor.set(kIntakeSpeed);
        },
        () -> {});
  }

  public Command l1ScoreModeB() {
    return startEnd(
        () -> {
          goToPivotPosition(kPivotRetractAngle);
          intakeMotor.set(kIntakeSpeed / 5);
          conveyorMotor.set(kConveyorSpeed);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake/Intake Motor Output", intakeMotor.get());
    SmartDashboard.putNumber("Intake/Pivot Motor Output", pivotMotor.get());
    SmartDashboard.putNumber("Intake/Conveyor Motor Output", conveyorMotor.get());
    SmartDashboard.putNumber("Intake/Pivot Setpoint", pivotSetpoint.in(Degrees));
    SmartDashboard.putNumber(
        "Intake/Pivot Position (Degrees)", pivotMotor.getPosition().getValue().in(Degrees));
    SmartDashboard.putNumber(
        "Intake/Absolute Encoder Position (Rotation)",
        Rotations.of(throughBoreEncoder.get()).in(Degrees));
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
      case IDLE:
        if (atSetpoint(kPivotExtendAngle) && intakeMotor.get() > 0) {
          t.start();
          if (t.hasElapsed(0.5)) {
            state = IntakeState.FLOOR_INTAKE;
          }
        } else {
          t.stop();
        }

        if (atSetpoint(kcoralStation) && intakeMotor.get() > 0) {
          t.start();
          if (t.hasElapsed(0.5)) {
            state = IntakeState.FLOOR_INTAKE;
          }
        } else {
          t.stop();
        }
    }

    /**
     * switch (state) { case IDLE: if (MathUtil.isNear( pivotSim.getAngleRads() *
     * kPivotExtendAngle.in(Radians), kPivotTolerance.in(Radians)) && Math.abs(intakeMotor.get()) >
     * 0 && Math.abs(conveyorMotor.get()) > 0) { timer.start(); } else { timer.stop(); }
     *
     * <p>if (timer.hasElapsed(2)) { simbeam.set(true); state = IntakeState.IDLE; timer.reset(); }
     * break; case FLOOR_INTAKE: if (Math.abs(conveyorMotor.get()) > 0) { if (!timer.isRunning()) {
     * timer.start(); } } else { if (timer.isRunning()) { timer.stop(); } }
     *
     * <p>if (timer.hasElapsed(2)) { simbeam.set(false); state = IntakeState.FLOOR_OUTTAKE;
     * timer.reset(); } break; case ALGAE_HOLD: state = IntakeState.ALGAE_HOLD; break;
     */
  }
}
