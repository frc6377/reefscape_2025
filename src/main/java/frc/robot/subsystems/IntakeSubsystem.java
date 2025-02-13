// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.IntakeConstants.*;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;
import utilities.DebugEntry;
import utilities.TOFSensorSimple;
import utilities.TOFSensorSimple.TOFType;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private TalonFX intakeMotor;

  private TalonFXConfiguration intakeConfig;

  private TalonFX conveyorMotor;
  private TalonFXConfiguration conveyorConfig;

  private TalonFX pivotMotor;
  private TalonFXSimState simPivotMotor;
  private Slot0Configs pivtoSlotConfigs;
  private TalonFXConfiguration pivotConfig;
  private FeedbackConfigs pivotFeedbackConfigs;
  private MotionMagicConfigs pivotMotionMagicConfigs;
  private Angle pivotSetpoint = kPivotRetractAngle;
  private DutyCycleEncoder throughBoreEncoder;

  // Sensor closest to birdhouse
  private TOFSensorSimple sensor;
  private SimDeviceSim simSensor;
  private SimBoolean simbeam;

  private Mechanism2d mech = new Mechanism2d(2, 2);
  private ComplexWidget widget;
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

  public IntakeSubsystem() {
    // intakeMotor
    intakeMotor = new TalonFX(CANIDs.kIntakeMotor);
    intakeConfig = new TalonFXConfiguration();
    intakeConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;
    intakeMotor.getConfigurator().apply(intakeConfig);

    conveyorMotor = new TalonFX(CANIDs.kConveyorMotor);
    conveyorConfig = new TalonFXConfiguration();
    conveyorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;
    conveyorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    conveyorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;
    conveyorMotor.getConfigurator().apply(conveyorConfig);
    conveyorMotor.getConfigurator().apply(new Slot0Configs().withKP(0.02));

    pivotMotor = new TalonFX(CANIDs.kPivotMotor);

    pivtoSlotConfigs =
        new Slot0Configs()
            .withKP(kPivotP)
            .withKI(kPivotI)
            .withKD(kPivotD)
            .withKG(kPivotG)
            .withKA(kPivotA)
            .withKV(kPivotV)
            .withGravityType(kPivotGravityType);

    pivotFeedbackConfigs = new FeedbackConfigs();
    pivotFeedbackConfigs.RotorToSensorRatio = 1;
    pivotFeedbackConfigs.SensorToMechanismRatio = kSensorToMechanism;

    pivotMotionMagicConfigs =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(kMotionMagicCruiseVelocity)
            .withMotionMagicAcceleration(kMotionMagicAcceleration)
            .withMotionMagicJerk(kMotionMagicJerk);

    pivotConfig = new TalonFXConfiguration();
    pivotConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;

    pivotMotor.getConfigurator().apply(pivotConfig);
    pivotMotor.getConfigurator().apply(pivtoSlotConfigs);
    pivotMotor.getConfigurator().apply(pivotFeedbackConfigs);
    pivotMotor.getConfigurator().apply(pivotMotionMagicConfigs);
    pivotMotor.setControl(new CoastOut()); // Temporary

    pivotOutput = new DebugEntry<Double>(0.0, "Pivot Output", this);
    currentCommand = new DebugEntry<String>("none", "Pivot Command", this);

    // pivotMotor.setPosition(throughBoreEncoder.get());

    sensor = new TOFSensorSimple(CANIDs.kIntakeTOFID2, Inches.of(1.5), TOFType.LASER_CAN);
    throughBoreEncoder = new DutyCycleEncoder(DIOConstants.kthroughBoreEncoderID, 1, armZero);

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
      simSensor = new SimDeviceSim("TOF", CANIDs.kIntakeTOFID2);
      simbeam = simSensor.getBoolean("BeamBroken");
    }
  }

  public Trigger getBeamBroken() {
    return sensor.beamBroken();
  }

  private boolean atSetpoint() {
    return pivotMotor.getPosition().getValue().isNear(pivotSetpoint, kPivotTolerance);
  }

  private void setPivotMotor(double speed) {
    pivotMotor.set(speed);
  }

  private void setConveyerMotor(double speed) {
    conveyorMotor.set(speed);
  }

  // private void setConveyerTorque(AngularVelocity input) {
  //   conveyorMotor.setControl(new VelocityDutyCycle(input));
  // }

  private void setIntakeMotor(double speed) {
    intakeMotor.set(speed);
  }

  private void setPivotAngle(Angle setPoint) {
    pivotMotor.setControl(new MotionMagicVoltage(setPoint));
    pivotSetpoint = setPoint;
  }

  public void seedEncoder() {
    pivotMotor.setPosition(throughBoreEncoder.get());
  }

  public Angle getPivotPosition() {
    return pivotMotor.getPosition().getValue();
  }

  // Pivot and Intake
  public Command intakePivotIntakeCommand() {
    return startEnd(
        () -> {
          setIntakeMotor(kIntakeSpeed);
          setPivotAngle(kPivotExtendAngle);
        },
        () -> {
          setPivotAngle(kPivotRetractAngle);
          setIntakeMotor(-kOuttakeSpeed);
        });
  }

  public Command intakePivotOutakeCommand() {
    return runEnd(
        () -> {
          setIntakeMotor(kOuttakeSpeed);
        },
        () -> {
          setIntakeMotor(0);
        });
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
          setConveyerMotor(kConveyorSpeed + 0.55);
          setIntakeMotor(kIntakeHandoffSpeed);
        },
        () -> {
          setConveyerMotor(0);
          setIntakeMotor(0);
        });
  }

  public Command setL1PivotPose() {
    return runOnce(
        () -> {
          setPivotAngle(kPivotOuttakePose);
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

    Logger.recordOutput(
        "TOFSensors/Intake Sensor Distance (Inches)", sensor.getDistance().in(Inches));
    Logger.recordOutput("TOFSensors/Intake Sensor Triggered", sensor.isBeamBroke());
    Logger.recordOutput(
        "Intake/Belt Velocity",
        RotationsPerSecond.of(conveyorMotor.getVelocity().getValueAsDouble()).in(RPM));
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
  }
}
