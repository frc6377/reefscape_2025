// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.IntakeConstants.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.Idle;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
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
import frc.robot.Constants.CtreCanID;
import frc.robot.Constants.RevCanID;
import frc.robot.Robot;
import utilities.DebugEntry;
import utilities.HowdyPID;
import utilities.TOFSensorSimple;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private SparkMax intakeMotor;

  private TalonFX pivotMotor;
  private SparkMax conveyorMotor;
  private HowdyPID pivotPID;
  private PIDController pid;
  private Angle pivotSetpoint = kPivotRetractAngle;

  private TOFSensorSimple sensor;
  private SimDeviceSim simSensor;
  private SimBoolean simbeam;
  private Mechanism2d mech = new Mechanism2d(2, 2);
  private ComplexWidget widget;
  private MechanismLigament2d pivotArmMech;

  enum intakeState {
    Idle,
    atSensor,
    coralLoaded
  }

  intakeState state = intakeState.Idle;
  Timer t = new Timer();

  private SingleJointedArmSim pivotSim;

  private DebugEntry<Double> pivotOutput;
  private DebugEntry<String> currentCommand;

  public IntakeSubsystem() {
    intakeMotor = new SparkMax(RevCanID.kIntakeMotor, MotorType.kBrushless);
    pivotMotor = new TalonFX(CtreCanID.kPivotMotor);
    conveyorMotor = new SparkMax(RevCanID.kConveyorMotor, MotorType.kBrushless);
    pivotPID = new HowdyPID(kPivotP, kPivotI, kPivotD);
    sensor = new TOFSensorSimple(RevCanID.kConveyorSensor, Inches.of(1));
    pid = pivotPID.getPIDController();
    pid.setTolerance(kPivotTolerance.in(Rotations));

    // pivotMotor.configFactoryDefault();
    var slot0Configs = new Slot0Configs();
    slot0Configs.kI = kPivotI;
    slot0Configs.kP = kPivotP;
    slot0Configs.kD = kPivotD;
    pivotMotor.getConfigurator().apply(slot0Configs);

    // pivotMotor.configMotionCruiseVelocity();
    // pivotMotor.configMotionAcceleration();

    pivotOutput = new DebugEntry<Double>(0.0, "Pivot Output", this);
    currentCommand = new DebugEntry<String>("none", "Pivot Command", this);

    /**
     * Once the gains are configured, the Position closed loop control request can be sent to the
     * TalonFX. The control request object has an optional feedforward term that can be used to add
     * an arbitrary value to the output, which can be useful to account for the effects of gravity
     * or friction.
     *
     * <p>// create a position closed-loop request, voltage output, slot 0 configs final
     * PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
     *
     * <p>// set position to 10 rotations m_talonFX.setControl(m_request.withPosition(10));
     */
    if (Robot.isSimulation()) {
      pivotSim =
          new SingleJointedArmSim(
              DCMotor.getFalcon500(1),
              kGearing,
              kMOI.in(KilogramSquareMeters),
              Feet.of(1).in(Meters),
              kPivotRetractAngle.minus(Degrees.of(30)).in(Radians),
              kPivotExtendAngle.plus(Degrees.of(30)).in(Radians),
              true,
              0);
      pivotArmMech =
          mech.getRoot("Root", 1, 0)
              .append(
                  new MechanismLigament2d("Pivot Mech", 1, 90, 10, new Color8Bit(Color.kPurple)));
      if (widget == null) {
        widget = Shuffleboard.getTab(getName()).add("Pivot Arm", mech);
      }
      simSensor = new SimDeviceSim("TOF", RevCanID.kConveyorSensor);
      simbeam = simSensor.getBoolean("BeamBroken");
    }
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

  public Angle getPivotPosition() {
    return pivotMotor.getPosition().getValue();
  }

  /** Pivot down */
  public Command extendPivotCommand() {
    return runOnce(() -> pivotMotor.setControl(new PositionVoltage(kPivotExtendAngle)));
  }

  /** Pivot up */
  public Command retractPivotCommand() {
    return runOnce(() -> pivotMotor.setControl(new PositionVoltage(kPivotRetractAngle)));
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
        .andThen(Commands.waitUntil(pid::atSetpoint)) // FIXME: Fix debouncing if neccesary
        .andThen(conveyorFeed().until(sensor.beamBroken().negate()));
  }

  public Command intakeToBirdhouse() {
    return intakeToBirdhousePhase1()
        .andThen(intakeToBirdhousePhase2())
        .withName("Intake Phase 1 and 2");
  }

  public Command ejectFromBirdhouse() {
    return conveyorEject()
        .until(sensor.beamBroken().negate())
        .andThen(extendPivotCommand())
        .andThen(Commands.waitSeconds(0.5))
        .andThen(outtakeCommand());
  }

  public double calculatePivotPID() {
    return pid.calculate(getPivotPosition().in(Rotations), pivotSetpoint.in(Rotations));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setPivotMotor(calculatePivotPID());
    SmartDashboard.putNumber("Intake Motor Output", intakeMotor.get());
    SmartDashboard.putNumber("Pivot Motor Output", pivotMotor.get());
    SmartDashboard.putNumber("Conveyor Motor Output", conveyorMotor.get());
    SmartDashboard.putNumber("Pivot Setpoint", pivotSetpoint.in(Rotations));
    SmartDashboard.putNumber(
        "Pivot Position in Degrees", pivotMotor.getPosition().getValue().in(Degrees));
    pivotOutput.log(pivotMotor.get());
    if (this.getCurrentCommand() != null) {
      currentCommand.log(this.getCurrentCommand().getName());
    } else {
      currentCommand.log("none");
    }
  }

  public void simulationPeriodic() {

    pivotSim.setInput(pivotMotor.getMotorVoltage().getValue().in(Volts));
    pivotSim.update(0.020);
    pivotMotor.setPosition(Radians.of(pivotSim.getAngleRads()));
    pivotArmMech.setAngle(Radians.of(pivotSim.getAngleRads()).plus(Degrees.of(90)).in(Degrees));

    switch (state) {
      case Idle:
        if (pivotSetpoint.equals(kPivotExtendAngle)
            && Math.abs(intakeMotor.get()) > 0
            && Math.abs(conveyorMotor.get()) > 0) {
          if (!t.isRunning()) {
            t.start();
          }
        } else {
          if (t.isRunning()) {
            t.stop();
          }
        }

        if (t.hasElapsed(2)) {
          simbeam.set(true);
          state = intakeState.atSensor;
          t.reset();
        }
        break;
      case atSensor:
        if (pivotSetpoint.equals(kPivotRetractAngle) && Math.abs(conveyorMotor.get()) > 0) {
          if (!t.isRunning()) {
            t.start();
          }
        } else {
          if (t.isRunning()) {
            t.stop();
          }
        }

        if (t.hasElapsed(2)) {
          simbeam.set(false);
          state = intakeState.coralLoaded;
          t.reset();
        }
        break;
      case coralLoaded:
        state = intakeState.Idle;
        break;
    }
  }
}
