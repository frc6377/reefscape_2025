// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDs;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DIOConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.PWMConstants;
import frc.robot.Robot;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private final DutyCycleEncoder climberFrontEncoder;

  private final DutyCycleEncoder climberBackEncoder;
  private TalonFX climberMotorFront;
  private Orchestra climberOrchestra;
  private TalonFX climberMotorBack;
  private FeedbackConfigs feedbackConfigs;
  private MotorOutputConfigs climberOutputConfigsFront;
  private MotorOutputConfigs climberOutputConfigsBack;
  private TalonFXConfiguration frontConfigs;
  private TalonFXConfiguration backConfigs;
  private Angle climberTargetAngle;
  private Servo frontClimberServo;
  private Servo backClimberServo;

  private boolean isFrontServoEngaged = false;
  private boolean isBackServoEngaged = false;

  private CurrentLimitsConfigs currentLimit = new CurrentLimitsConfigs();

  // for simulation
  private DCMotor simClimberGearbox;
  private SingleJointedArmSim climberSimNormal;
  private SingleJointedArmSim climberSimLifting;
  private Mechanism2d climbMech;
  private MechanismRoot2d climbMechRoot1;
  private MechanismRoot2d climbMechRoot2;
  private MechanismLigament2d climbMechLigament1;
  private MechanismLigament2d climbMechLigament2;
  private MechanismLigament2d climbMechTargetLigament;
  private boolean isClimbingStateSim;

  public Climber() {
    currentLimit.StatorCurrentLimit = 20;
    currentLimit.StatorCurrentLimitEnable = false;
    climberTargetAngle = ClimberConstants.kClimberRetractedSetpoint;
    climberFrontEncoder =
        new DutyCycleEncoder(DIOConstants.kClimberFrontEncoderID, 1, Degrees.of(20).in(Rotations));
    climberBackEncoder =
        new DutyCycleEncoder(DIOConstants.kClimberBackEncoderID, 1, Degrees.of(243).in(Rotations));
    climberMotorFront = new TalonFX(CANIDs.kClimberMotorFront);
    climberMotorBack = new TalonFX(CANIDs.kClimberMotorBack);
    feedbackConfigs = new FeedbackConfigs().withSensorToMechanismRatio(ClimberConstants.kGearRatio);
    frontClimberServo = new Servo(PWMConstants.kFrontClimberServoID);
    backClimberServo = new Servo(PWMConstants.kBackClimberServoID);
    // Boolean to check if the climber is climbing of if it is just idle
    isClimbingStateSim = false;
    // Set the configs
    climberOutputConfigsFront = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);
    climberOutputConfigsBack = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);

    frontConfigs =
        new TalonFXConfiguration()
            .withSlot0(ClimberConstants.kClimberPID0.getSlot0Configs())
            .withSlot1(ClimberConstants.kClimberPID1.getSlot1Configs())
            .withMotorOutput(
                climberOutputConfigsFront.withInverted(ClimberConstants.kClimberFrontInvert))
            .withFeedback(feedbackConfigs);
    backConfigs =
        new TalonFXConfiguration()
            .withSlot0(ClimberConstants.kClimberPID0.getSlot0Configs())
            .withSlot1(ClimberConstants.kClimberPID1.getSlot1Configs())
            .withMotorOutput(
                climberOutputConfigsBack.withInverted(ClimberConstants.kClimberBackInvert))
            .withFeedback(feedbackConfigs.withSensorToMechanismRatio(ClimberConstants.kGearRatio));

    frontConfigs.CurrentLimits = currentLimit;
    backConfigs.CurrentLimits = currentLimit;
    climberMotorFront.getConfigurator().apply(frontConfigs);
    climberMotorBack.getConfigurator().apply(backConfigs);
    climberOrchestra = new Orchestra();
    climberOrchestra.addInstrument(climberMotorFront, 2);
    climberOrchestra.addInstrument(climberMotorBack, 6);
    climberOrchestra.loadMusic("music/jeopardymusic.chrp");
    climberMotorFront.setPosition(ClimberConstants.kClimberAtCageSetpoint.in(Rotations));
    climberMotorBack.setPosition(ClimberConstants.kClimberAtCageSetpoint.in(Rotations));

    Logger.recordOutput("Climber/Front/isFrontServoEngaged", isFrontServoEngaged);
    Logger.recordOutput("Climber/Back/isBackServoEngaged", isBackServoEngaged);

    // For simulation
    // simulates the entire simulation, not just one arm
    if (Robot.isSimulation()) {
      simClimberGearbox = DCMotor.getKrakenX60(ClimberConstants.KClimberMotorsCount);
      climberSimNormal =
          new SingleJointedArmSim(
              simClimberGearbox,
              ClimberConstants.kGearRatio,
              SingleJointedArmSim.estimateMOI(
                  ClimberConstants.kClimberArmLength.in(Meters),
                  ClimberConstants.kClimberMass.in(Kilograms)),
              ClimberConstants.kClimberArmLength.in(Meters),
              ClimberConstants.kClimberArmMinAngle.in(Radians),
              ClimberConstants.kClimberArmMaxAngle.in(Radians),
              true,
              climberTargetAngle.in(Radians));
      climberSimLifting =
          new SingleJointedArmSim(
              simClimberGearbox,
              ClimberConstants.kGearRatio,
              Math.pow(ClimberConstants.kClimberArmLength.in(Meters), 2)
                  * DrivetrainConstants.ROBOT_MASS.in(Kilograms),
              ClimberConstants.kClimberArmLength.in(Meters),
              ClimberConstants.kClimberArmMinAngle.in(Radians),
              ClimberConstants.kClimberArmMaxAngle.in(Radians),
              true,
              ClimberConstants.kClimberRetractedSetpoint.in(Radians));

      climbMech = new Mechanism2d(4, 2);
      climbMechRoot1 = climbMech.getRoot("Climb Mech right", 3, 1);
      climbMechRoot2 = climbMech.getRoot("Climb Mech left", 1, 1);
      climbMechLigament1 =
          climbMechRoot1.append(
              new MechanismLigament2d(
                  "Climb Mech Ligament 1",
                  1,
                  Radians.of(climberSimNormal.getAngleRads()).in(Degrees)));
      climbMechLigament2 =
          climbMechRoot2.append(
              new MechanismLigament2d(
                  "Climb Mech Ligament 2",
                  1,
                  Radians.of(climberSimNormal.getAngleRads()).in(Degrees)));
      climbMechTargetLigament =
          climbMechRoot1.append(
              new MechanismLigament2d(
                  "Climb Mech Target Ligament",
                  0.8,
                  climberTargetAngle.in(Degrees),
                  10,
                  new Color8Bit(255, 255, 0)));
      SmartDashboard.putData("Mech2Ds/Climb Mech", climbMech);
    }

    // Temp until we have real climb code
    Logger.recordOutput("Odometry/Mech Poses/Climber 1 Pose", DrivetrainConstants.kClimber1Pose);
    Logger.recordOutput("Odometry/Mech Poses/Climber 2 Pose", DrivetrainConstants.kClimber2Pose);
  }

  public Command playJeopardy() {
    return runOnce(
        () -> {
          climberOrchestra.play();
        });
  }

  public Command stopJeopardy() {
    return Commands.runOnce(
        () -> {
          climberOrchestra.stop();
        });
  }

  public Command toggleJeopardy() {
    return Commands.runOnce(
        () -> {
          if (climberOrchestra.isPlaying()) {
            climberOrchestra.stop();
          } else {
            climberOrchestra.play();
          }
        });
  }

  public Command zero() {
    return runOnce(
        () -> {
          seedEncoder();
        });
  }

  public void seedEncoder() {
    if (1 - climberFrontEncoder.get() < 0.5) {
      climberMotorFront.setPosition(
          1 - climberFrontEncoder.get() + ClimberConstants.kClimberOffsetAngle.in(Rotations));
    } else {
      climberMotorFront.setPosition(
          -climberFrontEncoder.get() + ClimberConstants.kClimberOffsetAngle.in(Rotations));
    }
    if (climberBackEncoder.get() < 0.5) {
      climberMotorBack.setPosition(
          climberBackEncoder.get() + ClimberConstants.kClimberOffsetAngle.in(Rotations));
    } else {
      climberMotorBack.setPosition(
          climberBackEncoder.get() - 1 + ClimberConstants.kClimberOffsetAngle.in(Rotations));
    }
  }

  public Command runRaw(Voltage voltage) {
    return startEnd(
        () -> {
          climberMotorFront.setControl(new VoltageOut(voltage));
          climberMotorBack.setControl(new VoltageOut(voltage));
        },
        () -> {
          climberMotorFront.setControl(new VoltageOut(Volts.zero()));
          climberMotorBack.setControl(new VoltageOut(Volts.zero()));
        });
  }

  private Command runClimber(Angle position, int slot) {
    return Commands.sequence(
        stopJeopardy(),
        runOnce(
            () -> {
              if (position.lt(climberMotorFront.getPosition().getValue())
                  && (isFrontServoEngaged || isBackServoEngaged)) {
                new Alert("Attempted motor output against servo ratchet", AlertType.kError)
                    .set(true);
              } else {
                climberTargetAngle = position;
                climberMotorFront.setControl(new PositionVoltage(position).withSlot(slot));
                climberMotorBack.setControl(new PositionVoltage(position).withSlot(slot));
                Logger.recordOutput("Climber/Climber Position Setpoint", position.in(Degrees));
              }
            }));
  }

  private BooleanSupplier isClimberAtPosition(Angle position) {
    if (Robot.isReal()) {
      return () ->
          position.isNear(
                  climberMotorFront.getPosition().getValue(),
                  ClimberConstants.kClimberSensorTolerance)
              && position.isNear(
                  climberMotorBack.getPosition().getValue(),
                  ClimberConstants.kClimberSensorTolerance);
    } else {
      return () ->
          position.isNear(
              climberMotorFront.getPosition().getValue(), ClimberConstants.kClimberSensorTolerance);
    }
  }

  public void setCurrentLimit(Current current) {
    climberMotorFront.getConfigurator().apply(currentLimit.withStatorCurrentLimit(current));
    climberMotorBack.getConfigurator().apply(currentLimit.withStatorCurrentLimit(current));
  }

  public Command climb() {
    return Commands.sequence(extendToCage(), engageServo(), extendFully());
  }

  public Command retract() {
    return disengageServo()
        .andThen(runClimber(ClimberConstants.kClimberEmergencyUndoAngle, 0))
        .until(isClimberAtPosition(ClimberConstants.kClimberEmergencyUndoAngle))
        .andThen(Commands.waitSeconds(1))
        .andThen(runClimber(ClimberConstants.kClimberRetractedSetpoint, 0))
        .until(isClimberAtPosition(ClimberConstants.kClimberRetractedSetpoint))
        .andThen(engageServo())
        .andThen(Commands.waitSeconds(1));
  }

  public Command extendToCage() {
    return runClimber(ClimberConstants.kClimberAtCageSetpoint, 0)
        .andThen(Commands.waitUntil(isClimberAtPosition(ClimberConstants.kClimberAtCageSetpoint)));
  }

  public Command extendFully() {
    return runClimber(ClimberConstants.kClimberExtendedSetpoint, 1);
  }

  private void setServoAngle(Servo servo, double angle) {
    servo.setAngle(angle);
  }

  public Command engageServo() {
    return runOnce(
        () -> {
          setServoAngle(frontClimberServo, ClimberConstants.kFrontServoEngageAngle.in(Rotations));
          setServoAngle(
              backClimberServo,
              ClimberConstants.kFrontServoEngageAngle.in(
                  Rotations)); // Change to back servo angle if needed
          isFrontServoEngaged = true;
          isBackServoEngaged = true;
          setCurrentLimit(Amps.of(70));
          Logger.recordOutput("Climber/Front/isFrontServoEngaged", isFrontServoEngaged);
          Logger.recordOutput("Climber/Back/isBackServoEngaged", isBackServoEngaged);
        });
  }

  public Command disengageServo() {
    return runOnce(
        () -> {
          setServoAngle(
              frontClimberServo, ClimberConstants.kFrontServoDisengageAngle.in(Rotations));
          setServoAngle(
              backClimberServo,
              ClimberConstants.kFrontServoDisengageAngle.in(
                  Rotations)); // Change to back servo angle if needed
          isFrontServoEngaged = false;
          isBackServoEngaged = false;
          setCurrentLimit(Amps.of(20));
          Logger.recordOutput("Climber/Front/isFrontServoEngaged", isFrontServoEngaged);
          Logger.recordOutput("Climber/Back/isBackServoEngaged", isBackServoEngaged);
        });
  }

  private SingleJointedArmSim getSimulator() {
    if (isClimbingStateSim) {
      return climberSimLifting;
    } else {
      return climberSimNormal;
    }
  }

  private void toggleClimbingSim() {
    if (isClimbingStateSim) {
      climberSimNormal.setState(
          climberSimLifting.getAngleRads(), climberSimLifting.getVelocityRadPerSec());
      isClimbingStateSim = false;
    } else {
      climberSimLifting.setState(
          climberSimNormal.getAngleRads(), climberSimNormal.getVelocityRadPerSec());
      isClimbingStateSim = true;
    }
    Logger.recordOutput("Climber/Climbing", isClimbingStateSim);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput(
        "Climber/Front/Motor Voltage (Volts)",
        climberMotorFront.getMotorVoltage().getValue().in(Volts));
    Logger.recordOutput(
        "Climber/Front/Motor Position (Degrees)",
        climberMotorFront.getPosition().getValue().in(Degrees));
    Logger.recordOutput(
        "Climber/Front/Absolute Encoder Position (Degrees)",
        Rotations.of(1 - climberFrontEncoder.get()).in(Degrees));
    Logger.recordOutput(
        "Climber/Front/Motor Current (Amps)",
        climberMotorFront.getStatorCurrent().getValue().in(Amps));
    Logger.recordOutput(
        "Climber/Back/Motor Voltage (Volts)",
        climberMotorBack.getMotorVoltage().getValue().in(Volts));
    Logger.recordOutput(
        "Climber/Back/Motor Position (Degrees)",
        climberMotorBack.getPosition().getValue().in(Degrees));
    Logger.recordOutput(
        "Climber/Back/Absolute Encoder Position (Degrees)",
        Rotations.of(climberBackEncoder.get()).in(Degrees));
    Logger.recordOutput(
        "Climber/Back/Motor Current (Amps)",
        climberMotorBack.getStatorCurrent().getValue().in(Amps));
  }

  @Override
  public void simulationPeriodic() {
    climbMechTargetLigament.setAngle(
        climberTargetAngle.minus(ClimberConstants.kClimberOffsetAngle).in(Degrees));

    SingleJointedArmSim simulator = getSimulator();
    simulator.setInputVoltage(climberMotorFront.getMotorVoltage().getValue().in(Volts));
    simulator.update(Robot.defaultPeriodSecs);
    climberMotorFront.setPosition(Radians.of(simulator.getAngleRads()));
    climbMechLigament1.setAngle(
        Radians.of(simulator.getAngleRads())
            .minus(ClimberConstants.kClimberOffsetAngle)
            .in(Degrees));
    climbMechLigament2.setAngle(
        Radians.of(simulator.getAngleRads())
            .minus(ClimberConstants.kClimberOffsetAngle)
            .times(-1)
            .plus(Degrees.of(180))
            .in(Degrees));

    Logger.recordOutput("Climber/Climber Angle", Radians.of(simulator.getAngleRads()).in(Degrees));
    Logger.recordOutput("Climber/Climbing", isClimbingStateSim);
    if (climberMotorFront.getPosition().getValue().gt(ClimberConstants.kClimberAtCageSetpoint)) {
      if (simulator == climberSimNormal) {
        toggleClimbingSim();
      }
    } else {
      if (simulator == climberSimLifting) {
        toggleClimbingSim();
      }
    }
  }
}
