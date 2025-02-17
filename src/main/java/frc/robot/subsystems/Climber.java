// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
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
import frc.robot.Robot;
import java.util.function.BooleanSupplier;

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
  private Slot0Configs climberConfigsToClimber;
  private Slot1Configs climberConfigsAtClimber;

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
    climberTargetAngle = ClimberConstants.kClimberRetractedSetpoint;
    climberFrontEncoder =
        new DutyCycleEncoder(DIOConstants.kClimberFrontEncoderID, 1, Degrees.of(20).in(Rotations));
    climberBackEncoder =
        new DutyCycleEncoder(DIOConstants.kClimberBackEncoderID, 1, Degrees.of(243).in(Rotations));
    climberMotorFront = new TalonFX(CANIDs.kClimberMotorFront);
    climberMotorBack = new TalonFX(CANIDs.kClimberMotorBack);
    feedbackConfigs = new FeedbackConfigs().withSensorToMechanismRatio(ClimberConstants.kGearRatio);
    climberConfigsToClimber =
        new Slot0Configs()
            .withKP(ClimberConstants.kClimberP0)
            .withKI(ClimberConstants.kClimberI0)
            .withKD(ClimberConstants.kClimberD0)
            .withKG(ClimberConstants.kClimberkG0)
            .withKV(ClimberConstants.kClimberkV0);
    climberConfigsAtClimber =
        new Slot1Configs()
            .withKP(ClimberConstants.kClimberP1)
            .withKI(ClimberConstants.kClimberI1)
            .withKD(ClimberConstants.kClimberD1)
            .withKG(ClimberConstants.kClimberkG1)
            .withKV(ClimberConstants.kClimberkV1);
    // Boolean to check if the climber is climbing of if it is just idle
    isClimbingStateSim = false;
    // Set the configs
    climberOutputConfigsFront = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);
    climberOutputConfigsBack = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);
    frontConfigs =
        new TalonFXConfiguration()
            .withSlot0(climberConfigsToClimber)
            .withSlot1(climberConfigsAtClimber)
            .withMotorOutput(
                climberOutputConfigsFront.withInverted(ClimberConstants.kClimberFrontInvert))
            .withFeedback(feedbackConfigs);
    backConfigs =
        new TalonFXConfiguration()
            .withSlot0(climberConfigsToClimber)
            .withSlot1(climberConfigsAtClimber)
            .withMotorOutput(
                climberOutputConfigsBack.withInverted(ClimberConstants.kClimberBackInvert))
            .withFeedback(feedbackConfigs.withSensorToMechanismRatio(ClimberConstants.kGearRatio));
    climberMotorFront.getConfigurator().apply(frontConfigs);
    climberMotorBack.getConfigurator().apply(backConfigs);
    climberMotorFront.setPosition(climberTargetAngle.in(Rotations));
    climberMotorBack.setPosition(climberTargetAngle.in(Rotations));
    climberOrchestra = new Orchestra();
    climberOrchestra.addInstrument(climberMotorFront, 2);
    climberOrchestra.addInstrument(climberMotorBack, 6);
    climberOrchestra.loadMusic("music/jeopardymusic.chrp");
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
                  * DrivetrainConstants.ROBOT_MASS_KG,
              ClimberConstants.kClimberArmLength.in(Meters),
              ClimberConstants.kClimberArmMinAngle.in(Radians),
              ClimberConstants.kClimberArmMaxAngle.in(Radians),
              true,
              ClimberConstants.kClimberRetractedSetpoint.in(Radians));

      climbMech = new Mechanism2d(4, 2);
      climbMechRoot1 = climbMech.getRoot("Climb Mech root", 3, 1);
      climbMechRoot2 = climbMech.getRoot("Climb Mech root 2", 1, 1);
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
      SmartDashboard.putData("Climber/Climb Mech", climbMech);
    }
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
          climberMotorFront.setPosition(
              1.0 - climberFrontEncoder.get() + ClimberConstants.kClimberOffsetAngle.in(Rotations));
          climberMotorBack.setPosition(
              climberBackEncoder.get() + ClimberConstants.kClimberOffsetAngle.in(Rotations));
        });
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
              climberTargetAngle = position;
              climberMotorFront.setControl(new PositionVoltage(position).withSlot(slot));
              climberMotorBack.setControl(new PositionVoltage(position).withSlot(slot));
              SmartDashboard.putNumber("Climber/Climber Position Setpoint", position.in(Degrees));
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

  public Command climb() {
    return Commands.sequence(
        runClimber(ClimberConstants.kClimberAtCageSetpoint, 0),
        Commands.waitUntil(isClimberAtPosition(ClimberConstants.kClimberAtCageSetpoint)),
        runClimber(ClimberConstants.kClimberExtendedSetpoint, 1));
  }

  public Command extendToCage() {
    return runClimber(ClimberConstants.kClimberAtCageSetpoint, 0)
        .andThen(Commands.waitUntil(isClimberAtPosition(ClimberConstants.kClimberAtCageSetpoint)));
  }

  public Command extendFully() {
    return runClimber(ClimberConstants.kClimberExtendedSetpoint, 1);
  }

  public Command retract() {
    return runClimber(ClimberConstants.kClimberRetractedSetpoint, 0);
  }

  private SingleJointedArmSim getSimulator() {
    if (isClimbingStateSim) {
      return climberSimLifting;
    } else {
      return climberSimNormal;
    }
  }

  public void toggleClimb() {
    if (Robot.isSimulation()) {
      toggleClimbingSim();
    } else {
      // Implement real robot behavior for toggling climb state
      if (isClimbingStateSim) {
        climberMotorFront.getConfigurator().apply(frontConfigs.withSlot0(climberConfigsToClimber));
        climberMotorBack.getConfigurator().apply(backConfigs.withSlot0(climberConfigsToClimber));
        isClimbingStateSim = false;
      } else {
        climberMotorFront.getConfigurator().apply(frontConfigs.withSlot1(climberConfigsAtClimber));
        climberMotorBack.getConfigurator().apply(backConfigs.withSlot1(climberConfigsAtClimber));
        isClimbingStateSim = true;
      }
      SmartDashboard.putBoolean("Climbing", isClimbingStateSim);
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
    SmartDashboard.putBoolean("Climber/Climbing", isClimbingStateSim);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(
        "Climber/Motor Voltage Front", climberMotorFront.getMotorVoltage().getValue().in(Volts));
    SmartDashboard.putNumber(
        "Climber/Motor Voltage Back", climberMotorBack.getMotorVoltage().getValue().in(Volts));
    SmartDashboard.putNumber(
        "Climber/Climber Position Front", climberMotorFront.getPosition().getValue().in(Degrees));
    SmartDashboard.putNumber(
        "Climber/Climber Position Back", climberMotorBack.getPosition().getValue().in(Degrees));
    SmartDashboard.putNumber(
        "Climber/Absolute Encoder Front", Rotations.of(1 - climberFrontEncoder.get()).in(Degrees));
    SmartDashboard.putNumber(
        "Climber/Absolute Encoder Back", Rotations.of(climberBackEncoder.get()).in(Degrees));
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

    SmartDashboard.putNumber(
        "Climber/Climber Angle", Radians.of(simulator.getAngleRads()).in(Degrees));
    SmartDashboard.putBoolean("Climber/Climbing", isClimbingStateSim);
    if (climberMotorFront.getPosition().getValue().gt(ClimberConstants.kClimberAtCageSetpoint)) {
      if (simulator == climberSimNormal) {
        toggleClimb();
      }
    } else {
      if (simulator == climberSimLifting) {
        toggleClimb();
      }
    }
  }
}
