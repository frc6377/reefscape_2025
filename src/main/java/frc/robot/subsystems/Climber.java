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
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
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
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.Robot;
import java.util.ArrayList;
import java.util.Collections;
import java.util.function.BooleanSupplier;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private final DutyCycleEncoder climberFrontEncoder;

  private final DutyCycleEncoder climberBackEncoder;
  private TalonFX climberMotorFront;
  private Orchestra climberOrchestra;
  private TalonFX climberMotorBack;
  private FeedbackConfigs feedbackConfigs;
  private MotorOutputConfigs climberOutputConfigs;
  private TalonFXConfiguration frontConfigs;
  private TalonFXConfiguration backConfigs;
  private DCMotor simClimberGearbox;
  private SingleJointedArmSim climberSimNormal;
  private SingleJointedArmSim climberSimLifting;
  private Mechanism2d climbMech;
  private MechanismRoot2d climbMechRoot;
  private MechanismLigament2d climbMechLigament;
  private MechanismLigament2d climbMechTargetLigament;
  private Slot0Configs climberConfigsToClimber;
  private Slot1Configs climberConfigsAtClimber;
  private Angle climberTargetAngle;
  private boolean isClimbing;

  public Climber() {
    climberTargetAngle = ClimberConstants.kClimberRetractedSetpoint;
    climberFrontEncoder = new DutyCycleEncoder(10);
    climberBackEncoder = new DutyCycleEncoder(11);
    climberMotorFront = new TalonFX(MotorIDConstants.kCLimberMotorLeader);
    climberMotorBack = new TalonFX(MotorIDConstants.kCLimberMotorFollower);
    feedbackConfigs = new FeedbackConfigs().withSensorToMechanismRatio(ClimberConstants.KGearRatio);
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
    isClimbing = false;
    // Set the configs
    climberOutputConfigs = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);
    frontConfigs =
        new TalonFXConfiguration()
            .withSlot0(climberConfigsToClimber)
            .withSlot1(climberConfigsAtClimber)
            .withMotorOutput(
                climberOutputConfigs.withInverted(InvertedValue.CounterClockwise_Positive))
            .withFeedback(feedbackConfigs);
    backConfigs =
        new TalonFXConfiguration()
            .withSlot0(climberConfigsToClimber)
            .withSlot1(climberConfigsAtClimber)
            .withMotorOutput(climberOutputConfigs.withInverted(InvertedValue.Clockwise_Positive))
            .withFeedback(feedbackConfigs);
    climberMotorFront.getConfigurator().apply(frontConfigs);
    climberMotorBack.getConfigurator().apply(backConfigs);
    climberMotorFront.setPosition(climberTargetAngle.in(Rotations));
    climberOrchestra =
        new Orchestra(new ArrayList<ParentDevice>(Collections.singletonList(climberMotorFront)));
    climberOrchestra.loadMusic("music/jeopardymusic.chrp");
    // For simulation

    if (Robot.isSimulation()) {
      simClimberGearbox = DCMotor.getKrakenX60(2);
      climberSimNormal =
          new SingleJointedArmSim(
              simClimberGearbox,
              ClimberConstants.KGearRatio,
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
              ClimberConstants.KGearRatio,
              Math.pow(ClimberConstants.kClimberArmLength.in(Meters), 2)
                  * DrivetrainConstants.ROBOT_MASS_KG,
              ClimberConstants.kClimberArmLength.in(Meters),
              ClimberConstants.kClimberArmMinAngle.in(Radians),
              ClimberConstants.kClimberArmMaxAngle.in(Radians),
              true,
              ClimberConstants.kClimberRetractedSetpoint.in(Radians));

      climbMech = new Mechanism2d(2, 2);
      climbMechRoot = climbMech.getRoot("Climb Mech root", 1, 1);
      climbMechLigament =
          climbMechRoot.append(
              new MechanismLigament2d(
                  "Climb Mech Ligament",
                  1,
                  Radians.of(climberSimNormal.getAngleRads()).in(Degrees)));
      climbMechTargetLigament =
          climbMechRoot.append(
              new MechanismLigament2d(
                  "Climb Mech Target Ligament",
                  0.8,
                  climberTargetAngle.in(Degrees),
                  10,
                  new Color8Bit(255, 255, 0)));
      SmartDashboard.putData("Climb Mech", climbMech);
    }
  }

  public Command playJeopardy() {
    return runOnce(
        () -> {
          climberOrchestra.play();
        });
  }

  public Command stopJeopardy() {
    return runOnce(
        () -> {
          climberOrchestra.stop();
        });
  }

  public Command zero() {
    return runOnce(
        () -> {
          climberMotorFront.setPosition(
              Degrees.of((climberFrontEncoder.get() + 90) * ClimberConstants.KGearRatio)
                  .in(Rotations));
          climberMotorBack.setPosition(
              Degrees.of((climberBackEncoder.get() + 90) * ClimberConstants.KGearRatio)
                  .in(Rotations));
        });
  }

  public Command runRaw(Voltage voltage) {
    return runOnce(
        () -> {
          climberMotorFront.setControl(new VoltageOut(voltage));
          climberMotorBack.setControl(new VoltageOut(voltage));
        });
  }

  private Command runClimber(Angle position, int slot) {
    return runOnce(
        () -> {
          climberTargetAngle = position;
          climberMotorFront.setControl(new PositionVoltage(position).withSlot(slot));
          climberMotorBack.setControl(new PositionVoltage(position).withSlot(slot));
          SmartDashboard.putNumber("Climber Position Setpoint", position.in(Degrees));
        });
  }

  private BooleanSupplier isClimberAtPosition(Angle position) {
    if (Robot.isReal()) {
      return () ->
          position.isNear(
                  climberMotorFront.getPosition().getValue(), ClimberConstants.kClimberSensorError)
              && position.isNear(
                  climberMotorFront.getPosition().getValue(), ClimberConstants.kClimberSensorError);
    } else {
      return () ->
          position.isNear(
              climberMotorFront.getPosition().getValue(), ClimberConstants.kClimberSensorError);
    }
  }

  public Command climb() {
    return Commands.sequence(
        runClimber(ClimberConstants.kClimberAtCageSetpoint, 0),
        Commands.waitUntil(isClimberAtPosition(ClimberConstants.kClimberAtCageSetpoint)),
        runClimber(ClimberConstants.kClimberExtendedSetpoint, 1));
  }

  public Command retract() {
    return runClimber(ClimberConstants.kClimberRetractedSetpoint, 0);
  }

  private SingleJointedArmSim getSimulator() {
    if (isClimbing) {
      return climberSimLifting;
    } else {
      return climberSimNormal;
    }
  }

  private void toggleClimbingSim() {
    if (isClimbing) {
      climberSimNormal.setState(
          climberSimLifting.getAngleRads(), climberSimLifting.getVelocityRadPerSec());
      isClimbing = false;
    } else {
      climberSimLifting.setState(
          climberSimNormal.getAngleRads(), climberSimNormal.getVelocityRadPerSec());
      isClimbing = true;
    }
    SmartDashboard.putBoolean("Climbing", isClimbing);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(
        "Motor Voltage Front", climberMotorFront.getMotorVoltage().getValue().in(Volts));
    SmartDashboard.putNumber(
        "Motor Voltage Back", climberMotorBack.getMotorVoltage().getValue().in(Volts));
    SmartDashboard.putNumber(
        "Climber Position Front",
        climberMotorFront.getPosition().getValue().div(ClimberConstants.KGearRatio).in(Degrees));
    SmartDashboard.putNumber(
        "Climber Position Back",
        climberMotorBack.getPosition().getValue().div(ClimberConstants.KGearRatio).in(Degrees));
  }

  @Override
  public void simulationPeriodic() {
    climbMechTargetLigament.setAngle(
        climberTargetAngle.minus(ClimberConstants.kClimberOffsetAngle).in(Degrees));

    getSimulator().setInputVoltage(climberMotorFront.getMotorVoltage().getValue().in(Volts));
    getSimulator().update(Robot.defaultPeriodSecs);
    climberMotorFront.setPosition(Radians.of(getSimulator().getAngleRads()));
    climbMechLigament.setAngle(
        Radians.of(getSimulator().getAngleRads())
            .minus(ClimberConstants.kClimberOffsetAngle)
            .in(Degrees));

    SmartDashboard.putNumber(
        "Climber Angle", Radians.of(getSimulator().getAngleRads()).in(Degrees));
    if (climbMechLigament.getAngle() > ClimberConstants.kClimberAtCageSetpoint.in(Degrees)) {
      if (getSimulator() == climberSimNormal) {
        toggleClimbingSim();
      }
    } else {
      if (getSimulator() == climberSimLifting) {
        toggleClimbingSim();
      }
    }
  }
}
