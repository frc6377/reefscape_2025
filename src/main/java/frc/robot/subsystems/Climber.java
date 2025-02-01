// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
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
import frc.robot.Constants.MotorIDConstants;
import frc.robot.Robot;
import java.util.function.BooleanSupplier;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private TalonFX climberMotorFront;

  private TalonFX climberMotorBack;
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

  public Climber() {
    climberMotorFront = new TalonFX(MotorIDConstants.kCLimberMotorLeader);
    climberMotorBack = new TalonFX(MotorIDConstants.kCLimberMotorFollower);
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

    // Set the configs
    climberOutputConfigs = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);
    frontConfigs =
        new TalonFXConfiguration()
            .withSlot0(climberConfigsToClimber)
            .withSlot1(climberConfigsAtClimber)
            .withMotorOutput(
                climberOutputConfigs.withInverted(InvertedValue.CounterClockwise_Positive));
    backConfigs =
        new TalonFXConfiguration()
            .withSlot0(climberConfigsToClimber)
            .withSlot1(climberConfigsAtClimber)
            .withMotorOutput(climberOutputConfigs.withInverted(InvertedValue.Clockwise_Positive));
    climberMotorFront.getConfigurator().apply(frontConfigs);
    climberMotorBack.getConfigurator().apply(backConfigs);
    // For simulation
    climberTargetAngle = ClimberConstants.kClimberRetractedSetpoint;
    if (Robot.isSimulation()) {
      simClimberGearbox = DCMotor.getKrakenX60(1);
      climberSimNormal =
          new SingleJointedArmSim(
              simClimberGearbox,
              ClimberConstants.KGearRatio,
              ClimberConstants.kClimberArmMOI.in(KilogramSquareMeters),
              ClimberConstants.kClimberArmLength.in(Meters),
              ClimberConstants.kClimberArmMinAngle.in(Radians),
              ClimberConstants.kClimberArmMaxAngle.in(Radians),
              true,
              climberTargetAngle.in(Radians));
      climberSimLifting =
          new SingleJointedArmSim(
              simClimberGearbox,
              ClimberConstants.KGearRatio,
              SingleJointedArmSim.estimateMOI(ClimberConstants.kClimberArmLength.in(Meters), ClimberConstants.kClimberMass.in(Kilograms)),
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


private Command runClimber(Angle position, int slot) {
    return runOnce(
        () -> {
          climberTargetAngle = position;
          climberMotorFront.setControl(
              new PositionDutyCycle(position.times(ClimberConstants.KGearRatio)).withSlot(slot));
          climberMotorBack.setControl(
              new PositionDutyCycle(position.times(ClimberConstants.KGearRatio)).withSlot(slot));
          SmartDashboard.putNumber("Climber Position Setpoint", position.in(Degrees));
        });
  }

  private BooleanSupplier isClimberAtPosition(Angle position) {
    return () ->
        Math.abs(
                (climberMotorFront.getPosition().getValue().in(Degrees)
                        / ClimberConstants.KGearRatio
                    - position.in(Degrees)))
            < ClimberConstants.kClimberSensorError.in(Degrees);
  }

  public Command climb() {
    return Commands.sequence(
        runClimber(ClimberConstants.kClimberAtCageSetpoint, 0),
        Commands.waitUntil(isClimberAtPosition(ClimberConstants.kClimberAtCageSetpoint)),
        runClimber(ClimberConstants.kClimberExtendedSetpoint, 1),
        Commands.waitUntil(isClimberAtPosition(ClimberConstants.kClimberExtendedSetpoint)));
  }

  public Command retract() {
    return Commands.sequence(
        runClimber(ClimberConstants.kClimberRetractedSetpoint, 0),
        Commands.waitUntil(isClimberAtPosition(ClimberConstants.kClimberRetractedSetpoint)));
  }

  @Override
  public void simulationPeriodic() {
    climbMechTargetLigament.setAngle(climberTargetAngle.in(Degrees));
    climberSimNormal.setInput(climberMotorFront.getMotorVoltage().getValue().in(Volts));
    climberSimNormal.update(Robot.defaultPeriodSecs);
    climberMotorFront.setPosition(
        Radians.of(climberSimNormal.getAngleRads() * ClimberConstants.KGearRatio));
    climbMechLigament.setAngle(Radians.of(climberSimNormal.getAngleRads()).in(Degrees));

    SmartDashboard.putNumber(
        "Climber Angle", Radians.of(climberSimNormal.getAngleRads()).in(Degrees));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(
        "Motor Voltage", climberMotorFront.getMotorVoltage().getValue().in(Volts));
  }
}
