// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.CoralScorerConstants.*;
import static frc.robot.Constants.SensorIDs.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.CANIDs;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import utilities.TOFSensorSimple;
import utilities.TOFSensorSimple.TOFType;

public class CoralScorer extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private TalonFX scorerMotor;

  private TalonFXConfiguration scoreMotorConfig = new TalonFXConfiguration();

  private TOFSensorSimple TOFSensor;
  private TOFSensorSimple alignSensor;

  public CoralScorer() {
    scorerMotor = new TalonFX(CANIDs.kScorerMotor, Constants.RIOName);
    scoreMotorConfig = new TalonFXConfiguration();
    scoreMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    scoreMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;
    scorerMotor.getConfigurator().apply(scoreMotorConfig);

    TOFSensor = new TOFSensorSimple(kScorerSensorID, kSensorDistnace, TOFType.LASER_CAN);
    alignSensor =
        new TOFSensorSimple(
            kAlignmentSensorID, kAlignSensorDistnace, TOFSensorSimple.TOFType.LASER_CAN);
  }

  public Trigger hasCoralTrigger() {
    return TOFSensor.getBeamBrokenTrigger();
  }

  public Trigger scorerAlignedTrigger() {
    return alignSensor.getBeamBrokenTrigger().debounce(kAlignSensorDebounce.in(Seconds));
  }

  public void stopMotor() {
    scorerMotor.stopMotor();
  }

  public void setScoreMotor(double percent) {
    scorerMotor.set(percent);
  }

  // Made a command to spin clockwise
  public Command scoreCommand() {
    return startEnd(() -> setScoreMotor(kScoreSpeed), () -> stopMotor());
  }

  public Command scoreAutoCommand() {
    return startEnd(() -> setScoreMotor(kScoreAutoSpeed), () -> stopMotor());
  }

  public Command intakeCommand() {
    return startEnd(() -> setScoreMotor(kIntakeSpeed), () -> stopMotor());
  }

  // Made a command to spin counter clockwise
  public Command reverseCommand() {
    return startEnd(() -> setScoreMotor(kReverseSpeed), () -> stopMotor());
  }

  public Command runScorer(Supplier<Double> percent) {
    return runEnd(
        () -> scorerMotor.set(Math.abs(percent.get()) * kScoreSpeed),
        () -> scorerMotor.stopMotor());
  }

  public boolean getReefSensorBool() {
    return ReefTOFSensor.getBeamBroke();
  }

  public Trigger getReefSensorTrigger() {
    return ReefTOFSensor.getBeamBrokenTrigger();
  }

  public boolean getReefSensorBool() {
    return ReefTOFSensor.getBeamBroke();
  }

  public Trigger getReefSensorTrigger() {
    return ReefTOFSensor.getBeamBrokenTrigger();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("CoralScorer/Motor/Output", scorerMotor.get());
    Logger.recordOutput(
        "CoralScorer/Motor/Velocity (RPS)", scorerMotor.getVelocity().getValueAsDouble());

    Logger.recordOutput(
        "CoralScorer/Coral Sensor/Distance (Inches)", TOFSensor.getDistance().in(Inches));
    Logger.recordOutput("CoralScorer/Coral Sensor/Bool", TOFSensor.getBeamBroke());

    Logger.recordOutput(
        "CoralScorer/Motor Velocity (RPS)", scorerMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput("CoralScorer/Sensor Distance (Inches)", TOFSensor.getDistance().in(Inches));
    Logger.recordOutput("CoralScorer/Sensor Bool", TOFSensor.getBeamBroke());
  }
}
