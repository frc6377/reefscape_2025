// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static frc.robot.Constants.CoralScorerConstants.*;
import static frc.robot.Constants.SensorIDs.kScorerSensorID;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.CANIDs;
import org.littletonrobotics.junction.Logger;
import utilities.TOFSensorSimple;
import utilities.TOFSensorSimple.TOFType;

public class CoralScorer extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private TalonFX scorerMotor;

  private TalonFXConfiguration scoreMotorConfig;

  private TOFSensorSimple TOFSensor;

  public CoralScorer() {
    scorerMotor = new TalonFX(CANIDs.kScorerMotor, Constants.RIOName);
    scoreMotorConfig = new TalonFXConfiguration();
    scoreMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    scoreMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;
    scorerMotor.getConfigurator().apply(scoreMotorConfig);

    TOFSensor = new TOFSensorSimple(kScorerSensorID, Inches.of(2.5), TOFType.LASER_CAN);
  }

  public Trigger hasCoral() {
    return TOFSensor.getBeamBrokenTrigger();
  }

  public void stopMotor() {
    scorerMotor.stopMotor();
  }

  public void setMotor(Current current) {
    scorerMotor.setControl(new TorqueCurrentFOC(current));
  }

  // Made a command to spin clockwise
  public Command scoreCommand() {
    return startEnd(() -> setMotor(kScoreAMPs), () -> stopMotor());
  }

  public Command intakeCommand() {
    return startEnd(() -> setMotor(kIntakeAMPs), () -> stopMotor());
  }

  // Made a command to spin counter clockwise
  public Command reverseCommand() {
    return startEnd(() -> setMotor(kIntakeAMPs.times(-1)), () -> stopMotor());
  }

  public Command stopCommand() {
    return runOnce(() -> scorerMotor.stopMotor());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("CoralScorer/Motor Output", scorerMotor.get());
    Logger.recordOutput(
        "CoralScorer/Motor Velocity (RPS)", scorerMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput("CoralScorer/Sensor Distance (Inches)", TOFSensor.getDistance().in(Inches));
    Logger.recordOutput("CoralScorer/Sensor Bool", TOFSensor.getBeamBroke());
  }
}
