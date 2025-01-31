// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private DCMotor simClimberGearbox;
  private SingleJointedArmSim climberSim;
  private Mechanism2d climbMech;
  private MechanismRoot2d climbMechRoot;
  private MechanismLigament2d climbMechLigament;
  private Slot0Configs climberConfigsToClimber;
  private Slot1Configs climberConfigsAtClimber;

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

    if (Robot.isSimulation()) {
      simClimberGearbox = DCMotor.getKrakenX60(1);
      climberSim =
          new SingleJointedArmSim(
              simClimberGearbox,
              ClimberConstants.KGearRatio,
              ClimberConstants.ClimberSimConstants.kClimberArmMOI.in(KilogramSquareMeters),
              ClimberConstants.ClimberSimConstants.kClimberArmLength.in(Meters),
              0,
              Math.PI,
              false,
              1);
      climbMech = new Mechanism2d(2, 2);
      climbMechRoot = climbMech.getRoot("Climb Mech root", 1, 1);
      climbMechLigament =
          climbMechRoot.append(
              new MechanismLigament2d(
                  "Climb Mech Ligament", 1, Radians.of(climberSim.getAngleRads()).in(Degrees)));
      SmartDashboard.putData("Climb Mech", climbMech);
    }
  }

  private Command runClimber(Angle position, int slot) {
    return runOnce(
        () -> {

          climberMotorFront.setControl(
              new PositionDutyCycle(position.times(ClimberConstants.KGearRatio)).withSlot(slot));
          climberMotorBack.setControl(
              new PositionDutyCycle(position.times(ClimberConstants.KGearRatio * -1)).withSlot(slot));
        });
  }

  private BooleanSupplier isClimberAtPosition(Angle position) {
    return () ->
        Math.abs((climberMotorFront.getPosition().getValue().in(Degrees) - position.in(Degrees)))
            < 5;
  }

  public Command climb() {
    return Commands.sequence(
        runClimber(ClimberConstants.kClimberCage,0),
        Commands.waitUntil(isClimberAtPosition(ClimberConstants.kClimberCage)),
        runClimber(ClimberConstants.kClimberExtended, 1),
        Commands.waitUntil(isClimberAtPosition(ClimberConstants.kClimberExtended)));
  }

  public Command retract() {
    return Commands.sequence(
        runClimber(ClimberConstants.kClimberCage, 1),
        Commands.waitUntil(isClimberAtPosition(ClimberConstants.kClimberCage)),
        runClimber(ClimberConstants.kClimberRetracted, 0),
        Commands.waitUntil(isClimberAtPosition(ClimberConstants.kClimberRetracted)));
  }

  @Override
  public void simulationPeriodic() {
    climberSim.setInput(climberMotorFront.get());
    climberSim.update(0.02);
    climbMechLigament.setAngle(Radians.of(climberSim.getAngleRads()).in(Degrees));
    SmartDashboard.putData("Climb Mech", climbMech);
    SmartDashboard.putNumber("Climber Angle", climberSim.getAngleRads());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
