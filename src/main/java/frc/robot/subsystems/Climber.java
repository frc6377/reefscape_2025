// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.Robot;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private TalonFX climberMotorLeader;

  private TalonFX climberMotorFollower;
  private PIDController climberPID;
  private DCMotor simClimberGearbox;
  private Mechanism2d simClimberArm;
  private SingleJointedArmSim climberSim;
  private Mechanism2d climbMech;
  private MechanismRoot2d climbMechRoot;
  private MechanismLigament2d climbMechLigament;

  public Climber() {
    climberMotorLeader = new TalonFX(MotorIDConstants.kCLimberMotorLeader);
    climberMotorFollower = new TalonFX(MotorIDConstants.kCLimberMotorFollower);
    climberMotorFollower.setControl(new Follower(MotorIDConstants.kCLimberMotorLeader, true));
    climberPID =
        new PIDController(
            ClimberConstants.kClimberP, ClimberConstants.kClimberI, ClimberConstants.kClimberD);
    if (Robot.isSimulation()) {
      simClimberGearbox = DCMotor.getKrakenX60(1);
      climberSim = new SingleJointedArmSim(simClimberGearbox, 126, 1, 0.1, 0, Math.PI, false, 1);
      climbMech = new Mechanism2d(2, 2);
      climbMechRoot = climbMech.getRoot("Climb Mech root", 1, 1);
      climbMechLigament =
          climbMechRoot.append(
              new MechanismLigament2d(
                  "Climb Mech Ligament", 1, Radians.of(climberSim.getAngleRads()).in(Degrees)));
      SmartDashboard.putData("Climb Mech", climbMech);
    }
  }

  private double calcPID(Angle target) {
    return climberPID.calculate(
        climberMotorLeader.getPosition().getValue().in(Degrees), target.in(Degrees));
  }

  public Command climb() {
    return run(
        () -> {
          climberMotorLeader.set(calcPID(ClimberConstants.kClimberExtended));
          SmartDashboard.putNumber("Target Angle", ClimberConstants.kClimberExtended.in(Degrees));
        });
  }

  public Command retract() {
    return run(
        () -> {
          climberMotorLeader.set(calcPID(ClimberConstants.kClimberRetracted));
          SmartDashboard.putNumber("Target Angle", ClimberConstants.kClimberRetracted.in(Degrees));
        });
  }

  @Override
  public void simulationPeriodic() {
    if (Robot.isSimulation()) {
      climberSim.setInput(climberMotorLeader.get());
      climberSim.update(0.02);
      climbMechLigament.setAngle(Radians.of(climberSim.getAngleRads()).in(Degrees));
      SmartDashboard.putData("Climb Mech", climbMech);
      SmartDashboard.putNumber("Climber Angle", climberSim.getAngleRads());
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
