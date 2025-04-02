package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EverybotClimberConstants;

public class EverybotClimber extends SubsystemBase {

  private final SparkMax climbMotor;

  /** This is the subsytem that controls the climber. */
  public EverybotClimber() {

    // Set up the climb motor as a brushless motor
    climbMotor = new SparkMax(EverybotClimberConstants.kClimberMotorID, MotorType.kBrushless);

    /*
     * Set can timeout. Because this project only sets parameters once on
     * construction, the timeout can be long without blocking robot operation. Code
     * which sets or gets parameters during operation may need a shorter timeout.
     */
    climbMotor.setCANTimeout(250);

    /*
     * Create and apply configuration for climb motor. Voltage compensation helps
     * the climb behave the same as the battery
     * voltage dips. The current limit helps prevent breaker trips or burning out
     * the motor in the event the climb stalls.
     */
    SparkMaxConfig climbConfig = new SparkMaxConfig();
    climbConfig.voltageCompensation(EverybotClimberConstants.kClimberMotorVoltageComp);
    climbConfig.smartCurrentLimit(EverybotClimberConstants.kClimberMotorCurrentLimit);
    climbConfig.idleMode(IdleMode.kBrake);
    climbMotor.configure(
        climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {}

  /**
   * Use to run the climber, can be set to run from 100% to -100%. Keep in mind that the direction
   * changes based on which way the winch is wound.
   *
   * @param speed motor speed from -1.0 to 1, with 0 stopping it
   */
  public void runClimber(double speed) {
    climbMotor.set(speed);
  }

  public Command climberUp() {
    return startEnd(
        () -> {
          runClimber(EverybotClimberConstants.kClimberSpeedUp);
        },
        () -> {
          runClimber(0);
        });
  }

  public Command climberDown() {
    return startEnd(
        () -> {
          runClimber(EverybotClimberConstants.kClimberSpeedDown);
        },
        () -> {
          runClimber(0);
        });
  }
}
