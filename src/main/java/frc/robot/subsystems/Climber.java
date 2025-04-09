package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.IntakeConstants.kGearing;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  public static ClosedLoopConfig loopCfg =
      new ClosedLoopConfig()
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pidf(
              ClimberConstants.kP, ClimberConstants.kI, ClimberConstants.kD, ClimberConstants.kFF);

  private final SparkMax climbMotor;

  // For Simulation
  private SparkMaxSim climbMotorSim;
  private SingleJointedArmSim climberSim;
  private static Mechanism2d mech = new Mechanism2d(2, 2);
  private static ComplexWidget widget;
  private MechanismLigament2d climberLigament;

  /** This is the subsytem that controls the climber. */
  public Climber() {

    // Set up the climb motor as a brushless motor
    climbMotor = new SparkMax(ClimberConstants.kClimberMotorID, MotorType.kBrushless);

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
    climbConfig.voltageCompensation(ClimberConstants.kClimberMotorVoltageComp);
    climbConfig.smartCurrentLimit(ClimberConstants.kClimberMotorCurrentLimit);
    climbConfig.idleMode(IdleMode.kBrake);
    climbMotor.configure(
        climbConfig.apply(loopCfg), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Simulation
    if (Robot.isSimulation()) {
      climberSim =
          new SingleJointedArmSim(
              ClimberConstants.kGearbox,
              ClimberConstants.kGearing,
              ClimberConstants.kClimberMOI.in(KilogramSquareMeters),
              ClimberConstants.kClimberArmLength.in(Meters),
              ClimberConstants.kClimberMinAngle.in(Radians),
              ClimberConstants.kClimberMaxAngle.in(Radians),
              true,
              ClimberConstants.kClimberStartAngle.in(Radians));
      climbMotorSim = new SparkMaxSim(climbMotor, ClimberConstants.kGearbox);
      climberLigament =
          mech.getRoot("root", 1, 0)
              .append(
                  new MechanismLigament2d("Climb Mech", 1, 90, 10, new Color8Bit(Color.kPurple)));

      if (widget == null) {
        widget = Shuffleboard.getTab("Mechanism2d").add("Climber", mech);
      }
    }
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Encoder", climbMotor.getEncoder().getPosition());
  }

  @Override
  public void simulationPeriodic() {
    climberSim.setInputVoltage(climbMotorSim.getBusVoltage() * climbMotorSim.getAppliedOutput());
    climberSim.update(Robot.defaultPeriodSecs);
    climbMotorSim.setPosition(Radians.of(climberSim.getAngleRads()).in(Rotations));
    climberLigament.setAngle(Radians.of(climberSim.getAngleRads()).in(Degrees));

    climbMotorSim.iterate(
        RadiansPerSecond.of(climberSim.getVelocityRadPerSec() * kGearing).in(RPM),
        RobotController.getBatteryVoltage(),
        Robot.defaultPeriodSecs);

    Logger.recordOutput(
        "Climber/Climber Motor Voltage",
        climbMotorSim.getBusVoltage() * climbMotorSim.getAppliedOutput());
  }

  /**
   * Use to run the climber, can be set to run from 100% to -100%. Keep in mind that the direction
   * changes based on which way the winch is wound.
   *
   * @param speed motor speed from -1.0 to 1, with 0 stopping it
   */
  public void runClimber(double speed) {
    climbMotor.set(speed);
  }

  public Command goToPosition(Angle position) {
    return runOnce(
            () -> {
              climbMotor
                  .getClosedLoopController()
                  .setReference(
                      position.in(Rotations) * ClimberConstants.kGearing, ControlType.kPosition);
            })
        .until(
            () ->
                Rotations.of(climbMotor.getEncoder().getPosition() * ClimberConstants.kGearing)
                    .isNear(position, ClimberConstants.kTolerance));
  }

  public Command climberUp() {
    return startEnd(
        () -> {
          runClimber(ClimberConstants.kClimberSpeedUp);
        },
        () -> {
          runClimber(0);
        });
  }

  public Command climberDown() {
    return startEnd(
        () -> {
          runClimber(ClimberConstants.kClimberSpeedDown);
        },
        () -> {
          runClimber(0);
        });
  }

  public Command climb() {
    return goToPosition(ClimberConstants.kClimberMaxAngle)
        .andThen(goToPosition(ClimberConstants.kClimberClimbAngle));
  }
}
