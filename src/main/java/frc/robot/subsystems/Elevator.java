package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Millisecond;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.ElevatorSimConstants.*;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Elevator extends SubsystemBase {
  SparkMax elevatorMotor;
  SparkAbsoluteEncoder elevatorEncoder;
  private SparkMaxSim simElevatorMotor;
  private final Time sparkPeriod;

  public static final AbsoluteEncoderConfig encoderCfg =
      new AbsoluteEncoderConfig()
          .positionConversionFactor(Constants.ElevatorConstants.elevatorConversion);
  public static final ClosedLoopConfig loopCfg =
      new ClosedLoopConfig()
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          .pid(
              Constants.ElevatorConstants.elevatorP,
              Constants.ElevatorConstants.elevatorI,
              Constants.ElevatorConstants.elevatorD);

  public Elevator() {
    sparkPeriod = Millisecond.one();
    elevatorMotor = new SparkMax(0, MotorType.kBrushless);
    elevatorEncoder = elevatorMotor.getAbsoluteEncoder();

    elevatorMotor.configure(
        new SparkMaxConfig().apply(encoderCfg).apply(loopCfg),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
    if (Robot.isSimulation()) {
      simElevatorMotor =
          new SparkMaxSim(elevatorMotor, Constants.ElevatorSimConstants.m_elevatorGearbox);
    }
  }

  public Distance getElevatorHeight() {
    return Inches.of(elevatorEncoder.getPosition());
  }

  public Command goUp() {
    return startEnd(
        () -> {
          elevatorMotor.set(.5);
        },
        () -> elevatorMotor.set(0));
  }

  public Command goDown() {
    return startEnd(
        () -> {
          elevatorMotor.set(-.5);
        },
        () -> elevatorMotor.set(0));
  }

  public Command changeElevation(Distance heightLevel) {
    return startEnd(
        () -> {
          elevatorMotor
              .getClosedLoopController()
              .setReference(heightLevel.in(Inches), ControlType.kPosition);
        },
        () -> elevatorMotor.set(0));
  }

  public Command L0() {
    return changeElevation(Inches.zero());
  }

  public Command L1() {
    return changeElevation(Constants.ElevatorConstants.L1Height);
  }

  public Command L2() {
    return changeElevation(Constants.ElevatorConstants.L2Height);
  }

  public Command L3() {
    return changeElevation(Constants.ElevatorConstants.L3Height);
  }

  public Command L4() {
    return changeElevation(Constants.ElevatorConstants.L4Height);
  }

  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          m_elevatorGearbox,
          kElevatorGearing,
          kCarriageMass,
          kElevatorDrumRadius.in(Meters),
          kMinElevatorHeightMeters,
          kMaxElevatorHeightMeters,
          true,
          0,
          0.01,
          0.0);

  @Override
  public void simulationPeriodic() {
    Distance simDist = Meters.zero();
    LinearVelocity simVel = MetersPerSecond.zero();
    for (Time i = Seconds.zero(); i.lt(Robot.period); i = i.plus(sparkPeriod)) {
      m_elevatorSim.setInputVoltage(
          RobotController.getBatteryVoltage() * simElevatorMotor.getAppliedOutput());
      m_elevatorSim.update(sparkPeriod.in(Seconds));
      simDist = Meters.of(m_elevatorSim.getPositionMeters());
      simVel = MetersPerSecond.of(m_elevatorSim.getVelocityMetersPerSecond());
      simElevatorMotor.iterate(
          ((simVel.in(MetersPerSecond) / (2 * Math.PI * kElevatorDrumRadius.in(Meters)))
                  * kElevatorGearing)
              * 60,
          RobotController.getBatteryVoltage(),
          sparkPeriod.in(Seconds));
    }
  }
}
