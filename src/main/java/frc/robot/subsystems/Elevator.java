package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;

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
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  SparkMax elevatorMotor;
  SparkAbsoluteEncoder elevatorEncoder;

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
    elevatorMotor = new SparkMax(0, MotorType.kBrushless);
    elevatorEncoder = elevatorMotor.getAbsoluteEncoder();

    elevatorMotor.configure(
        new SparkMaxConfig().apply(encoderCfg).apply(loopCfg),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
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
          Constants.ElevatorSimConstants.kElevatorGearing,
          Constants.ElevatorSimConstants.kCarriageMass,
          Constants.ElevatorSimConstants.kElevatorDrumRadius,
          Constants.ElevatorSimConstants.kMinElevatorHeightMeters,
          Constants.ElevatorSimConstants.kMaxElevatorHeightMeters,
          true,
          0,
          0.01,
          0.0);
}
