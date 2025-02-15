package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgeaScorer extends SubsystemBase {
  private SparkMax algeaMotor;
  private SparkAbsoluteEncoder algeaEncoder;
  public static final ClosedLoopConfig algeaCfg =
      new ClosedLoopConfig()
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(
              Constants.AlgeaScorerConstants.kAlgeaP,
              Constants.AlgeaScorerConstants.kAlgeaI,
              Constants.AlgeaScorerConstants.kAlgeaD);

  public AlgeaScorer() {
    algeaMotor = new SparkMax(Constants.CANIDs.kAlgeaMotor, MotorType.kBrushless);
    algeaEncoder = algeaMotor.getAbsoluteEncoder();
    algeaMotor.configure(
        new SparkMaxConfig().apply(algeaCfg),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }

  public Command goUp() {
    return startEnd(
        () -> {
          algeaMotor.set(Constants.AlgeaScorerConstants.kAlgeaPercent);
        },
        () -> algeaMotor.set(0));
  }

  public Command goDown() {
    return startEnd(
        () -> {
          algeaMotor.set(-.2);
        },
        () -> algeaMotor.set(0));
  }

  public Command changeAngle(Angle angle) {
    return runOnce(
        () -> {
          algeaMotor
              .getClosedLoopController()
              .setReference(angle.in(Rotations), ControlType.kPosition);

          SmartDashboard.putNumber("Algea/Setpoint (Rotations)", angle.in(Rotations));
          SmartDashboard.putNumber("Algea/Setpoint (Degrees)", angle.in(Degrees));
        });
  }

  public Command stowAlgea() {
    return changeAngle(Constants.AlgeaScorerConstants.algeaStowed);
  }

  public Command removeAlgea() {
    return changeAngle(Constants.AlgeaScorerConstants.algeaRemove);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Algea/Motor Rotation", algeaMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Elevator/Motor Percent", algeaMotor.get());
  }
}
