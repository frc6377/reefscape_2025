package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DIOConstants;

public class AlgeaScorer extends SubsystemBase {
  private SparkMax algeaMotor;
  private DutyCycleEncoder algeaEncoder;
  private SparkMaxSim simAlgeaMotor;

  private Mechanism2d mech = new Mechanism2d(2, 2);
  private ComplexWidget widget;
  private MechanismLigament2d algeaMech;

  public static final ClosedLoopConfig algeaCfg =
      new ClosedLoopConfig()
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(
              Constants.AlgeaScorerConstants.kAlgeaP,
              Constants.AlgeaScorerConstants.kAlgeaI,
              Constants.AlgeaScorerConstants.kAlgeaD);

  public AlgeaScorer() {
    algeaMotor = new SparkMax(Constants.CANIDs.kAlgeaMotor, MotorType.kBrushless);
    algeaEncoder =
        new DutyCycleEncoder(
            DIOConstants.kAlgeaEncoderID, 1, Constants.AlgeaScorerConstants.algeaZero);
    algeaMotor.configure(
        new SparkMaxConfig().apply(algeaCfg),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);

    algeaMech =
        mech.getRoot("root", 1, 0)
            .append(
                new MechanismLigament2d(
                    "Algea Mech [0]", 15, 0, 10, new Color8Bit(Color.kDarkViolet)));
    if (widget == null) {
      widget = Shuffleboard.getTab(getName()).add("Algea Score", mech);
    }
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
          algeaMotor.set(-Constants.AlgeaScorerConstants.kAlgeaPercent);
        },
        () -> algeaMotor.set(0));
  }

  public Command changeAngle(Angle angle) {
    return runOnce(
        () -> {
          algeaMotor
              .getClosedLoopController()
              .setReference(
                  angle.in(Rotations) * Constants.AlgeaScorerConstants.kAlegeaGearRatio,
                  ControlType.kPosition);

          SmartDashboard.putNumber(
              "Algea/Setpoint (Rotations)",
              angle.in(Rotations) * Constants.AlgeaScorerConstants.kAlegeaGearRatio);
          SmartDashboard.putNumber(
              "Algea/Setpoint (Degrees)",
              angle.in(Degrees) * Constants.AlgeaScorerConstants.kAlegeaGearRatio);
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
    SmartDashboard.putNumber("Algea/Motor Percent", algeaMotor.get());
  }
}
