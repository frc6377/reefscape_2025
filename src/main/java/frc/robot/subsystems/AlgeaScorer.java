package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
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
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AlgeaScorerConstants;
import frc.robot.Robot;

public class AlgeaScorer extends SubsystemBase {
  private SingleJointedArmSim algeaSim;
  private SparkMax algeaMotor;
  private SparkMaxSim simAlgeaMotor;
  private double simAngle;

  private Mechanism2d mech = new Mechanism2d(2, 2);
  private ComplexWidget widget;
  private MechanismLigament2d algeaMech;

  public static final ClosedLoopConfig algeaCfg =
      new ClosedLoopConfig()
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(
              AlgeaScorerConstants.kAlgeaP,
              AlgeaScorerConstants.kAlgeaI,
              AlgeaScorerConstants.kAlgeaD);

  public AlgeaScorer() {
    algeaMotor = new SparkMax(Constants.CANIDs.kAlgeaMotor, MotorType.kBrushless);
    algeaMotor.getAlternateEncoder();
    algeaMotor.configure(
        new SparkMaxConfig().apply(algeaCfg),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
    if (Robot.isSimulation()) {
      simAlgeaMotor = new SparkMaxSim(algeaMotor, Constants.AlgeaScorerConstants.kAlgeaGearbox);
      algeaSim =
          new SingleJointedArmSim(
              AlgeaScorerConstants.kAlgeaGearbox,
              AlgeaScorerConstants.kAlegeaGearRatio,
              30, // Update with real value
              AlgeaScorerConstants.algeaArmLength.in(Meters),
              0,
              Math.PI * .75,
              true,
              0);
      algeaMech =
          mech.getRoot("root", 1, 0)
              .append(
                  new MechanismLigament2d(
                      "Algea Mech [0]", 1, 0, 10, new Color8Bit(Color.kDarkViolet)));
    }
    if (widget == null) {
      widget = Shuffleboard.getTab(getName()).add("Algea Score", mech);
    }
  }

  public Command goUp() {
    return startEnd(
        () -> {
          System.out.println("up");
          algeaMotor.set(AlgeaScorerConstants.kAlgeaPercent);
        },
        () -> algeaMotor.set(0));
  }

  public Command goDown() {
    return startEnd(
        () -> {
          System.out.println("down");
          algeaMotor.set(-AlgeaScorerConstants.kAlgeaPercent);
        },
        () -> algeaMotor.set(0));
  }

  public Command zeroAlgeaEncoder() {
    return Commands.runOnce(
        () -> {
          algeaMotor.getEncoder().setPosition(0);
        });
  }

  public Command changeAngle(Angle angle) {
    return runOnce(
        () -> {
          algeaMotor
              .getClosedLoopController()
              .setReference(
                  angle.in(Rotations) * AlgeaScorerConstants.kAlegeaGearRatio,
                  ControlType.kPosition);

          SmartDashboard.putNumber(
              "Algea/Setpoint (Rotations)",
              angle.in(Rotations) * AlgeaScorerConstants.kAlegeaGearRatio);
          SmartDashboard.putNumber(
              "Algea/Setpoint (Degrees)",
              angle.in(Degrees) * AlgeaScorerConstants.kAlegeaGearRatio);
        });
  }

  public Command stowAlgea() {
    return changeAngle(AlgeaScorerConstants.algeaStowed);
  }

  public Command removeAlgea() {
    return changeAngle(AlgeaScorerConstants.algeaRemove);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Algea/Motor Rotation", algeaMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Algea/Motor Percent", algeaMotor.get());
  }

  @Override
  public void simulationPeriodic() {
    algeaSim.setInputVoltage(algeaMotor.getBusVoltage());
    algeaSim.update(Robot.defaultPeriodSecs);
    simAlgeaMotor.setBusVoltage(RobotController.getBatteryVoltage());
    simAlgeaMotor.iterate(
        RadiansPerSecond.of(algeaSim.getVelocityRadPerSec()).in(RPM),
        RobotController.getBatteryVoltage(),
        Robot.defaultPeriodSecs);
    simAngle = simAlgeaMotor.getPosition();

    algeaMech.setAngle(simAngle);
    SmartDashboard.putNumber("Algea/Sim Angle", simAngle);
  }
}
