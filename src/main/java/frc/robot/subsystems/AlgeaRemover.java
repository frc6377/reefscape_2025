package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.AlgeaRemoverConstants;
import frc.robot.Constants.DIOConstants;
import frc.robot.Robot;

public class AlgeaRemover extends SubsystemBase {
  private SingleJointedArmSim algeaSim;
  private SparkMax algeaMotor;
  private SparkMaxSim simAlgeaMotor;
  private Angle simAngle;
  private Angle algeaSetpoint;
  private DutyCycleEncoder algeaEncoder;

  private Mechanism2d mech = new Mechanism2d(2, 2);
  private ComplexWidget widget;
  private MechanismLigament2d algeaMech;

  public static final ClosedLoopConfig algeaCfg =
      new ClosedLoopConfig()
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(
              AlgeaRemoverConstants.kAlgeaP,
              AlgeaRemoverConstants.kAlgeaI,
              AlgeaRemoverConstants.kAlgeaD);

  public AlgeaRemover() {
    algeaMotor = new SparkMax(Constants.CANIDs.kAlgeaMotor, MotorType.kBrushless);
    algeaMotor.configure(
        new SparkMaxConfig().apply(algeaCfg),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
    algeaEncoder =
        new DutyCycleEncoder(
            DIOConstants.kAlgeaEncoderID, 1, AlgeaRemoverConstants.encoderOffset.in(Rotations));
    algeaMotor.getEncoder().setPosition(algeaEncoder.get());
    if (Robot.isSimulation()) {
      simAlgeaMotor = new SparkMaxSim(algeaMotor, AlgeaRemoverConstants.kAlgeaGearbox);
      algeaSim =
          new SingleJointedArmSim(
              AlgeaRemoverConstants.kAlgeaGearbox,
              AlgeaRemoverConstants.kAlegeaGearRatio,
              SingleJointedArmSim.estimateMOI(
                  AlgeaRemoverConstants.algeaArmLength.in(Meters), .5), // Update with real value
              AlgeaRemoverConstants.algeaArmLength.in(Meters),
              -Math.PI / 2,
              Math.PI / 4,
              true,
              (AlgeaRemoverConstants.kAlgeaStartingAngle).in(Radians));
      simAlgeaMotor.setPosition(
          (AlgeaRemoverConstants.kAlgeaStartingAngle).in(Rotations)
              * AlgeaRemoverConstants.kAlegeaGearRatio);
      algeaMech =
          mech.getRoot("root", 1, 1)
              .append(
                  new MechanismLigament2d(
                      "Algea Mech [0]", 1, 0, 10, new Color8Bit(Color.kDarkViolet)));
    }
    if (widget == null) {
      widget = Shuffleboard.getTab(getName()).add("Algea Remover", mech);
    }
  }

  public Command goUp() {
    return startEnd(
        () -> {
          algeaMotor.set(AlgeaRemoverConstants.kAlgeaPercent);
        },
        () -> algeaMotor.set(0));
  }

  public Command goDown() {
    return startEnd(
        () -> {
          algeaMotor.set(-AlgeaRemoverConstants.kAlgeaPercent);
        },
        () -> algeaMotor.set(0));
  }

  public void seedEncoder() {
    algeaMotor.getEncoder().setPosition(algeaMotor.getAlternateEncoder().getPosition());
  }

  public Command zeroAlgeaEncoder() {
    return Commands.runOnce(
        () -> {
          algeaMotor.getEncoder().setPosition(algeaEncoder.get());
        });
  }

  public Command changeAngle(Angle angle) {
    return runOnce(
        () -> {
          algeaSetpoint = angle;
          algeaMotor
              .getClosedLoopController()
              .setReference(
                  angle.in(Rotations) * AlgeaRemoverConstants.kAlegeaGearRatio,
                  ControlType.kPosition);

          SmartDashboard.putNumber(
              "Algea/Setpoint (Rotations)",
              angle.in(Rotations) * AlgeaRemoverConstants.kAlegeaGearRatio);
          SmartDashboard.putNumber(
              "Algea/Setpoint (Degrees)",
              angle.in(Degrees) * AlgeaRemoverConstants.kAlegeaGearRatio);
        });
  }

  public Trigger algeaAngleAccurate() {
    return new Trigger(
            () ->
                Rotations.of(algeaEncoder.get())
                    .isNear(algeaSetpoint, AlgeaRemoverConstants.ksetpointTolerance))
        .debounce(.5);
  }

  public Command stowAlgea() {
    return changeAngle(AlgeaRemoverConstants.algeaStowed);
  }

  public Command removeAlgea() {
    return changeAngle(AlgeaRemoverConstants.algeaRemove);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Algea/Motor Rotations", algeaEncoder.get());
    SmartDashboard.putNumber("Algea/Motor Percent", algeaMotor.get());
  }

  @Override
  public void simulationPeriodic() {
    algeaSim.setInputVoltage(algeaMotor.getBusVoltage() * algeaMotor.getAppliedOutput());
    algeaSim.update(Robot.defaultPeriodSecs);
    simAlgeaMotor.iterate(
        RadiansPerSecond.of(algeaSim.getVelocityRadPerSec()).in(RPM)
            * AlgeaRemoverConstants.kAlegeaGearRatio,
        RobotController.getBatteryVoltage(),
        Robot.defaultPeriodSecs);
    simAngle = Rotations.of(simAlgeaMotor.getPosition() / AlgeaRemoverConstants.kAlegeaGearRatio);

    algeaMech.setAngle(simAngle.in(Degrees));
    SmartDashboard.putNumber("Algea/Sim Angle", simAngle.in(Degrees));
  }
}
