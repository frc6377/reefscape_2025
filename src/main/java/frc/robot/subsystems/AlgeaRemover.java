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
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.AlgeaRemoverConstants;
import frc.robot.Constants.DIOConstants;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class AlgeaRemover extends SubsystemBase {
  private SingleJointedArmSim algeaSim;
  private SparkMax algeaMotor;
  private SparkMaxSim simAlgeaMotor;
  private Angle simAngle;
  private Angle algeaSetpoint;
  private DutyCycleEncoder algeaEncoder;

  private Mechanism2d mech = new Mechanism2d(2, 2);
  private MechanismLigament2d algeaMech;

  public static final ClosedLoopConfig algeaCfg =
      new ClosedLoopConfig()
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(
              AlgeaRemoverConstants.kAlgeaP,
              AlgeaRemoverConstants.kAlgeaI,
              AlgeaRemoverConstants.kAlgeaD);
  private static final SparkMaxConfig algaeMotorConfig = new SparkMaxConfig();

  public AlgeaRemover() {
    algeaMotor = new SparkMax(Constants.CANIDs.kAlgeaMotor, MotorType.kBrushless);
    algaeMotorConfig.smartCurrentLimit(20);
    algaeMotorConfig.apply(algeaCfg);
    algaeMotorConfig.inverted(true);
    algeaMotor.configure(
        algaeMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    algeaEncoder =
        new DutyCycleEncoder(
            DIOConstants.kAlgeaEncoderID, 1, AlgeaRemoverConstants.kEncoderOffset.in(Rotations));
    if (Robot.isSimulation()) {
      simAlgeaMotor = new SparkMaxSim(algeaMotor, AlgeaRemoverConstants.kAlgeaGearbox);
      algeaSim =
          new SingleJointedArmSim(
              AlgeaRemoverConstants.kAlgeaGearbox,
              AlgeaRemoverConstants.kAlegeaGearRatio,
              SingleJointedArmSim.estimateMOI(
                  AlgeaRemoverConstants.kAlgeaArmLength.in(Meters), .5), // Update with real value
              AlgeaRemoverConstants.kAlgeaArmLength.in(Meters),
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
      SmartDashboard.putData("Mech2Ds/algea remover mech", mech);
    }
    Logger.recordOutput("Algea Remover/Absolute Setpoint (Degrees)", Degrees.zero().in(Degrees));
  }

  public void seedEncoder() {
    double encoderPose = algeaEncoder.get();
    double encoderOffset = encoderPose < 0.5 ? encoderPose : encoderPose - 1;
    algeaMotor.getEncoder().setPosition(encoderOffset * AlgeaRemoverConstants.kAlegeaGearRatio);
  }

  public void changeAngle(Angle angle) {
    algeaSetpoint = angle;
    algeaMotor
        .getClosedLoopController()
        .setReference(
            angle.in(Rotations) * AlgeaRemoverConstants.kAlegeaGearRatio, ControlType.kPosition);

    Logger.recordOutput("Algea Remover/Absolute Setpoint (Degrees)", angle.in(Degrees));
    Logger.recordOutput(
        "Algea Remover/Setpoint (Rotations)",
        angle.in(Rotations) * AlgeaRemoverConstants.kAlegeaGearRatio);
    Logger.recordOutput(
        "Algea Remover/Setpoint (Degrees)",
        angle.in(Degrees) * AlgeaRemoverConstants.kAlegeaGearRatio);
  }

  public Angle getAlgaeArmAngle() {
    return Rotations.of(algeaEncoder.get());
  }

  public Trigger algeaArmAtSetpoint() {
    return new Trigger(
            () ->
                Rotations.of(algeaEncoder.get())
                    .isNear(algeaSetpoint, AlgeaRemoverConstants.ksetpointTolerance))
        .debounce(.5);
  }

  public Command upCommand() {
    return startEnd(
        () -> algeaMotor.set(-AlgeaRemoverConstants.kAlgeaPercent), () -> algeaMotor.stopMotor());
  }

  public Command downCommand() {
    return startEnd(
        () -> algeaMotor.set(AlgeaRemoverConstants.kAlgeaPercent), () -> algeaMotor.stopMotor());
  }

  public Command removeUpCommand() {
    return startEnd(
        () -> changeAngle(AlgeaRemoverConstants.kRemoveUpAngle),
        () -> changeAngle(AlgeaRemoverConstants.kAlgeaStowed));
  }

  public Command removeDownCommand() {
    return startEnd(
        () -> changeAngle(AlgeaRemoverConstants.kRemoveDownAngle),
        () -> changeAngle(AlgeaRemoverConstants.kAlgeaStowed));
  }

  @Override
  public void periodic() {
    Logger.recordOutput(
        "Algea Remover/Motor/Position (Degrees)",
        Rotations.of(algeaMotor.getEncoder().getPosition()).in(Degrees));
    Logger.recordOutput("Algea Remover/Motor/Percent (%)", algeaMotor.get());
    Logger.recordOutput("Algea Remover/Motor/Voltage (Volts)", algeaMotor.getBusVoltage());
    Logger.recordOutput("Algea Remover/Motor/Current (Amps)", algeaMotor.getOutputCurrent());

    Logger.recordOutput(
        "Algea Remover/Absolute Position (Degrees)", Rotations.of(algeaEncoder.get()).in(Degrees));
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
    Logger.recordOutput("Algea Remover/Sim Angle (Degrees)", simAngle.in(Degrees));
  }
}
