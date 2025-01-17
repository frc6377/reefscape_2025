package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Millisecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.ElevatorConstants.kCarageFactor;
import static frc.robot.Constants.ElevatorConstants.kCarriageMass;
import static frc.robot.Constants.ElevatorConstants.kElevatorDrumCircumference;
import static frc.robot.Constants.ElevatorConstants.kElevatorDrumRadius;
import static frc.robot.Constants.ElevatorConstants.kElevatorGearbox;
import static frc.robot.Constants.ElevatorConstants.kElevatorGearing;
import static frc.robot.Constants.ElevatorConstants.kMaxElevatorHeight;
import static frc.robot.Constants.ElevatorConstants.kMinElevatorHeight;

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
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.Robot;

public class Elevator extends SubsystemBase {
  private SparkMax elevatorMotor;
  private SparkAbsoluteEncoder elevatorEncoder;
  private SparkMaxSim simElevatorMotor;
  private final Time sparkPeriod;
  private static Mechanism2d mech = new Mechanism2d(2, 2);
  private static ComplexWidget widg;
  private MechanismLigament2d elevatorMech;

  public static final AbsoluteEncoderConfig encoderCfg =
      new AbsoluteEncoderConfig()
          .positionConversionFactor(Constants.ElevatorConstants.kElevatorConversion);
  public static final SoftLimitConfig elvMaxSoftLimit =
      new SoftLimitConfig()
          .forwardSoftLimit(heightToRotations(Constants.ElevatorConstants.kTopLimit).in(Rotations));
  public static final SoftLimitConfig elvMinSoftLimit =
      new SoftLimitConfig()
          .reverseSoftLimit(
              heightToRotations(Constants.ElevatorConstants.kBottomLimit).in(Rotations));
  public static final ClosedLoopConfig loopCfg =
      new ClosedLoopConfig()
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pidf(
              Constants.ElevatorConstants.P,
              Constants.ElevatorConstants.I,
              Constants.ElevatorConstants.D,
              Constants.ElevatorConstants.FF);
  private ElevatorSim m_elevatorSim;

  public Elevator() {
    sparkPeriod = Millisecond.one();
    elevatorMotor = new SparkMax(MotorIDConstants.kElevatorMotor1, MotorType.kBrushless);
    elevatorEncoder = elevatorMotor.getAbsoluteEncoder();
    elevatorMotor.configure(
        new SparkMaxConfig()
            .apply(encoderCfg)
            .apply(loopCfg)
            .apply(elvMaxSoftLimit)
            .apply(elvMinSoftLimit),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);

    elevatorMotor.getEncoder().setPosition(elevatorEncoder.getPosition() * kElevatorGearing);

    // Simulation
    if (Robot.isSimulation()) {
      m_elevatorSim =
          new ElevatorSim(
              kElevatorGearbox,
              kElevatorGearing,
              kCarriageMass.in(Kilograms),
              kElevatorDrumRadius.in(Meters),
              kMinElevatorHeight.in(Meters),
              kMaxElevatorHeight.in(Meters),
              true,
              0);
      simElevatorMotor =
          new SparkMaxSim(elevatorMotor, Constants.ElevatorConstants.kElevatorGearbox);
      elevatorMech =
          mech.getRoot("root", 1, 0)
              .append(
                  new MechanismLigament2d(
                      "Elevator Mech [0]", 1, 90, 10, new Color8Bit(Color.kPurple)));

      if (widg == null) {
        widg = Shuffleboard.getTab(getName()).add("Elevator", mech);
      }
    }

    SmartDashboard.putNumber("Elevator/Setpoint", 0);
    SmartDashboard.putNumber("Elevator/Setpoint Rotations", 0);
  }

  public static Distance rotationsToHeight(Angle rotations) {
    return ElevatorConstants.kElevatorDrumCircumference
        .times(rotations.in(Rotations) * ElevatorConstants.kCarageFactor)
        .div(kElevatorGearing);
  }

  // height = C * rot * 2/75 -> rot = height * 75/2C
  public static Angle heightToRotations(Distance height) {
    return height
        .times(kElevatorGearing)
        .div(kElevatorDrumCircumference.times(kCarageFactor))
        .times(Rotations.one());
  }

  public Distance getElevatorHeight() {
    return rotationsToHeight(Rotations.of(elevatorMotor.getEncoder().getPosition()));
  }

  public Command goUp() {
    return startEnd(
        () -> {
          elevatorMotor.set(.2);
        },
        () -> elevatorMotor.set(0));
  }

  public Command goDown() {
    return startEnd(
        () -> {
          elevatorMotor.set(-.2);
        },
        () -> elevatorMotor.set(0));
  }

  public Command zeroMotorEncoder() {
    return Commands.runOnce(
        () -> {
          elevatorMotor.getEncoder().setPosition(0);
        });
  }

  public Command changeElevation(Distance heightLevel) {
    return runOnce(
        () -> {
          double adjustedSetpoint =
              heightToRotations(heightLevel).in(Rotations);
          elevatorMotor
              .getClosedLoopController()
              .setReference(adjustedSetpoint, ControlType.kPosition);
          SmartDashboard.putNumber("Elevator/Setpoint", heightLevel.in(Meters));
          SmartDashboard.putNumber("Elevator/Setpoint Rotations", adjustedSetpoint);
        });
  }

  public Command L0() {
    return changeElevation(Inches.zero());
  }

  public Command L1() {
    return changeElevation(Constants.ElevatorConstants.kL1Height);
  }

  public Command L2() {
    return changeElevation(Constants.ElevatorConstants.kL2Height);
  }

  public Command L3() {
    return changeElevation(Constants.ElevatorConstants.kL3Height);
  }

  public Command L4() {
    return changeElevation(Constants.ElevatorConstants.kL4Height);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator/Absolute Encoder rotations", elevatorEncoder.getPosition());
    SmartDashboard.putNumber(
        "Elevator/Motor Encoder Rotation", elevatorMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Elevator/Motor Percent", elevatorMotor.get());
    SmartDashboard.putNumber("Elevator/Height", getElevatorHeight().in(Meters));
  }

  @Override
  public void simulationPeriodic() {
    Distance simDist = Meters.zero();
    LinearVelocity simVel = MetersPerSecond.zero();

    for (Time i = Seconds.zero(); i.lt(Robot.period); i = i.plus(sparkPeriod)) {
      m_elevatorSim.setInputVoltage(
          simElevatorMotor.getBusVoltage() * simElevatorMotor.getAppliedOutput());
      m_elevatorSim.update(sparkPeriod.in(Seconds));
      simDist = Meters.of(m_elevatorSim.getPositionMeters());
      simVel = MetersPerSecond.of(m_elevatorSim.getVelocityMetersPerSecond());
      simElevatorMotor.iterate(
          ((simVel.in(MetersPerSecond) / kElevatorDrumCircumference.in(Meters)))
              * 60
              * kElevatorGearing
              / kCarageFactor,
          RobotController.getBatteryVoltage(),
          sparkPeriod.in(Seconds));
    }

    elevatorMech.setLength(0.1 + (simDist.in(Meters)));

    SmartDashboard.putNumber("Elevator/Sim Length", simDist.in(Meters));
    SmartDashboard.putNumber("Elevator/Sim velocity", simVel.in(MetersPerSecond));
    SmartDashboard.putNumber("Elevator/Sim Pose", m_elevatorSim.getPositionMeters());
  }
}
