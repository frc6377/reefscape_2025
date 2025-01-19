package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Millisecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ElevatorConstants.kCarageFactor;
import static frc.robot.Constants.ElevatorConstants.kCarriageMass;
import static frc.robot.Constants.ElevatorConstants.kElevatorDrumCircumference;
import static frc.robot.Constants.ElevatorConstants.kElevatorDrumRadius;
import static frc.robot.Constants.ElevatorConstants.kElevatorGearbox;
import static frc.robot.Constants.ElevatorConstants.kElevatorGearing;
import static frc.robot.Constants.ElevatorConstants.kMaxElevatorHeight;
import static frc.robot.Constants.ElevatorConstants.kMinElevatorHeight;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DigitalInput;
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
  private TalonFX elevatorMotor1;
  private TalonFX elevatorMotor2;
  private final Time sparkPeriod;
  private static Mechanism2d mech = new Mechanism2d(2, 2);
  private static ComplexWidget widg;
  private DigitalInput elvLimitSwitch;
  private MechanismLigament2d elevatorMech;

  public static final AbsoluteEncoderConfig encoderCfg =
      new AbsoluteEncoderConfig()
          .positionConversionFactor(Constants.ElevatorConstants.kElevatorConversion);
  public static final SoftwareLimitSwitchConfigs elvSoftLimit =
      new SoftwareLimitSwitchConfigs()
          .withForwardSoftLimitEnable(true)
          .withForwardSoftLimitThreshold(heightToRotations(Constants.ElevatorConstants.kTopLimit))
          .withReverseSoftLimitEnable(true)
          .withReverseSoftLimitThreshold(
              heightToRotations(Constants.ElevatorConstants.kBottomLimit));
  public static final Slot0Configs loopCfg =
      new Slot0Configs()
          .withKP(Constants.ElevatorConstants.P)
          .withKI(Constants.ElevatorConstants.I)
          .withKD(Constants.ElevatorConstants.D);
  private ElevatorSim m_elevatorSim;

  public Elevator() {
    sparkPeriod = Millisecond.one();
    // TODO: set up for canivore
    elevatorMotor1 = new TalonFX(MotorIDConstants.kElevatorMotor1, "");
    elevatorMotor2 = new TalonFX(MotorIDConstants.kElevatorMotor2, "");
    elevatorMotor1.getConfigurator().apply(loopCfg);
    elevatorMotor1.getConfigurator().apply(elvSoftLimit);
    elevatorMotor2.setControl(new Follower(MotorIDConstants.kElevatorMotor1, true));
    elvLimitSwitch = new DigitalInput(Constants.ElevatorConstants.elvLimitID);

    // elevatorEncoder = elevatorMotor1.getPosition().getValueAsDouble();

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

  public static AngularVelocity heightToRotations(LinearVelocity vel) {
    // rot/s = (G)/(C*2/vel)
    return Rotations.one()
        .times(kElevatorGearing)
        .div((kElevatorDrumCircumference.times(kCarageFactor)).div(vel));
  }

  public Distance getElevatorHeight() {
    return rotationsToHeight(elevatorMotor1.getPosition().getValue());
  }

  public Command goUp() {
    return startEnd(
        () -> {
          elevatorMotor1.set(.2);
        },
        () -> elevatorMotor1.set(0));
  }

  public Command limitHit() {
    return goDown().until(elvLimitSwitch::get).andThen(zeroMotorEncoder());
  }

  public Command goDown() {
    return startEnd(
        () -> {
          elevatorMotor1.set(-.2);
        },
        () -> elevatorMotor1.set(0));
  }

  public Command zeroMotorEncoder() {
    return Commands.runOnce(
        () -> {
          elevatorMotor1.setPosition(0);
        });
  }

  public Command changeElevation(Distance heightLevel) {
    return runOnce(
        () -> {
          Angle adjustedSetpoint = heightToRotations(heightLevel);
          elevatorMotor1.setControl(new PositionVoltage(adjustedSetpoint));
          SmartDashboard.putNumber("Elevator/Setpoint (Inches)", heightLevel.in(Inches));
          SmartDashboard.putNumber("Elevator/Setpoint Rotations", adjustedSetpoint.in(Rotations));
        });
  }

  public Command L0() {
    return changeElevation(Constants.ElevatorConstants.kL0Height);
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
    SmartDashboard.putNumber(
        "Elevator/Motor Encoder Rotation", elevatorMotor1.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Motor Percent", elevatorMotor1.get());
    SmartDashboard.putNumber("Elevator/Height (Inches)", getElevatorHeight().in(Inches));
  }

  @Override
  public void simulationPeriodic() {
    Distance simDist = Meters.zero();
    LinearVelocity simVel = MetersPerSecond.zero();
    var simElevatorMotor = elevatorMotor1.getSimState();
    simElevatorMotor.setSupplyVoltage(Volts.of(RobotController.getBatteryVoltage()));

    // for (Time i = Seconds.zero(); i.lt(Robot.period); i = i.plus(sparkPeriod)) {
    m_elevatorSim.setInputVoltage(simElevatorMotor.getMotorVoltage());
    m_elevatorSim.update(Robot.defaultPeriodSecs);
    simDist = Meters.of(m_elevatorSim.getPositionMeters());
    simVel = MetersPerSecond.of(m_elevatorSim.getVelocityMetersPerSecond());
    elevatorMotor1.setPosition(heightToRotations(simDist));
    // simElevatorMotor.setRawRotorPosition(heightToRotations(simDist));
    // simElevatorMotor.setRotorVelocity(heightToRotations(simVel));
    // }

    elevatorMech.setLength(0.1 + (simDist.in(Meters)));

    SmartDashboard.putNumber("Elevator/Sim Length", simDist.in(Inches));
    SmartDashboard.putNumber("Elevator/Sim velocity", simVel.in(InchesPerSecond));
    SmartDashboard.putNumber("Elevator/Sim Pose", m_elevatorSim.getPositionMeters());
  }
}
