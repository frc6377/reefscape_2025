package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ElevatorConstants.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.Robot;
import java.util.function.Consumer;
import java.util.function.Supplier;
import utilities.TunableNumber;

public class Elevator extends SubsystemBase {
  private TalonFX elevatorMotor1;
  private TalonFXSimState simElvMotor1;
  private TalonFX elevatorMotor2;
  private DutyCycleEncoder gear3;
  private DutyCycleEncoder gear11;
  private DutyCycleEncoderSim simGear3;
  private DutyCycleEncoderSim simGear11;
  private final SysIdRoutine m_sysIdElevator;

  private final VoltageOut m_voltReq;
  private CurrentLimitsConfigs currentLimit = new CurrentLimitsConfigs();
  private MotorOutputConfigs invertMotor =
      new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive);
  private static Mechanism2d mech = new Mechanism2d(2, 2);
  private DigitalInput elvLimitSwitch;
  private MechanismLigament2d elevatorMech;
  private MotionMagicConfigs elvMotionMagic =
      new MotionMagicConfigs()
          .withMotionMagicCruiseVelocity(MMVel)
          .withMotionMagicAcceleration(MMAcc)
          .withMotionMagicJerk(MMJerk);

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
  private TunableNumber tunableP;
  private TunableNumber tunableI;
  private TunableNumber tunableD;
  private Consumer<Double> consumerP;
  private Consumer<Double> consumerI;
  private Consumer<Double> consumerD;
  private TunableNumber tunableMMVel;
  private TunableNumber tunableMMAcc;
  private TunableNumber tunableMMJerk;
  private Consumer<Double> consumerMMVel;
  private Consumer<Double> consumerMMAcc;
  private Consumer<Double> consumerMMJerk;

  public Elevator() {
    // TODO: set up for canivore
    currentLimit.StatorCurrentLimit = 90;
    currentLimit.SupplyCurrentLimit = 70;
    m_voltReq = new VoltageOut(0.0);
    currentLimit.SupplyCurrentLowerLimit = 40;
    currentLimit.SupplyCurrentLowerTime = 1;
    currentLimit.StatorCurrentLimitEnable = true;
    currentLimit.SupplyCurrentLimitEnable = true;
    m_sysIdElevator =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // Use default ramp rate (1 V/s)
                Volts.of(2), // Reduce dynamic step voltage to 4 to prevent brownout
                Seconds.of(3), // Use default timeout (10 s)
                // Log state with Phoenix SignalLogger class
                (state) -> SignalLogger.writeString("Elevator/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (volts) -> elevatorMotor1.setControl(m_voltReq.withOutput(volts.in(Volts))),
                // state callback is optional and defaults to null (which will log the data to a
                // normal WPILog file).
                null,
                this));
    gear3 = new DutyCycleEncoder(MotorIDConstants.gear3ID, 1.0, ElevatorConstants.gear3Offset);
    gear11 = new DutyCycleEncoder(MotorIDConstants.gear11ID, 1.0, ElevatorConstants.gear11Offset);
    elevatorMotor1 = new TalonFX(MotorIDConstants.kElevatorMotor1, Constants.RIOName);
    elevatorMotor2 = new TalonFX(MotorIDConstants.kElevatorMotor2, Constants.RIOName);
    elevatorMotor1.getConfigurator().apply(loopCfg);
    elevatorMotor1.getConfigurator().apply(elvSoftLimit);
    elevatorMotor1.getConfigurator().apply(currentLimit);
    elevatorMotor1.getConfigurator().apply(invertMotor);
    elevatorMotor1.getConfigurator().apply(elvMotionMagic);
    elevatorMotor2.setControl(new Follower(MotorIDConstants.kElevatorMotor1, true));
    elvLimitSwitch = new DigitalInput(Constants.ElevatorConstants.elvLimitID);
    new Trigger(elvLimitSwitch::get).onTrue(zeroMotorEncoder());

    // PID Tunable Numbers
    // P
    consumerP =
        newP -> {
          loopCfg.kP = newP;
          elevatorMotor1.getConfigurator().apply(loopCfg);
        };
    tunableP =
        new TunableNumber("Tunable Number P", Constants.ElevatorConstants.P, consumerP, this);
    // I
    consumerI =
        newI -> {
          loopCfg.kI = newI;
          elevatorMotor1.getConfigurator().apply(loopCfg);
        };
    tunableI =
        new TunableNumber("Tunable Number I", Constants.ElevatorConstants.I, consumerI, this);
    // D
    consumerD =
        newD -> {
          loopCfg.kD = newD;
          elevatorMotor1.getConfigurator().apply(loopCfg);
        };
    tunableD =
        new TunableNumber("Tunable Number D", Constants.ElevatorConstants.D, consumerD, this);
    // Motion Magic Tunable Numbers
    // MM Velocity
    consumerMMVel =
        newMMVel -> {
          elvMotionMagic.MotionMagicCruiseVelocity = newMMVel;
          elevatorMotor1.getConfigurator().apply(elvMotionMagic);
        };
    tunableMMVel =
        new TunableNumber(
            "Tunable Number MMVel",
            (Constants.ElevatorConstants.MMVel).in(RotationsPerSecond),
            consumerMMVel,
            this);
    // MM Acceleration
    consumerMMAcc =
        newMMAcc -> {
          elvMotionMagic.MotionMagicAcceleration = newMMAcc;
          elevatorMotor1.getConfigurator().apply(elvMotionMagic);
        };
    tunableMMAcc =
        new TunableNumber(
            "Tunable Number MMAccel",
            (Constants.ElevatorConstants.MMAcc).in(RotationsPerSecondPerSecond),
            consumerMMAcc,
            this);
    // MM Jerk (Jerk = Meters per second per second per second)
    consumerMMJerk =
        newMMJerk -> {
          elvMotionMagic.MotionMagicJerk = newMMJerk;
          elevatorMotor1.getConfigurator().apply(elvMotionMagic);
        };
    ;
    tunableMMJerk =
        new TunableNumber(
            "Tunable Number MMJerk",
            (Constants.ElevatorConstants.MMJerk).in(RotationsPerSecondPerSecond.per(Second)),
            consumerMMJerk,
            this);

    // Simulation
    if (Robot.isSimulation()) {
      simElvMotor1 = elevatorMotor1.getSimState();
      simElvMotor1.Orientation = ChassisReference.Clockwise_Positive;
      simGear3 = new DutyCycleEncoderSim(gear3);
      simGear11 = new DutyCycleEncoderSim(gear11);

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
      SmartDashboard.putData("Elevator/2d mechanism", mech);
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

  public Angle ChineseRemander() {
    double Pos3 = gear3.get() * gear1Toothing;
    double Pos11 = gear11.get() * gear2Toothing;
    return Rotations.of(
        Constants.ElevatorConstants.CRTA[(int) (Pos3)][(int) (Pos11)] + Pos3 - (int) Pos3);
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

  public Command goUp(Supplier<Double> upPower) {
    return runEnd(
        () -> {
          elevatorMotor1.set(Math.abs(upPower.get()) * elevatorOutput);
        },
        () -> elevatorMotor1.set(0));
  }

  public Command goDown(Supplier<Double> downPower) {
    return runEnd(
        () -> {
          elevatorMotor1.set(Math.abs(downPower.get()) * -elevatorOutput);
        },
        () -> elevatorMotor1.set(0));
  }

  private void disableSoftLimits() {
    elevatorMotor1
        .getConfigurator()
        .apply(
            new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(false)
                .withReverseSoftLimitEnable(false));
  }

  private void enableSoftLimits() {
    elevatorMotor1.getConfigurator().apply(elvSoftLimit);
  }

  public Command limitHit() {
    return runOnce(this::disableSoftLimits)
        .andThen(goDown(() -> 1.0).until(elvLimitSwitch::get))
        .andThen(zeroMotorEncoder())
        .andThen(runOnce(this::enableSoftLimits));
  }

  public Command zeroMotorEncoder() {
    return runOnce(
        () -> {
          // elevatorMotor1.setPosition(ChineseRemander());
          elevatorMotor1.setPosition(0);
        });
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdElevator.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdElevator.dynamic(direction);
  }

  public Command changeElevation(Distance heightLevel) {
    return runOnce(
        () -> {
          Angle adjustedSetpoint = heightToRotations(heightLevel);
          elevatorMotor1.setControl(new MotionMagicVoltage(adjustedSetpoint));
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
    SmartDashboard.putBoolean("Elevator/limit switch state", elvLimitSwitch.get());
    SmartDashboard.putNumber(
        "Elevator/Motor Encoder Rotation", elevatorMotor1.getPosition().getValue().in(Revolutions));
    SmartDashboard.putNumber("Elevator/Motor1 Percent", elevatorMotor1.get());
    SmartDashboard.putNumber("Elevator/Motor2 Percent", elevatorMotor2.get());
    SmartDashboard.putNumber("Elevator/Height (Inches)", getElevatorHeight().in(Inches));
    SmartDashboard.putNumber("Elevator/CRT", ChineseRemander().in(Rotations));
  }

  @Override
  public void simulationPeriodic() {
    m_elevatorSim.setInputVoltage(elevatorMotor1.getMotorVoltage().getValueAsDouble());
    m_elevatorSim.update(Robot.defaultPeriodSecs);
    final Distance simDist = Meters.of(m_elevatorSim.getPositionMeters());
    final LinearVelocity simVel = MetersPerSecond.of(m_elevatorSim.getVelocityMetersPerSecond());
    simElvMotor1.setRawRotorPosition(heightToRotations(simDist));
    simElvMotor1.setRotorVelocity(heightToRotations(simVel));
    simElvMotor1.setSupplyVoltage(RobotController.getBatteryVoltage());
    simGear3.set((heightToRotations(simDist).in(Rotations) / gear1Toothing) % 1);
    simGear11.set((heightToRotations(simDist).in(Rotations) / gear2Toothing) % 1);

    elevatorMech.setLength(0.1 + (simDist.in(Meters)));

    SmartDashboard.putNumber("Elevator/Sim Length", simDist.in(Inches));
    SmartDashboard.putNumber("Elevator/Sim velocity", simVel.in(InchesPerSecond));
    SmartDashboard.putNumber("Elevator/Sim Pose", m_elevatorSim.getPositionMeters());
    SmartDashboard.putNumber("Gear3", simGear3.get());
    SmartDashboard.putNumber("Gear11", simGear11.get());
  }
}
