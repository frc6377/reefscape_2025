package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ElevatorConstants.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.CANIDs;
import frc.robot.Constants.DIOConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Robot;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private TalonFX elevatorMotor1;
  private TalonFXSimState simElvMotor1;
  private TalonFX elevatorMotor2;

  private TalonFXConfiguration elevatorConfig1;
  private TalonFXConfiguration elevatorConfig2;

  private DutyCycleEncoder gear3;
  private DutyCycleEncoder gear11;
  private DutyCycleEncoderSim simGear3;
  private DutyCycleEncoderSim simGear11;
  private final SysIdRoutine m_sysIdElevator;

  private Distance tuneOffset = Inches.zero();
  private Distance L2TuneOffset = Inches.zero();
  private Distance L3TuneOffset = Inches.zero();
  private Distance L4TuneOffset = Inches.zero();

  private CurrentLimitsConfigs currentLimit = new CurrentLimitsConfigs();
  private MotorOutputConfigs invertMotor =
      new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive);
  private static Mechanism2d mech = new Mechanism2d(2, 2);
  private DigitalInput elvLimitSwitch;
  private MechanismLigament2d elevatorMech;
  private MotionMagicConfigs elvMotionMagic = ElevatorConstants.kElevatorMM.getMotionMagicConfigs();

  public static final SoftwareLimitSwitchConfigs elvSoftLimit =
      new SoftwareLimitSwitchConfigs()
          .withForwardSoftLimitEnable(true)
          .withForwardSoftLimitThreshold(heightToRotations(ElevatorConstants.kTopLimit))
          .withReverseSoftLimitEnable(true)
          .withReverseSoftLimitThreshold(
              heightToRotations(Constants.ElevatorConstants.kBottomLimit));
  public static final Slot0Configs loopCfg = kElevatorPID.getSlot0Configs();

  private Pose3d elvSimPose1;
  private Pose3d elvSimPose2;

  private Distance currentSetpoint = Meter.zero();

  private ElevatorSim m_elevatorSim;

  public Elevator() {
    // TODO: set up for canivore (Do we still need this? -Jackson)
    elevatorMotor1 = new TalonFX(CANIDs.kElevatorMotor1, Constants.RIOName);
    elevatorMotor2 = new TalonFX(CANIDs.kElevatorMotor2, Constants.RIOName);

    currentLimit.StatorCurrentLimit = 120;
    currentLimit.SupplyCurrentLimit = 70;
    currentLimit.SupplyCurrentLowerLimit = 40;
    currentLimit.SupplyCurrentLowerTime = 1;
    currentLimit.StatorCurrentLimitEnable = true;
    currentLimit.SupplyCurrentLimitEnable = true;

    elevatorConfig1 = new TalonFXConfiguration();
    elevatorConfig1.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;
    elevatorConfig1.Slot0 = ElevatorConstants.kElevatorPID.getSlot0Configs();
    ElevatorConstants.kElevatorPID.createTunableNumbers("Elevator PID", this);
    elevatorConfig1.SoftwareLimitSwitch = elvSoftLimit;
    elevatorConfig1.CurrentLimits = currentLimit;
    elevatorConfig1.MotorOutput = invertMotor;
    elevatorConfig1.MotionMagic = elvMotionMagic;

    elevatorConfig2 = new TalonFXConfiguration();
    elevatorConfig2.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;
    elevatorConfig2.CurrentLimits = currentLimit;

    elevatorMotor1.getConfigurator().apply(elevatorConfig1);
    elevatorMotor2.getConfigurator().apply(elevatorConfig2);
    elevatorMotor2.setControl(new Follower(CANIDs.kElevatorMotor1, true));

    elvLimitSwitch = new DigitalInput(DIOConstants.elvLimitID);

    gear3 = new DutyCycleEncoder(DIOConstants.gear3ID, 1.0, ElevatorConstants.gear3Offset);
    gear11 = new DutyCycleEncoder(DIOConstants.gear11ID, 1.0, ElevatorConstants.gear11Offset);

    m_sysIdElevator =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(5).div(Seconds.of(1)), // Use default ramp rate (1 V/s)
                Volts.of(40), // Reduce dynamic step voltage to 4 to prevent brownout
                Seconds.of(10), // Use default timeout (10 s)
                (state) -> {
                  SignalLogger.writeString("Elevator/SysIdState", state.toString());
                  Logger.recordOutput("Elevator/SysID State", state.toString());
                }),
            new SysIdRoutine.Mechanism((volts) -> setElvCurrentFOC(volts.in(Volts)), null, this));

    // Simulation
    if (Robot.isSimulation()) {
      simElvMotor1 = elevatorMotor1.getSimState();
      simElvMotor1.Orientation = ChassisReference.CounterClockwise_Positive;
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
              false,
              0);
      elevatorMech =
          mech.getRoot("root", 1, 0)
              .append(
                  new MechanismLigament2d(
                      "Elevator Mech [0]", 1, 90, 10, new Color8Bit(Color.kPurple)));
      SmartDashboard.putData("Mech2Ds/Elevator Mech", mech);
    }

    elvSimPose1 = DrivetrainConstants.kElvStage1Pose;
    elvSimPose2 = DrivetrainConstants.kElvStage2Pose;
    Logger.recordOutput("Odometry/Mech Poses/Elv 1 Pose", elvSimPose1);
    Logger.recordOutput("Odometry/Mech Poses/Elv 2 Pose", elvSimPose2);
    Logger.recordOutput("Elevator/Elv/Setpoint (Inches)", 0.0);
    Logger.recordOutput("Elevator/Elv/Setpoint (Rotations)", 0.0);
  }

  public void setElvCurrentFOC(double amps) {
    elevatorMotor1.setControl(new TorqueCurrentFOC(amps));
  }

  public void setElvVoltage(Voltage voltage) {
    elevatorMotor1.setControl(new VoltageOut(voltage));
  }

  public static Distance rotationsToHeight(Angle rotations) {
    return ElevatorConstants.kElevatorDrumCircumference
        .times(rotations.in(Rotations))
        .div(kElevatorGearing);
  }

  // height = C * rot * 2/75 -> rot = height * 75/2C
  public static Angle heightToRotations(Distance height) {
    return height.times(kElevatorGearing).div(kElevatorDrumCircumference).times(Rotations.one());
  }

  public Angle ChineseRemander() {
    return Rotations.zero();
    // double Pos3 = Math.max(0.0, Math.min(gear3.get(), 0.999)) * gear1Toothing;
    // double Pos11 = Math.max(0.0, Math.min(gear11.get(), 0.999)) * gear2Toothing;
    // return Rotations.of(
    //     Constants.ElevatorConstants.CRTA[(int) Pos3][(int) Pos11] + Pos3 - (int) Pos3);
  }

  public static AngularVelocity heightToRotations(LinearVelocity vel) {
    // rot/s = (G)/(C*2/vel)
    return Rotations.one().times(kElevatorGearing).div((kElevatorDrumCircumference).div(vel));
  }

  public Distance getElevatorHeight() {
    return rotationsToHeight(elevatorMotor1.getPosition().getValue());
  }

  public Distance getElevatorMechHeight() {
    return Meters.of(elevatorMech.getLength());
  }

  public Trigger elevatorAtSetpoint(Distance setpoint) {
    return new Trigger(() -> getElevatorHeight().isNear(setpoint, kSetpointTolerance))
        .debounce(0.5);
  }

  public Trigger elevatorAtCurrentSetpoint() {
    return new Trigger(() -> getElevatorHeight().isNear(currentSetpoint, kSetpointTolerance))
        .debounce(0.5);
  }

  private void disableSoftLimits() {
    elevatorMotor1
        .getConfigurator()
        .apply(
            new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(false)
                .withReverseSoftLimitEnable(false));
  }

  private Distance getL0Setpoint() {
    return ElevatorConstants.kL0Height.plus(tuneOffset);
  }

  private Distance getL2Setpoint() {
    return ElevatorConstants.kL2Height.plus(tuneOffset).plus(L2TuneOffset);
  }

  private Distance getL3Setpoint() {
    return ElevatorConstants.kL3Height.plus(tuneOffset).plus(L3TuneOffset);
  }

  private Distance getL4Setpoint() {
    return ElevatorConstants.kL4Height.plus(tuneOffset).plus(L4TuneOffset);
  }

  private void enableSoftLimits() {
    elevatorMotor1.getConfigurator().apply(elvSoftLimit);
  }

  public Command elevatorUpOrDown(Supplier<Double> upPower) {
    return runEnd(
        () -> {
          elevatorMotor1.set(upPower.get() * elevatorOutput);
        },
        () -> elevatorMotor1.set(0));
  }

  public Command setElvPercent(double percentPower) {
    return runEnd(
        () -> {
          elevatorMotor1.set(percentPower);
        },
        () -> {
          elevatorMotor1.set(0);
        });
  }

  public Command limitHit() {
    return runOnce(this::disableSoftLimits)
        .andThen(setElvPercent(-0.2).until(elvLimitSwitch::get))
        .andThen(zeroMotorEncoder())
        .andThen(runOnce(this::enableSoftLimits))
        .withName("Elevator Zero");
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
          MotionMagicVoltage control = new MotionMagicVoltage(adjustedSetpoint);
          control.EnableFOC = true;
          elevatorMotor1.setControl(control);
          currentSetpoint = heightLevel;
          Logger.recordOutput("Elevator/Elv/Setpoint (Inches)", heightLevel.in(Inches));
          Logger.recordOutput("Elevator/Elv/Setpoint (Rotations)", adjustedSetpoint.in(Rotations));
        });
  }

  public Command L0() {
    return changeElevation(getL0Setpoint());
  }

  public Command L2() {
    return changeElevation(getL2Setpoint());
  }

  public Command L3() {
    return changeElevation(getL3Setpoint());
  }

  public Command L4() {
    return changeElevation(getL4Setpoint());
  }

  public Command tuneSetpoints(
      Supplier<Double> offset, Supplier<Double> L2, Supplier<Double> L3, Supplier<Double> L4) {
    return Commands.runOnce(
            () -> {
              tuneOffset = Inches.of(offset.get() * 10);
              L2TuneOffset = Inches.of(L2.get() * 10);
              L3TuneOffset = Inches.of(L3.get() * 10);
              L4TuneOffset = Inches.of(L4.get() * 10);
            })
        .alongWith(Commands.print(L2.get().toString()));
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Elevator/Motor1/Percent Out", elevatorMotor1.get());
    Logger.recordOutput(
        "Elevator/Motor1/Voltage (Volts)", elevatorMotor1.getMotorVoltage().getValue().in(Volts));
    Logger.recordOutput(
        "Elevator/Motor1/Velocity", elevatorMotor1.getVelocity().getValue().in(DegreesPerSecond));
    Logger.recordOutput(
        "Elevator/Motor1/Position (Degrees)", elevatorMotor1.getPosition().getValue().in(Degrees));
    Logger.recordOutput(
        "Elevator/Motor1/Stator Current (Amps)",
        elevatorMotor1.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput(
        "Elevator/Motor1/Supply Current (Amps)",
        elevatorMotor1.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput(
        "Elevator/Motor1/Stall Current (Amps)",
        elevatorMotor1.getMotorStallCurrent().getValueAsDouble());
    Logger.recordOutput(
        "Elevator/Motor1/Temp (Fahrenheit)",
        elevatorMotor1.getDeviceTemp().getValue().in(Fahrenheit));

    Logger.recordOutput("Elevator/Motor2/Percent Out", elevatorMotor2.get());
    Logger.recordOutput(
        "Elevator/Motor2/Voltage (Volts)", elevatorMotor2.getMotorVoltage().getValue().in(Volts));
    Logger.recordOutput(
        "Elevator/Motor2/Velocity", elevatorMotor2.getVelocity().getValue().in(DegreesPerSecond));
    Logger.recordOutput(
        "Elevator/Motor2/Position (Degrees)", elevatorMotor2.getPosition().getValue().in(Degrees));
    Logger.recordOutput(
        "Elevator/Motor1/Stator Current (Amps)",
        elevatorMotor2.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput(
        "Elevator/Motor1/Supply Current (Amps)",
        elevatorMotor2.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput(
        "Elevator/Motor1/Stall Current (Amps)",
        elevatorMotor2.getMotorStallCurrent().getValueAsDouble());
    Logger.recordOutput(
        "Elevator/Motor2/Temp (Fahrenheit)",
        elevatorMotor2.getDeviceTemp().getValue().in(Fahrenheit));

    Logger.recordOutput("Elevator/Elv/Height (Inches)", getElevatorHeight().in(Inches));

    Logger.recordOutput("Elevator/limit switch state", elvLimitSwitch.get());

    // Logger.recordOutput("Elevator/Setpoints/L0", getL0Setpoint().in(Inches));
    // Logger.recordOutput("Elevator/Setpoints/L2", getL2Setpoint().in(Inches));
    // Logger.recordOutput("Elevator/Setpoints/L3", getL3Setpoint().in(Inches));
    // Logger.recordOutput("Elevator/Setpoints/L4", getL4Setpoint().in(Inches));
    // Logger.recordOutput("Elevator/Setpoints/Overall Offset", tuneOffset);

    Logger.recordOutput(
        "Elevator/Current Command",
        this.getCurrentCommand() != null ? this.getCurrentCommand().getName() : "None");

    // Distance elvHeight = getElevatorHeight();
    // elvSimPose1 =
    //     new Pose3d(
    //         new Translation3d(
    //             elvSimPose1.getMeasureX(),
    //             elvSimPose1.getMeasureY(),
    //             DrivetrainConstants.kElvStage1Pose.getMeasureZ().plus(elvHeight.div(2))),
    //         new Rotation3d());
    // elvSimPose2 =
    //     new Pose3d(
    //         new Translation3d(
    //             elvSimPose2.getMeasureX(),
    //             elvSimPose2.getMeasureY(),
    //             DrivetrainConstants.kElvStage2Pose.getMeasureZ().plus(elvHeight)),
    //         new Rotation3d());
    // Logger.recordOutput("Odometry/Mech Poses/Elv 1 Pose", elvSimPose1);
    // Logger.recordOutput("Odometry/Mech Poses/Elv 2 Pose", elvSimPose2);
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

    elevatorMech.setLength(simDist.in(Meters));

    Logger.recordOutput("Elevator/Simulation/Length", simDist.in(Inches));
    Logger.recordOutput("Elevator/Simulation/velocity", simVel.in(InchesPerSecond));
    Logger.recordOutput("Elevator/Simulation/Pose", m_elevatorSim.getPositionMeters());
    Logger.recordOutput("Elevator/Chinese Remander/Gear3", simGear3.get());
    Logger.recordOutput("Elevator/Chinese Remander/Gear11", simGear11.get());
  }
}
