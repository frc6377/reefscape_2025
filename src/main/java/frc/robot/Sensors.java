package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static frc.robot.Constants.SensorIDs.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IntakeConstants.CoralEnum;
import utilities.TOFSensorSimple;
import utilities.TOFSensorSimple.TOFType;

public class Sensors {
  private TOFSensorSimple sensor2 =
      new TOFSensorSimple(kSensor2ID, Inches.of(1), TOFType.LASER_CAN);
  private TOFSensorSimple sensor3 =
      new TOFSensorSimple(kSensor3ID, Inches.of(1), TOFType.LASER_CAN);
  private TOFSensorSimple sensor4 =
      new TOFSensorSimple(kSensor4ID, Inches.of(1), TOFType.LASER_CAN);
  private TOFSensorSimple scorerSensor =
      new TOFSensorSimple(kScorerSensorID, Inches.of(1), TOFType.LASER_CAN);

  public CoralEnum simState = CoralEnum.NO_CORAL;

  public Sensors() {}

  public CoralEnum getSensorState() {
    if (Robot.isSimulation()) {
      return simState;
    }

    if (scorerSensor.isBeamBroke()) {
      return CoralEnum.IN_ELEVATOR;
    }
    int state = sensor2.isBeamBroke() ? 1 : 0;
    state += sensor3.isBeamBroke() ? 10 : 0;
    state += sensor4.isBeamBroke() ? 100 : 0;
    SmartDashboard.putNumber("Laser Can State", state);
    switch (state) {
      case 000:
        return CoralEnum.NO_CORAL;
      case 001:
        return CoralEnum.CORAL_TOO_CLOSE;
      case 011:
        return CoralEnum.DONE;
      case 010:
        return CoralEnum.CORAL_TOO_FAR;
      case 110:
        return CoralEnum.CORAL_TOO_FAR;
      case 100:
        return CoralEnum.CORAL_TOO_FAR;
      case 101:
        return CoralEnum.NO_CORAL;
      case 111:
        return CoralEnum.NO_CORAL;
      default:
        return CoralEnum.NO_CORAL;
    }
  }

  public void setSimState(CoralEnum state) {
    simState = state;
  }
}
