package frc.robot.subsystems.signaling.patterns;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.signaling.RGB;
import java.util.NoSuchElementException;

public class AlliancePattern {
  private static RGB allianceColor = RGB.BLUE;

  static {
    try {
      if (DriverStation.getAlliance().get() == Alliance.Red) {
        allianceColor = RGB.RED;
      }
    } catch (NoSuchElementException e) {
      allianceColor = RGB.BLUE;
    }
  }

  private static final PatternNode[] pattern = {
    new PatternNode(RGB.WHITE, 10), new PatternNode(allianceColor, 10)
  };
  private static int patternLength;

  static {
    for (PatternNode p : pattern) {
      patternLength += p.repeat;
    }
  }

  public static int getPatternLength() {
    return patternLength;
  }

  public static PatternNode[] getPattern() {
    return pattern;
  }
}
