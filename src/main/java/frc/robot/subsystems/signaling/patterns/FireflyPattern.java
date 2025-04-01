package frc.robot.subsystems.signaling.patterns;

import frc.robot.subsystems.signaling.RGB;

public class FireflyPattern {
  private static final PatternNode[] pattern = {
    new PatternNode(RGB.WHITE, 10), new PatternNode(RGB.FIRE_FLY_GREEN, 10)
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
