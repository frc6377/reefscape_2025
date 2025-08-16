package frc.robot.subsystems.signaling.patterns;

import frc.robot.subsystems.signaling.RGB;

public class SweepPattern {
  private static final PatternNode[] pattern = {
    new PatternNode(RGB.WHITE, 2), new PatternNode(RGB.BLACK, 2)
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
