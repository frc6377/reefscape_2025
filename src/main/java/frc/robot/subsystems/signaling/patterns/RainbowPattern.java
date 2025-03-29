package frc.robot.subsystems.signaling.patterns;

import frc.robot.subsystems.signaling.RGB;

public class RainbowPattern {
  private static final PatternNode[] pattern = {
    new PatternNode(RGB.RED, 10),
    new PatternNode(RGB.ORANGE, 10),
    new PatternNode(RGB.YELLOW, 10),
    new PatternNode(new RGB(255 / 2, 255, 0), 10),
    new PatternNode(RGB.GREEN, 10),
    new PatternNode(new RGB(0, 255, 255 / 2), 10),
    new PatternNode(new RGB(0, 255, 255), 10),
    new PatternNode(new RGB(0, 255 / 2, 255), 10),
    new PatternNode(RGB.BLUE, 10),
    new PatternNode(RGB.PURPLE, 10),
    new PatternNode(new RGB(255, 0, 255), 10),
    new PatternNode(new RGB(255, 0, 255 / 2), 10)
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
