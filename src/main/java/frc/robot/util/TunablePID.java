package frc.robot.util;

public class TunablePID {
  private double kP;
  private double kI;
  private double kD;

  public TunablePID(double kP, double kI, double kD) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
  }

  public void withP(double newKp) {
    kP = newKp;
  }

  public void withI(double newKi) {
    kI = newKi;
  }

  public void withD(double newKd) {
    kD = newKd;
  }
}
