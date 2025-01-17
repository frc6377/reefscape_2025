// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import utilities.OI_Utils;

/** Add your docs here. */
public class OI extends OI_Utils {

  public static final class Driver {
    public static final XboxController controller = new XboxController(driverJoystickPort);
    // A B Y X Buttons
    public static final Control A = new Control(XboxController.Button.kA, "", controller);
    public static final Control B = new Control(XboxController.Button.kB, "", controller);
    public static final Control Y = new Control(XboxController.Button.kY, "", controller);
    public static final Control X = new Control(XboxController.Button.kX, "", controller);

    // Bumpers & Triggers
    public static final Control LTrigger =
        new Control(XboxController.Axis.kLeftTrigger, "", controller, 0.5);
    public static final Control RTrigger =
        new Control(XboxController.Axis.kRightTrigger, "", controller, 0.5);
    public static final Control LBumper =
        new Control(XboxController.Button.kLeftBumper, "", controller);
    public static final Control RBumper =
        new Control(XboxController.Button.kRightBumper, "", controller);

    // Start, End & Left/Right stick buttons
    public static final Control Start = new Control(XboxController.Button.kStart, "", controller);
    public static final Control Back = new Control(XboxController.Button.kBack, null, controller);
    public static final Control LSB =
        new Control(XboxController.Button.kLeftStick, null, controller);
    public static final Control RSB =
        new Control(XboxController.Button.kRightStick, null, controller);

    // POV Buttons
    public static final Control POV0 = new Control(0, "", controller);
    public static final Control POV90 = new Control(90, "", controller);
    public static final Control POV180 = new Control(180, "", controller);
    public static final Control POV270 = new Control(270, "", controller);

    // Control Curves
    private static final ControlCurve xTranslationCurve = new ControlCurve(1, 0, 1, 0.01, true);
    private static final ControlCurve yTranslationCurve = new ControlCurve(1, 0, 1, 0.01);
    public static final ControlCurve translationMagnitudeCurve = new ControlCurve(1, 0, 1, 0.01);
    public static final ControlCurve rotationCurve = new ControlCurve(1, 0, 1, 0.01, true);

    // Joystick Axes
    public static final Control LeftX =
        new Control(XboxController.Axis.kLeftX, "", controller, xTranslationCurve);
    public static final Control LeftY =
        new Control(XboxController.Axis.kLeftY, "", controller, yTranslationCurve);
    public static final Control RightX =
        new Control(XboxController.Axis.kRightX, "", controller, rotationCurve);
    public static final Control RightY =
        new Control(XboxController.Axis.kRightY, null, controller, null);

    public static void setRumble(double rumbleIntensity) {
      controller.setRumble(RumbleType.kBothRumble, rumbleIntensity);
    }
  }

  public static final class Operator {
    public static final XboxController controller = new XboxController(operatorJoystickPort);

    // A B Y X Buttons
    public static final Control A = new Control(XboxController.Button.kA, "", controller);
    public static final Control B = new Control(XboxController.Button.kB, "", controller);
    public static final Control X = new Control(XboxController.Button.kX, "", controller);
    public static final Control Y = new Control(XboxController.Button.kY, "", controller);

    // Bumpers & Triggers
    public static final Control LTrigger =
        new Control(XboxController.Axis.kLeftTrigger, "", controller, 0.5);
    public static final Control RTrigger =
        new Control(XboxController.Axis.kRightTrigger, "", controller, 0.5);
    public static final Control LBumper =
        new Control(XboxController.Button.kLeftBumper, "", controller);
    public static final Control RBumper =
        new Control(XboxController.Button.kRightBumper, "", controller);

    // Start, End & Left/Right stick buttons
    public static final Control Start = new Control(XboxController.Button.kStart, "", controller);
    public static final Control LeftStick =
        new Control(XboxController.Button.kLeftStick, null, controller);
    public static final Control RightStick =
        new Control(XboxController.Button.kRightStick, null, controller);
    public static final Control Back = new Control(XboxController.Button.kBack, null, controller);

    // Control Curves
    private static final ControlCurve xTranslationCurve = new ControlCurve(1, 0, 0, 0.0);
    private static final ControlCurve yTranslationCurve = new ControlCurve(1, 0, 0, 0.0);
    public static final ControlCurve translationMagnitudeCurve = new ControlCurve(1, 0, 1, 0.0);
    public static final ControlCurve rotationCurve = new ControlCurve(0.8, 0, 1, 0.0, true);

    // Joystick Axes
    public static final Control LeftX =
        new Control(XboxController.Axis.kLeftX, "", controller, xTranslationCurve);
    public static final Control LeftY =
        new Control(XboxController.Axis.kLeftY, "", controller, yTranslationCurve);
    public static final Control RightX =
        new Control(XboxController.Axis.kRightX, "", controller, rotationCurve);
    public static final Control RightY =
        new Control(XboxController.Axis.kRightY, null, controller, null);

    public static void setRumble(double rumbleIntensity) {
      controller.setRumble(RumbleType.kBothRumble, rumbleIntensity);
    }
  }
}
