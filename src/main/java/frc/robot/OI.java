// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import utilities.OI_Utils;
import utilities.OI_Utils.Control.ControlType;

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

    // Control Curves
    private static final ControlCurve xTranslationCurve = new ControlCurve(1, 0, 0, 0.0, true);
    private static final ControlCurve yTranslationCurve = new ControlCurve(1, 0, 0, 0.0);
    public static final ControlCurve translationMagnitudeCurve = new ControlCurve(1, 0, 1, 0.1);
    public static final ControlCurve rotationCurve = new ControlCurve(0.8, 0, 1, 0, true);

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

  public static final class Keyboard {
    private static final XboxController keyboard1 = new XboxController(0);
    private static final XboxController keyboard2 = new XboxController(1);
    private static final XboxController keyboard3 = new XboxController(2);

    private static final ControlCurve defualtCurve = new ControlCurve(1, 0, 0, 0);

    public static final Control Z = new Control(1, null, null, keyboard1, ControlType.BUTTON);
    public static final Control X = new Control(2, null, null, keyboard1, ControlType.BUTTON);
    public static final Control C = new Control(3, null, null, keyboard1, ControlType.BUTTON);
    public static final Control V = new Control(4, null, null, keyboard1, ControlType.BUTTON);

    public static final Control WS =
        new Control(XboxController.Axis.kLeftX, null, keyboard1, defualtCurve);
    public static final Control AD =
        new Control(XboxController.Axis.kLeftY, null, keyboard1, defualtCurve);

    public static final Control M = new Control(1, null, null, keyboard2, ControlType.BUTTON);
    public static final Control Comma = new Control(2, null, null, keyboard2, ControlType.BUTTON);
    public static final Control Period = new Control(3, null, null, keyboard2, ControlType.BUTTON);
    public static final Control ForwardSlash =
        new Control(4, null, null, keyboard2, ControlType.BUTTON);

    public static final Control IK =
        new Control(XboxController.Axis.kLeftX, null, keyboard2, defualtCurve);
    public static final Control JL =
        new Control(XboxController.Axis.kLeftY, null, keyboard2, defualtCurve);

    public static final Control ArrowUpDown =
        new Control(XboxController.Axis.kLeftY, null, keyboard3, defualtCurve);
    public static final Control ArrowLeftRight =
        new Control(XboxController.Axis.kLeftX, null, keyboard3, defualtCurve);
  }
}
