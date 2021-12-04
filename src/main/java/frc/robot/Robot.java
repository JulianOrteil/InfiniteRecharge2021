// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // ID Constants
  private final int FRONT_LEFT_ID = 0;
  private final int FRONT_RIGHT_ID = 2;
  private final int BACK_LEFT_ID = 1;
  private final int BACK_RIGHT_ID = 3;

  private final int RACK_AND_PINION_ID = 5;
  private final int DRAWBRIDGE_ID = 6;
  private final int INTAKE_ID = 7;

  private final int DRAWBRIDGE_UPPER_LIMIT_ID = 3;
  private final int DRAWBRIDGE_LOWER_LIMIT_ID = 5;

  private final int RACK_AND_PINION_UPPER_LIMIT_ID = 1;
  private final int RACK_AND_PINION_LOWER_LIMIT_ID = 2;

  private final int CONTROLLER_ID = 0;

  private final int CONTROLLER_LEFT_X_AXIS = 0;
  private final int CONTROLLER_LEFT_Y_AXIS = 1;

  private final int FORWARD_CAMERA_ID = 0;
  private final int INTAKE_CAMERA_ID = 1;

  // Drive train
  private WPI_TalonSRX frontLeft;
  private WPI_TalonSRX frontRight;
  private WPI_TalonSRX backLeft;
  private WPI_TalonSRX backRight;
  private WPI_TalonSRX middleMotorLeft;
    private WPI_TalonSRX middleMotorRight;

  private SpeedControllerGroup leftTrain;
  private SpeedControllerGroup rightTrain;

  private DifferentialDrive robot;

  // Other mechanisms
  private VictorSPX rackAndPinion;
  private VictorSPX drawbridge;
  private VictorSPX intake;

  // Limit switches
  private DigitalInput drawbridgeUpperLimit;
  private DigitalInput drawbridgeLowerLimit;

  private DigitalInput rackAndPinionUpperLimit;
  private DigitalInput rackAndPinionLowerLimit;

  // Controllers
  private XboxController controller;

  // Cameras
  private UsbCamera forwardCamera;
  private UsbCamera intakeCamera;

  // Input debounces
  private boolean wasIntakeTogglePressed;
  private boolean wasDrawbridgeDownPressed;
  private boolean wasDrawbridgeUpPressed;
  private boolean wasRackAndPinionDownPressed;
  private boolean wasRackAndPinionUpPressed;

  // Mechanism states
  private boolean isIntakeRunning;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // Construct drive train
    frontLeft = new WPI_TalonSRX(FRONT_LEFT_ID);
    frontRight = new WPI_TalonSRX(FRONT_RIGHT_ID);
    backLeft = new WPI_TalonSRX(BACK_LEFT_ID);
    backRight = new WPI_TalonSRX(BACK_RIGHT_ID);

    leftTrain = new SpeedControllerGroup(frontLeft, backLeft);
    rightTrain = new SpeedControllerGroup(frontRight, backRight);

    robot = new DifferentialDrive(leftTrain, rightTrain);

    // Construct other mechanisms
    rackAndPinion = new VictorSPX(RACK_AND_PINION_ID);
    drawbridge = new VictorSPX(DRAWBRIDGE_ID);
    intake = new VictorSPX(INTAKE_ID);

    // Construct limit switches
    drawbridgeUpperLimit = new DigitalInput(DRAWBRIDGE_UPPER_LIMIT_ID);
    drawbridgeLowerLimit = new DigitalInput(DRAWBRIDGE_LOWER_LIMIT_ID);

    rackAndPinionUpperLimit = new DigitalInput(RACK_AND_PINION_UPPER_LIMIT_ID);
    rackAndPinionLowerLimit = new DigitalInput(RACK_AND_PINION_LOWER_LIMIT_ID);

    // Construct controllers
    controller = new XboxController(CONTROLLER_ID);

    // Construct cameras
    forwardCamera = CameraServer.getInstance().startAutomaticCapture(FORWARD_CAMERA_ID);
    intakeCamera = CameraServer.getInstance().startAutomaticCapture(INTAKE_CAMERA_ID);

    forwardCamera.setResolution(640, 480);
    intakeCamera.setResolution(640, 480);

    // Construct input debounces
    wasIntakeTogglePressed = false;
    wasDrawbridgeDownPressed = false;
    wasDrawbridgeUpPressed = false;
    wasRackAndPinionDownPressed = false;
    wasRackAndPinionUpPressed = false;

    // Construct mechanism states
    isIntakeRunning = false;
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // Reset debounces
    wasIntakeTogglePressed = false;
    wasDrawbridgeDownPressed = false;
    wasDrawbridgeUpPressed = false;
    wasRackAndPinionDownPressed = false;
    wasRackAndPinionUpPressed = false;

    // Reset states
    isIntakeRunning = false;
  }

  @Override
  public void teleopPeriodic() {
    // Get button states
    boolean isIntakeToggledPressed = controller.getXButtonPressed();
    boolean isRackAndPinionUpPressed = controller.getYButtonPressed();
    boolean isRackAndPinionDownPressed = controller.getAButtonPressed();
    boolean isDrawbridgeDownPressed = controller.getBumperPressed(Hand.kLeft);
    boolean isDrawbridgeUpPressed = controller.getBumperPressed(Hand.kRight);

    // Get limit switch states
    boolean isDrawbridgeUpTriggered = drawbridgeUpperLimit.get();
    boolean isDrawbridgeDownTriggered = drawbridgeLowerLimit.get();
    boolean isRackAndPinionUpTriggered = rackAndPinionUpperLimit.get();
    boolean isRackAndPinionDownTriggered = rackAndPinionLowerLimit.get();

    // Get drive stick speeds
    double forwardSpeed = controller.getRawAxis(CONTROLLER_LEFT_Y_AXIS);
    double rotateSpeed = controller.getRawAxis(CONTROLLER_LEFT_X_AXIS);

    // Move the robot
    // Y-stick mimics airplane joysticks: the more you push up, the more negative
    // the value. Invert it so the opposite happens.
    robot.arcadeDrive(-forwardSpeed, rotateSpeed);

    // Run the intake
    if (isIntakeToggledPressed) {
      if (!wasIntakeTogglePressed) {
        wasIntakeTogglePressed = true; // Debounces should always be set first
        if (isIntakeRunning) {
          intake.set(ControlMode.PercentOutput, 0.0); // Stop the intake
        } else {
          intake.set(ControlMode.PercentOutput, 1.0); // Start the intake
        }
      }
    } else {
      wasIntakeTogglePressed = false; // Reset the debounce
    }

    // Drop the drawbridge
    if (isDrawbridgeDownPressed) {
      if (!wasDrawbridgeDownPressed) {
        wasDrawbridgeDownPressed = true;
        if (!isDrawbridgeDownTriggered) { // Only run the motor in this direction if we aren't at our limit!!
          drawbridge.set(ControlMode.PercentOutput, 0.5);
        } else {
          // Safe guard; ensure the motor is not moving
          drawbridge.set(ControlMode.PercentOutput, 0.0);
        }
      }
    } else {
      wasDrawbridgeDownPressed = false;
    }

    // Raise the drawbridge
    if (isDrawbridgeUpPressed) {
      if (!wasDrawbridgeUpPressed) {
        wasDrawbridgeUpPressed = true;
        if (!isDrawbridgeUpTriggered) { // Only run the motor in this direction if we aren't at our limit!!
          drawbridge.set(ControlMode.PercentOutput, -0.5);
        } else {
          // Safe guard; ensure the motor is not moving
          drawbridge.set(ControlMode.PercentOutput, 0.0);
        }
      }
    } else {
      wasDrawbridgeUpPressed = false;
    }

    // Drop the rack and pinion
    if (isRackAndPinionDownPressed) {
      if (!wasRackAndPinionDownPressed) {
        wasRackAndPinionDownPressed = true;
        if (!isRackAndPinionDownTriggered) {
          rackAndPinion.set(ControlMode.PercentOutput, 0.5);
        } else {
          // Safe guard; ensure the motor is not moving
          rackAndPinion.set(ControlMode.PercentOutput, 0.0);
        }
      }
    } else {
      wasRackAndPinionDownPressed = false;
    }

    // Raise the rack and pinion
    if (isRackAndPinionUpPressed) {
      if (!wasRackAndPinionUpPressed) {
        wasRackAndPinionUpPressed = true;
        if (!isRackAndPinionUpTriggered) {
          rackAndPinion.set(ControlMode.PercentOutput, 0.5);
        } else {
          // Safe guard; ensure the motor is not moving
          rackAndPinion.set(ControlMode.PercentOutput, 0.0);
        }
      }
    } else {
      wasRackAndPinionUpPressed = false;
    }
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }
}
