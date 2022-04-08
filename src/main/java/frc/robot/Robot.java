
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//Clean up and turn everything into function.
//Use pelles sketchy split arcade code in this
//Need to use variables for all values set to motors so everyting is syncronised.
package frc.robot;

import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.lang.ref.Cleaner.Cleanable;
import java.util.Scanner;

import javax.sound.midi.Receiver;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.vision.VisionPipeline;
import edu.wpi.first.vision.VisionThread;

//import org.graalvm.compiler.lir.amd64.vector.AMD64VectorShuffle.ShuffleBytesOp;

//import org.opencv.core.Mat;

public class Robot extends TimedRobot {

  // private double current_left;
  private final int kJoystickPort = 0;
  private final int kJoystickPort2 = 1;
  private final int rleadDeviceID = 1; // Front Right Motor
  private final int rfollowDeviceID = 2; // Back Right Motor
  private final int leadDeviceID = 3; // Front Left Motor
  private final int followDeviceID = 4; // Back Left Motor
  private final int shooterID = 5;// Shooter Motor
  private final int climberID = 6; // Climber Motor
  private final int intakeID = 2;
  private final int uptake1ID = 0;
  private final int uptake2ID = 1;

  private CANSparkMax shooter;

  private CANSparkMax l_leadMotor;
  private CANSparkMax l_followMotor;
  private CANSparkMax r_leadMotor;
  private CANSparkMax r_followMotor;
  private CANSparkMax climber;
  private Joystick driver_joystick;
  private Joystick shooter_joystick;

  private double angle;

  private RelativeEncoder r_encoder, l_encoder, S_Encoder, r_fEncoder, l_fEncoder, climberEconder, shooterEncoder;
  public PWMSparkMax intake = new PWMSparkMax(intakeID);
  public PWMSparkMax uptake1 = new PWMSparkMax(uptake1ID);
  public PWMSparkMax uptake2 = new PWMSparkMax(uptake2ID);

  private double speed = 1;

  private double targetAngle = 90;
  private double incval = .0005;

  // Split drive
  private double splitRight;
  private double splitLeft;

  double changeInThetaRad;

  public final ADIS16448_IMU imu = new ADIS16448_IMU(ADIS16448_IMU.IMUAxis.kZ, SPI.Port.kMXP,
      ADIS16448_IMU.CalibrationTime._1s);

  // IR Sensor
  DigitalInput ir = new DigitalInput(3);
  DigitalInput ir2 = new DigitalInput(2);

  // Odometry
  private double x;
  private double y;

  private boolean firstBreak = true;

  private double distance = 0;
  private double currentDistance = 0;

  private double rLastEcoder = 0;
  private double rCurrentDistance = 0;

  private double lLastEcoder = 0;
  private double lCurrentDistance = 0;

  private double counta;
  private double counter, startT;

  private double theta = 0;
  private double lastTheta = 0;

  private double rEncoder = 0;

  private boolean goOrNo = false;

  private double l;
  private double s = 0;
  private boolean initShoot;
  private boolean irRead = false;

  private double targetDistance = 0;
  private double xTarget = 100;
  private double yTarget = 100;
  private double dX, dY, leftSpeed, rightSpeed, smartSpeed, intspeed, yintShoot;

  private double deltaTheta;
  private double intakeCuh = 0;
  private double gyroAngle;

  private int invert = 1;

  private double limeDistance;

  // Limelight
  private double Lx;
  private double Ly;
  private double Larea;
  private double Ltv;

  // Camera
  private CvSource outputStream;

  // Grip Pipeline
  private final Object imgLock = new Object();
  public double centerX = 0.0;
  private VisionThread visionThread;

  // Diving
  private double leftOutput = 0;
  private double rightOutput = 0;
  private double leftJoyStick = 0;
  private double rightJoyStick = 0;
  private int slow = 1;

  double deadZone = 0.05;
  private double splitTurnScale = 0.35;
  private double powerScale = .7;

  private double rightCurrentPower = 0;
  private double leftCurrentPower = 0;

  // Climber
  private boolean climberOverride = false;
  private double climberBottomStopValue = -8.3;
  private double climberTopStopValue = 37;
  private double climberCalibrationValue = 0;
  private double calibratedClimberValue;

  private double climberVoltage = 0;

  private double lastREcoder;
  private double lastLEncoder;

  // Shooter
  private double rpmAdjustedSpeed = 0;
  private double adjustedAngle;
  private double autoSpeed;
  private double autoShooterSpeed = 0;
  private double testShooterSpeed = 0;
  private double rpmS = 0;
  private boolean triggerBool = true;

  private double shooterSpeed = 0;

  // Intake
  private double intakeSpeed = 0;

  // Uptake
  private double uptake1Speed = 0;
  private double uptake2Speed = 0;

  // Rumble
  private double rumble = 0;
  private double rumbleL = 0;
  // auto
  private double auton = 1;

  // Timer
  private Timer timer = new Timer();

  @Override
  public void autonomousInit() {
    timer.reset();
    timer.start();
    firstBreak = true;

    lastREcoder = r_encoder.getPosition();
    lastLEncoder = l_encoder.getPosition();
  }

  @Override
  public void autonomousPeriodic() {
    

    double rightEconder = -r_encoder.getPosition() + lastREcoder;
    double leftEncoder = l_encoder.getPosition() - lastLEncoder;
    double autoDistance = -45;
    double autoTurnSpeed;
    double turnSpeed = 0;

    // calculate turn speed from camera data
    turnSpeed = (0.00000496032) * (centerX - 280) * (centerX - 360);

    // Lime light
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv");

    Lx = tx.getDouble(0.0);
    Ly = ty.getDouble(0.0);
    Larea = ta.getDouble(0.0);
    Ltv = tv.getDouble(0.0);

    autoTurnSpeed = .001 * Math.abs(Lx);
    if (autoTurnSpeed < 0.3) {
      autoTurnSpeed = 0.3;
    }

    if (auton == 1) {
      // Calibrate climber
      calibratedClimberValue = climberCalibrationValue + climberEconder.getPosition();

      if (!ir2.get() && firstBreak) {
        climberCalibrationValue = -climberEconder.getPosition();
        firstBreak = false;
        climber.set(0);
      } else if (firstBreak) {
        climber.set(-0.2);
      } else {
        climber.set(0);
      }

      // Drive backwards
      if ((rightEconder + leftEncoder) / 2 / 10.75 * 18.5 > autoDistance) {
        r_leadMotor.set(0.3);
        l_leadMotor.set(-0.3);
      } else {
        r_leadMotor.set(0);
        l_leadMotor.set(0);
      }

      if ((rightEconder + leftEncoder) / 2 / 10.75 * 18.5 < autoDistance && timer.get() < 5 && timer.get() < 3) {
        if (Lx > 2.5) {
          r_leadMotor.set(0.05);
          l_leadMotor.set(0.05);
        } else if (Lx < -2.5) {
          r_leadMotor.set(-0.05);
          l_leadMotor.set(-0.05);
        } else {
          r_leadMotor.set(0);
          l_leadMotor.set(0);
        }
      }

      if (timer.get() < 6) {
        shooter.set(0.43);
      } else {
        shooter.set(0);
      }

      if (timer.get() > 4 && timer.get() < 5 && (rightEconder + leftEncoder) / 2 / 10.75 * 18.5 < autoDistance) {
        uptake2.set(0.5);
      } else {
        uptake2.set(0);
      }

    } else if(auton == 2){
      calibratedClimberValue = climberCalibrationValue + climberEconder.getPosition();

      if (!ir2.get() && firstBreak) {
        climberCalibrationValue = -climberEconder.getPosition();
        firstBreak = false;
        climber.set(0);
      } else if (firstBreak) {
        climber.set(-0.2);
      } else {
        climber.set(0);
      }

      if (timer.get() < 1) {
        uptake1.set(0.6);
        uptake2.set(-0.6);
      } else if (timer.get() > 1 && timer.get() < 3) {
        uptake1.set(-0.5);
        if (ir.get()) {
          uptake2.set(0.5);
        } else {
          uptake2.set(0);
        }
        intake.set(0.8);
      } else if (timer.get() > 7.5 && timer.get() < 7.9) {
        uptake2.set(0.5);
      } else if (timer.get() > 7.9 && timer.get() < 11) {
        uptake2.set(0);
      } else if (timer.get() > 11 && timer.get() < 12){
        uptake2.set(0.5);
      }
      else {
        uptake1.set(0);
        uptake2.set(0);
        intake.set(0);
      }


      // shooter
      if (timer.get() < 13 && timer.get() > 5) {
        shooter.set(0.431);
      } else {
        shooter.set(0);
      }

      // Drive
      if ((rightEconder + leftEncoder) / 2 / 10.75 * 18.5 < -autoDistance && timer.get() < 2.5) {
        r_leadMotor.set(-0.3);
        l_leadMotor.set(0.3);
      } else if (timer.get() > 3 && timer.get() < 4) {
        r_leadMotor.set(0.28);
        l_leadMotor.set(0.28);
      } else if (timer.get() > 4 && timer.get() < 6) {
        double lessgo = .0006*(Lx*Lx)-.002;
        if (Lx > 0) {
          l_leadMotor.set(lessgo);
          r_leadMotor.set(lessgo);
        } else if (Lx < 0) {
          l_leadMotor.set(-lessgo);
          r_leadMotor.set(-lessgo);
        } else {
          l_leadMotor.set(0);
          r_leadMotor.set(0);
        }
      } else {
        r_leadMotor.set(0);
        l_leadMotor.set(0);
      }

    }else{
      calibratedClimberValue = climberCalibrationValue + climberEconder.getPosition();

      if (!ir2.get() && firstBreak) {
        climberCalibrationValue = -climberEconder.getPosition();
        firstBreak = false;
        climber.set(0);
      } else if (firstBreak) {
        climber.set(-0.2);
      } else {
        climber.set(0);
      }

      if (timer.get() < 1) {
        uptake1.set(0.75);
        uptake2.set(-0.75);
      } else if (timer.get() > 1 && timer.get() < 3) {
        uptake1.set(-0.5);
        if (ir.get()) {
          uptake2.set(0.5);
        } else {
          uptake2.set(0);
        }
        intake.set(0.8);
      } else if (timer.get() > 7.5 && timer.get() < 7.9) {
        uptake2.set(0.5);
      } else if (timer.get() > 7.9 && timer.get() < 11) {
        uptake2.set(0);
      } else if (timer.get() > 11 && timer.get() < 12){
        uptake2.set(0.5);
      }
      else {
        uptake1.set(0);
        uptake2.set(0);
        intake.set(0);
      }


      // shooter
      if (timer.get() < 13 && timer.get() > 5) {
        shooter.set(0.39);
      } else {
        shooter.set(0);
      }

      // Drive
      if ((rightEconder + leftEncoder) / 2 / 10.75 * 18.5 < 30 && timer.get() < 2.5) {
        r_leadMotor.set(-0.22);
        l_leadMotor.set(0.22);
      } else if (timer.get() > 3 && timer.get() < 4) {
        r_leadMotor.set(0.28);
        l_leadMotor.set(0.28);
      } else if (timer.get() > 4 && timer.get() < 6) {
        double lessgo = .0006*(Lx*Lx)-.002;
        if (Lx > 0) {
          l_leadMotor.set(lessgo);
          r_leadMotor.set(lessgo);
        } else if (Lx < 0) {
          l_leadMotor.set(-lessgo);
          r_leadMotor.set(-lessgo);
        } else {
          l_leadMotor.set(0);
          r_leadMotor.set(0);
        }
      } else {
        r_leadMotor.set(0);
        l_leadMotor.set(0);
      }
    }

    auton = SmartDashboard.getNumber("auton", 0);
    SmartDashboard.putNumber("rightAutoEncoder", rightEconder);
    SmartDashboard.putNumber("leftAutoEncoder", leftEncoder);
    SmartDashboard.putNumber("backauto", (rightEconder + leftEncoder) / 2 / 10.75 * 18.5);

    SmartDashboard.putNumber("cex", centerX);
    SmartDashboard.putNumber("time", timer.get());
    SmartDashboard.putNumber("ballturnSpeed", autoSpeed);
    SmartDashboard.putNumber("autoDistance", (rightEconder + leftEncoder) / 2 / 10.75 * 18.5);    

  }

  @Override
  public void robotInit() {
    SmartDashboard.putNumber("auton", 0);
    // startLogs("fake");
    // Camera
    UsbCamera camera = CameraServer.startAutomaticCapture();
    camera.setResolution(640, 480);
    CameraServer.startAutomaticCapture();

    r_leadMotor = new CANSparkMax(rleadDeviceID, MotorType.kBrushless);
    r_followMotor = new CANSparkMax(rfollowDeviceID, MotorType.kBrushless);

    l_leadMotor = new CANSparkMax(leadDeviceID, MotorType.kBrushless);
    l_followMotor = new CANSparkMax(followDeviceID, MotorType.kBrushless);
    climber = new CANSparkMax(climberID, MotorType.kBrushless);
    shooter = new CANSparkMax(shooterID, MotorType.kBrushless);

    l_leadMotor.restoreFactoryDefaults();
    l_followMotor.restoreFactoryDefaults();
    r_leadMotor.restoreFactoryDefaults();
    r_followMotor.restoreFactoryDefaults();
    climber.restoreFactoryDefaults();
    shooter.restoreFactoryDefaults();

    l_followMotor.follow(l_leadMotor);
    r_followMotor.follow(r_leadMotor);

    S_Encoder = shooter.getEncoder();
    r_encoder = r_leadMotor.getEncoder();
    l_encoder = l_leadMotor.getEncoder();
    r_fEncoder = r_followMotor.getEncoder();
    l_fEncoder = l_followMotor.getEncoder();
    climberEconder = climber.getEncoder();
    SmartDashboard.putNumber("xInput", 0);
    SmartDashboard.putNumber("yInput", 0);
    SmartDashboard.putNumber("shooterSpeed", .2);
    SmartDashboard.putNumber("Yint", .3687);
    SmartDashboard.putNumber("intakeSpeed", .5);
    SmartDashboard.putNumber("AUTOSPEED", 0);
    SmartDashboard.putNumber("testShooterSpeed", 0);

    driver_joystick = new Joystick(kJoystickPort);
    shooter_joystick = new Joystick(kJoystickPort2);

  }

  @Override
  public void teleopPeriodic() {

    // Launch pad to hoop 19.5 feet
    // autoSpeed = (Math.pow(limeDistance, x) + (limeDistance * x) + x);

    driving();
    climing();
    uptakes();
    intake();
    Limelight();
    shooter();
    smartDashboardUpdate();

    // Driving
    r_leadMotor.set(-leftOutput * powerScale);
    l_leadMotor.set(rightOutput * powerScale);

    // Climber
    climber.setVoltage(climberVoltage);

    // Shooter
    shooter.set(shooterSpeed);

    // Intake
    intake.set(intakeSpeed);

    // Uptake
    uptake1.set(uptake1Speed);
    uptake2.set(uptake2Speed);

    // Rumble
    driver_joystick.setRumble(RumbleType.kLeftRumble, rumble);
    shooter_joystick.setRumble(RumbleType.kLeftRumble, rumbleL);
  }

  public double applyDeadZone(double in) {
    if (in <= deadZone && in >= -1 * deadZone) {
      return 0;
    }
    return in;
  }

  private void driving() {

    // Slow Speed
    if (driver_joystick.getRawButtonReleased(3)) {
      slow *= -1;
    }
    if (slow == 1) {
      powerScale = 0.9;
    } else if (slow == -1) {
      powerScale = 0.5;
    }
    leftOutput = 0;
    rightOutput = 0;
    // Split arcade
    leftJoyStick = applyDeadZone(driver_joystick.getRawAxis(1) * -1);
    // if (leftJoyStick != 0) {
    // leftJoyStick = Math.cbrt(leftJoyStick);
    // //leftJoyStick = leftJoyStick;
    // }
    rightJoyStick = applyDeadZone(driver_joystick.getRawAxis(4));
    // if (rightJoyStick != 0) {
    // rightJoyStick = Math.cbrt(rightJoyStick);

    // }
    leftOutput = leftJoyStick - rightJoyStick * splitTurnScale;
    rightOutput = leftJoyStick + rightJoyStick * splitTurnScale;
    double powerDiff = 0;
    if (leftOutput > 1 || rightOutput > 1) {
      powerDiff = Math.abs(Math.max(leftOutput, rightOutput) - 1);
      leftOutput -= powerDiff;
      rightOutput -= powerDiff;
    } else if (leftOutput < -1 || rightOutput < -1) {
      powerDiff = Math.abs(Math.min(leftOutput, rightOutput) + 1);
      leftOutput += powerDiff;
      rightOutput += powerDiff;
    }

    if (rightCurrentPower + 0.05 < rightOutput) {
      rightOutput += 0.05;
    } else if (rightCurrentPower - 0.05 > rightOutput) {
      rightOutput -= 0.05;
    }

    if (leftCurrentPower + 0.05 < leftOutput) {
      leftOutput += 0.05;
    } else if (leftCurrentPower - 0.05 > leftOutput) {
      leftOutput -= 0.05;
    }

    if (rightOutput > 1) {
      rightOutput = 1;
    } else if (rightOutput < -1) {
      rightOutput = -1;
    }

    if (leftOutput > 1) {
      leftOutput = 1;
    } else if (leftOutput < -1) {
      leftOutput = -1;
    }
  }

  private void Limelight() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv");

    // read values periodically
    Lx = tx.getDouble(0.0);
    Ly = ty.getDouble(0.0);
    Larea = ta.getDouble(0.0);
    Ltv = tv.getDouble(0.0);
    SmartDashboard.putNumber("ooks", centerX);

    // post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", Lx);
    SmartDashboard.putNumber("LimelightY", Ly);
    SmartDashboard.putNumber("LimelightArea", Larea);
    SmartDashboard.putNumber("Distance (lime)", limeDistance);

    counter = System.currentTimeMillis();

    // Limelight distance
    if (Ltv == 1) {
      limeDistance = (104 - 39) / Math.tan(Math.toRadians(20 + Ly));
    }
  }

  private void climing() {

    calibratedClimberValue = climberCalibrationValue + climberEconder.getPosition();

    if (!ir2.get() && firstBreak) {

      climberCalibrationValue = -climberEconder.getPosition();
      firstBreak = false;
    } else {

      // Rubble
      if (calibratedClimberValue > 21) {
        rumble = (calibratedClimberValue - 21) / 15;
      } else {
        rumble = 0;
      }

      // Normal climbing
      if (driver_joystick.getRawAxis(2) >= 0.1 && calibratedClimberValue > climberBottomStopValue
      && driver_joystick.getPOV() == 0) {
        climberVoltage = driver_joystick.getRawAxis(2) * -12;
      }  else if (driver_joystick.getRawAxis(2) >= 0.1 && calibratedClimberValue > -4) {
        climberVoltage = driver_joystick.getRawAxis(2) * -3;
      }else if (driver_joystick.getRawAxis(3) >= 0.1 && calibratedClimberValue < climberTopStopValue) {
        climberVoltage = driver_joystick.getRawAxis(3) * 6;
      } else if (calibratedClimberValue < -4 && calibratedClimberValue > -4.5) {
        climberVoltage = 3;
      } else {
        climberVoltage = 0;
      }

      // Overide toggle
      if (driver_joystick.getRawButton(8) && !climberOverride) {
        climberBottomStopValue = -100;
        climberTopStopValue = 100;
        climberOverride = true;
      } else if (driver_joystick.getRawButton(8) && climberOverride) {
        // -5 stopping point(don't go lower)
        climberBottomStopValue = -8.3;
        climberTopStopValue = 37;
        climberOverride = false;
      }

      // Resest IR stop value calibration
      if (driver_joystick.getRawButton(7)) {
        firstBreak = true;
      }
    }
    // }

    if (shooter_joystick.getRawButton(7)) {
      firstBreak = true;
    }
  }

  private void uptakes() {
    // Uptake1

    // Shoot Button
    if (shooter_joystick.getRawAxis(3) >= 0.1 && !ir.get() && triggerBool) {
      startT = System.currentTimeMillis();
      rpmS = S_Encoder.getVelocity();
      // intakeCuh = 0;
      triggerBool = false;
    } else if (!triggerBool && shooter_joystick.getRawAxis(3) >= 0.1) {
      triggerBool = false;
    } else {
      triggerBool = true;
    }

    if (counter - startT <= 400) {
      s = .5;
    } else if (counter - startT > 400 && !ir.get()) {
      s = 0;
      startT = 0;
    } else if (counter - startT > 400 && ir.get()) {
      s = 0;
      startT = 0;
    }

    // Joystick
    if (shooter_joystick.getRawAxis(1) > 0.1 || shooter_joystick.getRawAxis(1) < -0.1) {
      uptake1Speed = shooter_joystick.getRawAxis(1) * .7;
    } else {
      uptake1Speed = intakeCuh;
    }

    // Uptake2
    if (shooter_joystick.getRawAxis(5) < -0.1 || shooter_joystick.getRawButton(3) && ir.get()) {
      uptake2Speed = 0.4;
    } else if (shooter_joystick.getRawAxis(5) > 0.1) {
      uptake2Speed = -0.4;

    } else {
      // Why??????????????
      uptake2Speed = s;
    }

  }

  private void intake() {
    if (shooter_joystick.getRawButton(6) || driver_joystick.getRawButton(6)) {
      intakeSpeed = 0.95;
      intakeCuh = -.6;
    } else if (shooter_joystick.getRawButton(5) || driver_joystick.getRawButton(5)) {
      intakeCuh = 0;
      intakeSpeed = -.35;
    } else {
      intakeCuh = 0;
      intakeSpeed = 0;
    }

  }

  private void shooter() {

    

    // Auto aim
    if (driver_joystick.getRawButton(4)) {
      if (Lx > 2.5) {
        leftOutput = -0.05;
        rightOutput = 0.05;
      } else if (Lx < -2.5) {
        leftOutput = 0.05;
        rightOutput = -0.05;
      } else {
        leftOutput = 0;
        rightOutput = 0;
      }
    }

    // Limelight autospeedd
    autoShooterSpeed = -.026798 * (limeDistance * limeDistance) + (25.2379 * limeDistance) - 632.833;

    // D-Pad set points
    if (shooter_joystick.getPOV() == 0) {
      shooterSpeed = 0.431;
      if(S_Encoder.getVelocity() > 2150){
        rumbleL = 1;
      }

    }else if (shooter_joystick.getPOV() == 90) {
      shooterSpeed = 0.45;
      if(S_Encoder.getVelocity() > 2200){
        rumbleL = 1;
      }

    } else if (shooter_joystick.getPOV() == 180) {
      shooterSpeed = 0.2;
      if(S_Encoder.getVelocity() > 900 && S_Encoder.getVelocity() < 1000){
        rumbleL = 1;
      }

    } else if (shooter_joystick.getRawAxis(2) >= 0.1) {
      shooterSpeed = shooter_joystick.getRawAxis(2);

    } else if (shooter_joystick.getPOV() == 270) {
      shooterSpeed = autoShooterSpeed / 5000;

    } else {
      shooterSpeed = testShooterSpeed;
      rumbleL = 0;
    }

  }

  private void smartDashboardUpdate() {

    // 10 Axis Gyro
    SmartDashboard.putNumber("Angle", angle);
    SmartDashboard.putNumber("Adjusted Angle", adjustedAngle);
    SmartDashboard.putNumber("acceleration", imu.getYFilteredAccelAngle());
    SmartDashboard.putNumber("climbVal", calibratedClimberValue);
    SmartDashboard.putBoolean("climbIr", ir2.get());
    SmartDashboard.putBoolean("shootIr", ir.get());

    SmartDashboard.putBoolean("fBrake", firstBreak);
    SmartDashboard.putBoolean("go", goOrNo);

    // Encoders
    SmartDashboard.putNumber("right motor power", splitRight);
    SmartDashboard.putNumber("left motor power", splitLeft);

    SmartDashboard.putNumber("Right Wheels", r_encoder.getVelocity() * -1);
    SmartDashboard.putNumber("Left Wheels", l_encoder.getVelocity());

    SmartDashboard.putNumber("rEncoder", (r_encoder.getPosition() / 12.75 * 18 * -1));
    SmartDashboard.putNumber("lEncoder", l_encoder.getPosition() / 12.75 * 18);
    SmartDashboard.putNumber("distance", distance);
    SmartDashboard.putNumber("targetAngle", targetAngle);
    SmartDashboard.putNumber("Climb Encoder", climberEconder.getPosition());
    SmartDashboard.putNumber("FINAL RPM", rpmS);
    SmartDashboard.putNumber("tSp", speed);

    // smartSpeed = SmartDashboard.getNumber("shooterSpeed", .2);
    SmartDashboard.putNumber("Shooter RPM", S_Encoder.getVelocity());
    SmartDashboard.putNumber("Target RPM", autoShooterSpeed);
    SmartDashboard.putNumber("woah", rpmAdjustedSpeed);
    SmartDashboard.putBoolean("trigger Bool", triggerBool);
    yintShoot = SmartDashboard.getNumber("Yint", .3687);
    intspeed = SmartDashboard.getNumber("intakeSpeed", .5);
    autoShooterSpeed = SmartDashboard.getNumber("WHEELSPEED", 0);
    testShooterSpeed = SmartDashboard.getNumber("testShooterSpeed", 0);
    // auton = SmartDashboard.getNumber("auton", 0);
  }
}