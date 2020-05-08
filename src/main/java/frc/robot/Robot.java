/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpiutil.net.PortForwarder;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;

import frc.robot.Input.*;
import frc.robot.Constants;

import frc.robot.OI;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ControlPanelManipulator;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.FalconShooter;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Popup;


import frc.robot.Auto.AutoCommands.*;
import frc.robot.Auto.AutoPaths;
import frc.robot.Auto.BasicAuto;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Limelight;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */

 /**
 * @author Fernando Matias
 */

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public AHRS ahrs;
  private static OI m_oi;

  

  Limelight limelight = Limelight.getInstance();
  Popup popup = Popup.getInstance();
  Intake intake = Intake.getInstance();
  DriveTrain driveTrain = DriveTrain.getInstance();
  ControlPanelManipulator cpm = ControlPanelManipulator.GetInstance();
  AutoPaths paths = AutoPaths.getInstance();

  Climber climber = Climber.getInstance();
  FalconShooter falconShooter = FalconShooter.getInstance();

  Command autonomousCommand;
  SendableChooser<Command> autoProgram = new SendableChooser<>();
  Turret turret = Turret.getInstance();

  //private RobotContainer m_robotContainer;
  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.

    //m_robotContainer = new RobotContainer();
    m_oi = new OI();
    m_oi.registerControls();
    
    PortForwarder.add(5800, "limelight.local", 5800);
    PortForwarder.add(5801, "limelight.local", 5801);
    PortForwarder.add(5805, "limelight.local", 5805);
    
    popup.PopDown();
    intake.RetractIntake();
    //turret.TurretStayStill();

    
    driveTrain.UpShift();
    popup.FalshlighOff();
    driveTrain.ClimberShifterOff();

    climber.ClimberNotReady();

    //autoProgram.setDefaultOption("PathA", new PathFollower(paths.getFirstPath()));
    //autoProgram.setDefaultOption("PathB", new PathFollower(paths.getSecondPath()));

    autoProgram.setDefaultOption("basic", new BasicAuto());

    CameraServer.getInstance().startAutomaticCapture();

    SmartDashboard.putData("Selected Auto", autoProgram);

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    
    CommandScheduler.getInstance().run();
    limelight.LimelightOutput();
    driveTrain.NavXOutput();
    popup.UpdateLoadState();
    SmartDashboard.putNumber("Relay", Constants.flashlightMode);
    SmartDashboard.putNumber("Tracking ", Constants.trackState);

    // Creates UsbCamera and MjpegServer [1] and connects them
    //CameraServer.getInstance().startAutomaticCapture();

    // Creates the CvSink and connects it to the UsbCamera
    //CvSink cvSink = CameraServer.getInstance().getVideo();

    // Creates the CvSource and MjpegServer [2] and connects them
    //CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 640, 480);

    
    //CameraServer.getInstance().startAutomaticCapture();
    //CvSink cvSink = CameraServer.getInstance().getVideo();
    //CvSource outputStream = CameraServer.getInstance().putVideo("blur", 640, 480);
    
    
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  BasicAuto auto;
  @Override
  public void autonomousInit() {
    //m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    //schedule the autonomous command (example)
    //autonomousCommand = autoProgram.getSelected();
    auto = new BasicAuto();
    auto.schedule();

    if (m_autonomousCommand != null) {
     // m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    driveTrain.setCoast();
    popup.PopDown();
    intake.RetractIntake();
    falconShooter.StopShootingCells();
    driveTrain.UpShift();
    cpm.StopSpinControlPanel();
    Constants.ClimbMode = Constants.ClimbModeOff;
    Constants.triggerState = Constants.triggerNotPressed;
    turret.TurretStayStill();


  }

  /** 
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
        // Driver Orientation
        driveTrain.Curvature(OI.getLeftThrottleInput(), OI.getRightSteeringInputInverted());

/*     
      if (Constants.DriverOrientation == Constants.FrontOrientation) {
      
    }
    else if (Constants.DriverOrientation == Constants.BackOrientation) {
      driveTrain.Curvature(OI.getLeftThrottleInputInverted(), OI.getRightSteeringInputInverted());
    } */

    //turret.UseManualInput();

    // Turret Control
/*     if (Constants.TurretAimState == Constants.TurretAimStateManual) {
      turret.UseManualInput();
    }

    else if (Constants.TurretAimState == Constants.TurretAimStateAuto) {
      
    }  */

    //driveTrain.Curvature(OI.getLeftThrottleInput(), OI.getRightSteeringInputInverted());

    if (Constants.climbState == Constants.climbResting){
      climber.ClimberMotor.set(ControlMode.PercentOutput, OI.controllerThrottleInput());
    }


  } 
  

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
