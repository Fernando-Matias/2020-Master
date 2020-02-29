/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpiutil.math.MathUtil;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.*;

import frc.robot.utility.DefaultTalonFXDrive;
import frc.robot.RobotMap;
import frc.robot.Constants;

/**
 * @author Fernando Matias
 */

public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new DriveTrain.
   */

  public static final DriveTrain instance = new DriveTrain();

  public static WPI_TalonFX mDriveLeftMaster, mDriveLeftB;
  public static WPI_TalonFX mDriveRightMaster, mDriveRightB;
  private static Solenoid mShifter_High, mShifter_Low;

  AHRS ahrs;
  public PIDController PIDTurn;

  public static DriveTrain getInstance(){
    return instance;
  }

  public double TurnRateCurved;
  public double AccelRateCurved;

  public static DifferentialDrive mDrive;


  public DriveTrain() {
  //Setting both falcons to the default fx script
  mDriveLeftMaster = new WPI_TalonFX(RobotMap.mDriveLeftA_ID);
  mDriveLeftB = new WPI_TalonFX(RobotMap.mDriveLeftB_ID);

  mDriveRightMaster = new WPI_TalonFX(RobotMap.mDriveRightA_ID);
  mDriveRightB = new WPI_TalonFX(RobotMap.mDriveRightB_ID);

  //enabling follower mode for the other  two motors
  mDriveLeftB.set(ControlMode.Follower,RobotMap.mDriveLeftA_ID);
  mDriveRightB.set(ControlMode.Follower,RobotMap.mDriveRightA_ID);

  //Left Encoder Values
  mDriveLeftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, Constants.kTimeoutms);
    mDriveLeftMaster.setSensorPhase(false);
    mDriveLeftMaster.setInverted(false);
    mDriveLeftB.setInverted(false);

  //Right Encoder Values
  mDriveRightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, Constants.kTimeoutms);
    mDriveRightMaster.setSensorPhase(false);
    mDriveRightMaster.setInverted(false);
    mDriveRightB.setInverted(false);
  
  //creating the differential drive and disabling the saftey
  mDrive = new DifferentialDrive(mDriveLeftMaster, mDriveRightMaster);
  mDrive.setSafetyEnabled(false);

  //Setting Shifter varibales 
  mShifter_Low = new Solenoid(RobotMap.PCM_A, RobotMap.pShiftLow_ID);
  mShifter_High = new Solenoid(RobotMap.PCM_B, RobotMap.pShiftHigh_ID);

  //NavX AHRS
  ahrs = new AHRS(SerialPort.Port.kMXP);

 /*  //PIDTurn
  PIDController PIDTurn = new PIDController(Constants.kTurn_P, Constants.kTurn_I, Constants.kTurn_D);

  PIDTurn.enableContinuousInput( -180.0 , 180.0);
  MathUtil.clamp(PIDTurn.calculate(mDriveLeftMaster.getSelectedSensorPosition()), -0.65, 0.65);
  MathUtil.clamp(PIDTurn.calculate(mDriveRightMaster.getSelectedSensorPosition()), -0.65, 0.65);
  PIDTurn.setTolerance(Constants.kToleranceDegrees);
  */
  } 


   public void UpShift() {
  mShifter_High.set(Constants.On);
   mShifter_Low.set(Constants.Off);
   Constants.currentGear = Constants.highGear;
  }

  public void DownShift() {
    mShifter_High.set(Constants.Off);
    mShifter_Low.set(Constants.On);
    Constants.currentGear = Constants.lowGear;
  } 
  
  public double GetLeftEncoderValue() {
    return mDriveLeftMaster.getSelectedSensorPosition(0);
  }
   
  public double GetRightEncoderValue() {
    return mDriveRightMaster.getSelectedSensorPosition(0);
  }
  
  public void setCoast() {
    mDriveLeftMaster.setNeutralMode(NeutralMode.Coast);
    mDriveLeftB.setNeutralMode(NeutralMode.Coast);
    mDriveRightMaster.setNeutralMode(NeutralMode.Coast);
    mDriveRightB.setNeutralMode(NeutralMode.Coast);
  }
  public void setBrake() {
    mDriveLeftMaster.setNeutralMode(NeutralMode.Brake);
    mDriveLeftB.setNeutralMode(NeutralMode.Brake);
    mDriveRightMaster.setNeutralMode(NeutralMode.Brake);
    mDriveRightB.setNeutralMode(NeutralMode.Brake);
  }
  
  public void EnableVoltComp() {
    mDriveLeftMaster.enableVoltageCompensation(true);
    mDriveLeftB.enableVoltageCompensation(true);
    mDriveRightMaster.enableVoltageCompensation(true);
    mDriveRightB.enableVoltageCompensation(true);
  }
  public void DisableVoltComp() {
    mDriveLeftMaster.enableVoltageCompensation(false);
    mDriveLeftB.enableVoltageCompensation(false);
    mDriveRightMaster.enableVoltageCompensation(false);
    mDriveRightB.enableVoltageCompensation(false);
  }
  public void StopDrivetrain() {
    mDriveLeftMaster.set(ControlMode.PercentOutput, 0.0);
    mDriveRightMaster.set(ControlMode.PercentOutput, 0.0);
  }
  
  public void ResetNavX() {
    ahrs.reset();
    Constants.NavxState = Constants.NavxResete;

  }

  public double getYaw() {
    return ahrs.getYaw();
  }


  public void NavX0deg() {
    PIDTurn.setSetpoint(0.0);
  }

  public void NavX90deg() {
    PIDTurn.setSetpoint(90.0);
  }

  public void NavX180deg() {
    PIDTurn.setSetpoint(179.9);
  }

  public void NavX270() {
    PIDTurn.setSetpoint(-90.0);
  }

  public void NavXOutput() {
    SmartDashboard.putNumber("getYaw", ahrs.getYaw() );
  }

  public void DriveFoward(){
    
  }

  //turning algirithm
  public void Curvature(double ThrottleAxis, double TurnAxis) {
    TurnRateCurved = (Constants.kTurnrateCurve*Math.pow(TurnAxis,3)+(1-Constants.kTurnrateCurve)*TurnAxis*Constants.kTurnrateLimit);
    AccelRateCurved = (Constants.kAccelRateCurve*Math.pow(ThrottleAxis,3)+(1-Constants.kAccelRateCurve)*ThrottleAxis*Constants.kAccelRateLimit);
    mDrive.curvatureDrive(AccelRateCurved, TurnRateCurved, true);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
