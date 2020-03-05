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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpiutil.math.MathUtil;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.*;

import frc.robot.utility.DefaultTalonFXDrive;
import frc.robot.utility.PurePursuit.AdaptivePurePursuitController;
import frc.robot.utility.PurePursuit.Kinematics;
import frc.robot.utility.PurePursuit.Path;
import frc.robot.utility.PurePursuit.RigidTransform2d;
import frc.robot.utility.PurePursuit.SynchronousPID;
import frc.robot.RobotMap;
import frc.robot.RobotState;
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

  private double TurnrateCurved, mLastHeadingErrorDegrees, leftvelo_,  rightvelo_, left_distance, right_distance, time;

  //Velocity & base lock control for K
  protected static final int kVelocityControlSlot = 0;

    //PID
  private static AdaptivePurePursuitController pathFollowingController_;
  private static SynchronousPID velocityHeadingPid_;

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

  //PIDTurn
  PIDController PIDTurn = new PIDController(Constants.kTurn_P, Constants.kTurn_I, Constants.kTurn_D);

  PIDTurn.enableContinuousInput( -180.0 , 180.0);
  MathUtil.clamp(PIDTurn.calculate(mDriveLeftMaster.getSelectedSensorPosition()), -0.65, 0.65);
  MathUtil.clamp(PIDTurn.calculate(mDriveRightMaster.getSelectedSensorPosition()), -0.65, 0.65);
  PIDTurn.setTolerance(Constants.kToleranceDegrees);
  

    //PID setting
    velocityHeadingPid_ = new SynchronousPID(Constants.kDriveHeadingVelocityKp, Constants.kDriveHeadingVelocityKi, Constants.kDriveHeadingVelocityKd);
    velocityHeadingPid_.setOutputRange(-30, 30);

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

  public void NavX40deg(){
    if (getYaw() < -40 && getYaw() > -35){
      mDrive.arcadeDrive(0.0, 0.0);
    }
    else if (getYaw() > -40){
      mDrive.arcadeDrive(0.0, 0.2);
    }
    else if (getYaw() < -35){
      mDrive.arcadeDrive(0.0, 0.2);
    }
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
  

  public void followPath(Path path, boolean reversed) {
    configureTalonsForSpeedControl();
    velocityHeadingPid_.reset();
    pathFollowingController_ = new AdaptivePurePursuitController(Constants.kPathFollowingLookahead,
      Constants.kPathFollowingMaxAccel, Constants.kLooperDt, path, reversed, 0.25);
    updatePathFollower();
    }
    private void configureTalonsForSpeedControl() {
      mDriveLeftMaster.set(ControlMode.Velocity, 0.0);
      mDriveLeftMaster.selectProfileSlot(kVelocityControlSlot, 0);
      mDriveLeftMaster.configAllowableClosedloopError(0, Constants.kDriveVelocityAllowableError);
      mDriveRightMaster.set(ControlMode.Velocity, 0.0);
      mDriveRightMaster.selectProfileSlot(kVelocityControlSlot, 0);
      mDriveRightMaster.configAllowableClosedloopError(0, Constants.kDriveVelocityAllowableError);
      setBrake();
    }

    public void updateVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
      leftvelo_ = inchesPerSecondToVelo(left_inches_per_sec);
      rightvelo_ = inchesPerSecondToVelo(right_inches_per_sec);
      mDriveLeftMaster.set(ControlMode.Velocity, -leftvelo_);
      mDriveRightMaster.set(ControlMode.Velocity, rightvelo_);
      SmartDashboard.putNumber("leftvelo", leftvelo_);
      SmartDashboard.putNumber("rightvelo", rightvelo_);
    }

    public void updatePathFollower() {
      RigidTransform2d robot_pose = RobotState.getInstance().getLatestFieldToVehicle().getValue();
      RigidTransform2d.Delta command = pathFollowingController_.update(robot_pose, Timer.getFPGATimestamp());
      Kinematics.DriveVelocity setpoint = Kinematics.inverseKinematics(command);
    
      // Scale the command to respect the max velocity limits
      double max_vel = 0.0;
      max_vel = Math.max(max_vel, Math.abs(setpoint.left));
      max_vel = Math.max(max_vel, Math.abs(setpoint.right));
      if (max_vel > Constants.kPathFollowingMaxVel) {
          double scaling = Constants.kPathFollowingMaxVel / max_vel;
          setpoint = new Kinematics.DriveVelocity(setpoint.left * scaling, setpoint.right * scaling);
      }
      updateVelocitySetpoint(setpoint.left, setpoint.right);
      SmartDashboard.putNumber("setpoint.left", setpoint.left);
      SmartDashboard.putNumber("setpoint.right", setpoint.right);
    }

    private double inchesPerSecondToVelo(double inches_per_second) {
      return inches_per_second * Constants.kRatioFactor;
    }

    // 		/* Set acceleration and vcruise velocity for motion magic - see documentation */
		// _talon.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
    // _talon.configMotionAcceleration(6000, Constants.kTimeoutMs);
    
//     		/* Configure Sensor Source for Pirmary PID */
// 		_talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx,
//     Constants.kTimeoutMs);

// /* set deadband to super small 0.001 (0.1 %).
//   The default deadband is 0.04 (4 %) */
// _talon.configNeutralDeadband(0.001, Constants.kTimeoutMs);

		// /* Configure Sensor Source for Pirmary PID */
		// _talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx,
		// 		Constants.kTimeoutMs);

    	/**
	 * Decide if positive motor-output/sensor-velocity should be when motor spins
	 * clockwise or counter-clockwise.
	 */
  //final TalonFXInvertType kInvertType = TalonFXInvertType.CounterClockwise; // <<< What direction you want "forward/up" to be.
  
  	/** electic brake during neutral */
  //final NeutralMode kBrakeDurNeutral = NeutralMode.Coast;
  
  		/* get the selected sensor for PID0 */
      // double appliedMotorOutput = _talon.getMotorOutputPercent();
      // int selSenPos = _talon.getSelectedSensorPosition(0); /* position units */
      // int selSenVel = _talon.getSelectedSensorVelocity(0); /* position units per 100ms */
  
      // /* scaling depending on what user wants */
      // double pos_Rotations = (double) selSenPos / kUnitsPerRevolution;
      // double vel_RotPerSec = (double) selSenVel / kUnitsPerRevolution * 10; /* scale per100ms to perSecond */
      // double vel_RotPerMin = vel_RotPerSec * 60.0;
  

    //differentail drive
    // double leftVelUnitsPer100ms = _leftFront.getSelectedSensorVelocity(0);
    // double rghtVelUnitsPer100ms = _rghtFront.getSelectedSensorVelocity(0);

    
        // /*
        //  * drive motor at least 25%, Talons will auto-detect if sensor is out of phase
        //  */
        // _leftFront.getFaults(_faults_L);
        // _rghtFront.getFaults(_faults_R);

        // if (_faults_L.SensorOutOfPhase) {
        //     work += " L sensor is out of phase";
        // }
        // if (_faults_R.SensorOutOfPhase) {
        //     work += " R sensor is out of phase";
        // }

        // /* print to console if btn1 is held down */
        // if (btn1) {
        //     System.out.println(work);
        // }

        // _rghtFront.configFactoryDefault();
        // _rghtFollower.configFactoryDefault();
        // _leftFront.configFactoryDefault();
        // _leftFollower.configFactoryDefault();


    /* [3] flip values so robot moves forward when stick-forward/LEDs-green */
    //_rghtFront.setInverted(TalonFXInvertType.Clockwise); // !< Update this
    //_leftFront.setInverted(TalonFXInvertType.CounterClockwise); // !< Update this

        /* get gamepad stick values */
        // double forw = -1 * _joystick.getRawAxis(1); /* positive is forward */
        // double turn = +1 * _joystick.getRawAxis(2); /* positive is right */
        // boolean btn1 = _joystick.getRawButton(1); /* is button is down, print joystick values */
          //   /* deadband gamepad 10% */
          //   if (Math.abs(forw) < 0.10) {
          //     forw = 0;
          // }
          // if (Math.abs(turn) < 0.10) {
          //     turn = 0;
          // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
