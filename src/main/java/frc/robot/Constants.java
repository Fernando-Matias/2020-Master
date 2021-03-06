/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//import frc.robot.subsystems.Limelight.LimelightConstants;
import frc.robot.utility.geometry.Pose2d;
import frc.robot.utility.geometry.Translation2d;
import frc.robot.utility.geometry.Rotation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

 /**
 * @author Fernando Matias
 */

public final class Constants {

    //looper 
    public static final double kLooperDt = 0.01;

    //Varibles for Turning on Drivetrain
    public static double kTurnrateCurve = 0.4;
    public static double kTurnrateLimit = 0.8;

    //Variables for throttle axis
    public static double kAccelRateCurve = 0.4; // test variables
    public static double kAccelRateLimit = 0.8; // test variables

    //yes just yes (easier this way)
    public static final boolean On = true;
    public static final boolean Off = false;

    //pnuematics on driveTrain
    public static final int highGear = 0;
    public static final int lowGear = 1;
    public static int currentGear = highGear;

    //if intake is extended or retreacted
    public static final int intakeStateExtended = 0;
    public static final int intakeStateRetracted = 1;
    public static int intakeState = intakeStateRetracted;

    //if popup mechanism is up or down
    public static final int popupStateUp = 0;
    public static final int popupStateDown = 1;
    public static int popupState = popupStateDown; 

    //Control pannel manipulator 
    public static final int cpmStateRetracted = 0;
    public static final int cpmStateExtended = 1;
    public static int cpmState = cpmStateRetracted;

    //NavX State
    public static final int NavxResete = 0;
    public static final int Navxold = 1;
    public static int NavxState = Navxold;

    //Climbing button states
    public static final int climbing = 0;
    public static final int climbResting = 1;
    public static final int pullingRobotUp = 2;
    public static int climbState = climbResting;

    //Ramping Shooter State
    public static final int RampingReset = 0;
    public static final int RampingnotReset = 1;
    public static int RampingState = RampingnotReset;

    //Orientation State 
    public static final int ClimbModeOff = 0;
    public static final int ClimbModeOn = 1;
    public static int ClimbMode = ClimbModeOff;

    //Manual or Auto Aiming State 
    public static final int TurretAimStateManual = 0;
    public static final int TurretAimStateAuto = 1;
    public static int TurretAimState = TurretAimStateAuto;

    //Hood State
    public static final int HoodOut = 0;
    public static final int HoodIn = 1;
    public static int HoodState = TurretAimStateAuto;

    //Timeout for Encoders
    public static final int kTimeoutms = 10;

    public static double kDriveHeadingVelocityKp = 4.0; // 6.0;
    public static double kDriveHeadingVelocityKi = 0.0;
    public static double kDriveHeadingVelocityKd = 50.0;

    //yes 
    public static boolean rampingcompete = false;

    public static double kRatioFactor = 43;

    //for Drivetrain
    public static int kDriveVelocityAllowableError = 0;

    public static final int notDoneRamping = 0;
    public static final int doneRamping = 1;
    public static int shootState = notDoneRamping;

    //controls 
    public static final int trackON = 0;
    public static final int trackOFF = 1;
    public static int trackState = trackOFF; 

    public static final int leftSideGood = 0;
    public static final int leftSideBad = 1;
    public static int leftSide = leftSideBad;

    public static final int rightSideGood = 0;
    public static final int rightSideBad = 1;
    public static int rightSide = rightSideBad;

    public static final int bottomPulleyStill = 0;
    public static final int bottomPulleySpinning = 1;
    public static int bottomPulleyState = bottomPulleyStill;

    public static final int autopulley = 0;
    public static final int manualPulley = 1;
    public static int pulleystate = manualPulley;

    public static final int flashlightOn = 0;
    public static final int flashlightOff = 1;
    public static int flashlightMode = flashlightOff;
    
    public static final int TopPulleySpinning = 0;
    public static final int TopPulleyStill = 1;
    public static int TopPulleyState = TopPulleyStill;
     
    public static final int intakeMotorSpinning = 0;
    public static final int intakeMotorStill = 1;
    public static int intakeMotorState = intakeMotorStill;
    
    public static final int neoShooterSpin = 0;
    public static final int neoShooterStill = 1;
    public static int neoShooterState = neoShooterStill;
    
    public static final int driveBaseOrintationFront = 0;
    public static final int driveBaseOrintationBack = 1;
    public static int driveBaseOrintation = driveBaseOrintationFront;
    
    public static final int turretMotorSpin = 0;
    public static final int turretMotorStill = 1;
    public static int turretMotorState = turretMotorStill;
    
    public static final int buttonAPressed = 0;
    public static final int buttonAPressed2 = 1;
    public static int buttonAState = buttonAPressed2;
    
    public static final int buttonBPressed = 0;
    public static final int buttonBPressed2 = 1;
    public static int buttonBState = buttonBPressed2;
    
    public static final int triggerPressed = 0;
    public static final int triggerNotPressed = 1;
    public static int triggerState = triggerNotPressed;
    
    public static final int buttonYPressed = 0;
    public static final int buttonYPressed2 = 1;
    public static int buttonYState = buttonYPressed2;

    public static final int topBallLoaded = 0;
    public static final int topBallUnloaded = 1;
    public static int topLimitSwitch = topBallUnloaded;

    public static final int bottomBallLoaded = 0;
    public static final int bottomBallUnloaded = 1;
    public static int bottomLimitSwitch = bottomBallUnloaded;

    public static boolean WantBall = false;

    public static final int firstballStaged = 1;
    public static final int noBallStaged = 0;
    public static final int secondBallStaged = 3;
    public static int ballsStaged = noBallStaged;


    //difficult things
    public static double kTrackLengthInches = 8.265;
    public static double kTrackWidthInches = 23.8;
    public static double kTrackEffectiveDiameter = (kTrackWidthInches * kTrackWidthInches
            + kTrackLengthInches * kTrackLengthInches) / kTrackWidthInches;
    public static double kTrackScrubFactor = 0.5;
    public static double kDriveWheelDiameterInches = 6.25;


    // limelight
    public static final double kHorizontalFOV = 59.6; // degrees
    public static final double kVerticalFOV = 49.7; // degrees
    public static final double kVPW = 2.0 * Math.tan(Math.toRadians(kHorizontalFOV / 2.0));
    public static final double kVPH = 2.0 * Math.tan(Math.toRadians(kVerticalFOV / 2.0));
    public static final double kImageCaptureLatency = 11.0 / 1000.0; // seconds
    
    public static final double kMaxTrackerDistance = 9.0;
    public static final double kMaxGoalTrackAge = 2.5;
    public static final double kMaxGoalTrackAgeNotTracking = 0.1;
    public static final double kMaxGoalTrackSmoothingTime = 0.5;
    public static final double kTrackStabilityWeight = 0.0;
    public static final double kTrackAgeWeight = 10.0;
    public static final double kTrackSwitchingWeight = 100.0;
    
    public static final double kCameraFrameRate = 90.0;
    public static final double kMinStability = 0.5;
    public static final int kPortPipeline = 0;
    public static final int kBallPipeline = 2;
    public static final double kPortTargetHeight = 39.125;
    public static final double kHatchTargetHeight = 31.5;

    public static final double kTurretToArmOffset = -2.5;  // in
    public static final double kWristToTremorsEnd = 15.75;  // in

 /*    //Limelight
    public static final LimelightConstants kLimelightConstants = new LimelightConstants();
    static {
        kLimelightConstants.kName = "Limelight";
        kLimelightConstants.kTableName = "limelight";
        kLimelightConstants.kHeight = 7.221;  // inches
        kLimelightConstants.kTurretToLens = new Pose2d(new Translation2d(-1.293, 2.556), Rotation2d.fromDegrees(0.0));
        kLimelightConstants.kHorizontalPlaneToLens = Rotation2d.fromDegrees(-24.0);
    } */

    public static final double kMaxTopLimelightHeight = 16.0;

    public static final double kInertiaSteeringGain = 0.0; // angular velocity command is multiplied by this gain *
    // our speed
    // in inches per sec
/*     public static final double kPathFollowingMaxAccel = 80.0;  // inches per second ^ 2
    public static final double kPathFollowingMaxVel = 120.0; // inches per second
    public static final double kPathFollowingProfileKp = 0.3 / 12.0;  // % throttle per inch of error
    public static final double kPathFollowingProfileKi = 0.0;
    public static final double kPathFollowingProfileKv = 0.01 / 12.0;  // % throttle per inch/s of error
    public static final double kPathFollowingProfileKffv = 0.003889;  // % throttle per inch/s
    public static final double kPathFollowingProfileKffa = 0.001415;  // % throttle per inch/s^2
    public static final double kPathFollowingProfileKs = 0.1801 / 12.0;  // % throttle
    public static final double kPathFollowingGoalPosTolerance = 3.0;
    public static final double kPathFollowingGoalVelTolerance = 12.0;
    public static final double kPathStopSteeringDistance = 12.0;
    public static final double kDriveVoltageRampRate = 0.0;
    public static final int kDriveCurrentThrottledLimit = 30; // amps
    public static final int kDriveCurrentUnThrottledLimit = 80; // amps */

    public static double kPathFollowingLookahead = 72.0; // inches
    public static double kPathFollowingMaxVel = 120.0; // inches/sec
    public static double kPathFollowingMaxAccel = 50.0; // inches/sec^2
    

    //PIDTurn
    public static final double kTurn_P = 0.00172;
    public static final double kTurn_I = 0;
    public static final double kTurn_D = 0;
    public static final double kToleranceDegrees = 2.0;
}
