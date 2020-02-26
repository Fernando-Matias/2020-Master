/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

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
    //Varibles for Turning on Drivetrain
    public static double kTurnrateCurve = 0.1;
    public static double kTurnrateLimit = 0.8;

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
    public static int cpmState = cpmStateExtended;

    //Control pannel manipulator 
    public static final int NavxResete = 0;
    public static final int Navxold = 1;
    public static int NavxState = Navxold;

    //Timeout for Encoders
    public static final int kTimeoutms = 10;

    //controls 
    public static final int pulleyMotorStateSpinning = 0;
    public static final int pulleyMotorStateStill = 1;
    public static int pulleyMotorState = pulleyMotorStateStill;
    
    public static final int pulleyMotorTopStateSpinning = 0;
    public static final int pulleyMotorTopStateStill = 1;
    public static int pulleyMotorTopState = pulleyMotorTopStateStill;
     
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
    
    public static final int buttonXPressed = 0;
    public static final int buttonXPressed2 = 1;
    public static int buttonXState = buttonXPressed2;
    
    public static final int buttonYPressed = 0;
    public static final int buttonYPressed2 = 1;
    public static int buttonYState = buttonYPressed2;

    //PIDTurn
    public static final double kTurn_P = 0.2;
    public static final double kTurn_I = 0;
    public static final double kTurn_D = 0;
    public static final double kToleranceDegrees = 2.0;
}
