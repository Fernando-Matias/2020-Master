/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.Input.*;
import frc.robot.commands.*;
//import frc.robot.subsystems.NeoShooter;

/**
 * @author Fernando Matias
 */
public class OI {
    private static final LogitechAttack3Joystick LeftStick = new LogitechAttack3Joystick(RobotMap.mLeftStickPort);
    private static final LogitechAttack3Joystick RightStick = new LogitechAttack3Joystick(RobotMap.mRightStickPort);
    private static final LogitechController Gamepad = new LogitechController(RobotMap.mGamepadPort);

    public static void registerControls(){

      //LeftStick.getButton2().whenPressed(new IntakePowerCells());
      LeftStick.getButton3().whenPressed(new TargetOff());
      Gamepad.getRightBumper().whenPressed(new ClimberModeOn());
      Gamepad.getRightBumper().whenInactive(new ClimberModeOff());
      //LeftStick.getButton4().whenHeld(new BottomPulley());
      RightStick.getButtonTrigger().whenPressed(new TopPulley());
      RightStick.getButtonTrigger().whenInactive(new StopTopPulley());

      RightStick.getButton4().whenPressed(new ShiftLow());
      RightStick.getButton4().whenInactive(new ShiftHigh());
      RightStick.getButton6().whenPressed(new ptoEngaged());
      RightStick.getButton7().whenPressed(new ptoOff());
      
      

      //RightStick.getButton2().whenPressed(new ManualPopup());
      RightStick.getButton3().whenPressed(new Target());
      //RightStick.getButton4().whenHeld(new TopPulley());
      //RightStick.getButton3().whenPressed(new DriverBackOrientation());
      LeftStick.getButton11().whenPressed(new ClimberReady());
      LeftStick.getButton10().whenPressed(new ClimberNotReady());
      LeftStick.getButtonTrigger().whenPressed(new Ramping());
      //LeftStick.getButtonTrigger().whenPressed(new ShootPowerCell());
      
      LeftStick.getButtonTrigger().whenInactive(new StopShootPowerCell());

      //Gamepad.getButtonBack().whenPressed(new AutoTurretMode());
      //Gamepad.getButtonStart().whenPressed(new ManualTurretMode());
      //Gamepad.getButtonY().whenHeld(new ClimberGoUp());
      //Gamepad.getButtonX().whenHeld(new ClimberGoDown());
      Gamepad.getButtonA().whenPressed(new IntakePowerCells());
      Gamepad.getButtonA().whenPressed(new PulleyAutomated());

      
      Gamepad.getButtonY().whenPressed(new SpinCPMTimed());
      Gamepad.getButtonX().whenPressed(new Backfeed());
      Gamepad.getButtonX().whenInactive(new StopBackfeed());

      //Gamepad.getButtonX().whenPressed(new ClimberGoUp());
      //Gamepad.getButtonX().whenInactive(new Climbstop());

      //Gamepad.getButtonB().whenPressed(new ClimberGoDown());
      //Gamepad.geotButtonB().whenInactive(new Climbstop());

      Gamepad.getLeftBumper().whenPressed(new BottomPulley());
      Gamepad.getLeftBumper().whenInactive(new StopBottomPully());  
      //Gamepad.getButtonBack().whenPressed(new ShootingModeFar());
      //Gamepad.getButtonStart().whenPressed(new ShootingModeNear());
    
      //Gamepad.getRightBumper().whenPressed(new TopPulley());


    }

    public static double controllerThrottleInput(){
      return Gamepad.getLeftY();
    }
    public static double controllerSteeringInput(){
      return Gamepad.getRightX();
    }
    public static double controllerThrottleInputInverted(){
      return Gamepad.getLeftYAxisInverted();
    }
    public static double controllerSteeringInputInverted(){
      return Gamepad.getRightXAxisInverted();
    }

    
    public static double getLeftThrottleInput() {
      return LeftStick.getYAxis();
    }
    public static double getRightThrottleInput() {
      return RightStick.getYAxis(); 
    }
    public static double getLeftSteeringInput() {
      return LeftStick.getXAxis();
    }
    public static double getRightSteeringInput() {
      return RightStick.getXAxis();
    }
    public static double getLeftThrottleInputInverted() {
      return LeftStick.getYAxisInverted();
    }
    public static double getRightThrottleInputInverted() {
      return RightStick.getYAxisInverted();
    }
    public static double getLeftSteeringInputInverted() {
      return LeftStick.getXAxisInverted();
    }
    public static double getRightSteeringInputInverted() {
      return RightStick.getXAxisInverted();
    }

}
 
    
