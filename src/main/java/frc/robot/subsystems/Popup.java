/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.ControlMode;

/**
 * @author Fernando Matias
 */

public class Popup extends SubsystemBase {

  private static final Popup instance = new Popup();

  public static Popup getInstance(){
    return instance;
  }
  //Soleniod names
  public Solenoid popupUp, popupDown;

  //Creating motors
  public TalonSRX pulleyTopMotor, pulleyBottomMotor;

  public DigitalInput bottomLimit;
  public DigitalInput topLimit;
  public Relay lightRelay;
  
  public Popup() {
    popupUp = new Solenoid(RobotMap.PCM_A, RobotMap.pPopupUp_ID);
    popupDown = new Solenoid(RobotMap.PCM_B, RobotMap.pPopupDown_ID);

    bottomLimit = new DigitalInput(3);
    topLimit = new DigitalInput(2);

    

    pulleyTopMotor = new TalonSRX(RobotMap.mTopPulley_ID);
    pulleyBottomMotor = new TalonSRX(RobotMap.mBottomPulley_ID);
    lightRelay = new Relay(0);

    pulleyTopMotor.configFactoryDefault();
    pulleyTopMotor.setNeutralMode(NeutralMode.Brake);
    pulleyTopMotor.configContinuousCurrentLimit(40);
    pulleyTopMotor.configPeakCurrentLimit(0);
    pulleyTopMotor.enableCurrentLimit(true);
    pulleyTopMotor.setInverted(false);

    pulleyBottomMotor.configFactoryDefault();
    pulleyBottomMotor.setNeutralMode(NeutralMode.Brake);
    pulleyBottomMotor.configContinuousCurrentLimit(40);
    pulleyBottomMotor.configPeakCurrentLimit(0);
    pulleyBottomMotor.enableCurrentLimit(true);
    pulleyBottomMotor.setInverted(false);
  }

  public void UpBottomPulley(){
    pulleyBottomMotor.set(ControlMode.PercentOutput, 0.4);
    //Constants.bottomPulleyState = Constants.bottomPulleySpinning;
  }

  public void StopBottomPulley(){
    pulleyBottomMotor.set(ControlMode.PercentOutput, 0.0);
    //Constants.bottomPulleyState = Constants.bottomPulleyStill;
  }
  
  public void UpTopPulley(){
    pulleyTopMotor.set(ControlMode.PercentOutput, 0.5);
    //Constants.TopPulleyState = Constants.TopPulleySpinning;
  }

  public void AutoPulleyUp(){
    pulleyTopMotor.set(ControlMode.PercentOutput, 0.3);
    pulleyTopMotor.set(ControlMode.PercentOutput, 0.3);
  }
  public void AutoPulleyDown (){
    pulleyTopMotor.set(ControlMode.PercentOutput, -0.3);
    pulleyTopMotor.set(ControlMode.PercentOutput, -0.3);
  }

  public void StopTopPulley() {
    pulleyTopMotor.set(ControlMode.PercentOutput, 0.0);
    Constants.TopPulleyState = Constants.TopPulleyStill;
  }

  public void DownTopPulley(){
    pulleyTopMotor.set(ControlMode.PercentOutput, -0.2);
  }
  public void DownBottomPulley(){
    pulleyBottomMotor.set(ControlMode.PercentOutput, -0.2);
  }

  public void PopUp() {
    popupUp.set(Constants.On);
    popupDown.set(Constants.Off);
    Constants.popupState = Constants.popupStateUp;
  }

  public void PopDown(){
    popupUp.set(Constants.Off);
    popupDown.set(Constants.On);
    Constants.popupState = Constants.popupStateDown;
  }

  public void FlashlightOn(){
    lightRelay.set(Value.kForward);
    Constants.flashlightMode = Constants.flashlightOn;
  }
  public void FalshlighOff(){
    lightRelay.set(Value.kOff);
    Constants.flashlightMode = Constants.flashlightOff;
  }
  

  public void UpdateLoadState() {
    if (!topLimit.get()) {
      Constants.topLimitSwitch = Constants.topBallLoaded;
      SmartDashboard.putNumber("topLimitSwitch", Constants.topLimitSwitch);
      
    }
    else if (topLimit.get()) {
      Constants.topLimitSwitch = Constants.topBallUnloaded;
      SmartDashboard.putNumber("topLimitSwitch", Constants.topLimitSwitch);
    }
    if (!bottomLimit.get()) {
      Constants.bottomLimitSwitch = Constants.bottomBallLoaded;
      SmartDashboard.putNumber("BottomLimitSwitch", Constants.bottomLimitSwitch);
    }
    else if (bottomLimit.get()) {
      Constants.bottomLimitSwitch = Constants.bottomBallUnloaded;
      SmartDashboard.putNumber("BottomLimitSwitch", Constants.bottomLimitSwitch);
    }
  }
  public void PulleyAuto(){
    if (Constants.topLimitSwitch == Constants.topBallLoaded && Constants.bottomLimitSwitch == Constants.bottomBallLoaded){
      StopBottomPulley();
      StopTopPulley();
      Constants.ballsStaged = Constants.secondBallStaged;
      SmartDashboard.putNumber("balls staged", Constants.ballsStaged);
    }
    if(Constants.topLimitSwitch == Constants.topBallUnloaded && Constants.bottomLimitSwitch == Constants.bottomBallLoaded){
      UpTopPulley();
      UpBottomPulley();
      
    }
    if(Constants.topLimitSwitch == Constants.topBallLoaded && Constants.bottomLimitSwitch == Constants.bottomBallUnloaded ){ //&& Constants.ballsStaged == Constants.noBallStaged
      DownBottomPulley();
      DownTopPulley();
      Constants.ballsStaged = Constants.firstballStaged;
      SmartDashboard.putNumber("balls staged", Constants.ballsStaged);
    }
/*     if (Constants.topLimitSwitch == Constants.topBallUnloaded && Constants.bottomLimitSwitch == Constants.bottomBallUnloaded && Constants.ballsStaged == Constants.firstballStaged){
      Constants.ballsStaged = Constants.firstballStaged;
       popup.StopBottomPulley();
       popup.StopTopPulley();
    } */
    if(Constants.topLimitSwitch == Constants.topBallUnloaded && Constants.bottomLimitSwitch == Constants.bottomBallUnloaded){
      UpTopPulley();
      UpBottomPulley();

    }
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
