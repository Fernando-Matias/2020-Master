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

import frc.robot.Constants;
import frc.robot.RobotMap;

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
  
  public Popup() {
    popupUp = new Solenoid(RobotMap.PCM_A, RobotMap.pPopupUp_ID);
    popupDown = new Solenoid(RobotMap.PCM_B, RobotMap.pPopupDown_ID);

    bottomLimit = new DigitalInput(3);
    topLimit = new DigitalInput(2);

    pulleyTopMotor = new TalonSRX(RobotMap.mTopPulley_ID);
    pulleyBottomMotor = new TalonSRX(RobotMap.mBottomPulley_ID);

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
    pulleyBottomMotor.set(ControlMode.PercentOutput, 0.5);
    Constants.bottomPulleyState = Constants.bottomPulleySpinning;
  }

  public void StopBottomPulley(){
    pulleyBottomMotor.set(ControlMode.PercentOutput, 0.0);
    Constants.bottomPulleyState = Constants.bottomPulleyStill;
  }
  
  public void UpTopPulley(){
    pulleyTopMotor.set(ControlMode.PercentOutput, 0.5);
    Constants.TopPulleyState = Constants.TopPulleySpinning;
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

  public void UpdateLoadState() {
    if (!topLimit.get()) {
      Constants.topLimitSwitch = Constants.topBallLoaded;
    }
    else if (topLimit.get()) {
      Constants.topLimitSwitch = Constants.topBallUnloaded;
    }
    if (!bottomLimit.get()) {
      Constants.bottomLimitSwitch = Constants.bottomBallLoaded;
    }
    else if (bottomLimit.get()) {
      Constants.bottomLimitSwitch = Constants.bottomBallUnloaded;
    }
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
