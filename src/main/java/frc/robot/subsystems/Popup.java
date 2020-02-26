/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
  
  public Popup() {
/*     popupUp = new Solenoid(RobotMap.PCM_A, RobotMap.pPopupUp_ID);
    popupDown = new Solenoid(RobotMap.PCM_B, RobotMap.pPopupDown_ID);
 */
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
    pulleyTopMotor.set(ControlMode.PercentOutput, 0.5);
    Constants.pulleyMotorTopState = Constants.pulleyMotorStateSpinning;
  }

  public void UpTopPulley(){
    pulleyBottomMotor.set(ControlMode.PercentOutput, 0.5);
    Constants.pulleyMotorState = Constants.pulleyMotorStateSpinning;
  }

  public void StopBottomPulley(){
    pulleyBottomMotor.set(ControlMode.PercentOutput, 0.0);
    Constants.pulleyMotorState = Constants.pulleyMotorStateStill;
  }

  public void StopTopPulley() {
    pulleyTopMotor.set(ControlMode.PercentOutput, 0.0);
    Constants.pulleyMotorTopState = Constants.pulleyMotorTopStateStill;
  }

  /* public void PopUp() {
    popupUp.set(Constants.On);
    popupDown.set(Constants.Off);
    Constants.popupState = Constants.popupStateUp;
  }

  public void PopDown(){
    popupUp.set(Constants.Off);
    popupDown.set(Constants.On);
    Constants.popupState = Constants.popupStateDown;
  } */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
