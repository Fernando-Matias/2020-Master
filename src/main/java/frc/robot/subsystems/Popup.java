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
  public TalonSRX pulleyMotor;
  
  public Popup() {
/*     popupUp = new Solenoid(RobotMap.PCM_A, RobotMap.pPopupUp_ID);
    popupDown = new Solenoid(RobotMap.PCM_B, RobotMap.pPopupDown_ID);
 */
    pulleyMotor.configFactoryDefault();
    pulleyMotor.setNeutralMode(NeutralMode.Brake);
    pulleyMotor.configContinuousCurrentLimit(40);
    pulleyMotor.configPeakCurrentLimit(0);
    pulleyMotor.enableCurrentLimit(true);
    pulleyMotor.setInverted(false);
  }

  public void UpThePulley(){
    pulleyMotor.set(ControlMode.PercentOutput, 1.0);
  }

  public void StopThePulley() {
    pulleyMotor.set(ControlMode.PercentOutput, 0.0);
  }

/*   public void PopUp() {
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
