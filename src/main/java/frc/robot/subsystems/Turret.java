/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.RobotMap;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Input.LogitechController;
import frc.robot.Constants;

public class Turret extends SubsystemBase {

  //calling limelight Class
  //Limelight limelight = Limelight.getInstance();
  //creating Turret Class
  private static final LogitechController Gamepad = new LogitechController(RobotMap.mGamepadPort);
  private static final Turret instance = new Turret();
  

  public static Turret GetInstance() {
    return instance;
  }

  //declaring servo name
  public Servo TurretServo;
  public Servo HoodServo;
  
  public static Turret getInstance(){
    return instance;
  }


  public Turret(){
    TurretServo = new Servo(RobotMap.mTurretServo_ID);
    HoodServo = new Servo(RobotMap.mHoodServo_ID);
  } 

  public void turretRightTurn(){
    TurretServo.setSpeed(1.0);
    TurretServo.set(0.0);

  }
  public void TurretStayStill(){
    TurretServo.set(0.5);
  }

  public void turretLeftTurn(){
    TurretServo.setSpeed(0.0);
    TurretServo.set(1.0);
  }

  public void HoodOut(){
    HoodServo.setSpeed(0.0);
    HoodServo.set(1.0);
  }

  public void HoodIn(){
    HoodServo.setSpeed(1.0);
    HoodServo.set(0.0);
  }

  public void UseManualInput() {
    if ( Gamepad.getRightXAxis()  >= -0.25 && Gamepad.getRightXAxis() <= 0.25 ){
      TurretServo.set(0.5);
     }
    else if (Gamepad.getRightXAxis() > 0.25 ){
    TurretServo.set(1.0);
     }
    else if (Gamepad.getRightXAxis() < -0.25){
    TurretServo.set(0.0);

    if (Gamepad.getRightYAxis() > 0) {
      HoodServo.set(1.0);
    }
    else if (Gamepad.getRightYAxis() < 0) {
      HoodServo.set(0.0);
    }
    }  

  

  }
 // deadband

}

  

 // @Override
  //public void periodic() {
    // This method will be called once per scheduler run
  //}

