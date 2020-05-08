/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;



import edu.wpi.first.wpilibj.Servo;
import frc.robot.Input.*;

public class Climber extends SubsystemBase {
  /**
   * Creates a new Climber.
   */
    //callable now
  private static final Climber instance = new Climber();

  public static Climber getInstance() {
    return instance;
  }

  //declaring servo name
  public Servo letfServo;
  public Servo rightServo;

  public TalonSRX ClimberMotor ;


  public Climber() {
    ClimberMotor = new TalonSRX(RobotMap.mClimberMotor_ID);

    letfServo = new Servo(1);
    rightServo = new Servo(0);

    ClimberMotor.configFactoryDefault();
    ClimberMotor.setNeutralMode(NeutralMode.Brake);
    ClimberMotor.configContinuousCurrentLimit(40);
    ClimberMotor.configPeakCurrentLimit(0);
    ClimberMotor.enableCurrentLimit(true);
    ClimberMotor.setInverted(false);
    
  }

  public void ClimberGoUp() {
    ClimberMotor.set(ControlMode.PercentOutput, -0.6);
    Constants.climbState = Constants.climbing;
  }

  public void ClimberStop(){
    ClimberMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public void ClimberGoDown() {
    ClimberMotor.set(ControlMode.PercentOutput, 0.6);
    Constants.climbState = Constants.pullingRobotUp;
  }

  public void ClimberInPosition(){
    rightServo.setSpeed(0.5);
    rightServo.set(0.7);
    
    letfServo.setSpeed(0.5);
    letfServo.set(0.3);
  }
  public void ClimberNotReady(){
    rightServo.setSpeed(0.5);
    rightServo.set(0.4);

    letfServo.setSpeed(0.5);
    letfServo.set(0.5);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
