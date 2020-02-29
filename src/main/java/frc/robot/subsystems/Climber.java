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

public class Climber extends SubsystemBase {
  /**
   * Creates a new Climber.
   */
    //callable now
  private static final Climber instance = new Climber();

  public static Climber getInstance() {
    return instance;
  }

  public TalonSRX ClimberMotor ;


  public Climber() {
    ClimberMotor = new TalonSRX(RobotMap.mClimberMotor_ID);

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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
