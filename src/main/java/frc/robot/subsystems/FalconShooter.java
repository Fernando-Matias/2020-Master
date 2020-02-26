/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.utility.FalconShooterConfig;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;


public class FalconShooter extends SubsystemBase {
  /**
   * Creates a new FalconShooter.
   */
  public static final FalconShooter instance = new FalconShooter();

  public static FalconShooter getInstance(){
    return instance;
  }
  Timer timer = new Timer();

  public static FalconShooterConfig falconShooter;

  public FalconShooter() {
    falconShooter = new FalconShooterConfig(RobotMap.mFalconShooter_ID);
  }

  public void RampingSequence(){
    timer.start();
    for (double i = 0; i <= 0.5; i = i + 0.1){
      SmartDashboard.putNumber("ramping", i);
      falconShooter.set(ControlMode.PercentOutput, i);
      Timer.delay(1);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
