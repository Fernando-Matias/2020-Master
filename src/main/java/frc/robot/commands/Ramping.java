/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FalconShooter;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;

public class Ramping extends CommandBase {
  /**
   * Creates a new Ramping.
   */
  Timer timer = new Timer();
  FalconShooter falconShooter = new FalconShooter();
  double inittime;
  public Ramping() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    inittime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if((Timer.getFPGATimestamp() - inittime) >= .5){
      falconShooter.falconShooter.set(ControlMode.PercentOutput, 0.1);
    }

    if((Timer.getFPGATimestamp() - inittime) >= 1){
      falconShooter.falconShooter.set(ControlMode.PercentOutput, 0.2);
    }

    if((Timer.getFPGATimestamp() - inittime) >= 1.5){
      falconShooter.falconShooter.set(ControlMode.PercentOutput, 0.3);
    }

    if((Timer.getFPGATimestamp() - inittime) >= 2){
      falconShooter.falconShooter.set(ControlMode.PercentOutput, 0.4);
    }
    if((Timer.getFPGATimestamp() - inittime) >= 2.25){
      falconShooter.falconShooter.set(ControlMode.PercentOutput, 0.5);
    }
     
    if((Timer.getFPGATimestamp() - inittime) >= 2.5){
      falconShooter.falconShooter.set(ControlMode.PercentOutput, 0.6);
    } 

/*     if((Timer.getFPGATimestamp() - inittime) >= 2.5){
      falconShooter.falconShooter.set(ControlMode.PercentOutput, 0.5);
    } */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    falconShooter.falconShooter.set(ControlMode.PercentOutput, 0.62);
    Constants.shootState = Constants.doneRamping;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("CHECK FINISH");
    return (Timer.getFPGATimestamp() - inittime) >= 2.5;
  }
}
