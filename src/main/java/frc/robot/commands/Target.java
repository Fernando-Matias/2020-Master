/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Turret;
import frc.robot.Constants;

public class Target extends CommandBase {
  /**
   * Creates a new Target.
   */

  Limelight limelight = Limelight.getInstance();
  Turret turret = Turret.GetInstance();
  DriveTrain driveTrain = DriveTrain.getInstance();

  public Target() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Constants.trackState == Constants.trackOFF){
      limelight.setOn();
      driveTrain.setBrake();
      limelight.SteeringAdjust();
    }
      
    
/* 
    if (limelight.getLimelightTarget() == 1 && limelight.getXOffsetFromTarget() > 1.0 && Constants.TurretAimState == Constants.TurretAimStateAuto){
      turret.turretLeftTurn();
    }
    if (limelight.getLimelightTarget() == 1 && limelight.getXOffsetFromTarget() < 1.0){
      turret.turretRightTurn();
    }

    if (limelight.getXOffsetFromTarget() > 1.0 && limelight.getXOffsetFromTarget() < 1.0 && limelight.getLimelightTarget() == 1){
      Constants.rightSide = Constants.rightSideGood;
      Constants.leftSide = Constants.leftSideGood;
    } */

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Constants.trackState == Constants.trackOFF){
      return true;
    }
    else{
      return false;
    }
  }
}
