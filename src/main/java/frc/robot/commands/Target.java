/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Turret;
import frc.robot.Constants;

public class Target extends CommandBase {
  /**
   * Creates a new Target.
   */

  Limelight limelight = Limelight.getInstance();
  Turret turret = Turret.GetInstance();

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
    if (limelight.getLimelightTarget() == 1 && limelight.getXOffsetFromTarget() > 1.0){
      turret.turretLeftTurn();
      Constants.leftSide = Constants.leftSideGood;
    }
    if (limelight.getLimelightTarget() == 1 && limelight.getXOffsetFromTarget() < 1.0){
      turret.turretRightTurn();
      Constants.rightSide = Constants.rightSideGood;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
