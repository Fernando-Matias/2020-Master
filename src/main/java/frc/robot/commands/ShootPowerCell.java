/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FalconShooter;
import frc.robot.Constants;

public class ShootPowerCell extends CommandBase {
  /**
   * Creates a new ShootPowerCell.
   */

  FalconShooter falconShooter = FalconShooter.getInstance();

  public ShootPowerCell() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
 /*     if (Constants.RampingState == Constants.RampingnotReset) { */
      falconShooter.RampingSequence();
      Timer.delay(.5);
      falconShooter.ShootPowerCell();
/*       Constants.RampingState = Constants.RampingReset;
    }
    else {

    } */

    
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