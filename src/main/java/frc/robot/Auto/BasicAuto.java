/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Popup;
import frc.robot.subsystems.FalconShooter;
import frc.robot.subsystems.DriveTrain;

import java.sql.Time;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class BasicAuto extends CommandBase {
  /**
   * Creates a new BasicAuto.
   */

  DriveTrain driveTrain = DriveTrain.getInstance();
  FalconShooter falconShooter = FalconShooter.getInstance();
  Popup popup = Popup.getInstance();

  public BasicAuto() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.mDrive.arcadeDrive(.5, .0);
    Timer.delay(1);
    driveTrain.mDrive.arcadeDrive(0.0, 0.0);
    Timer.delay(0.2);
    driveTrain.NavX40deg();
    Timer.delay(0.2);
    popup.PopUp();
    falconShooter.RampingSequence();

    Timer.delay(.1);
    popup.UpBottomPulley();
    popup.UpTopPulley();
    Timer.delay(3);
    falconShooter.StopShootingCells();
    popup.StopBottomPulley();
    popup.StopTopPulley();



    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
