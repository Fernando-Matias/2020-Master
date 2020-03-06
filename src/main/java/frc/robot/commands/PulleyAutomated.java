/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Popup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

public class PulleyAutomated extends CommandBase {
  /**
   * Creates a new PulleyAutomated.
   */

  Popup popup = Popup.getInstance();
  Timer timer = new Timer();
  double inittime;

  public PulleyAutomated() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Constants.ballsStaged = Constants.noBallStaged;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //popup.UpdateLoadState();
    //popup.PulleyAuto();
    popup.UpdateLoadState();
    if (Constants.topLimitSwitch == Constants.topBallLoaded && Constants.bottomLimitSwitch == Constants.bottomBallLoaded){
      popup.StopBottomPulley();
      popup.StopTopPulley();
      Constants.ballsStaged = Constants.secondBallStaged;
    }
    else if(Constants.topLimitSwitch == Constants.topBallUnloaded && Constants.bottomLimitSwitch == Constants.bottomBallLoaded){
      // popup.UpTopPulley();
      // popup.UpBottomPulley();
      popup.AutoPulleyUp();
    }
    else if(Constants.topLimitSwitch == Constants.topBallLoaded && Constants.bottomLimitSwitch == Constants.bottomBallUnloaded){
      // popup.DownBottomPulley();
      // popup.DownTopPulley();
      popup.AutoPulleyDown();
      Constants.ballsStaged = Constants.firstballStaged;
    }
/*     else if (Constants.topLimitSwitch == Constants.topBallUnloaded && Constants.bottomLimitSwitch == Constants.bottomBallUnloaded){
       popup.StopBottomPulley();
       popup.StopTopPulley();
    } */
    else if(Constants.topLimitSwitch == Constants.topBallUnloaded && Constants.bottomLimitSwitch == Constants.bottomBallUnloaded){
      // popup.UpTopPulley();
      // popup.UpBottomPulley();
      popup.AutoPulleyUp();

    }
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Constants.ballsStaged == Constants.secondBallStaged ){ // || Constants.ballsStaged == Constants.firstballStaged
      return true;
    }
    else {
      return false;
    }
    
  }
}
