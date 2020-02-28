/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Popup;
import frc.robot.Constants;

public class ManualPopup extends CommandBase {
  /**
   * Creates a new ManualPopup.
   */
  Popup popup = Popup.getInstance();
  public ManualPopup() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Constants.popupState == Constants.popupStateDown) {
      popup.PopUp();
      Constants.popupState = Constants.popupStateUp;
    }
    else if (Constants.popupState == Constants.popupStateUp) {
      popup.PopDown();
      Constants.popupState = Constants.popupStateDown;
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