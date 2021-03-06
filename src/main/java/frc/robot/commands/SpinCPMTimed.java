/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ControlPanelManipulator;

public class SpinCPMTimed extends CommandBase {
  /**
   * Creates a new SpinCPMTimed.
   */
  ControlPanelManipulator cpm = ControlPanelManipulator.GetInstance();
  public SpinCPMTimed() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  if (Constants.cpmState == Constants.cpmStateRetracted){
    cpm.SpinControlPanel();
  }
  else if (Constants.cpmState == Constants.cpmStateExtended){
    cpm.StopSpinControlPanel();
  }
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
