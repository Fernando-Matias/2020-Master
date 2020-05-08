/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.FalconShooter;
//import frc.robot.subsystems.NeoShooter;
import frc.robot.subsystems.Popup;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Turret;

public class StopShootPowerCell extends CommandBase {
  /**
   * Creates a new StopShootPowerCell.
   */
  FalconShooter falconShooter = FalconShooter.getInstance();
  //NeoShooter neoShooter = NeoShooter.getInstance();
  Popup popup = Popup.getInstance();
  Limelight limelight = Limelight.getInstance();
  Turret turret = Turret.getInstance();

  public StopShootPowerCell() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    //falconShooter.StopShootingCells();

    popup.PopDown();
    //neoShooter.NeoNoShoot();
/*     if (Constants.flashlightMode == Constants.flashlightOn){
      popup.FalshlighOff();
    } */

    falconShooter.StopShootingCells();
    Constants.shootState = Constants.notDoneRamping;
    Constants.triggerState = Constants.triggerNotPressed;
    Constants.trackState = Constants.trackOFF;
    // if (Constants.trackState == Constants.trackOFF){
    //   limelight.setOff();
    //   turret.TurretServo.setSpeed(0.5);
    // }
    
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
