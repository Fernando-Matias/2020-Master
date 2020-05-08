/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.Timer;

public class ClimberNotReady extends CommandBase {
  /**
   * Creates a new ClimberNotReady.
   */

  Climber climber = Climber.getInstance();
  DriveTrain driveTrain = DriveTrain.getInstance();
  Timer timer = new Timer();
  double inittime;

  public ClimberNotReady() {
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
    driveTrain.UpShift();
    driveTrain.setCoast();
    if ((Timer.getFPGATimestamp() - inittime) > 1){ //starts going backward
      climber.ClimberNotReady();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Timer.getFPGATimestamp() - inittime) >= 1.1 ;
  }
}
