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


public class ClimberReady extends CommandBase {
  /**
   * Creates a new ClimberReady.
   */

  Climber climber = Climber.getInstance();
  DriveTrain driveTrain = DriveTrain.getInstance();
  Timer timer = new Timer();
  double inittime;

  public ClimberReady() {
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

    climber.ClimberInPosition();
    driveTrain.setBrake();
    if ((Timer.getFPGATimestamp() - inittime) > 1){ //starts going backward
        driveTrain.DownShift();
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
