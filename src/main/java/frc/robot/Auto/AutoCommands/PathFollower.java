/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Auto.AutoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import frc.robot.subsystems.DriveTrain;
import frc.robot.utility.PurePursuit.Path;
import frc.robot.utility.PurePursuit.Path.Waypoint;
import frc.robot.utility.PurePursuit.Rotation2d;
import frc.robot.utility.PurePursuit.Translation2d;
import frc.robot.utility.PurePursuit.RigidTransform2d;
import frc.robot.utility.PurePursuit.Kinematics;
import frc.robot.RobotState;
import frc.robot.Auto.AutoPaths;
import edu.wpi.first.wpilibj.Timer;

public class PathFollower extends CommandBase {
  /**
   * Creates a new PathFollower.
   */

  Path path;

  Rotation2d gyro_angle;
  RigidTransform2d odometry;
  RigidTransform2d.Delta velocity;
  RobotState robotstate = RobotState.getInstance();
  DriveTrain driveTrain = DriveTrain.getInstance();

  public PathFollower(Path path) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.path = path;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
