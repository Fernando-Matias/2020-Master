/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Auto;

import edu.wpi.first.wpilibj2.command.Command;
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
import edu.wpi.first.wpilibj.Timer;

import frc.robot.subsystems.Popup;
import frc.robot.subsystems.FalconShooter;
import frc.robot.subsystems.Intake;

/**
 * Add your docs here.
 */
public class AutoPaths {

    private Path PathA, PathB, PathC;

    private static final AutoPaths instance = new AutoPaths();
    Popup popup = Popup.getInstance();
    FalconShooter fShooter = FalconShooter.getInstance();
    Intake intake = Intake.getInstance();

    public static AutoPaths getInstance() {
      return instance;
    }

    public AutoPaths() {

        List<Waypoint> first_path = new ArrayList<>();
        //first_path.add(new Waypoint(new Translation2d(0, 0), 120.0));
        first_path.add(new Waypoint(new Translation2d(0,0), 50));
        first_path.add(new Waypoint(new Translation2d(-100, -100), 50));
        //first_path.add(new Popup())
        popup.PopUp();
        // popup.UpBottomPulley();
        // popup.UpTopPulley();
        // fShooter.RampingSequence();
        // fShooter.ShootPowerCell();


        PathA = new Path(first_path);

        List<Waypoint> second_path = new ArrayList<>();
        second_path.add(new Waypoint(new Translation2d(0, 0), 50.0));
        second_path.add(new Waypoint(new Translation2d(-150, 100), 50.0));
        intake.ExtendIntake();
        
        

        PathB = new Path(second_path);
    }
    public Path getFirstPath() {
        return PathA;
    }
    public Path getSecondPath() {
        return PathB;
    }
}
