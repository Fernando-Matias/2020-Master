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
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.DriveTrain;

//import java.sql.Time;

import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BasicAuto extends CommandBase {
  /**
   * Creates a new BasicAuto.
   */

  DriveTrain driveTrain = DriveTrain.getInstance();
  FalconShooter falconShooter = FalconShooter.getInstance();
  Intake intake = Intake.getInstance();
  Popup popup = Popup.getInstance();
  Timer timer = new Timer();
  double inittime;

  public BasicAuto() {
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

    //double yes = 1.0;
    //SmartDashboard.putNumber("working", yes);

    if ((Timer.getFPGATimestamp() - inittime) < 4){

    intake.ExtendIntake();
    driveTrain.Curvature(0.35, 0.0);
    System.out.println(Timer.getFPGATimestamp());

    }
 
     if ((Timer.getFPGATimestamp() - inittime) >= 4 && (Timer.getFPGATimestamp() - inittime) <= 4.1 ){
       driveTrain.Curvature(0.0, 0.0);
       //popup.PopUp();

     }
     if ((Timer.getFPGATimestamp() - inittime) >= 5){
      //driveTrain.Curvature(0.0, 0.0);
      intake.RetractIntake();
      driveTrain.Curvature(-0.3, 0.0);
    //   falconShooter.AutoRamping();
    //   falconShooter.ShootPowerCell();
    //   popup.UpBottomPulley();
    //   popup.UpTopPulley();


    }  

     //if ((Timer.getFPGATimestamp() - inittime) >= 10){
      //driveTrain.NavX40deg();
     //}

    //Timer.delay(1);
    //driveTrain.mDrive.arcadeDrive(0.0, 0.0);
    //Timer.delay(0.2);
    //driveTrain.NavX40deg();
    //Timer.delay(0.2);
    //popup.PopUp();
    //falconShooter.RampingSequence();

    //Timer.delay(.1);
    //popup.UpBottomPulley();
    //popup.UpTopPulley();
    //Timer.delay(3);
    //falconShooter.StopShootingCells();
    //popup.StopBottomPulley();
    //popup.StopTopPulley();
    //popup.PopDown(); 



    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    //popup.StopBottomPulley();
    //popup.StopTopPulley();
    //popup.PopDown();
    //falconShooter.StopShootingCells();
    driveTrain.Curvature(0.0, 0.0);



  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("CHECK FINISH");
    return (Timer.getFPGATimestamp() - inittime) >= 10 ;
  }
}
