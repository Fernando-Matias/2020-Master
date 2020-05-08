/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import frc.robot.subsystems.NeoShooter;
import frc.robot.subsystems.FalconShooter;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.subsystems.Popup;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Turret;

import edu.wpi.first.wpilibj.Timer;

public class Ramping extends CommandBase {
  /**
   * Creates a new Ramping.
   */
  Timer timer = new Timer();
  //NeoShooter neoShooter = NeoShooter.getInstance();
  FalconShooter falconShooter = FalconShooter.getInstance();
  
  Popup popup = Popup.getInstance();
  Limelight limelight = Limelight.getInstance();
  Turret turret = Turret.getInstance();
  
  double inittime;
  public Ramping() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    inittime = Timer.getFPGATimestamp();
    Constants.triggerState = Constants.triggerPressed;
    Constants.trackState = Constants.trackON;
    limelight.setOn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (Constants.triggerState == Constants.triggerPressed){
      // if (limelight.getLimelightTarget() == 1){
      //   double kp = .1;
      //   turret.turretSetSpeed(limelight.getXOffsetFromTarget()*kp);
      // }
      // else {
      //   turret.TurretStayStill();
      // }
      popup.PopUp();
      // if (Constants.flashlightMode == Constants.flashlightOff){
      //   popup.FlashlightOn();
      // }
      
      if((Timer.getFPGATimestamp() - inittime) >= .5){
        falconShooter.falconShooter.set(ControlMode.PercentOutput, 0.1);
        //neoShooter.NeoShooter.set(-.2);
      }
  
      if((Timer.getFPGATimestamp() - inittime) >= .75){
        falconShooter.falconShooter.set(ControlMode.PercentOutput, 0.2);
        //neoShooter.NeoShooter.set(-.4);
      }
  
      if((Timer.getFPGATimestamp() - inittime) >= 1){
        falconShooter.falconShooter.set(ControlMode.PercentOutput, 0.3);
        //neoShooter.NeoShooter.set(-0.6);
      }
  
      if((Timer.getFPGATimestamp() - inittime) >= 1.25){
        falconShooter.falconShooter.set(ControlMode.PercentOutput, 0.4);
        //neoShooter.NeoShooter.set(-0.8);
      }
      if((Timer.getFPGATimestamp() - inittime) >= 1.5){
        falconShooter.falconShooter.set(ControlMode.PercentOutput, 0.5);
        //neoShooter.NeoShooter.set(-0.8);
      }

      // if((Timer.getFPGATimestamp() - inittime) >= 1.75){
      //   falconShooter.falconShooter.set(ControlMode.PercentOutput, 0.6);
      //   //neoShooter.NeoShooter.set(-0.8);
      // }
      // if((Timer.getFPGATimestamp() - inittime) >= 2){
      //   falconShooter.falconShooter.set(ControlMode.PercentOutput, 0.7);
      //   //neoShooter.NeoShooter.set(-0.8);
      // }

    }



    // if((Timer.getFPGATimestamp() - inittime) >= 2.25){
    //   falconShooter.falconShooter.set(ControlMode.PercentOutput, 0.5);
    // }
     
    // if((Timer.getFPGATimestamp() - inittime) >= 2.5){
    //   falconShooter.falconShooter.set(ControlMode.PercentOutput, 0.6);
    // } 

/*     if((Timer.getFPGATimestamp() - inittime) >= 2.5){
      falconShooter.falconShooter.set(ControlMode.PercentOutput, 0.5);
    } */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //neoShooter.NeoShooter.set(-0.8);
    falconShooter.falconShooter.set(ControlMode.PercentOutput, 0.53);
    Constants.shootState = Constants.doneRamping;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("CHECK FINISH");
    if (Constants.triggerState == Constants.triggerPressed || (Timer.getFPGATimestamp() - inittime) >= 1.75){ //Constants.triggerState == Constants.triggerPressed || 
      return true;
    }
    else {
      return false;
    }
  }

}
