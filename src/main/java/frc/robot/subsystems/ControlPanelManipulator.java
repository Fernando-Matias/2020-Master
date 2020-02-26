/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.RobotMap;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;

public class ControlPanelManipulator extends SubsystemBase {
  
  //call back to
  private static final ControlPanelManipulator instance = new ControlPanelManipulator();

  public static ControlPanelManipulator GetInstance() {
    return instance;
  }

  Timer timer = new Timer();

  public Solenoid CPManipulatorExtend, CPManipulatorRetract;
  public TalonSRX CPManipulatorMotor;

  public ControlPanelManipulator() {
    CPManipulatorExtend = new Solenoid(RobotMap.PCM_B, RobotMap.pCPManipulatorExtend_ID);
    CPManipulatorRetract = new Solenoid(RobotMap.PCM_A, RobotMap.PCPManipulatorRetract_ID);
    
    CPManipulatorMotor = new TalonSRX(RobotMap.mCPManipulator_ID);

    CPManipulatorMotor.configFactoryDefault();
    CPManipulatorMotor.setNeutralMode(NeutralMode.Brake);
    CPManipulatorMotor.configContinuousCurrentLimit(30);
    CPManipulatorMotor.configPeakCurrentLimit(0);
    CPManipulatorMotor.enableCurrentLimit(true);
    CPManipulatorMotor.setInverted(false);
  }

//Manipulator Control
   public void SpinControlPanel() {

    timer.start();
    for (double i = 0; i <= 0.5; i = i + 0.1){
      SmartDashboard.putNumber("ramping", i);
      CPManipulatorMotor.set(ControlMode.PercentOutput, i);
      Timer.delay(1);
    }

  }
  public void StopSpinControlPanel() {
    CPManipulatorMotor.set(ControlMode.PercentOutput, 0.0);
  }

//Pneumatics Control
/*   public void PushUpManipulator() {
    CPManipulatorRetract.set(Constants.On);
    CPManipulatorExtend.set(Constants.Off);
    Constants.cpmState = Constants.cpmStateRetracted;
  }
  public void PushDownManipulator() {
    CPManipulatorRetract.set(Constants.Off);
    CPManipulatorExtend.set(Constants.On);
    Constants.cpmState = Constants.cpmStateExtended;
  } */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
