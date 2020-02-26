/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.RobotMap;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;

/**
 * @author Fernando Matias
 */

public class Intake extends SubsystemBase {

  //callable now
  private static final Intake instance = new Intake();

  public static Intake getInstance() {
    return instance;
  }

  public Solenoid intakeExtend, intakeRetract;
  public TalonSRX intakeMotor;


  public Intake() {
    intakeExtend = new Solenoid(RobotMap.PCM_A, RobotMap.pIntakeExtended_ID);
    intakeRetract = new Solenoid(RobotMap.PCM_A, RobotMap.pIntakeRetract_ID);

    intakeMotor = new TalonSRX(RobotMap.mIntakeMotor_ID);

    intakeMotor.configFactoryDefault();
    intakeMotor.setNeutralMode(NeutralMode.Brake);
    intakeMotor.configContinuousCurrentLimit(40);
    intakeMotor.configPeakCurrentLimit(0);
    intakeMotor.enableCurrentLimit(true);
    intakeMotor.setInverted(false);

  }

  public void IntakePowerCell() {
    intakeMotor.set(ControlMode.PercentOutput, 1.0);
  }
  public void StopIntakePowerCell() {
    intakeMotor.set(ControlMode.PercentOutput, 0.0);
  }
 /*  public void ExtendIntake() {
    intakeExtend.set(Constants.On);
    intakeRetract.set(Constants.Off);
    Constants.intakeState = Constants.intakeStateExtended;
  }
  public void RetractIntake() {
    intakeExtend.set(Constants.Off);
    intakeRetract.set(Constants.On);
    Constants.intakeState = Constants.intakeStateRetracted;
  } */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
