/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utility;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
/**
 * Author Fernando Matias
 */

public class DefaultTalonFXDrive extends WPI_TalonFX{
    private double prevValue = 0;
	private ControlMode prevControlMode = ControlMode.Disabled;

	public DefaultTalonFXDrive(int deviceNumber) {
		super(deviceNumber);
		configFactoryDefault();
		configVoltageCompSaturation(12, 10);
	//	enableCurrentLimit(true);
	//	configContinuousCurrentLimit(40);
	//	configPeakCurrentDuration(100);
	//	configPeakCurrentLimit(40);
		new StatorCurrentLimitConfiguration(true, 40, 40, 100);
		enableVoltageCompensation(true);
		setNeutralMode(NeutralMode.Coast);
		setSafetyEnabled(false);
	}

	@Override
	public void set(ControlMode controlMode, double outputValue) {
		//return;
		
		if (outputValue != prevValue || controlMode != prevControlMode) {
			super.set(controlMode, outputValue);
			prevValue = outputValue;
		} 
	}

	public double getSetpoint() {
		return prevValue;
	}

}
