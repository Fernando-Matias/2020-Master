/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

 import edu.wpi.first.networktables.NetworkTable;
 import edu.wpi.first.networktables.NetworkTableEntry;
 import edu.wpi.first.networktables.NetworkTableInstance;
 import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

 public class Limelight extends SubsystemBase {

  private static final Limelight instance = new Limelight();

  public static Limelight getInstance() {
    return instance;
  }
  double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
  double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

  public double kp = -0.1f;
  public double minCommand = 0.05f; 

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
/*   NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;
  NetworkTableEntry tv; */
  NetworkTableEntry ts;
  NetworkTableEntry pipeline;
  NetworkTableEntry ledMode;
  NetworkTableEntry camMode;

  public boolean m_LimelightHasValidTarget = false;

  // public Limelight() {
  //   tx = table.getEntry("tx");
  //   ty = table.getEntry("ty");
  //   ta = table.getEntry("ta");
  //   tv = table.getEntry("tv");
  //   ts = table.getEntry("ts");
  //   pipeline = table.getEntry("pipeline");
  //   camMode = table.getEntry("camMode");
  //   ledMode = table.getEntry("ledMode");

  //   pipeline.setValue(1);
  //   camMode.setValue(1);

  // }

  // public double getLimelightTarget(){
  //   double v = tv.getDouble(0.0);
  //   return v;
  // }

  // public double getXOffsetFromTarget() {
  //   double x = tx.getDouble(0.0);
  //   return x;
  // }

  // public double getYOffsetFromTarget() {
  //   double y = ty.getDouble(0.0);
  //   return y;
  // }

  // public double getTargetArea() {
  //   double area = ta.getDouble(0.0);
  //   return area;
  // }

  public double getTargetAngle(){
    double skew = ts.getDouble(0.0);
    return skew;
  }

  public void setOn(){
    ledMode.setNumber(0);
  }

  public void blink() {
    ledMode.setNumber(2);
  }

  public void setOff(){
    ledMode.setNumber(1);
  }

/*   public void LimelightOutput() {
    SmartDashboard.putNumber("Limelight X", getXOffsetFromTarget());
    SmartDashboard.putNumber("Limelight Y", getYOffsetFromTarget());
    SmartDashboard.putNumber("Limelight Area", getTargetArea());
    SmartDashboard.putNumber("ValidTarget", getLimelightTarget());
  } */
  public void UpdateLimelightTraking () {

    if (tv == 1.0) {
      m_LimelightHasValidTarget = true;
      // m_LimelightDriveCommand = 0.0;
      // m_LimelightSteerCommand = 0.0;
      return;
    } else {
      m_LimelightHasValidTarget = false;
    }
  }

  public void SteeringAdjust() {

    double heading_error = tx;
    double steeringAdjust = 0.0f;

    if (tx > 1.0){
      steeringAdjust = kp*heading_error - minCommand;
      //m_LimelightTurnSuccess = true;
    } 
    else if (tx < 1.0){
      steeringAdjust = kp*heading_error + minCommand;
    }

    double leftCommand =+ steeringAdjust;
    double rightCommand =- steeringAdjust;
    DriveTrain.mDrive.tankDrive(leftCommand, rightCommand);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}