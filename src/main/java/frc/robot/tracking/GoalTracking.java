/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.tracking;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class GoalTracking {
    private static GoalTracking mInstance;

    public static GoalTracking getInstance() {
      if (mInstance == null)
        mInstance = new GoalTracking();
      return mInstance;
    }

    private final double LENGTH = 2.5;
    private final double HEIGHT = 8.1875;
    private final double LIMELIGHT_HEIGHT = 2.4167;
    public double tv;
    public double tx;
    public double ty;
    public double ta;
    private double ts;
    private double tl;
    private double tshort;
    private double tlong;
    private double thor;
    private double tvert;
    private double distanceToGoal;
    private double angleToGoal;
    public GoalTracking() {
        tv = 0.0;
        tx = 0.0;
        ty = 0.0;
        ta = 0.0;
        tlong = 0.0;
        tshort = 0.0;
        distanceToGoal = 0.0;
    }
    public void run() {
        tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
        tlong = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tlong").getDouble(0);
        tshort = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tshort").getDouble(0);
        calculateDistance();
        logToDashBoard();
    }
    public void calculateDistance() {
        double w = 320.0, a = 59.6, b = 49.7, distMultiplier = 1.0 / 10.0;
        distanceToGoal = ((LENGTH * w)/(2.0 * tlong * Math.tan(Math.toRadians(b / 2.0))));
        //distanceToGoal = (HEIGHT - LIMELIGHT_HEIGHT) / Math.tan((ty + (b / 2.0)) * (Math.PI / 180.0));
        distanceToGoal += distanceToGoal * distMultiplier;
    }
    public void logToDashBoard() {
        SmartDashboard.putNumber("Vision/tv", tv);
        //System.out.println("tv: " + tv);
        SmartDashboard.putNumber("Vision/tx", tx);
        //System.out.println("tx: " + tx);
        SmartDashboard.putNumber("Vision/ty", ty);
        //System.out.println("ty: " + ty);
        SmartDashboard.putNumber("Vision/ta", ta);
        //System.out.println("ta: " + ta);
        SmartDashboard.putNumber("Vision/tlong", tlong);
        //System.out.println("tlong: " + tlong);
        SmartDashboard.putNumber("Vision/Distance To Goal", distanceToGoal);
        System.out.println("Distance To Goal: " + distanceToGoal);
    }
}
