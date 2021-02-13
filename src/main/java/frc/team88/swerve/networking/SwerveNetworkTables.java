package frc.team88.swerve.networking;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.Timer;

import frc.team88.swerve.SwerveChassis;


public class SwerveNetworkTables {
    NetworkTable odom_table;
    NetworkTableEntry odom_time;
    NetworkTableEntry odom_x;
    NetworkTableEntry odom_y;
    NetworkTableEntry odom_t;
    NetworkTableEntry odom_vx;
    NetworkTableEntry odom_vy;
    NetworkTableEntry odom_vt;

    NetworkTable command_table;
    NetworkTableEntry command_time;
    NetworkTableEntry command_linear_x;
    NetworkTableEntry command_linear_y;
    NetworkTableEntry command_angular_z;

    public SwerveNetworkTables()
    {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        inst.setUpdateRate(0.01);
        NetworkTable table = inst.getTable("Swerve");

        odom_table = table.getSubTable("odom");
        odom_time = odom_table.getEntry("time");
        odom_x = odom_table.getEntry("x");
        odom_y = odom_table.getEntry("y");
        odom_t = odom_table.getEntry("t");
        odom_vx = odom_table.getEntry("vx");
        odom_vy = odom_table.getEntry("vy");
        odom_vt = odom_table.getEntry("vt");

        command_table = table.getSubTable("command");
        command_time = command_table.getEntry("time");
        command_linear_x = command_table.getEntry("linear_x");
        command_linear_y = command_table.getEntry("linear_y");
        command_angular_z = command_table.getEntry("angular_z");
    }

    public void updateWithSwerve(SwerveChassis chassis)
    {
        
    }

    public void setOdom(double timestamp, double x, double y, double theta, double vx, double vy, double vtheta)
    {
        odom_time.setDouble(timestamp);
        odom_x.setDouble(x);
        odom_y.setDouble(y);
        odom_t.setDouble(theta);
        odom_vx.setDouble(vx);
        odom_vy.setDouble(vy);
        odom_vt.setDouble(vtheta);
    }

    public double getLinearXCmd()
    {
        return command_linear_x.getDouble(0.0);
    }

    public double getLinearYCmd()
    {
        return command_linear_y.getDouble(0.0);
    }

    public double getAngularZCmd()
    {
        return command_angular_z.getDouble(0.0);
    }

    public boolean didCmdUpdate()
    {
        double dt = Timer.getFPGATimestamp() - command_time.getDouble(0.0);
        return dt < 0.5;
    }
}
