package frc.robot.subsystems;
import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight 
{
    double m_hasValidTarget = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    double m_x_AngleOffset = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double m_y_AngleOffset = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    double m_areaDetected = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

    double m_tagHeightInches = 47.5;

    public void updateLimelightTracking() 
    {
        m_hasValidTarget = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        m_x_AngleOffset = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        m_y_AngleOffset = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        m_areaDetected = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);


    }

    public double find_Tag_Y_Distance() 
    {
        double m_y_angleToTagDegrees = Constants.LimelightConstants.m_limelightMountAngleDegree + m_y_AngleOffset;
        double m_y_angleToTagRadians = m_y_angleToTagDegrees * (3.14159 / 180.);

        double m_limelightToTagInches = (m_tagHeightInches - Constants.LimelightConstants.m_limelightLensHeightInches) / Math.tan(m_y_angleToTagRadians);

        return m_limelightToTagInches;
    }

    public double find_Tag_X_Distance() 
    {
        double X_Distance;
        double Y_Distance = find_Tag_Y_Distance();
        double m_x_angleToTagRadians = m_x_AngleOffset * (3.14159 / 180.);

        // o / a: o = y: y / tan(x_offset)
        boolean toRight = rightOfTag();

        X_Distance = Math.abs(Y_Distance / (Math.tan(m_x_angleToTagRadians)));

        if (toRight) 
        {
            X_Distance = -X_Distance;
        }

        return X_Distance;
    }
    public void LimeToDrive() 
    {
        double X_Distance = find_Tag_X_Distance();

        SmartDashboard.putNumber("Horizontal Displacement to Tag", X_Distance);
    }
    public double detectTarget() 
    {
        return m_hasValidTarget;
    }
    public double findXOffset() 
    {
        return m_x_AngleOffset;
    }
    public boolean rightOfTag() 
    {
        //need to check if pos or not
        if (m_x_AngleOffset < 0) 
        {
            return true;
        }
        return false;
    }
    public double findYOffset() 
    {
        return m_y_AngleOffset;
    }
    public double findAreaDetected() 
    {
        return m_areaDetected;
    }
}