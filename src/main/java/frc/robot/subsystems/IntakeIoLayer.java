package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IntakeIoLayer {
    @AutoLog
    public class IOInputs {
        public double rollerVoltage = 0.0;
        public double pivotVoltage = 0.0;
        public double rollerVelocity = 0.0;
        public double pivotVelocity = 0.0;
        public double rollerStatorCurrent = 0.0;
        public double pivotStatorCurrent = 0.0;
        public double rollerSupplyCurrent = 0.0;
        public double pivotSupplyCurrent = 0.0;
        public Rotation2d pivotRotation = new Rotation2d();
    }
    
    void updateInputs(IOInputs inputs);

    void setRollerVoltage(double voltage);

    void setPivotRotation(Rotation2d rotation);

    void setRollerVelocity(double velocity);

    

}
