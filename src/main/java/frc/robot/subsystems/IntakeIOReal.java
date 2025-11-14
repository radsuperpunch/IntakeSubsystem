// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.ObjectInputFilter.Status;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
@SuppressWarnings("unused")
public class IntakeIOReal implements IntakeIoLayer{
    TalonFX pivotMotor = new TalonFX(12);
    TalonFX rollerMotor = new TalonFX(13);

    VoltageOut rollerVoltageOut = new VoltageOut(0.0).withEnableFOC(true);
    VoltageOut pivotVoltageOut = new VoltageOut(0.0).withEnableFOC(true);

    PositionVoltage pivotPositionVoltage = new PositionVoltage(0.0).withEnableFOC(true);

    VelocityVoltage rollerVelocityVoltage = new VelocityVoltage(0.0).withEnableFOC(true);

    StatusSignal<Angle> pivotPosition = pivotMotor.getPosition();
    StatusSignal<Voltage> rollerVoltage = rollerMotor.getMotorVoltage();
    StatusSignal<Voltage> pivotVoltage = pivotMotor.getMotorVoltage();
    StatusSignal<AngularVelocity> rollerVelocity = rollerMotor.getVelocity();
    StatusSignal<AngularVelocity> pivotVelocity = pivotMotor.getVelocity();
    StatusSignal<Current> rollerStatorCurrent = rollerMotor.getStatorCurrent();
    StatusSignal<Current> pivotStatorCurrent = pivotMotor.getStatorCurrent();
    StatusSignal<Current> rollerSupplyCurrent = rollerMotor.getSupplyCurrent();
    StatusSignal<Current> pivotSupplyCurrent = pivotMotor.getSupplyCurrent();

    @Override
    public void updateInputs(IOInputs inputs) {
        inputs.pivotVoltage = this.pivotVoltage.getValueAsDouble();
        inputs.pivotVelocity = this.pivotVelocity.getValueAsDouble();
        inputs.pivotStatorCurrent = this.pivotStatorCurrent.getValueAsDouble();
        inputs.pivotSupplyCurrent = this.pivotSupplyCurrent.getValueAsDouble();

        inputs.rollerVoltage = this.rollerVoltage.getValueAsDouble();
        inputs.rollerVelocity = this.rollerVelocity.getValueAsDouble();
        inputs.rollerStatorCurrent = this.rollerStatorCurrent.getValueAsDouble();
        inputs.rollerSupplyCurrent = this.rollerSupplyCurrent.getValueAsDouble();
        
        inputs.pivotRotation = Rotation2d.fromRotations(this.pivotPosition.getValueAsDouble());
    }

    @Override
    public void setRollerVoltage(double voltage) {
        rollerMotor.setControl(rollerVoltageOut.withOutput(voltage));
    }

    @Override
    public void setPivotRotation(Rotation2d rotation) {
        pivotMotor.setControl(pivotPositionVoltage.withPosition(rotation.getMeasure()));
    }

    @Override
    public void setRollerVelocity(double velocity) {
        rollerMotor.setControl(rollerVelocityVoltage.withVelocity(velocity));
    }
    
}
