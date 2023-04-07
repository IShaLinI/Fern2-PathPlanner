// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.sim;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.PivotConstants;


@SuppressWarnings("unused")
public class FernPivotSim {

    private TalonFXSimCollection mMotorSimCollection;
    private Supplier<Double> mMotorSet;
    private Supplier<Double> mArmAngle;

    private SingleJointedArmSim mArmSim = new SingleJointedArmSim(
        LinearSystemId.createSingleJointedArmSystem(
            DCMotor.getFalcon500(2), SingleJointedArmSim.estimateMOI(0.57, Units.lbsToKilograms(7.169)), (20d * 60d/16d)
        ),
        DCMotor.getFalcon500(2),
        1 / PivotConstants.kGearing,
        0.57,
        Units.degreesToRadians(-90),
        Units.degreesToRadians(140),
        true
    );

    private final Mechanism2d mMechanismVisualizer = new Mechanism2d(60, 60);
    private final MechanismRoot2d mPivot = mMechanismVisualizer.getRoot("Pivot", 30, 30);
    private final MechanismLigament2d mArmTower = mPivot.append(new MechanismLigament2d("PivotTower", 30, -90));
    private final MechanismLigament2d mArm = mPivot.append(
        new MechanismLigament2d("Arm", 20, Units.degreesToRadians(mArmSim.getAngleRads()),
        6,
        new Color8Bit(Color.kYellow)
    ));


    public FernPivotSim(TalonFXSimCollection _motorSimCollection, Supplier<Double> _motorSet, Supplier<Double> _armAngle){

        mMotorSimCollection = _motorSimCollection;
        mMotorSet = _motorSet;
        mArmAngle = _armAngle;

        SmartDashboard.putData("Arm Visualizer", mMechanismVisualizer);

    }

    public void update(){
        mArmSim.setInput(mMotorSet.get() * RobotController.getBatteryVoltage());
        mArmSim.update(0.02);
        
        double poseDelta = Units.radiansPerSecondToRotationsPerMinute(mArmSim.getVelocityRadPerSec()) * 1d/60d * 1 / PivotConstants.kGearing;
        mMotorSimCollection.addIntegratedSensorPosition((int)(poseDelta * (1d / 10) * 2048));

        mArm.setAngle(mArmAngle.get() + 90);
    }

}
