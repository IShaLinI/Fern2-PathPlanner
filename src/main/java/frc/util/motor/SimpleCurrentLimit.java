// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.util.motor;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

/** Add your docs here. */
public class SimpleCurrentLimit {

    public static final SupplyCurrentLimitConfiguration getSimpleCurrentLimit(double limit){
        return new SupplyCurrentLimitConfiguration(true, limit, limit, 0);
    }

}
