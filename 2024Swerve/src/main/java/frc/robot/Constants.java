// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public final class Constants {
    public static final String canivoreBus = "can2";

    public static final class SwerveDrivetrain{

    }
    
    public static final class Shooter{
        public static final int Shooter1_ID = 10;
        public static final int Shooter2_ID = 11;
        public static final int Feeder_ID = 13;
        public static final int noteSensor_DIO = 0;

        public static final boolean Feeder_Inverted = false;

        public static final boolean Shooter1_Inverted = true;
        public static final boolean Shooter2_Inverted = false;

        public static final double Shooter_kP = 0.1;
        public static final double Shooter_kI = 0.0;
        public static final double Shooter_kD = 0.0;

        public static final double Gear_Ratio = 84/74;
    }

    public static final class Arm{
        public static final int leftArmMotor = 21;
        public static final int rightArmMotor = 22;
        public static final int telescopeMotor = 23;

        public static final boolean leftArm_Inverted = false;
        public static final boolean rightArm_Inverted = false;
        
        public static final double telescope_kP = 0.1;
        public static final double telescope_kI = 0.0;
        public static final double telescope_kD = 0.0;

        public static final double arm_kP = 0.1;
        public static final double arm_kI = 0.0;
        public static final double arm_kD = 0.0;
    }

    public static final class Limelight{

    }
}
