package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;

public class ConsIntake {
    //ID
    public static final int ANGLE_MOTOR_ID = 11;//
    public static final int ROLLING_MOTOR_ID = 12;//
    public static final int LEFT_TRANSPORT_MOTOR_ID = 13;//
    public static final int RIGHT_TRANSPORT_MOTOR_ID = 14;//

    //speed
    public static final double transportSpeed = 0.14;//
    public static final double rollingSpeed =0.9;  


    //PID
    private static final Slot0Configs angleSlot0 = new Slot0Configs().withKP(1.3).withKI(0.05).withKD(0.2);//

    //limit
    private static final SoftwareLimitSwitchConfigs angleLimit = new SoftwareLimitSwitchConfigs()
    .withForwardSoftLimitEnable(false).withForwardSoftLimitThreshold(0)//true
    .withReverseSoftLimitEnable(false).withReverseSoftLimitThreshold(0);//true

    // voltage limit
    private static final VoltageConfigs angleVoltageLimit = new VoltageConfigs().withPeakForwardVoltage(12).withPeakReverseVoltage(-12);//need to test

    //config
    public static final TalonFXConfiguration angleConfigs = new TalonFXConfiguration().withSlot0(angleSlot0).withSoftwareLimitSwitch(angleLimit).withVoltage(angleVoltageLimit);

    //position
    public static final double upPosition = 0;//
    public static final double downPosition =-34;//-19.13134765625//-19.69
}