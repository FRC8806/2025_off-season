package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;

public class ConsClimber {
    //ID
    public static final int CLIMB_MOTOR_ID = 10;

    //position
    public static final double readyPosition = -115.69;
    public static final double finalPosition = -422.26;

    //PID
    private static final Slot0Configs angleSlot0 = new Slot0Configs().withKP(2).withKI(0).withKD(0);

    //limit
    private static final SoftwareLimitSwitchConfigs angleLimit = new SoftwareLimitSwitchConfigs()
    .withForwardSoftLimitEnable(false).withForwardSoftLimitThreshold(0)
    .withReverseSoftLimitEnable(false).withReverseSoftLimitThreshold(0);

    //voltage limit
    private static final VoltageConfigs voltageLimit = new VoltageConfigs().withPeakForwardVoltage(14).withPeakReverseVoltage(-14);

    //config
    public static final TalonFXConfiguration climberConfigs = new TalonFXConfiguration().withSlot0(angleSlot0).withSoftwareLimitSwitch(angleLimit).withVoltage(voltageLimit);

}
