package org.rivierarobotics.util;

import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class StatusFrameDemolisher {
    public static void demolishStatusFrames(BaseTalon motor, boolean isFollower) {
        if(isFollower) {
            motor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255);
            motor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255);
        }
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255);
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255);
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
    }
}
