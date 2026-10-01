package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class LimeLightHelperBLUE {

    private final Limelight3A limelight;

    public LimeLightHelperBLUE(HardwareMap hardwareMap) {

        limelight = hardwareMap.get(
                Limelight3A.class,
                "limelight"
        );

        limelight.pipelineSwitch(6);
        limelight.start();
    }


    public double getTx() {

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            return result.getTx();
        }

        return 0.0;
    }

    public double getTxForTag(int wantedId) {

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return 0.0;
        }

        List<LLResultTypes.FiducialResult> fiducials =
                result.getFiducialResults();

        for (LLResultTypes.FiducialResult tag : fiducials) {

            if (tag.getFiducialId() == wantedId) {
                return tag.getTargetXDegrees();
            }
        }

        return 0.0;
    }


    public boolean hasTag(int wantedId) {

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return false;
        }

        List<LLResultTypes.FiducialResult> fiducials =
                result.getFiducialResults();

        for (LLResultTypes.FiducialResult tag : fiducials) {

            if (tag.getFiducialId() == wantedId) {
                return true;
            }
        }

        return false;
    }


    public boolean hasTarget() {

        LLResult result = limelight.getLatestResult();

        return result != null && result.isValid();
    }
}