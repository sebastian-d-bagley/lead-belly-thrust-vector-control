function [thetaControllerOutput,phiControllerOutput]= PD_controller(recordedDeltaTheta,recordedDeltaPhi, recordedTheta, recordedPhi, P, D)
thetaControllerOutput = -P * recordedTheta - D * recordedDeltaTheta;
phiControllerOutput = -P * recordedPhi - D * recordedDeltaPhi;
end

