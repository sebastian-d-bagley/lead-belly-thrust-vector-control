function [torqueTheta, torquePhi] = Mount_Angle_to_Torque(motorForce, mountAnglePhi, mountAngleTheta,leverArm)
    torqueTheta = motorForce * leverArm * sin(mountAngleTheta);
    torquePhi = motorForce * leverArm * sin(mountAnglePhi);
end
