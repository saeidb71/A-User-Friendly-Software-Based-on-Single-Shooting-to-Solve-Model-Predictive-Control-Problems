function output = Rotor_drivetrain_Endpoint(input)
q = input.phase.integral; 

output.objective =(-q); %[GW.s]
end