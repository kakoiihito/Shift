extends Node

var max_brake_torque = Values.max_brake_torque
var wheel_brake_torque = Data.wheel_brake_torque
var brake_torque: float 

var FR_torque_brake = Values.FR_torque_brake
var FL_torque_brake = Values.FL_torque_brake
var RR_torque_brake = Values.RR_torque_brake
var RL_torque_brake = Values.RL_torque_brake

		
func brake_proccess() -> void:

	var brake_wheels = [FL_torque_brake, FL_torque_brake, RR_torque_brake, RL_torque_brake]
	var input_brake = Input.get_action_strength("Brake") # strength of input
	
	if input_brake > 0.0:
		
		brake_torque = (input_brake * max_brake_torque) / Data.active_wheels_brake # formula of torque divided by wheels using it
		
		for i in range(4):
			if brake_wheels[i] == true:
				var brake_direction = 1.0
				if Data.wheel_angular_velocity[i] != 0.0:
					brake_direction = sign(Data.wheel_angular_velocity[i])
				else:
					brake_direction = 0.0
				wheel_brake_torque[i] = brake_torque * brake_direction
				
	else:
		for i in range(4):
			wheel_brake_torque[i] = 0.0
			
