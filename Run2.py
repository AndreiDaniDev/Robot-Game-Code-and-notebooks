#be aware that this was made in Spike Prime 3.4.3
import runloop, hub, motor, motor_pair, color


# ---> Robot Ports <---
leftDriveBaseMotor = hub.port.B
rightDriveBaseMotor = hub.port.D
leftSystemMotor = hub.port.A
rightSystemMotor = hub.port.C
#leftColorSensor = NULL
#rightColorSensor = NULL

async def gyroForward(distance: float, speed: int, target: int = 0, * , kp: float = 0.25, kd: float = 0.3, stop: bool = True, reset: bool = True):
    d = int(distance * 20.45); lastError = 0
    if(reset): hub.motion_sensor.reset_yaw(0)
    motor.reset_relative_position(leftDriveBaseMotor, 0)
    while(abs(motor.relative_position(leftDriveBaseMotor)) < d):
        error = (hub.motion_sensor.tilt_angles()[0] - target) / 10
        proportional = error * -kp
        differential = (error - lastError) * -kd
        lastError = error + target
        motor_pair.move_tank(motor_pair.PAIR_1, int(speed + proportional + differential), int(speed - proportional - differential))
    if(stop): motor_pair.stop(motor_pair.PAIR_1, stop=motor.SMART_BRAKE)

async def gyroBackwards(distance: float, speed: int, target: int = 0, * , kp: float = 0.25, kd: float = 0.3 , stop: bool = True, reset: bool = True):
    d = int(distance * 20.45); lastError = 0; speed *= -1
    if(reset): hub.motion_sensor.reset_yaw(0)
    motor.reset_relative_position(leftDriveBaseMotor, 0)
    while(abs(motor.relative_position(leftDriveBaseMotor)) < d):
        error = (hub.motion_sensor.tilt_angles()[0] - target) / 10
        proportional = error * kp
        differential = (error - lastError) * kd
        lastError = error + target
        motor_pair.move_tank(motor_pair.PAIR_1, int(speed - proportional - differential), int(speed + proportional + differential))
    if(stop): motor_pair.stop(motor_pair.PAIR_1, stop=motor.SMART_BRAKE)

async def turnLeft(grade: float, speedMax: int = 500, speedMin: int = 150, * , reset: bool = True, kp: float = 0.25, kd: float = 0.35):
    if(reset): hub.motion_sensor.reset_yaw(0)
    target = int(grade * 10)
    error = float(target - abs(hub.motion_sensor.tilt_angles()[0]))
    lastError = target
    while(error < -5 or error > 5):
        error = target - abs(hub.motion_sensor.tilt_angles()[0])
        output = kp * error + kd * (error - lastError)
        lastError = error
        if(abs(output) > speedMax): output = speedMax
        if(abs(output) < speedMin): output = speedMin
        motor_pair.move_tank(motor_pair.PAIR_1, -int(output), int(output))
    motor_pair.stop(motor_pair.PAIR_1, stop=motor.SMART_BRAKE)

async def turnRight(grade: float, speedMax: int = 500, speedMin: int = 150, * , reset: bool = True, kp: float = 0.25, kd: float = 0.35):
    if(reset): hub.motion_sensor.reset_yaw(0)
    target = int(grade * 10)
    error = float(target - abs(hub.motion_sensor.tilt_angles()[0]))
    lastError = target
    while(error < -5 or error > 5):
        error = target - abs(hub.motion_sensor.tilt_angles()[0])
        output = kp * error + kd * (error - lastError)
        lastError = error
        if(abs(output) > speedMax): output = speedMax
        if(abs(output) < speedMin): output = speedMin
        motor_pair.move_tank(motor_pair.PAIR_1, int(output), -int(output))
    motor_pair.stop(motor_pair.PAIR_1, stop=motor.SMART_BRAKE)

async def turnLeftOneWheel(grade: float, speedMax: int = 500, speedMin: int = 150, * , reset: bool = True, kp: float = 0.25, kd: float = 0.35):
    if(reset): hub.motion_sensor.reset_yaw(0)
    target = int(grade * 10)
    error = float(target - abs(hub.motion_sensor.tilt_angles()[0]))
    lastError = target
    while(error < -5 or error > 5):
        error = target - abs(hub.motion_sensor.tilt_angles()[0])
        output = kp * error + kd * (error - lastError)
        lastError = error
        if(abs(output) > speedMax): output = speedMax
        if(abs(output) < speedMin): output = speedMin
        motor_pair.move_tank(motor_pair.PAIR_1, 0, int(output))
    motor_pair.stop(motor_pair.PAIR_1, stop=motor.SMART_BRAKE)

async def turnRightOneWheel(grade: float, speedMax: int = 500, speedMin: int = 150, * , reset: bool = True, kp: float = 0.25, kd: float = 0.35):
    if(reset): hub.motion_sensor.reset_yaw(0)
    target = int(grade * 10)
    error = float(target - abs(hub.motion_sensor.tilt_angles()[0]))
    lastError = target
    while(error < -5 or error > 5):
        error = target - abs(hub.motion_sensor.tilt_angles()[0])
        output = kp * error + kd * (error - lastError)
        lastError = error
        if(abs(output) > speedMax): output = speedMax
        if(abs(output) < speedMin): output = speedMin
        motor_pair.move_tank(motor_pair.PAIR_1, int(output), 0)
    motor_pair.stop(motor_pair.PAIR_1, stop=motor.SMART_BRAKE)

async def turnLeftOtherWheel(grade: float, speedMax: int = 500, speedMin: int = 150, * , reset: bool = True, kp: float = 0.25, kd: float = 0.35):
    if(reset): hub.motion_sensor.reset_yaw(0)
    target = int(grade * 10)
    error = float(target - abs(hub.motion_sensor.tilt_angles()[0]))
    lastError = target
    while(error < -5 or error > 5):
        error = target - abs(hub.motion_sensor.tilt_angles()[0])
        output = kp * error + kd * (error - lastError)
        lastError = error
        if(abs(output) > speedMax): output = speedMax
        if(abs(output) < speedMin): output = speedMin
        motor_pair.move_tank(motor_pair.PAIR_1, -int(output), 0)
    motor_pair.stop(motor_pair.PAIR_1, stop=motor.SMART_BRAKE)

async def turnRightOtherWheel(grade: float, speedMax: int = 500, speedMin: int = 150, * , reset: bool = True, kp: float = 0.25, kd: float = 0.35):
    if(reset): hub.motion_sensor.reset_yaw(0)
    target = int(grade * 10)
    error = float(target - abs(hub.motion_sensor.tilt_angles()[0]))
    lastError = target
    while(error < -5 or error > 5):
        error = target - abs(hub.motion_sensor.tilt_angles()[0])
        output = kp * error + kd * (error - lastError)
        lastError = error
        if(abs(output) > speedMax): output = speedMax
        if(abs(output) < speedMin): output = speedMin
        motor_pair.move_tank(motor_pair.PAIR_1, 0, -int(output))
    motor_pair.stop(motor_pair.PAIR_1, stop=motor.SMART_BRAKE)

async def run2_WPI():
    hub.light_matrix.write("2")
    #---> You can modify after this <---
    await gyroForward(72.5, 1000, 150)
    await runloop.sleep_ms(1000)
    await gyroBackwards(57, 1000, -175)
    #---> You can't modify after this <---
    return

def play(): return not (hub.button.pressed(hub.button.RIGHT) or hub.button.pressed(hub.button.LEFT))
def pressPlay(): return (bool)(hub.button.pressed(hub.button.RIGHT) or hub.button.pressed(hub.button.LEFT))
async def main():
    motor_pair.pair(motor_pair.PAIR_1, leftDriveBaseMotor, rightDriveBaseMotor)
    await run2_WPI()
runloop.run(main())
