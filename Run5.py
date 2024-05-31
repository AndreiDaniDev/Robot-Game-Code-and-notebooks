#be aware that this was made in Spike Prime 3.4.0
import runloop, hub, motor, motor_pair, color

async def gyroForward(distance: float, speed: int, target: int = 0, * , kp: float = 0.25, kd: float = 0.3, stop: bool = True, reset: bool = True):
    d = int(distance * 20.45); lastError = 0
    if(reset): hub.motion_sensor.reset_yaw(0)
    motor.reset_relative_position(hub.port.B, 0)
    while(abs(motor.relative_position(hub.port.B)) < d):
        error = (hub.motion_sensor.tilt_angles()[0] - target) / 10
        proportional = error * -kp
        differential = (error - lastError) * -kd
        lastError = error + target
        motor_pair.move_tank(motor_pair.PAIR_1, int(speed + proportional + differential), int(speed - proportional - differential))
    if(stop): motor_pair.stop(motor_pair.PAIR_1, stop=motor.SMART_BRAKE)

async def gyroBackwards(distance: float, speed: int, target: int = 0, * , kp: float = 0.25, kd: float = 0.3 , stop: bool = True, reset: bool = True):
    d = int(distance * 20.45); lastError = 0; speed *= -1
    if(reset): hub.motion_sensor.reset_yaw(0)
    motor.reset_relative_position(hub.port.B, 0)
    while(abs(motor.relative_position(hub.port.B)) < d):
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
    motor_pair.stop(motor_pair.PAIR_1)

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
    motor_pair.stop(motor_pair.PAIR_1)

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
    motor_pair.stop(motor_pair.PAIR_1)

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
    motor_pair.stop(motor_pair.PAIR_1)

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
    motor_pair.stop(motor_pair.PAIR_1)

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
    motor_pair.stop(motor_pair.PAIR_1)

async def run5_WPI():
    #---> You can modify after this <---
    print("Run 5")

    hub.motion_sensor.reset_yaw(0)
    await motor.run_for_degrees(hub.port.C, 300, -1100)
    await runloop.sleep_ms(500)
    # ---> Getting in position <---
    await gyroForward(45, 600, kp = 0.4, kd = 0.25, reset = False)
    await runloop.sleep_ms(250)
    await turnLeft(42.5, 650, 125, kp = 0.75, kd = 1)
    await runloop.sleep_ms(250)

    # ---> Slowing down <---
    await gyroForward(5.5, 400, kp = 0.3, kd = 0.25, stop = False)
    await gyroForward(14.5, 500, kp = 0.3, kd = 0.25, reset = False, stop = False)
    await gyroForward(5, 250, kp = 0.3, kd = 0.25, reset = False)

    # ---> The marge of error <---
    await runloop.sleep_ms(500)
    await turnRight(77.5, 500, 175, kp = 0.75, kd = 1.2, reset = False)

    # ---> Dummy proof <---
    #    ->>> 20 lei <<<-
    await motor.run_for_degrees(hub.port.C, 300, 1100)
    await runloop.sleep_ms(250)
    await gyroForward(14, 500, target = 35, kp = 0.4, kd = 0.25)
    await gyroForward(10, 800, kp = 0.4, kd = 0.25)
    await runloop.sleep_ms(250)
    await gyroBackwards(14.5, 500, kp = 0.4, kd = 0.25)

    await turnLeft(60, 500, 250, kp = 0.4, kd = 0.25)
    await gyroBackwards(75, 1000, kp = 0.4, kd = 0.25)

    #---> You can't modify after this <---
    return

def play():
    return not (hub.button.pressed(hub.button.RIGHT) or hub.button.pressed(hub.button.LEFT))
def pressPlay():
    return (bool)(hub.button.pressed(hub.button.RIGHT) or hub.button.pressed(hub.button.LEFT))

async def main():
    motor_pair.pair(motor_pair.PAIR_1, hub.port.B, hub.port.D)
    await run5_WPI()

runloop.run(main())
