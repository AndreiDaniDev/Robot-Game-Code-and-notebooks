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


async def run1_WPI():
    hub.light_matrix.write("1")
    #---> You can modify after this <---
    print("Run 1 - Part 1")

    await gyroForward(60, 1000)
    await runloop.sleep_ms(100)
    await gyroBackwards(2, 400)
    await gyroForward(5, 1000)
    await runloop.sleep_ms(100)
    await motor.run_for_degrees(hub.port.A, -475, 1000)
    await runloop.sleep_ms(100)
    await gyroBackwards(57.5, 1000)

    hub.light.color(0, color.ORANGE)
    await runloop.until(pressPlay)
    hub.light.color(0, color.RED)

    print("Run 1 - Part 2")

    await motor.run_for_degrees(hub.port.C, -10, 200)
    motor.run_for_degrees(hub.port.A, 200, 1000)
    motor.run_for_degrees(hub.port.C, 350, 400)
    await gyroForward(50, 500, -18, stop=False, kp = 0.4)
    await gyroForward(16.75, 275)
    await runloop.sleep_ms(500)
    await motor.run_for_degrees(hub.port.C, -500, 400)
    await gyroForward(7, 200)
    await runloop.sleep_ms(500)
    await gyroBackwards(70, 1000, 750)
    #---> You can't modify after this <---
    return

async def run2_WPI():
    hub.light_matrix.write("2")
    #---> You can modify after this <---
    print("Run 2")
    await gyroForward(72.5, 1000, 150)
    await runloop.sleep_ms(1000)
    await gyroBackwards(57, 1000, -175)
    #---> You can't modify after this <---
    return

async def run3_WPI():
    hub.light_matrix.write("3")
    #---> You can modify after this <---
    print("Run 3")

    # ---> Getting in position <---
    await gyroForward(52.5, 750, kp = 0.4, kd = 0.25)
    await runloop.sleep_ms(100)
    await turnRightOneWheel(42, 500, 75, kp = 0.47, kd = 0.3, reset = False)
    await runloop.sleep_ms(250)
    await gyroForward(18.5, 375, kp = 0.4, kd = 0.25)
    await runloop.sleep_ms(100)
    await turnRightOneWheel(42, 500, 100, kp = 0.5, kd = 0.3)
    await runloop.sleep_ms(250)

    # ---> Solving M11 (Light Show) <---
    await gyroForward(51.6, 750, kp = 0.4, kd = 0.25)
    await runloop.sleep_ms(200)
    await turnRightOtherWheel(81.25, 500, 100, kp = 0.5, kd = 0.3, reset = False)
    await runloop.sleep_ms(250)
    await gyroForward(12.35, 250, kp = 0.4, kd = 0.3)
    await motor.run_for_degrees(hub.port.A, 500, 1000)

    # ---> Solving M5 (Augmented Reality Statue) <---
    await gyroBackwards(14.5, 250, kp = 0.4, kd = 0.3)
    await turnLeft(85, 400, 100, reset=False, kp = 0.5, kd = 0.3)
    await gyroForward(10.5, 300, kp = 0.4, kd = 0.3)
    await motor.run_for_degrees(hub.port.C, 75, 1000)

    await turnRight(40, 500, 100, reset=False, kp = 0.5, kd = 3) #
    await gyroForward(10, 250)
    await turnLeft(40, 500, 100, reset=False, kp = 0.3, kd = 0)
    await gyroForward(7.5, 500, stop = False)
    await gyroForward(115, 800, 200)

    #---> You can't modify after this <---
    return


async def run4_WPI():
    hub.light_matrix.write("4")
    #---> You can modify after this <---
    print("Run 4")
    await gyroForward(21.5, 800)
    await gyroForward(10, 400, reset=True)
    await runloop.sleep_ms(50)
    await motor.run_for_degrees(hub.port.A, 500, 325)
    await gyroForward(9.2, 300)
    await gyroBackwards(1, 200)
    await gyroForward(1.1, 200)
    await motor.run_for_degrees(hub.port.C, -2500, 1000)
    await gyroBackwards(50, 1000)
    #---> You can't modify after this <---
    return

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

async def run6_WPI():
    hub.light_matrix.write("6")
    #---> You can modify after this <---
    print("Run 6")
    await gyroForward(50, 1000, kp = 0.2, kd = 0.3)
    await turnRight(30, 500, 100, kp = 0.2, kd = 0)
    await gyroForward(120, 1000, -425, kp = 0.2, kd = 0.3)
    #---> You can't modify after this <---
    return

async def run7_WPI():
    hub.light_matrix.write("7")
    #---> You can modify after this <---
    print("Run 7")
    await gyroForward(27, 600, -35)
    await gyroBackwards(35, 1000)
    #---> You can't modify after this <---
    return

async def run8_WPI():
    hub.light_matrix.write("8")
    #---> You can modify after this <---
    print("Run 8")
    motor.run_for_degrees(hub.port.A, 375, 300)
    await gyroForward(20, 500, kp = 0.2, kd = 0.3)
    await turnLeft(10, 400, 150, reset = False, kp = 0.02, kd = 0)
    await gyroForward(37.5, 500, kp = 0.2, kd = 0.3)
    await turnLeft(65, 500, 150, reset = False)
    await gyroForward(45, 750, 50, kp = 0.2, kd = 0.3)

    motor.run_for_degrees(hub.port.A, -360, 500)
    await motor.run_for_degrees(hub.port.C, 500, 1000)
    await gyroBackwards(5, 500)
    #---> You can't modify after this <---
    return

async def playProgram(p: int):
    if(p == 1): await run1_WPI(); return;
    elif(p==2): await run2_WPI(); return;
    elif(p==3): await run3_WPI(); return;
    elif(p==4): await run4_WPI(); return;
    elif(p==5): await run5_WPI(); return;
    elif(p==6): await run6_WPI(); return;
    elif(p==7): await run7_WPI(); return;
    elif(p==8): await run8_WPI(); return;
    else: return
global program, releasedButton, frequency
def play():
    return not (hub.button.pressed(hub.button.RIGHT) or hub.button.pressed(hub.button.LEFT))
def pressPlay():
    return (bool)(hub.button.pressed(hub.button.RIGHT) or hub.button.pressed(hub.button.LEFT))
async def main():
    motor_pair.pair(motor_pair.PAIR_1, hub.port.B, hub.port.D)
    #await playProgram(3)
    program = 1; frequency = 250
    hub.light_matrix.write("1")
    hub.light.color(0, color.YELLOW)
    while(1):
        if(hub.button.pressed(hub.button.LEFT)):
            await runloop.sleep_ms(frequency)
            if(hub.button.pressed(hub.button.RIGHT)):
                hub.light.color(0, color.ORANGE)
                await runloop.until(play)
                await runloop.sleep_ms(50)
                hub.light.color(0, color.RED)
                await playProgram(program)
                hub.light_matrix.write(str(program))
                hub.light.color(0, color.YELLOW)
                program+=1
                if(program == 9): program = 1
                hub.light_matrix.write(str(program))
            else:
                program-=1
                if(program == 0): program = 8
                hub.light_matrix.write(str(program))
        elif(hub.button.pressed(hub.button.RIGHT)):
            await runloop.sleep_ms(frequency)
            if(hub.button.pressed(hub.button.LEFT)):
                hub.light.color(0, color.ORANGE)
                await runloop.until(play)
                await runloop.sleep_ms(50)
                hub.light.color(0, color.RED)
                await playProgram(program)
                hub.light_matrix.write(str(program))
                hub.light.color(0, color.YELLOW)
                program+=1
                if(program == 9): program = 1
                hub.light_matrix.write(str(program))
            else:
                program+=1
                if(program == 9): program = 1
                hub.light_matrix.write(str(program))
        await runloop.sleep_ms(50)

runloop.run(main())
