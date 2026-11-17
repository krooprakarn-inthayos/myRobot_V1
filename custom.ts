/**
 * iBIT Robot — บล็อกแบบเดียวกับ PTKidsBiT สำหรับ iBIT V2A
 * ต้องเพิ่ม Extension https://github.com/inexglobal/pxt-iBit2 ในโปรเจกต์ก่อน
 *
 * หุ่น: เซนเซอร์ 8 ตัว ADC0 (ซ้ายสุด) ... ADC7 (ขวาสุด)
 *       มอเตอร์ M1 = ล้อซ้าย, M2 = ล้อขวา
 *       Servo P8 (SV1) = แขนยก, Servo P12 (SV2) = ตัวคีบ
 *
 * ค่าเริ่มต้นแบบเดียวกับ PTKidsBiT:
 *   - PD ใช้เซนเซอร์แถวกลาง ADC1–ADC6 (GETPosition = 0 ถึง 5000, กลาง = 2500)
 *   - จับทางแยกด้วยเซนเซอร์ริม ADC0 (ซ้าย) และ ADC7 (ขวา)
 */

enum IR_Motor {
    //% block="Left"
    Left,
    //% block="Right"
    Right
}

enum IR_Dir {
    //% block="Left"
    Left,
    //% block="Right"
    Right
}

enum IR_Find {
    //% block="Left"
    Left,
    //% block="Center"
    Center,
    //% block="Right"
    Right
}

enum IR_ADC {
    //% block="ADC0"
    ADC0 = 0,
    //% block="ADC1"
    ADC1 = 1,
    //% block="ADC2"
    ADC2 = 2,
    //% block="ADC3"
    ADC3 = 3,
    //% block="ADC4"
    ADC4 = 4,
    //% block="ADC5"
    ADC5 = 5,
    //% block="ADC6"
    ADC6 = 6,
    //% block="ADC7"
    ADC7 = 7
}

enum IR_Servo {
    //% block="P8"
    P8,
    //% block="P12"
    P12
}

//% color="#51cb57" icon="" block="iBIT Robot"
//% groups='["Motor Control", "Sensor and ADC", "Line Follower", "Gripper"]'
namespace iBITRobot {
    const CH: ibitReadADC[] = [
        ibitReadADC.ADC0, ibitReadADC.ADC1, ibitReadADC.ADC2, ibitReadADC.ADC3,
        ibitReadADC.ADC4, ibitReadADC.ADC5, ibitReadADC.ADC6, ibitReadADC.ADC7
    ]

    // ค่าดำ (เส้น) และขาว (พื้น) ของเซนเซอร์แต่ละตัว
    let LINE = [4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095]
    let GROUND = [0, 0, 0, 0, 0, 0, 0, 0]

    // เซนเซอร์ที่ใช้คำนวณ PD และเซนเซอร์จับทางแยก
    let pdFirst = 1
    let pdLast = 6
    let junctionLeft = 0
    let junctionRight = 7

    let lastError = 0
    let dFilter = 0
    let lastPos = 2500
    let started = false

    // ตัวคีบ
    let armUpDeg = 45
    let armDownDeg = 110
    let gripOpenDeg = 45
    let gripCloseDeg = 0
    let armStepMs = 8
    let armNow = -1

    // ---------------- ภายใน ----------------

    function init() {
        if (!started) {
            iBIT.setADC_Address(adcAddress.iBIT_V2)
            started = true
        }
    }

    function raw(i: number): number {
        init()
        return iBIT.ReadADC(CH[i])
    }

    // 0 = พื้น, 1000 = เส้น (ใช้ได้ทั้งเซนเซอร์ที่ดำให้ค่าสูงหรือต่ำ)
    function val(i: number): number {
        let l = LINE[i]
        let g = GROUND[i]
        if (l == g) return 0
        return Math.constrain((raw(i) - g) * 1000 / (l - g), 0, 1000)
    }

    function isLine(i: number): boolean {
        return val(i) > 500
    }

    function setOne(ch: ibitMotorCH, v: number) {
        v = Math.round(Math.constrain(v, -100, 100))
        if (v >= 0) iBIT.setMotor(ch, ibitMotor.Forward, v)
        else iBIT.setMotor(ch, ibitMotor.Backward, -v)
    }

    function drive(left: number, right: number) {
        setOne(ibitMotorCH.M1, left)
        setOne(ibitMotorCH.M2, right)
    }

    function resetPD() {
        lastError = 0
        dFilter = 0
    }

    function pdStep(minSpeed: number, maxSpeed: number, kp: number, kd: number) {
        let center = (pdLast - pdFirst) * 500
        let e = GETPosition() - center
        dFilter = dFilter * 0.5 + (e - lastError) * 0.5
        lastError = e
        let pd = kp * e + kd * dFilter
        let l = minSpeed + pd
        let r = minSpeed - pd
        // ล้อใดเกิน Max Speed: ลดทั้งสองล้อเท่ากัน แรงเลี้ยวไม่หาย
        if (l > maxSpeed) {
            r = r - (l - maxSpeed)
            l = maxSpeed
        }
        if (r > maxSpeed) {
            l = l - (r - maxSpeed)
            r = maxSpeed
        }
        drive(Math.max(l, -maxSpeed), Math.max(r, -maxSpeed))
    }

    function readAverage(): number[] {
        let s = [0, 0, 0, 0, 0, 0, 0, 0]
        for (let k = 0; k < 30; k++) {
            for (let i = 0; i < 8; i++) {
                s[i] += raw(i)
            }
            basic.pause(5)
        }
        for (let i = 0; i < 8; i++) {
            s[i] = s[i] / 30
        }
        return s
    }

    function waitButtonA() {
        while (!input.buttonIsPressed(Button.A)) {
            basic.pause(10)
        }
        while (input.buttonIsPressed(Button.A)) {
            basic.pause(10)
        }
    }

    function beep() {
        music.playTone(Note.C5, music.beat(BeatFraction.Eighth))
    }

    // ---------------- Motor Control ----------------

    /**
     * สั่งมอเตอร์ทีละข้าง ความเร็ว -100 ถึง 100 (ค่าลบ = ถอยหลัง)
     */
    //% blockId=ir_motorWrite block="motorWrite %motor|Speed %speed"
    //% speed.min=-100 speed.max=100 speed.defl=0
    //% group="Motor Control" weight=100
    export function motorWrite(motor: IR_Motor, speed: number): void {
        if (motor == IR_Motor.Left) setOne(ibitMotorCH.M1, speed)
        else setOne(ibitMotorCH.M2, speed)
    }

    /**
     * สั่งมอเตอร์ซ้าย-ขวาพร้อมกัน ค่าละ -100 ถึง 100
     */
    //% blockId=ir_motorGo block="Motor Left %left|Motor Right %right"
    //% left.min=-100 left.max=100 left.defl=0 right.min=-100 right.max=100 right.defl=0
    //% inlineInputMode=inline
    //% group="Motor Control" weight=99
    export function motorGo(left: number, right: number): void {
        drive(left, right)
    }

    /**
     * เลี้ยวโดยหมุนมอเตอร์ข้างเดียว
     */
    //% blockId=ir_turn block="Turn %dir|Speed %speed"
    //% speed.min=0 speed.max=100 speed.defl=50
    //% group="Motor Control" weight=98
    export function turn(dir: IR_Dir, speed: number): void {
        if (dir == IR_Dir.Left) drive(0, speed)
        else drive(speed, 0)
    }

    /**
     * หมุนตัวอยู่กับที่โดยใช้มอเตอร์ 2 ตัว
     */
    //% blockId=ir_spin block="Spin %dir|Speed %speed"
    //% speed.min=0 speed.max=100 speed.defl=50
    //% group="Motor Control" weight=97
    export function spin(dir: IR_Dir, speed: number): void {
        if (dir == IR_Dir.Left) drive(-speed, speed)
        else drive(speed, -speed)
    }

    //% blockId=ir_stop block="Motor Stop"
    //% group="Motor Control" weight=96
    export function motorStop(): void {
        iBIT.MotorStop()
    }

    /**
     * หมุน Servo 0–180 องศา
     */
    //% blockId=ir_servo block="Servo %servo|Degree %degree"
    //% degree.min=0 degree.max=180 degree.defl=90
    //% group="Motor Control" weight=95
    export function servoWrite(servo: IR_Servo, degree: number): void {
        if (servo == IR_Servo.P8) {
            iBIT.Servo(ibitServo.SV1, degree)
            armNow = degree
        } else {
            iBIT.Servo(ibitServo.SV2, degree)
        }
    }

    // ---------------- Sensor and ADC ----------------

    /**
     * อ่านค่า ADC 0–4095
     */
    //% blockId=ir_adc block="ADCRead %ch"
    //% group="Sensor and ADC" weight=90
    export function ADCRead(ch: IR_ADC): number {
        return raw(ch)
    }

    // ---------------- Line Follower ----------------

    /**
     * Calibrate แบบ PTKidsBiT: วาง Sensor บนเส้น กด A รอเสียง แล้ววาง Sensor บนพื้น กด A รอเสียง
     * วางไว้ใน on start
     */
    //% blockId=ir_calibrate block="SensorCalibrate"
    //% group="Line Follower" weight=100
    export function sensorCalibrate(): void {
        // ขั้น 1: เส้น (จอทึบ)
        basic.showLeds(`
            # # # # #
            # # # # #
            # # # # #
            # # # # #
            # # # # #
            `)
        waitButtonA()
        LINE = readAverage()
        beep()
        // ขั้น 2: พื้น (จอโปร่ง)
        basic.showLeds(`
            # # # # #
            # . . . #
            # . . . #
            # . . . #
            # # # # #
            `)
        waitButtonA()
        GROUND = readAverage()
        beep()
        // ตรวจ: เซนเซอร์ตัวใดเส้นกับพื้นต่างกันไม่ถึง 200 ให้แสดงเลข ADC
        let bad = 0
        for (let i = 0; i < 8; i++) {
            if (Math.abs(LINE[i] - GROUND[i]) < 200) {
                bad += 1
                basic.showNumber(i)
                basic.pause(300)
            }
        }
        if (bad == 0) basic.showIcon(IconNames.Yes)
        else basic.showIcon(IconNames.Sad)
        resetPD()
    }

    /**
     * ใส่ค่าเส้นและพื้นเอง (8 ค่า เรียง ADC0–ADC7) ใช้แทน SensorCalibrate
     */
    //% blockId=ir_setcolor block="SETColorLine %line|Ground %ground"
    //% group="Line Follower" weight=95
    export function setColorLine(line: number[], ground: number[]): void {
        for (let i = 0; i < 8; i++) {
            if (i < line.length) LINE[i] = line[i]
            if (i < ground.length) GROUND[i] = ground[i]
        }
        resetPD()
    }

    /**
     * ส่งค่าเซนเซอร์ทั้ง 8 ตัวไปดูที่ Show data Device
     */
    //% blockId=ir_print block="PrintSensorValue"
    //% group="Line Follower" weight=94
    export function printSensorValue(): void {
        let s = ""
        for (let i = 0; i < 8; i++) {
            s = s + raw(i) + " "
        }
        serial.writeLine(s)
    }

    /**
     * ตำแหน่งเส้น 0 (ADC1) ถึง 5000 (ADC6), กลาง = 2500
     */
    //% blockId=ir_position block="GETPosition"
    //% group="Line Follower" weight=93
    export function GETPosition(): number {
        let sum = 0
        let wsum = 0
        let seen = false
        for (let i = pdFirst; i <= pdLast; i++) {
            let v = val(i)
            if (v > 200) seen = true
            sum += v
            wsum += v * (i - pdFirst) * 1000
        }
        if (!seen) {
            // หลุดเส้น: คืนค่าสุดขอบฝั่งที่เห็นเส้นครั้งสุดท้าย
            let c = (pdLast - pdFirst) * 500
            if (lastPos < c) return 0
            return (pdLast - pdFirst) * 1000
        }
        lastPos = wsum / sum
        return lastPos
    }

    /**
     * วิ่งตามเส้น 1 จังหวะ (ใส่ใน while/forever)
     */
    //% blockId=ir_pd block="Min Speed %minSpeed|Max Speed %maxSpeed|KP %kp|KD %kd"
    //% minSpeed.min=0 minSpeed.max=100 minSpeed.defl=50
    //% maxSpeed.min=0 maxSpeed.max=100 maxSpeed.defl=100
    //% kp.defl=0.012 kd.defl=0.15
    //% inlineInputMode=inline
    //% group="Line Follower" weight=90
    export function pdControl(minSpeed: number, maxSpeed: number, kp: number, kd: number): void {
        pdStep(minSpeed, maxSpeed, kp, kd)
    }

    /**
     * วิ่งตามเส้นจนเจอแยกที่เลือก แล้วหยุด
     * Left = ADC0 เจอเส้น, Right = ADC7 เจอเส้น, Center = ทั้งสองตัวเจอเส้น
     */
    //% blockId=ir_find block="Find %find|Min Speed %minSpeed|Max Speed %maxSpeed|KP %kp|KD %kd"
    //% minSpeed.min=0 minSpeed.max=100 minSpeed.defl=50
    //% maxSpeed.min=0 maxSpeed.max=100 maxSpeed.defl=100
    //% kp.defl=0.012 kd.defl=0.15
    //% inlineInputMode=inline
    //% group="Line Follower" weight=85
    export function findLine(find: IR_Find, minSpeed: number, maxSpeed: number, kp: number, kd: number): void {
        resetPD()
        while (true) {
            let l = isLine(junctionLeft)
            let r = isLine(junctionRight)
            if (find == IR_Find.Left && l) break
            if (find == IR_Find.Right && r) break
            if (find == IR_Find.Center && l && r) break
            pdStep(minSpeed, maxSpeed, kp, kd)
            basic.pause(1)
        }
        iBIT.MotorStop()
    }

    // เงื่อนไขเจอแยกตามชนิดที่เลือก
    function junctionFound(find: IR_Find): boolean {
        let l = isLine(junctionLeft)
        let r = isLine(junctionRight)
        if (find == IR_Find.Left) return l
        if (find == IR_Find.Right) return r
        return l && r
    }

    /**
     * วิ่งตามเส้นและนับแยกที่เลือก หยุดเมื่อเจอแยกครบตามจำนวน (Count)
     * ถ้าเริ่มบนแยกอยู่แล้ว จะไม่นับแยกนั้น
     * Left = ADC0 เจอเส้น, Right = ADC7 เจอเส้น, Center = ทั้งสองตัวเจอเส้น
     */
    //% blockId=ir_findcount block="Find %find|Count %count|Min Speed %minSpeed|Max Speed %maxSpeed|KP %kp|KD %kd"
    //% count.min=1 count.defl=2
    //% minSpeed.min=0 minSpeed.max=100 minSpeed.defl=50
    //% maxSpeed.min=0 maxSpeed.max=100 maxSpeed.defl=100
    //% kp.defl=0.012 kd.defl=0.15
    //% inlineInputMode=inline
    //% group="Line Follower" weight=84
    export function findCount(find: IR_Find, count: number, minSpeed: number, maxSpeed: number, kp: number, kd: number): void {
        resetPD()
        let found = 0
        // ถ้าเริ่มบนแยก ให้วิ่งจนพ้นแยกก่อน (ไม่นับ)
        while (junctionFound(find)) {
            pdStep(minSpeed, maxSpeed, kp, kd)
            basic.pause(1)
        }
        while (found < count) {
            if (junctionFound(find)) {
                found += 1
                if (found >= count) {
                    break
                }
                // ยังไม่ครบ: วิ่งต่อจนพ้นแยกนี้ แล้ววิ่งต่ออีก 30 ms กันนับซ้ำ
                while (junctionFound(find)) {
                    pdStep(minSpeed, maxSpeed, kp, kd)
                    basic.pause(1)
                }
                let t = input.runningTime()
                while (input.runningTime() - t < 30) {
                    pdStep(minSpeed, maxSpeed, kp, kd)
                    basic.pause(1)
                }
            } else {
                pdStep(minSpeed, maxSpeed, kp, kd)
                basic.pause(1)
            }
        }
        iBIT.MotorStop()
    }

    /**
     * วิ่งตามเส้นตามเวลาที่กำหนด (ms) แล้วหยุด
     */
    //% blockId=ir_time block="Time %ms|Min Speed %minSpeed|Max Speed %maxSpeed|KP %kp|KD %kd"
    //% ms.defl=200
    //% minSpeed.min=0 minSpeed.max=100 minSpeed.defl=50
    //% maxSpeed.min=0 maxSpeed.max=100 maxSpeed.defl=100
    //% kp.defl=0.012 kd.defl=0.15
    //% inlineInputMode=inline
    //% group="Line Follower" weight=80
    export function forwardTime(ms: number, minSpeed: number, maxSpeed: number, kp: number, kd: number): void {
        resetPD()
        let t = input.runningTime()
        while (input.runningTime() - t < ms) {
            pdStep(minSpeed, maxSpeed, kp, kd)
            basic.pause(1)
        }
        iBIT.MotorStop()
    }

    /**
     * หมุนเลี้ยว: ใช้ความเร็วเต็มช่วง Fast Time แล้วลดความเร็ว หยุดเมื่อ Sensor ที่เลือกเจอเส้น
     */
    //% blockId=ir_turnline block="TurnLINE %dir|Speed %speed|Sensor %sensor|Fast Time %fastTime"
    //% speed.min=0 speed.max=100 speed.defl=50
    //% fastTime.defl=200
    //% inlineInputMode=inline
    //% group="Line Follower" weight=75
    export function turnLine(dir: IR_Dir, speed: number, sensor: IR_ADC, fastTime: number): void {
        spin(dir, speed)
        basic.pause(fastTime)
        let slow = Math.max(Math.round(speed * 0.4), 20)
        spin(dir, slow)
        let t = input.runningTime()
        while (!isLine(sensor) && input.runningTime() - t < 4000) {
            basic.pause(1)
        }
        iBIT.MotorStop()
        resetPD()
        lastPos = (pdLast - pdFirst) * 500
    }

    /**
     * เลือกเซนเซอร์ที่ใช้คำนวณ PD (ค่าเริ่มต้น ADC1–ADC6)
     */
    //% blockId=ir_pdsensors block="Set PD Sensor from %first|to %last"
    //% first.defl=IR_ADC.ADC1 last.defl=IR_ADC.ADC6
    //% group="Line Follower" weight=70 advanced=true
    export function setPDSensors(first: IR_ADC, last: IR_ADC): void {
        if (last > first) {
            pdFirst = first
            pdLast = last
            lastPos = (pdLast - pdFirst) * 500
        }
    }

    /**
     * เลือกเซนเซอร์จับทางแยกซ้าย-ขวา (ค่าเริ่มต้น ADC0 และ ADC7)
     */
    //% blockId=ir_junction block="Set Junction Sensor Left %left|Right %right"
    //% left.defl=IR_ADC.ADC0 right.defl=IR_ADC.ADC7
    //% group="Line Follower" weight=69 advanced=true
    export function setJunctionSensors(left: IR_ADC, right: IR_ADC): void {
        junctionLeft = left
        junctionRight = right
    }

    // ---------------- Gripper ----------------

    /**
     * ตั้งค่ามุม Servo ของแขน (P8) และตัวคีบ (P12)
     */
    //% blockId=ir_setgripper block="Set Gripper Arm Up %up|Arm Down %down|Open %open|Close %close"
    //% up.defl=45 down.defl=110 open.defl=45 close.defl=0
    //% inlineInputMode=inline
    //% group="Gripper" weight=100
    export function setGripper(up: number, down: number, open: number, close: number): void {
        armUpDeg = up
        armDownDeg = down
        gripOpenDeg = open
        gripCloseDeg = close
    }

    function moveArm(target: number) {
        if (armNow < 0) {
            iBIT.Servo(ibitServo.SV1, target)
            armNow = target
            basic.pause(300)
            return
        }
        while (armNow != target) {
            armNow += armNow < target ? 1 : -1
            iBIT.Servo(ibitServo.SV1, armNow)
            basic.pause(armStepMs)
        }
    }

    //% blockId=ir_armup block="Arm Up"
    //% group="Gripper" weight=95
    export function armUp(): void {
        moveArm(armUpDeg)
    }

    //% blockId=ir_armdown block="Arm Down"
    //% group="Gripper" weight=94
    export function armDown(): void {
        moveArm(armDownDeg)
    }

    //% blockId=ir_gripopen block="Grip Open"
    //% group="Gripper" weight=93
    export function gripOpen(): void {
        iBIT.Servo(ibitServo.SV2, gripOpenDeg)
        basic.pause(300)
    }

    //% blockId=ir_gripclose block="Grip Close"
    //% group="Gripper" weight=92
    export function gripClose(): void {
        iBIT.Servo(ibitServo.SV2, gripCloseDeg)
        basic.pause(300)
    }

    /**
     * หยิบ: หยุด -> อ้า -> แขนลง -> คีบ -> ยก
     */
    //% blockId=ir_pick block="Pick Up"
    //% group="Gripper" weight=90
    export function pickUp(): void {
        iBIT.MotorStop()
        gripOpen()
        armDown()
        basic.pause(150)
        gripClose()
        armUp()
    }

    /**
     * วาง: หยุด -> แขนลง -> ปล่อย -> ถอยหลังเล็กน้อย -> ยกแขน
     */
    //% blockId=ir_put block="Put Down"
    //% group="Gripper" weight=89
    export function putDown(): void {
        iBIT.MotorStop()
        armDown()
        basic.pause(150)
        gripOpen()
        drive(-30, -30)
        basic.pause(200)
        iBIT.MotorStop()
        armUp()
    }

    /**
     * ความเร็วแขน: หน่วงต่อ 1 องศา (ms) ค่ามาก = ช้าและนุ่ม
     */
    //% blockId=ir_armspeed block="Set Arm Step %ms ms"
    //% ms.defl=8
    //% group="Gripper" weight=85 advanced=true
    export function setArmStep(ms: number): void {
        armStepMs = ms
    }
}
