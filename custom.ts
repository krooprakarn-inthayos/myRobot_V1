/**
 * iBIT Robot — บล็อกแบบเดียวกับ PTKidsBITRobot (iBuilds/pxt-PTKidsBIT-Robot) สำหรับ iBIT V2A
 * ต้องเพิ่ม Extension https://github.com/inexglobal/pxt-iBit2 ในโปรเจกต์ก่อน
 *
 * หุ่น: เซนเซอร์เส้น 8 ตัว ADC0 (ซ้ายสุด) ... ADC7 (ขวาสุด)
 *       มอเตอร์ M1 = ล้อซ้าย, M2 = ล้อขวา
 *       Servo P8 (SV1) = แขนยก, Servo P12 (SV2) = ตัวคีบ
 *       Ultrasonic (ถ้าติด): Trig = P1, Echo = P2 (ขาเดียวกับ PTKidsBiT)
 *
 * เทียบกับ PTKidsBiT (เซนเซอร์ 6 ตัว):
 *   - PD ใช้เซนเซอร์แถวกลาง: PTKidsBiT ใช้ 4 ตัวกลาง, iBIT Robot ใช้ ADC1–ADC6 (ปรับได้)
 *   - Find Center: PTKidsBiT = เห็นเส้น >= 5 จาก 6 ตัว, iBIT Robot = >= 6 จาก 8 ตัว
 *   - Find Left:   ADC0, ADC1, ADC2 เห็นเส้น และ ADC7 ไม่เห็น
 *   - Find Right:  ADC5, ADC6, ADC7 เห็นเส้น และ ADC0 ไม่เห็น
 *   - ไม่มีบล็อก LED Color / Sensor Color เพราะ iBIT ไม่มีไฟ NeoPixel
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

enum IR_TurnSensor {
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
    ADC7 = 7,
    //% block="Center"
    Center = 8
}

enum IR_Servo {
    //% block="P8"
    P8,
    //% block="P12"
    P12
}

// จุดบนสนาม PT KidsBiT ภารกิจส่งด่วน (นับจากมุมมองยืนที่ START มองเข้าสนาม)
// B1 B2 B3 B4 = ฝั่งซ้ายมือ (ฝั่งช่องขาว), B5 B6 B7 B8 = ฝั่งขวามือ (ฝั่งช่องดำ)
enum NavPlace {
    //% block="START"
    START = 0,
    //% block="B1"
    B1 = 13,
    //% block="B2"
    B2 = 19,
    //% block="B3"
    B3 = 25,
    //% block="B4"
    B4 = 26,
    //% block="B5"
    B5 = 27,
    //% block="B6"
    B6 = 28,
    //% block="B7"
    B7 = 16,
    //% block="B8"
    B8 = 22,
    //% block="จุดวางกล่องขาว"
    WhiteZone = 9,
    //% block="จุดวางกล่องดำ"
    BlackZone = 10,
    //% block="กล่องเหลือง"
    Yellow = 6
}

//% color="#51cb57" icon="" block="iBIT Robot"
//% groups='["Motor Control", "Servo Control", "Sensor and ADC", "Line Follower", "Gripper", "Navigation"]'
namespace iBITRobot {
    // MakeCode อาจรันโค้ด on start ของผู้ใช้ก่อนบรรทัดตั้งค่าตัวแปรในไฟล์นี้
    // จึงประกาศตัวแปรไว้เฉยๆ แล้วตั้งค่าเริ่มต้นใน initState() ซึ่งทุกบล็อกเรียกก่อนทำงาน
    let CH: ibitReadADC[]
    let ON_LINE: number          // ค่าที่ถือว่าเห็นเส้น (0–1000) เหมือน PTKidsBiT
    let CENTER_COUNT: number     // Find Center: เห็นเส้นอย่างน้อยกี่ตัวจาก 8

    // ค่าเส้น (Color Line) และพื้น (Color Ground) ของเซนเซอร์แต่ละตัว
    let LINE: number[]
    let GROUND: number[]
    // ค่าเซนเซอร์ล่าสุด 0 (พื้น) – 1000 (เส้น)
    let V: number[]

    let pdFirst: number
    let pdLast: number
    let previousError: number
    let lastPos: number
    let started: boolean

    // ส่วนเสริม (PTKidsBiT ไม่มี): ชดเชยมอเตอร์, เบรก, ตัวคีบ
    let compLeft: number
    let compRight: number
    let brakeMs: number
    let armUpDeg: number
    let armDownDeg: number
    let gripOpenDeg: number
    let gripCloseDeg: number
    let armStepMs: number
    let armNow: number

    let ready: boolean
    function initState() {
        if (ready) return
        ready = true
        CH = [
            ibitReadADC.ADC0, ibitReadADC.ADC1, ibitReadADC.ADC2, ibitReadADC.ADC3,
            ibitReadADC.ADC4, ibitReadADC.ADC5, ibitReadADC.ADC6, ibitReadADC.ADC7
        ]
        ON_LINE = 500
        CENTER_COUNT = 6
        LINE = [4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095]
        GROUND = [0, 0, 0, 0, 0, 0, 0, 0]
        V = [0, 0, 0, 0, 0, 0, 0, 0]
        pdFirst = 1
        pdLast = 6
        previousError = 0
        lastPos = 2500
        started = false
        compLeft = 0
        compRight = 0
        brakeMs = 0
        armUpDeg = 45
        armDownDeg = 110
        gripOpenDeg = 45
        gripCloseDeg = 0
        armStepMs = 8
        armNow = -1
    }

    // ================= ภายใน =================

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

    // อ่านเซนเซอร์ทั้ง 8 ตัว แปลงเป็น 0 (พื้น) – 1000 (เส้น) เก็บใน V
    function readAll() {
        for (let i = 0; i < 8; i++) {
            let l = LINE[i]
            let g = GROUND[i]
            if (l == g) {
                V[i] = 0
            } else {
                V[i] = Math.constrain((raw(i) - g) * 1000 / (l - g), 0, 1000)
            }
        }
    }

    function onLine(i: number): boolean {
        return V[i] > ON_LINE
    }

    function countOnLine(): number {
        let c = 0
        for (let i = 0; i < 8; i++) {
            if (V[i] > ON_LINE) c += 1
        }
        return c
    }

    // ตำแหน่งเส้นจากค่า V: 0 = ตัวแรกของ PD (ซ้าย) ... (n-1)*1000 = ตัวสุดท้าย (ขวา)
    function positionFromV(): number {
        let n = pdLast - pdFirst + 1
        let sum = 0
        let wsum = 0
        let seen = false
        for (let k = 0; k < n; k++) {
            let v = V[pdFirst + k]
            if (v > 200) seen = true
            sum += v
            wsum += v * k * 1000
        }
        if (!seen) {
            // หลุดเส้น: คืนค่าสุดขอบฝั่งที่เห็นเส้นครั้งสุดท้าย (แบบ PTKidsBiT)
            if (lastPos < (n - 1) * 500) return 0
            return (n - 1) * 1000
        }
        lastPos = wsum / sum
        return Math.round(lastPos)
    }

    function setOne(ch: ibitMotorCH, v: number) {
        v = Math.round(Math.constrain(v, -100, 100))
        if (v >= 0) iBIT.setMotor(ch, ibitMotor.Forward, v)
        else iBIT.setMotor(ch, ibitMotor.Backward, -v)
    }

    // ชดเชยความเร็วเฉพาะตอนล้อหมุน และชดเชยตามทิศ (ถอยหลังก็เร็วขึ้นเท่ากัน) สั่ง 0 ล้อหยุดจริง
    function withComp(v: number, comp: number): number {
        if (v == 0) return 0
        if (v > 0) return v + comp
        return v - comp
    }

    // เหมือน motorGo ของ PTKidsBiT: บวกค่าชดเชยแล้วจำกัด -100..100
    function drive(left: number, right: number) {
        setOne(ibitMotorCH.M1, withComp(left, compLeft))
        setOne(ibitMotorCH.M2, withComp(right, compRight))
    }

    // หยุด: ถ้าตั้ง Brake Time จะสั่งกลับทิศสั้น ๆ ก่อนหยุด
    function stopWithBrake(left: number, right: number) {
        if (brakeMs > 0) {
            drive(-left, -right)
            basic.pause(brakeMs)
        }
        iBIT.MotorStop()
    }

    // PD 1 จังหวะ จากค่า V ที่อ่านไว้แล้ว (สูตรเดียวกับ PTKidsBiT)
    function pdFromV(minSpeed: number, maxSpeed: number, kp: number, kd: number) {
        let n = pdLast - pdFirst + 1
        let error = positionFromV() - (n - 1) * 500
        let pd = kp * error + kd * (error - previousError)
        previousError = error
        let l = minSpeed + pd
        let r = minSpeed - pd
        // ส่วนเสริม: ล้อใดเกิน Max Speed ให้ลดทั้งสองล้อเท่ากัน แรงเลี้ยวไม่หาย
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

    // เงื่อนไขเจอแยก (ใช้กับ Find และ Find Count)
    function junction(find: IR_Find): boolean {
        if (find == IR_Find.Center) {
            return countOnLine() >= CENTER_COUNT
        } else if (find == IR_Find.Left) {
            return onLine(0) && onLine(1) && onLine(2) && !onLine(7)
        } else {
            return onLine(5) && onLine(6) && onLine(7) && !onLine(0)
        }
    }

    function turnSensorOnLine(sensor: IR_TurnSensor): boolean {
        if (sensor == IR_TurnSensor.Center) return V[3] >= ON_LINE || V[4] >= ON_LINE
        return V[sensor] >= ON_LINE
    }

    function tone(freq: number) {
        music.playTone(freq, music.beat(BeatFraction.Quarter))
    }

    function waitButtonA() {
        while (!input.buttonIsPressed(Button.A)) {
            basic.pause(10)
        }
        while (input.buttonIsPressed(Button.A)) {
            basic.pause(10)
        }
    }

    function readAverage20(): number[] {
        let s = [0, 0, 0, 0, 0, 0, 0, 0]
        for (let k = 0; k < 20; k++) {
            for (let i = 0; i < 8; i++) {
                s[i] += raw(i)
            }
            basic.pause(50)
        }
        for (let i = 0; i < 8; i++) {
            s[i] = s[i] / 20
        }
        return s
    }

    function printValues(title: string, values: number[]) {
        let s = title
        for (let i = 0; i < 8; i++) {
            s = s + " " + Math.round(values[i])
        }
        serial.writeLine(s)
    }

    // ================= Motor Control =================

    /**
     * ควบคุมมอเตอร์ 1 ช่อง ความเร็ว -100 ถึง 100 (มากกว่า 0 = เดินหน้า, น้อยกว่า 0 = ถอยหลัง)
     */
    //% blockId=ir_motorWrite block="motorWrite %motor|Speed %speed"
    //% speed.min=-100 speed.max=100 speed.defl=50
    //% group="Motor Control" weight=100
    export function motorWrite(motor: IR_Motor, speed: number): void {
        initState()
        if (motor == IR_Motor.Left) drive(speed, 0)
        else drive(0, speed)
    }

    /**
     * ควบคุมมอเตอร์ซ้าย-ขวาพร้อมกัน ค่าละ -100 ถึง 100
     */
    //% blockId=ir_motorGo block="Motor Left %left|Motor Right %right"
    //% left.min=-100 left.max=100 left.defl=50 right.min=-100 right.max=100 right.defl=50
    //% inlineInputMode=inline
    //% group="Motor Control" weight=99
    export function motorGo(left: number, right: number): void {
        initState()
        drive(left, right)
    }

    /**
     * เลี้ยว: มอเตอร์ข้างหนึ่งหยุด อีกข้างหมุน
     */
    //% blockId=ir_turn block="Turn %dir|Speed %speed"
    //% speed.min=0 speed.max=100 speed.defl=50
    //% group="Motor Control" weight=98
    export function Turn(dir: IR_Dir, speed: number): void {
        initState()
        if (dir == IR_Dir.Left) drive(0, speed)
        else drive(speed, 0)
    }

    /**
     * หมุนตัวอยู่กับที่: มอเตอร์หมุนสวนทางกัน
     */
    //% blockId=ir_spin block="Spin %dir|Speed %speed"
    //% speed.min=0 speed.max=100 speed.defl=50
    //% group="Motor Control" weight=97
    export function Spin(dir: IR_Dir, speed: number): void {
        initState()
        if (dir == IR_Dir.Left) drive(-speed, speed)
        else drive(speed, -speed)
    }

    //% blockId=ir_stop block="Motor Stop"
    //% group="Motor Control" weight=96
    export function motorStop(): void {
        initState()
        iBIT.MotorStop()
    }

    /**
     * (ส่วนเสริม) ชดเชยความเร็วเมื่อหุ่นวิ่งไม่ตรง ค่าจะถูกบวกเข้าไปในความเร็วของล้อนั้นทุกครั้ง
     */
    //% blockId=ir_compensate block="Compensate Left %left|Compensate Right %right"
    //% left.min=-100 left.max=100 left.defl=0 right.min=-100 right.max=100 right.defl=0
    //% inlineInputMode=inline
    //% group="Motor Control" weight=90 advanced=true
    export function motorCompensate(left: number, right: number): void {
        initState()
        compLeft = left
        compRight = right
    }

    /**
     * (ส่วนเสริม) เวลาเบรก (ms) หลัง Find / Count / Custom / Time / TurnLINE: สั่งกลับทิศสั้น ๆ ก่อนหยุด 0 = ไม่เบรก (แบบ PTKidsBiT)
     */
    //% blockId=ir_brake block="Set Brake Time %ms ms"
    //% ms.min=0 ms.max=200 ms.defl=0
    //% group="Motor Control" weight=89 advanced=true
    export function setBrakeTime(ms: number): void {
        initState()
        brakeMs = ms
    }

    // ================= Servo Control =================

    /**
     * ควบคุม Servo 0–180 องศา (P8 = Servo1, P12 = Servo2 ของ iBIT)
     */
    //% blockId=ir_servo block="Servo %servo|Degree %degree"
    //% degree.min=0 degree.max=180 degree.defl=90
    //% group="Servo Control" weight=100
    export function servoWrite(servo: IR_Servo, degree: number): void {
        initState()
        if (servo == IR_Servo.P8) {
            iBIT.Servo(ibitServo.SV1, degree)
            armNow = degree
        } else {
            iBIT.Servo(ibitServo.SV2, degree)
        }
    }

    /**
     * (ส่วนเสริม) ปล่อย Servo หยุดจ่ายสัญญาณ ลดเสียงหึ่งและประหยัดแบต
     */
    //% blockId=ir_servostop block="Servo Stop %servo"
    //% group="Servo Control" weight=95
    export function servoStop(servo: IR_Servo): void {
        initState()
        if (servo == IR_Servo.P8) {
            iBIT.ServoStop(ibitServo.SV1)
            armNow = -1
        } else {
            iBIT.ServoStop(ibitServo.SV2)
        }
    }

    // ================= Sensor and ADC =================

    /**
     * อ่านค่า Analog จากช่อง ADC 0–4095
     */
    //% blockId=ir_adc block="ADCRead %ch"
    //% group="Sensor and ADC" weight=100
    export function ADCRead(ch: IR_ADC): number {
        initState()
        return raw(ch)
    }

    /**
     * ระยะจาก Ultrasonic (cm) Trig = P1, Echo = P2 เหมือน PTKidsBiT ถ้าไม่มีเสียงสะท้อนหรือเกิน 400 cm จะได้ 400
     */
    //% blockId=ir_distance block="GETDistance"
    //% group="Sensor and ADC" weight=95
    export function distanceRead(): number {
        initState()
        pins.setPull(DigitalPin.P2, PinPullMode.PullNone)
        pins.digitalWritePin(DigitalPin.P1, 0)
        control.waitMicros(2)
        pins.digitalWritePin(DigitalPin.P1, 1)
        control.waitMicros(10)
        pins.digitalWritePin(DigitalPin.P1, 0)
        let d = pins.pulseIn(DigitalPin.P2, PulseValue.High, 500 * 58)
        let x = d / 58
        if (x <= 0 || x > 400) {
            return 400
        }
        return Math.round(x)
    }

    // ================= Line Follower =================

    /**
     * Calibrate แบบ PTKidsBiT: เสียงเริ่ม -> วางเซนเซอร์บนเส้น กด A -> เสียง -> วางบนพื้น กด A -> เสียงจบ
     * ส่งค่า Color Line / Color Ground ไปที่ Show data Device
     */
    //% blockId=ir_calibrate block="SensorCalibrate"
    //% group="Line Follower" weight=100
    export function SensorCalibrate(): void {
        initState()
        iBIT.MotorStop()
        serial.writeLine("")
        tone(587)
        tone(784)
        // ขั้น 1: เส้น (จอทึบ)
        basic.showLeds(`
            # # # # #
            # # # # #
            # # # # #
            # # # # #
            # # # # #
            `)
        waitButtonA()
        tone(784)
        LINE = readAverage20()
        tone(784)
        // ขั้น 2: พื้น (จอโปร่ง)
        basic.showLeds(`
            # # # # #
            # . . . #
            # . . . #
            # . . . #
            # # # # #
            `)
        waitButtonA()
        tone(784)
        GROUND = readAverage20()
        tone(784)
        tone(587)
        serial.writeLine("------------------------------------")
        printValues("Color Line:", LINE)
        printValues("Color Ground:", GROUND)
        // ส่วนเสริม: แจ้งเลข ADC ที่เส้นกับพื้นต่างกันไม่ถึง 200
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
        previousError = 0
    }

    /**
     * ใส่ค่าเส้น (Line) และพื้น (Ground) เอง 8 ค่า เรียง ADC0–ADC7 ใช้แทน SensorCalibrate
     */
    //% blockId=ir_setcolor block="SETColorLine $line|Ground $ground"
    //% group="Line Follower" weight=95
    export function ValueSensorSET(line: number[], ground: number[]): void {
        initState()
        for (let i = 0; i < 8; i++) {
            if (i < line.length) LINE[i] = line[i]
            if (i < ground.length) GROUND[i] = ground[i]
        }
        previousError = 0
    }

    /**
     * ส่งค่าดิบของเซนเซอร์ทั้ง 8 ตัวไปที่ Show data Device
     */
    //% blockId=ir_print block="PrintSensorValue"
    //% group="Line Follower" weight=94
    export function PrintSensorValue(): void {
        initState()
        let s = "Sensor Value:"
        for (let i = 0; i < 8; i++) {
            s = s + " " + raw(i)
        }
        serial.writeLine(s)
    }

    /**
     * ตำแหน่งเส้น: 0 = ซ้ายสุดของ PD ... 5000 = ขวาสุด, กลาง = 2500 (ค่าเริ่มต้น PD = ADC1–ADC6)
     */
    //% blockId=ir_position block="GETPosition"
    //% group="Line Follower" weight=93
    export function GETPosition(): number {
        initState()
        readAll()
        return positionFromV()
    }

    /**
     * วิ่งตามเส้น 1 จังหวะ ใส่ใน forever หรือ while
     */
    //% blockId=ir_pd block="Min Speed %minSpeed|Max Speed %maxSpeed|KP %kp|KD %kd"
    //% minSpeed.min=0 minSpeed.max=100 minSpeed.defl=30
    //% maxSpeed.min=0 maxSpeed.max=100 maxSpeed.defl=100
    //% kp.defl=0.01 kd.defl=0
    //% inlineInputMode=inline
    //% group="Line Follower" weight=90
    export function Follower(minSpeed: number, maxSpeed: number, kp: number, kd: number): void {
        initState()
        readAll()
        pdFromV(minSpeed, maxSpeed, kp, kd)
    }

    /**
     * วิ่งตามเส้นจนเจอแยกที่เลือกแล้วหยุด (ถ้าเริ่มบนแยกกลาง จะวิ่งตรงให้พ้นก่อน)
     */
    //% blockId=ir_find block="Find %find|Min Speed %minSpeed|Max Speed %maxSpeed|KP %kp|KD %kd"
    //% find.defl=IR_Find.Center
    //% minSpeed.min=0 minSpeed.max=100 minSpeed.defl=30
    //% maxSpeed.min=0 maxSpeed.max=100 maxSpeed.defl=100
    //% kp.defl=0.01 kd.defl=0
    //% inlineInputMode=inline
    //% group="Line Follower" weight=85
    export function ForwardLINE(find: IR_Find, minSpeed: number, maxSpeed: number, kp: number, kd: number): void {
        initState()
        // ช่วงที่ 1: ถ้ายืนบนแยก ให้วิ่งตรงจนพ้น แล้วเดินหน้าต่อ 20 ms
        readAll()
        while (countOnLine() >= CENTER_COUNT) {
            drive(minSpeed, minSpeed)
            basic.pause(1)
            readAll()
        }
        drive(50, 50)
        basic.pause(20)
        // ช่วงที่ 2: วิ่งตามเส้นจนเจอแยก แล้วเดินหน้าต่อ 20 ms ก่อนหยุด
        while (true) {
            readAll()
            if (junction(find)) {
                drive(50, 50)
                basic.pause(20)
                stopWithBrake(50, 50)
                break
            }
            pdFromV(minSpeed, maxSpeed, kp, kd)
        }
    }

    /**
     * วิ่งตามเส้นและนับแยกที่เลือก หยุดเมื่อครบจำนวน (แยกที่ยังไม่ครบ หุ่นจะวิ่งตรงผ่านไป)
     */
    //% blockId=ir_findcount block="Find %find|Count %count|Min Speed %minSpeed|Max Speed %maxSpeed|KP %kp|KD %kd"
    //% find.defl=IR_Find.Center
    //% count.min=1 count.defl=1
    //% minSpeed.min=0 minSpeed.max=100 minSpeed.defl=30
    //% maxSpeed.min=0 maxSpeed.max=100 maxSpeed.defl=100
    //% kp.defl=0.01 kd.defl=0
    //% inlineInputMode=inline
    //% group="Line Follower" weight=84
    export function ForwardLINECount(find: IR_Find, count: number, minSpeed: number, maxSpeed: number, kp: number, kd: number): void {
        initState()
        let found = 0
        while (true) {
            readAll()
            if (junction(find)) {
                found += 1
                if (found >= count) {
                    stopWithBrake(minSpeed, minSpeed)
                    break
                }
                // ยังไม่ครบ: วิ่งตรงจนพ้นแยก แล้ววิ่งต่ออีก 100 ms
                while (junction(find)) {
                    drive(minSpeed, minSpeed)
                    basic.pause(1)
                    readAll()
                }
                basic.pause(100)
                readAll()
            }
            pdFromV(minSpeed, maxSpeed, kp, kd)
        }
    }

    /**
     * วิ่งตามเส้นจนเซนเซอร์ตรงกับรูปแบบที่กำหนด แล้วหยุด
     * รูปแบบ 8 ตัวอักษร เรียง ADC0–ADC7: 1 = ต้องเห็นเส้น, 0 = ต้องเห็นพื้น, - = ไม่สนใจ
     * เช่น "1------1" = ADC0 และ ADC7 เห็นเส้น
     */
    //% blockId=ir_findcustom block="Find %pattern|Min Speed %minSpeed|Max Speed %maxSpeed|KP %kp|KD %kd"
    //% pattern.defl="1------1"
    //% minSpeed.min=0 minSpeed.max=100 minSpeed.defl=30
    //% maxSpeed.min=0 maxSpeed.max=100 maxSpeed.defl=100
    //% kp.defl=0.01 kd.defl=0
    //% inlineInputMode=inline
    //% group="Line Follower" weight=83
    export function ForwardLINECustom(pattern: string, minSpeed: number, maxSpeed: number, kp: number, kd: number): void {
        initState()
        let want = [2, 2, 2, 2, 2, 2, 2, 2]
        let need = 0
        for (let i = 0; i < 8; i++) {
            let c = pattern.charAt(i)
            if (c == "1") {
                want[i] = 1
                need += 1
            } else if (c == "0") {
                want[i] = 0
                need += 1
            }
        }
        while (true) {
            readAll()
            let match = 0
            for (let i = 0; i < 8; i++) {
                if (want[i] == 1 && V[i] > 800) match += 1
                else if (want[i] == 0 && V[i] < 100) match += 1
            }
            if (match >= need) {
                stopWithBrake(minSpeed, minSpeed)
                break
            }
            pdFromV(minSpeed, maxSpeed, kp, kd)
        }
    }

    /**
     * วิ่งตามเส้นตามเวลาที่กำหนด (ms) แล้วหยุด
     */
    //% blockId=ir_time block="Time %ms|Min Speed %minSpeed|Max Speed %maxSpeed|KP %kp|KD %kd"
    //% ms.shadow="timePicker" ms.defl=200
    //% minSpeed.min=0 minSpeed.max=100 minSpeed.defl=30
    //% maxSpeed.min=0 maxSpeed.max=100 maxSpeed.defl=100
    //% kp.defl=0.01 kd.defl=0
    //% inlineInputMode=inline
    //% group="Line Follower" weight=80
    export function ForwardTIME(ms: number, minSpeed: number, maxSpeed: number, kp: number, kd: number): void {
        initState()
        let t = control.millis()
        while (control.millis() - t < ms) {
            readAll()
            pdFromV(minSpeed, maxSpeed, kp, kd)
        }
        stopWithBrake(minSpeed, minSpeed)
    }

    /**
     * ถอยหลังตรงตามเวลา (ms) แล้วเบรก ไม่ใช้เซนเซอร์
     */
    //% blockId=ir_backtime block="Backward Time %ms|Speed %speed"
    //% ms.shadow="timePicker" ms.defl=300
    //% speed.min=0 speed.max=100 speed.defl=40
    //% inlineInputMode=inline
    //% group="Line Follower" weight=79
    export function BackwardTIME(ms: number, speed: number): void {
        initState()
        drive(-speed, -speed)
        basic.pause(ms)
        stopWithBrake(-speed, -speed)
    }

    /**
     * ถอยหลังจนเซนเซอร์เจอเส้นตัด (Center / Left / Right) แล้วเบรก
     * ระหว่างถอยปรับทิศเล็กน้อยด้วย KP ให้เซนเซอร์อยู่บนเส้น (ไม่มี KD)
     * เหมาะกับระยะสั้นและความเร็วต่ำ เพราะเซนเซอร์อยู่หน้าล้อ ถอยไกลตัวหุ่นจะค่อย ๆ เฉียงออก
     * ถ้าเริ่มบนเส้นตัด จะถอยพ้นเส้นนั้นก่อน
     * หยุดแล้วแกนล้อจะเลยเส้นตัดไป ถ้าจะเลี้ยวต่อให้เดินหน้าสั้น ๆ ก่อน
     */
    //% blockId=ir_backfind block="Backward Find %find|Speed %speed|KP %kp"
    //% speed.min=0 speed.max=100 speed.defl=35
    //% kp.defl=0.01
    //% inlineInputMode=inline
    //% group="Line Follower" weight=78
    export function BackwardLINE(find: IR_Find, speed: number, kp: number): void {
        initState()
        let n = pdLast - pdFirst + 1
        // ถอยพ้นเส้นตัดที่ยืนอยู่
        readAll()
        while (junction(find)) {
            drive(-speed, -speed)
            readAll()
        }
        while (true) {
            readAll()
            if (junction(find)) break
            let error = positionFromV() - (n - 1) * 500
            // จำกัดแรงแก้ไม่เกินความเร็ว: ล้อจะไม่หมุนไปข้างหน้า หุ่นไม่หมุนวน
            let pd = Math.constrain(kp * error, -speed, speed)
            drive(-speed + pd, -speed - pd)
        }
        stopWithBrake(-speed, -speed)
        previousError = 0
    }

    /**
     * หมุนเลี้ยวแบบ PTKidsBiT: หมุนให้พ้นเส้นเดิมก่อน แล้วหมุนจน Sensor ที่เลือกเจอเส้นใหม่
     * ใช้ความเร็ว Speed ช่วง Fast Time (ms) จากนั้นลดเหลือ 20
     * Center = ADC3 หรือ ADC4 เจอเส้น
     */
    //% blockId=ir_turnline block="TurnLINE %dir|Speed %speed|Sensor %sensor|Fast Time %fastTime"
    //% speed.min=0 speed.max=100 speed.defl=50
    //% sensor.defl=IR_TurnSensor.Center
    //% fastTime.shadow="timePicker" fastTime.defl=200
    //% inlineInputMode=inline
    //% group="Line Follower" weight=75
    export function TurnLINE(dir: IR_Dir, speed: number, sensor: IR_TurnSensor, fastTime: number): void {
        initState()
        let slow = 20
        let timer = control.millis()
        let s = 0
        // ช่วงที่ 1: หมุนจนเซนเซอร์กลาง 4 ตัว (ADC2–ADC5) พ้นเส้นเดิม
        while (true) {
            readAll()
            if (!(onLine(2) || onLine(3) || onLine(4) || onLine(5))) break
            s = timer + fastTime - control.millis()
            if (s > speed) s = speed
            else if (s < 0) s = slow
            if (dir == IR_Dir.Left) drive(-s, s)
            else drive(s, -s)
        }
        // ช่วงที่ 2: หมุนจน Sensor ที่เลือกเจอเส้น
        while (true) {
            readAll()
            if (turnSensorOnLine(sensor)) break
            s = timer + fastTime - control.millis()
            if (s > speed) s = speed
            else if (s < 0) s = slow
            if (dir == IR_Dir.Left) drive(-s, s)
            else drive(s, -s)
        }
        if (dir == IR_Dir.Left) stopWithBrake(-s, s)
        else stopWithBrake(s, -s)
        previousError = 0
        lastPos = (pdLast - pdFirst) * 500
    }

    /**
     * (ส่วนเสริม) เลือกเซนเซอร์ที่ใช้คำนวณ PD ค่าเริ่มต้น ADC1–ADC6
     */
    //% blockId=ir_pdsensors block="Set PD Sensor from %first|to %last"
    //% first.defl=IR_ADC.ADC1 last.defl=IR_ADC.ADC6
    //% group="Line Follower" weight=70 advanced=true
    export function setPDSensors(first: IR_ADC, last: IR_ADC): void {
        initState()
        if (last > first) {
            pdFirst = first
            pdLast = last
            lastPos = (pdLast - pdFirst) * 500
            previousError = 0
        }
    }

    // ================= Gripper (ส่วนเสริม) =================

    /**
     * ตั้งค่ามุมแขน (P8) และตัวคีบ (P12)
     */
    //% blockId=ir_setgripper block="Set Gripper Arm Up %up|Arm Down %down|Open %open|Close %close"
    //% up.defl=45 down.defl=110 open.defl=45 close.defl=0
    //% inlineInputMode=inline
    //% group="Gripper" weight=100
    export function setGripper(up: number, down: number, open: number, close: number): void {
        initState()
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
        } else {
            while (armNow != target) {
                armNow += armNow < target ? 1 : -1
                iBIT.Servo(ibitServo.SV1, armNow)
                basic.pause(armStepMs)
            }
        }
    }

    //% blockId=ir_armup block="Arm Up"
    //% group="Gripper" weight=95
    export function armUp(): void {
        initState()
        moveArm(armUpDeg)
    }

    //% blockId=ir_armdown block="Arm Down"
    //% group="Gripper" weight=94
    export function armDown(): void {
        initState()
        moveArm(armDownDeg)
    }

    //% blockId=ir_gripopen block="Grip Open"
    //% group="Gripper" weight=93
    export function gripOpen(): void {
        initState()
        iBIT.Servo(ibitServo.SV2, gripOpenDeg)
        basic.pause(300)
    }

    //% blockId=ir_gripclose block="Grip Close"
    //% group="Gripper" weight=92
    export function gripClose(): void {
        initState()
        iBIT.Servo(ibitServo.SV2, gripCloseDeg)
        basic.pause(300)
    }

    /**
     * หยิบ: หยุด -> อ้า -> แขนลง -> คีบ -> ยก
     */
    //% blockId=ir_pick block="Pick Up"
    //% group="Gripper" weight=90
    export function pickUp(): void {
        initState()
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
        initState()
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
     * ความเร็วแขน: หน่วงต่อ 1 องศา (ms) มาก = ช้าและนุ่ม
     */
    //% blockId=ir_armspeed block="Set Arm Step %ms ms"
    //% ms.defl=8
    //% group="Gripper" weight=85 advanced=true
    export function setArmStep(ms: number): void {
        initState()
        armStepMs = ms
    }

    // ======================= Navigation: วิ่งจากจุดหนึ่งไปอีกจุดเอง =======================
    // แผนที่สนาม (ซม. โดยประมาณจากภาพสนาม) มองจาก START: x = ไปทางกล่องเหลือง, y = ไปทางฝั่งช่องขาว
    // node: 0 START, 1-5 ทางแยก 1-5 บนเส้นหลัก, 6 กล่องเหลือง, 7/8 ทางแยกไปช่องขาว/ดำ,
    //       9/10 ช่องขาว/ดำ, 11-24 ทางแยกบนเส้นตั้งและแถว, 13 16 19 22 25-28 จุดวางกล่อง
    let NX: number[]
    let NY: number[]
    let NB: number[][]
    let navReady: boolean
    let navMin: number
    let navMax: number
    let navKp: number
    let navKd: number
    let navSlow: number
    let navTurnSpeed: number
    let navTurnFast: number
    let navCrossMs: number
    let navStartMs: number
    let navHomeMs: number
    let navBoxStraightMs: number
    let navBoxTurnMs: number
    let navBoxBackMs: number
    let navZoneMs: number
    let navZoneBackMs: number
    let navYellowMs: number
    let navApproach: number
    let navBrakeMs: number

    function initNav() {
        if (navReady) return
        navReady = true
        NX = [0, 16, 36, 51, 66, 76, 84, 16, 16, 0, 0,
            36, 36, 36, 36, 36, 36, 51, 51, 51, 51, 51, 51, 66, 66, 26, 74, 26, 74]
        NY = [0, 0, 0, 0, 0, 0, 0, 45, -45, 45, -45,
            22, 44, 52, -22, -44, -52, 22, 44, 52, -22, -44, -52, 22, -22, 22, 22, -22, -22]
        let E = [0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 1, 7, 1, 8, 7, 9, 8, 10,
            2, 11, 11, 12, 12, 13, 2, 14, 14, 15, 15, 16,
            3, 17, 17, 18, 18, 19, 3, 20, 20, 21, 21, 22,
            4, 23, 4, 24, 25, 11, 11, 17, 17, 23, 23, 26, 27, 14, 14, 20, 20, 24, 24, 28]
        NB = []
        for (let i = 0; i < 29; i++) NB.push([])
        for (let j = 0; j < E.length; j += 2) {
            NB[E[j]].push(E[j + 1])
            NB[E[j + 1]].push(E[j])
        }
        navMin = 50
        navMax = 100
        navKp = 0.012
        navKd = 0.15
        navSlow = 35
        navTurnSpeed = 50
        navTurnFast = 200
        navCrossMs = 80
        navApproach = 35
        navBrakeMs = 40
        navStartMs = 400
        navHomeMs = 300
        navBoxStraightMs = 300
        navBoxTurnMs = 350
        navBoxBackMs = 150
        navZoneMs = 250
        navZoneBackMs = 300
        navYellowMs = 250
    }

    // ทิศ: 0 = ไปทางกล่องเหลือง, 1 = ไปทางช่องขาว, 2 = ไปทาง START, 3 = ไปทางช่องดำ
    function dirOf(a: number, b: number): number {
        if (NX[b] > NX[a]) return 0
        if (NY[b] > NY[a]) return 1
        if (NX[b] < NX[a]) return 2
        return 3
    }

    function isBox(n: number): boolean {
        return n == 13 || n == 16 || n == 19 || n == 22 || n == 25 || n == 26 || n == 27 || n == 28
    }

    function isTerminal(n: number): boolean {
        return n == 0 || n == 6 || n == 9 || n == 10 || isBox(n)
    }

    // หาเส้นทางสั้นสุด (ระยะ + ค่าปรับเลี้ยว 15 ซม.) ห้ามกลับตัวกลางทาง ห้ามผ่านจุดปลายทางอื่น
    function navPlan(a: number, b: number): number[] {
        let S = 29 * 4
        let dist: number[] = []
        let prev: number[] = []
        let done: boolean[] = []
        for (let i = 0; i < S; i++) {
            dist.push(1000000)
            prev.push(-1)
            done.push(false)
        }
        let h0 = 0
        if (a != 0) h0 = dirOf(NB[a][0], a)
        dist[a * 4 + h0] = 0
        while (true) {
            let u = -1
            let best = 1000000
            for (let i = 0; i < S; i++) {
                if (!done[i] && dist[i] < best) {
                    best = dist[i]
                    u = i
                }
            }
            if (u < 0) break
            done[u] = true
            let n = Math.idiv(u, 4)
            let h = u % 4
            if (n == b) continue
            if (n != a && isTerminal(n)) continue
            for (let j = 0; j < NB[n].length; j++) {
                let v = NB[n][j]
                let g = dirOf(n, v)
                let rel = (g - h + 4) % 4
                let cost = Math.abs(NX[v] - NX[n]) + Math.abs(NY[v] - NY[n])
                if (rel == 2) {
                    if (n != a) continue
                    cost += 20
                } else if (rel != 0) {
                    cost += 15
                }
                let st = v * 4 + g
                if (dist[u] + cost < dist[st]) {
                    dist[st] = dist[u] + cost
                    prev[st] = u
                }
            }
        }
        let end = -1
        let best2 = 1000000
        for (let h = 0; h < 4; h++) {
            if (dist[b * 4 + h] < best2) {
                best2 = dist[b * 4 + h]
                end = b * 4 + h
            }
        }
        let path: number[] = []
        while (end >= 0) {
            path.push(Math.idiv(end, 4))
            end = prev[end]
        }
        path.reverse()
        return path
    }

    // หยุดทันที: ถอยกลับสั้น ๆ เป็นเบรก แล้วตัดมอเตอร์
    function navStop(speed: number) {
        if (navBrakeMs > 0) {
            drive(-speed, -speed)
            basic.pause(navBrakeMs)
        }
        iBIT.MotorStop()
    }

    // วิ่งตามเส้นผ่านทางแยก n-1 จุดด้วยความเร็วปกติ
    // ช่วงสุดท้ายก่อนทางแยกที่จะหยุด ใช้ความเร็ว Approach แล้วเบรกทันทีที่เซนเซอร์เจอเส้นตัด
    function navFind(n: number) {
        if (n <= 0) return
        let found = 0
        let sp = n == 1 ? navApproach : navMin
        while (true) {
            readAll()
            if (junction(IR_Find.Center)) {
                found += 1
                if (found >= n) {
                    navStop(sp)
                    break
                }
                while (junction(IR_Find.Center)) {
                    drive(sp, sp)
                    basic.pause(1)
                    readAll()
                }
                basic.pause(100)
                readAll()
                if (found >= n - 1) sp = navApproach
            }
            pdFromV(sp, navMax, navKp, navKd)
        }
    }

    function navSlowWalk(ms: number) {
        ForwardTIME(ms, navSlow, navMax, navKp, navKd)
    }

    // เลี้ยวที่ทางแยก: หุ่นอยู่ในท่า "เซนเซอร์เพิ่งเจอเส้นตัด"
    function navTurn(h: number, g: number) {
        let rel = (g - h + 4) % 4
        if (rel == 0) return
        if (rel == 2) {
            // กลับตัว: หมุนรอบล้อทันที (ไม่เดินเข้าทางแยกก่อน) จึงไม่ไปหยุดที่เส้นแยกด้านข้าง
            TurnLINE(IR_Dir.Left, navTurnSpeed, IR_TurnSensor.Center, navTurnFast)
            return
        }
        // เดินหน้านิดเดียวให้แกนล้อถึงทางแยก แล้วเลี้ยว
        if (navCrossMs > 0) {
            drive(navApproach, navApproach)
            basic.pause(navCrossMs)
            navStop(navApproach)
        }
        TurnLINE(rel == 1 ? IR_Dir.Left : IR_Dir.Right, navTurnSpeed, IR_TurnSensor.Center, navTurnFast)
    }

    /**
     * วิ่งจากจุดเริ่มต้นไปยังเป้าหมายเอง (สนาม PT KidsBiT ภารกิจส่งด่วน)
     * หาเส้นทางสั้นสุด นับทางแยก และเลี้ยวให้อัตโนมัติ
     * จุดเริ่มต้นต้องตรงกับที่หุ่นอยู่จริง: START = วางในกรอบหันเข้าสนาม,
     * จุดอื่น = หุ่นหยุดหันหน้าเข้าหาจุดนั้น (ท่าเมื่อจบบล็อกนี้ครั้งก่อน)
     * บล็อกนี้ไม่คีบ/ไม่วาง ให้ใส่ Pick Up / Put Down ต่อเอง
     */
    //% blockId=ir_navgo block="Go from %from|to %to"
    //% from.defl=NavPlace.START to.defl=NavPlace.B1
    //% inlineInputMode=inline
    //% group="Navigation" weight=100
    export function navGo(from: NavPlace, to: NavPlace): void {
        initState()
        initNav()
        let a = from as number
        let b = to as number
        if (a == b) return
        let path = navPlan(a, b)
        if (path.length < 2) return
        let k = path.length - 1
        let h = dirOf(path[0], path[1])
        let count = 0
        let i = 1
        if (a == 0) {
            // ออกจากกรอบ START
            navSlowWalk(navStartMs)
        } else {
            // ถอยออกจากจุดเดิมจนเซนเซอร์อยู่บนทางแยกแรก แล้วเลี้ยว/กลับตัวที่นั่น
            if (a == 6) {
                BackwardTIME(navYellowMs, navSlow)
            } else {
                BackwardTIME(isBox(a) ? navBoxBackMs : navZoneBackMs, navSlow)
                BackwardLINE(IR_Find.Center, navSlow, navKp)
            }
            h = dirOf(path[1], path[0])
            if (k >= 2) {
                let g = dirOf(path[1], path[2])
                navTurn(h, g)
                h = g
                i = 2
            }
        }
        for (; i < k; i++) {
            count += 1
            let g = dirOf(path[i], path[i + 1])
            if (g != h) {
                navFind(count)
                count = 0
                navTurn(h, g)
                h = g
            }
        }
        // ถึงเป้าหมาย
        if (b == 6) {
            navFind(count)
            drive(navSlow, navSlow)
            basic.pause(navYellowMs)
            stopWithBrake(navSlow, navSlow)
        } else if (isBox(b)) {
            if (count > 0) {
                navFind(count)
                navSlowWalk(navBoxStraightMs)
            } else {
                navSlowWalk(navBoxTurnMs)
            }
        } else {
            // ช่องขาว/ดำ หรือ START: นับขอบกรอบเป็นเส้นตัดอีก 1 เส้น แล้วเดินเข้าไป
            navFind(count + 1)
            navSlowWalk(b == 0 ? navHomeMs : navZoneMs)
        }
    }

    /**
     * ความเร็วที่ใช้กับบล็อก Go from ... to ...
     */
    //% blockId=ir_navspeed block="Set Navigation Min Speed %min|Max Speed %max|KP %kp|KD %kd|Slow %slow"
    //% min.defl=50 max.defl=100 kp.defl=0.012 kd.defl=0.15 slow.defl=35
    //% inlineInputMode=inline
    //% group="Navigation" weight=90
    export function setNavSpeed(min: number, max: number, kp: number, kd: number, slow: number): void {
        initState()
        initNav()
        navMin = min
        navMax = max
        navKp = kp
        navKd = kd
        navSlow = slow
    }

    /**
     * การเลี้ยว: ความเร็วเลี้ยว, Fast Time และเวลาเดินต่อหลังเจอเส้นตัดก่อนเลี้ยว (ms)
     */
    //% blockId=ir_navturn block="Set Navigation Turn Speed %speed|Fast Time %fast|Cross %cross ms"
    //% speed.defl=50 fast.defl=200 cross.defl=80
    //% inlineInputMode=inline
    //% group="Navigation" weight=85 advanced=true
    export function setNavTurn(speed: number, fast: number, cross: number): void {
        initState()
        initNav()
        navTurnSpeed = speed
        navTurnFast = fast
        navCrossMs = cross
    }

    /**
     * การหยุดที่ทางแยก: Approach = ความเร็วช่วงสุดท้ายก่อนถึงทางแยกที่จะหยุด (ช้า = หยุดตรงกว่า)
     * Brake = เวลาเบรก (ms) มาก = หยุดเร็วขึ้น ถ้ามากเกินหุ่นจะถอยกลับ
     */
    //% blockId=ir_navstop block="Set Navigation Junction Approach %approach|Brake %brake ms"
    //% approach.defl=35 brake.defl=40
    //% inlineInputMode=inline
    //% group="Navigation" weight=88
    export function setNavStop(approach: number, brake: number): void {
        initState()
        initNav()
        navApproach = approach
        navBrakeMs = brake
    }

    /**
     * เวลา (ms) ช่วงจุดวางกล่อง
     * Straight = เข้าหากล่องหลังวิ่งผ่านเส้นตัดตรง ๆ (B1 B2 B7 B8)
     * After Turn = เข้าหากล่องหลังเลี้ยวเข้าแถว (B3-B6)
     * Back = ถอยออกจากกล่องก่อนเริ่มหาเส้นตัด
     */
    //% blockId=ir_navbox block="Set Navigation Box Straight %straight|After Turn %turn|Back %back"
    //% straight.defl=300 turn.defl=350 back.defl=150
    //% inlineInputMode=inline
    //% group="Navigation" weight=80 advanced=true
    export function setNavBox(straight: number, turn: number, back: number): void {
        initState()
        initNav()
        navBoxStraightMs = straight
        navBoxTurnMs = turn
        navBoxBackMs = back
    }

    /**
     * เวลา (ms): เข้าไปในช่องสี, ถอยออกจากช่องสี, ออกจาก START, เข้า START,
     * วิ่งตรงจากขีดขวางทางแยก 5 ไปถึงกล่องเหลือง (ช่วงนี้ไม่มีเส้น)
     */
    //% blockId=ir_navzone block="Set Navigation Zone In %zin|Zone Back %zback|Start Out %sout|Home In %home|Yellow %yellow"
    //% zin.defl=250 zback.defl=300 sout.defl=400 home.defl=300 yellow.defl=250
    //% inlineInputMode=inline
    //% group="Navigation" weight=75 advanced=true
    export function setNavZone(zin: number, zback: number, sout: number, home: number, yellow: number): void {
        initState()
        initNav()
        navZoneMs = zin
        navZoneBackMs = zback
        navStartMs = sout
        navHomeMs = home
        navYellowMs = yellow
    }
}
