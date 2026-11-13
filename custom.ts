/**
 * หุ่นเดินตามเส้นแบบ PID สำหรับ iBIT V2A (เซนเซอร์ 2–8 ตัว)
 * ต้องเพิ่ม Extension iBIT: https://github.com/inexglobal/pxt-iBit2 ในโปรเจกต์ก่อน
 *
 * การต่อเซนเซอร์: เรียงจากซ้ายไปขวา (มองจากด้านบน หันหน้าไปทางเดียวกับหุ่น)
 * เซนเซอร์ 1 = ตัวซ้ายสุด ต่อที่ช่อง ADC แรกที่ตั้งค่าไว้ แล้วไล่ไปทางขวา
 * มอเตอร์: M1 = ล้อซ้าย, M2 = ล้อขวา (ถ้าต่อกลับ ใช้บล็อก "สลับมอเตอร์ซ้าย-ขวา")
 */

enum LBDirection {
    //% block="ซ้าย"
    Left,
    //% block="ขวา"
    Right
}

//% weight=90 color=#2E7D32 icon="" block="หุ่นเดินตามเส้น"
//% groups='["1. ตั้งค่า", "2. สอนหุ่น", "3. วิ่ง", "4. ข้อมูลเซนเซอร์"]'
namespace lineBot {
    const ADC_CH: ibitReadADC[] = [
        ibitReadADC.ADC0, ibitReadADC.ADC1, ibitReadADC.ADC2, ibitReadADC.ADC3,
        ibitReadADC.ADC4, ibitReadADC.ADC5, ibitReadADC.ADC6, ibitReadADC.ADC7
    ]

    let count = 5
    let firstCh = 0
    let reversed = false
    let swapMotors = false
    let baseSpeed = 40
    let kp = 0.6
    let ki = 0
    let kd = 4

    // ค่าพื้นสนามและค่าเส้น ของเซนเซอร์แต่ละตัว (ได้จากการสอนหุ่น)
    let floorVal: number[] = [0, 0, 0, 0, 0, 0, 0, 0]
    let lineVal: number[] = [4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095]
    let floorDone = false
    let lineDone = false

    let lastError = 0
    let integral = 0
    let lastSide = 0

    // ---------- ฟังก์ชันภายใน (ไม่แสดงเป็นบล็อก) ----------

    function clamp(v: number, lo: number, hi: number): number {
        if (v < lo) return lo
        if (v > hi) return hi
        return v
    }

    function readRaw(i: number): number {
        let idx = reversed ? (count - 1 - i) : i
        return iBIT.ReadADC(ADC_CH[firstCh + idx])
    }

    function norm(i: number): number {
        let f = floorVal[i]
        let l = lineVal[i]
        if (l == f) return 0
        return clamp((readRaw(i) - f) * 100 / (l - f), 0, 100)
    }

    function setOne(ch: ibitMotorCH, v: number) {
        v = Math.round(clamp(v, -100, 100))
        if (v >= 0) iBIT.setMotor(ch, ibitMotor.Forward, v)
        else iBIT.setMotor(ch, ibitMotor.Backward, -v)
    }

    function drive(left: number, right: number) {
        if (swapMotors) {
            setOne(ibitMotorCH.M1, right)
            setOne(ibitMotorCH.M2, left)
        } else {
            setOne(ibitMotorCH.M1, left)
            setOne(ibitMotorCH.M2, right)
        }
    }

    function centerSees(): boolean {
        if (count % 2 == 1) return norm((count - 1) / 2) > 50
        return norm(count / 2 - 1) > 50 || norm(count / 2) > 50
    }

    function followStep() {
        let e = linePosition()
        integral = clamp(integral + e, -2000, 2000)
        let d = e - lastError
        lastError = e
        let corr = kp * e + ki * integral + kd * d
        drive(baseSpeed + corr, baseSpeed - corr)
    }

    function resetPID() {
        lastError = 0
        integral = 0
    }

    // ---------- 1. ตั้งค่า ----------

    /**
     * ตั้งค่าหุ่น: ใช้เซนเซอร์กี่ตัว และเซนเซอร์ตัวซ้ายสุดต่อที่ช่อง ADC ใด วางไว้ใน on start
     * @param n จำนวนเซนเซอร์ 2–8 ตัว
     * @param first ช่อง ADC ของเซนเซอร์ตัวซ้ายสุด
     */
    //% blockId=lineBot_setup block="ตั้งค่าหุ่น ใช้เซนเซอร์ $n ตัว เริ่มที่ช่อง ADC $first"
    //% n.min=2 n.max=8 n.defl=5 first.min=0 first.max=6 first.defl=0
    //% inlineInputMode=inline
    //% group="1. ตั้งค่า" weight=100
    export function setup(n: number, first: number): void {
        count = Math.round(clamp(n, 2, 8))
        firstCh = Math.round(clamp(first, 0, 8 - count))
        iBIT.setADC_Address(adcAddress.iBIT_V2)
        floorDone = false
        lineDone = false
        resetPID()
        lastSide = 0
    }

    /**
     * ความเร็วตอนเดินตามเส้น (0–100)
     */
    //% blockId=lineBot_speed block="ตั้งความเร็ว $speed"
    //% speed.min=0 speed.max=100 speed.defl=40
    //% group="1. ตั้งค่า" weight=95
    export function setSpeed(speed: number): void {
        baseSpeed = clamp(speed, 0, 100)
    }

    /**
     * KP = แรงหักพวงมาลัย, KI = แก้การเบี้ยวสะสม, KD = เบรกกันส่าย
     */
    //% blockId=lineBot_pid block="ตั้งค่า PID KP $p KI $i KD $d"
    //% p.defl=0.6 i.defl=0 d.defl=4
    //% inlineInputMode=inline
    //% group="1. ตั้งค่า" weight=90
    export function setPID(p: number, i: number, d: number): void {
        kp = p
        ki = i
        kd = d
        resetPID()
    }

    /**
     * ใช้เมื่อหุ่นเลี้ยวหนีเส้น เพราะต่อมอเตอร์ซ้าย-ขวาสลับกัน
     */
    //% blockId=lineBot_swap block="สลับมอเตอร์ซ้าย-ขวา $on"
    //% on.shadow="toggleOnOff" on.defl=true
    //% group="1. ตั้งค่า" weight=80 advanced=true
    export function setSwapMotors(on: boolean): void {
        swapMotors = on
    }

    /**
     * ใช้เมื่อต่อเซนเซอร์เรียงจากขวาไปซ้าย วางก่อนสอนหุ่น
     */
    //% blockId=lineBot_reverse block="กลับลำดับเซนเซอร์ $on"
    //% on.shadow="toggleOnOff" on.defl=true
    //% group="1. ตั้งค่า" weight=75 advanced=true
    export function setReversed(on: boolean): void {
        reversed = on
    }

    // ---------- 2. สอนหุ่น ----------

    /**
     * วางหุ่นคร่อมเส้นให้เส้นอยู่กลางตัว แล้วสั่ง หุ่นจะหมุนซ้าย-ขวาเพื่อจำค่าพื้นและเส้นเอง
     * จอแสดงเครื่องหมายถูก = สอนสำเร็จ, หน้าเศร้า = ค่าดำ-ขาวต่างกันน้อย ให้ปรับความสูงเซนเซอร์
     */
    //% blockId=lineBot_autocal block="สอนหุ่นอัตโนมัติ (หมุนซ้าย-ขวา)"
    //% group="2. สอนหุ่น" weight=100
    export function autoCalibrate(): void {
        let mn: number[] = []
        let mx: number[] = []
        for (let i = 0; i < count; i++) {
            mn.push(4095)
            mx.push(0)
        }
        basic.showIcon(IconNames.SmallDiamond)
        let plan = [1, -1, -1, 1]
        for (let step = 0; step < plan.length; step++) {
            drive(30 * plan[step], -30 * plan[step])
            let start = control.millis()
            while (control.millis() - start < 400) {
                for (let i = 0; i < count; i++) {
                    let r = readRaw(i)
                    if (r < mn[i]) mn[i] = r
                    if (r > mx[i]) mx[i] = r
                }
            }
        }
        iBIT.MotorStop()
        basic.pause(200)
        // หุ่นกลับมาอยู่กลางเส้น: ดูเซนเซอร์ตัวกลางว่าเส้นให้ค่าสูงหรือต่ำ
        let c = Math.floor((count - 1) / 2)
        let cr = readRaw(c)
        let lineIsHigh = (cr - mn[c]) > (mx[c] - cr)
        let ok = true
        for (let i = 0; i < count; i++) {
            if (lineIsHigh) {
                floorVal[i] = mn[i]
                lineVal[i] = mx[i]
            } else {
                floorVal[i] = mx[i]
                lineVal[i] = mn[i]
            }
            if (mx[i] - mn[i] < 200) ok = false
        }
        floorDone = true
        lineDone = true
        resetPID()
        if (ok) basic.showIcon(IconNames.Yes)
        else basic.showIcon(IconNames.Sad)
    }

    /**
     * วางเซนเซอร์ทุกตัวบนพื้นสนาม แล้วสั่งบันทึก
     */
    //% blockId=lineBot_floor block="บันทึกค่าพื้นสนาม"
    //% group="2. สอนหุ่น" weight=90
    export function recordFloor(): void {
        for (let i = 0; i < count; i++) {
            let sum = 0
            for (let k = 0; k < 10; k++) {
                sum += readRaw(i)
                basic.pause(5)
            }
            floorVal[i] = sum / 10
        }
        floorDone = true
        basic.showIcon(IconNames.Square)
    }

    /**
     * วางเซนเซอร์ทุกตัวบนเส้น แล้วสั่งบันทึก
     */
    //% blockId=lineBot_line block="บันทึกค่าเส้น"
    //% group="2. สอนหุ่น" weight=85
    export function recordLine(): void {
        for (let i = 0; i < count; i++) {
            let sum = 0
            for (let k = 0; k < 10; k++) {
                sum += readRaw(i)
                basic.pause(5)
            }
            lineVal[i] = sum / 10
        }
        lineDone = true
        basic.showIcon(IconNames.SmallSquare)
    }

    /**
     * true เมื่อบันทึกค่าพื้นและค่าเส้นครบแล้ว
     */
    //% blockId=lineBot_ready block="สอนหุ่นเสร็จแล้ว"
    //% group="2. สอนหุ่น" weight=80
    export function isCalibrated(): boolean {
        return floorDone && lineDone
    }

    // ---------- 3. วิ่ง ----------

    /**
     * เดินตามเส้น 1 จังหวะ วางไว้ใน forever หรือใน while
     */
    //% blockId=lineBot_follow block="เดินตามเส้น"
    //% group="3. วิ่ง" weight=100
    export function follow(): void {
        followStep()
    }

    /**
     * เดินตามเส้นไปจนถึงทางแยกถัดไปแล้วหยุด (ถ้าเริ่มบนทางแยก จะข้ามทางแยกนั้นก่อน)
     */
    //% blockId=lineBot_junction block="เดินตามเส้นไปจนถึงทางแยกถัดไป"
    //% group="3. วิ่ง" weight=95
    export function followToJunction(): void {
        resetPID()
        while (isJunction()) {
            drive(baseSpeed, baseSpeed)
            basic.pause(1)
        }
        while (!isJunction()) {
            followStep()
            basic.pause(1)
        }
        iBIT.MotorStop()
    }

    /**
     * เดินตามเส้นไปจนถึงทางแยกที่ n แล้วหยุด
     */
    //% blockId=lineBot_junctions block="เดินตามเส้นผ่านทางแยก $n ครั้ง แล้วหยุด"
    //% n.min=1 n.defl=2
    //% group="3. วิ่ง" weight=93
    export function followJunctions(n: number): void {
        for (let k = 0; k < n; k++) {
            followToJunction()
        }
    }

    /**
     * เดินตามเส้นตามเวลาที่กำหนด แล้วหยุด
     */
    //% blockId=lineBot_time block="เดินตามเส้นนาน $ms มิลลิวินาที"
    //% ms.shadow=timePicker ms.defl=1000
    //% group="3. วิ่ง" weight=90
    export function followFor(ms: number): void {
        resetPID()
        let start = control.millis()
        while (control.millis() - start < ms) {
            followStep()
            basic.pause(1)
        }
        iBIT.MotorStop()
    }

    /**
     * หมุนตัวจนเซนเซอร์ตรงกลางเจอเส้นใหม่ แล้วหยุด
     */
    //% blockId=lineBot_turn block="เลี้ยว $dir จนเจอเส้น ความเร็ว $speed"
    //% speed.min=0 speed.max=100 speed.defl=40
    //% inlineInputMode=inline
    //% group="3. วิ่ง" weight=85
    export function turnToLine(dir: LBDirection, speed: number): void {
        let s = dir == LBDirection.Left ? -speed : speed
        drive(s, -s)
        basic.pause(250)
        let start = control.millis()
        while (!centerSees() && control.millis() - start < 3000) {
            basic.pause(1)
        }
        iBIT.MotorStop()
        resetPID()
        lastSide = 0
    }

    /**
     * เดินหน้าตรงโดยไม่ดูเส้น (ใช้ข้ามทางแยก หรือให้ล้อไปถึงเส้นก่อนเลี้ยว)
     */
    //% blockId=lineBot_forward block="เดินหน้าตรง ความเร็ว $speed นาน $ms มิลลิวินาที"
    //% speed.min=0 speed.max=100 speed.defl=40 ms.shadow=timePicker ms.defl=200
    //% inlineInputMode=inline
    //% group="3. วิ่ง" weight=80
    export function forward(speed: number, ms: number): void {
        drive(speed, speed)
        basic.pause(ms)
        iBIT.MotorStop()
    }

    //% blockId=lineBot_stop block="หยุด"
    //% group="3. วิ่ง" weight=75
    export function stop(): void {
        iBIT.MotorStop()
    }

    // ---------- 4. ข้อมูลเซนเซอร์ ----------

    /**
     * ตำแหน่งเส้น -100 (ซ้ายสุด) ถึง 100 (ขวาสุด), 0 = กลางพอดี
     */
    //% blockId=lineBot_pos block="ตำแหน่งเส้น"
    //% group="4. ข้อมูลเซนเซอร์" weight=100
    export function linePosition(): number {
        let sum = 0
        let wsum = 0
        let maxV = 0
        for (let i = 0; i < count; i++) {
            let v = norm(i)
            let w = -100 + 200 * i / (count - 1)
            sum += v
            wsum += v * w
            if (v > maxV) maxV = v
        }
        if (maxV < 30) {
            // เซนเซอร์ 2 ตัวแบบคร่อมเส้น: ไม่มีตัวไหนเห็นเส้น = อยู่กลางเส้นพอดี
            if (count == 2) return 0
            // 3 ตัวขึ้นไป: หลุดเส้น ให้หมุนกลับไปทางที่เห็นเส้นครั้งสุดท้าย
            return lastSide * 100
        }
        let pos = wsum / sum
        if (pos > 10) lastSide = 1
        else if (pos < -10) lastSide = -1
        return pos
    }

    /**
     * ค่าเซนเซอร์ 0 (พื้น) ถึง 100 (เส้น)
     * @param n เซนเซอร์ตัวที่ นับจากซ้าย เริ่มที่ 1
     */
    //% blockId=lineBot_value block="ค่าเซนเซอร์ตัวที่ $n"
    //% n.min=1 n.max=8 n.defl=1
    //% group="4. ข้อมูลเซนเซอร์" weight=95
    export function sensorValue(n: number): number {
        let i = Math.round(clamp(n, 1, count)) - 1
        return Math.round(norm(i))
    }

    /**
     * @param n เซนเซอร์ตัวที่ นับจากซ้าย เริ่มที่ 1
     */
    //% blockId=lineBot_sees block="เซนเซอร์ตัวที่ $n เห็นเส้น"
    //% n.min=1 n.max=8 n.defl=1
    //% group="4. ข้อมูลเซนเซอร์" weight=90
    export function seesLine(n: number): boolean {
        return sensorValue(n) > 50
    }

    //% blockId=lineBot_count block="จำนวนเซนเซอร์ที่เห็นเส้น"
    //% group="4. ข้อมูลเซนเซอร์" weight=85
    export function lineCount(): number {
        let c = 0
        for (let i = 0; i < count; i++) {
            if (norm(i) > 50) c++
        }
        return c
    }

    /**
     * เจอทางแยก = เซนเซอร์เกือบทุกตัวเห็นเส้นพร้อมกัน
     */
    //% blockId=lineBot_isjunction block="เจอทางแยก"
    //% group="4. ข้อมูลเซนเซอร์" weight=80
    export function isJunction(): boolean {
        let need = count <= 3 ? count : Math.ceil(count * 3 / 4)
        return lineCount() >= need
    }

    //% blockId=lineBot_lost block="หลุดเส้น"
    //% group="4. ข้อมูลเซนเซอร์" weight=75
    export function isLost(): boolean {
        return lineCount() == 0
    }

    /**
     * ส่งค่าดิบของเซนเซอร์ทุกตัวไปดูที่ Show data Device
     */
    //% blockId=lineBot_serial block="ส่งค่าเซนเซอร์ไปดูบนคอมพิวเตอร์"
    //% group="4. ข้อมูลเซนเซอร์" weight=70 advanced=true
    export function sendToSerial(): void {
        for (let i = 0; i < count; i++) {
            serial.writeValue("s" + (i + 1), readRaw(i))
        }
    }
}
