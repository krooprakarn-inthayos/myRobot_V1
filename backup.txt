// ตัวอย่างโปรแกรมแบบ PTKidsBiT สำหรับหุ่น iBIT V2A (ใช้คู่กับ custom.ts)
// ภารกิจตัวอย่าง: ออกจาก START -> แยกที่ 1 เลี้ยวซ้าย -> แยกถัดไป หยิบกระป๋อง
//                 -> กลับตัว -> กลับมาแยกที่ 1 เลี้ยวขวา -> เข้า START วางกระป๋อง
// ค่าเวลา ความเร็ว KP KD เป็นค่าตั้งต้น ต้องทดสอบและปรับตามหุ่นและสนามจริง

// ---- ค่าที่ใช้ซ้ำ ปรับที่นี่ที่เดียว ----
let MIN = 50
let MAX = 100
let KP = 0.012
let KD = 0.15

// ---- เริ่มต้น: ตั้งค่าตัวคีบ แล้ว Calibrate (A บนเส้น, A บนพื้น) ----
iBITRobot.setGripper(45, 110, 45, 0)
iBITRobot.armUp()
iBITRobot.gripOpen()
iBITRobot.SensorCalibrate()

// ---- กด B เริ่มภารกิจ ----
input.onButtonPressed(Button.B, function () {
    // 1. ออกจาก START (ใช้ Time ก่อน ไม่ให้นับขอบกล่องเป็นแยก)
    iBITRobot.ForwardTIME(500, MIN, MAX, KP, KD)
    // 2. ไปแยกที่ 1 แล้วเดินต่อให้ล้อถึงเส้นตัด
    iBITRobot.ForwardLINECount(IR_Find.Center, 1, MIN, MAX, KP, KD)
    iBITRobot.ForwardTIME(100, MIN, MAX, KP, KD)
    basic.pause(100)
    // 3. เลี้ยวซ้าย หยุดเมื่อเซนเซอร์กลางเจอเส้น
    iBITRobot.TurnLINE(IR_Dir.Left, 50, IR_TurnSensor.Center, 200)
    // 4. ไปแยกถัดไป แล้วหยิบ
    iBITRobot.ForwardTIME(200, MIN, MAX, KP, KD)
    iBITRobot.ForwardLINECount(IR_Find.Center, 1, MIN, MAX, KP, KD)
    iBITRobot.pickUp()
    // 5. กลับตัวบนจุดตัด: เลี้ยวขวา 2 ครั้ง (ครั้งแรกจะหยุดที่เส้นตั้งฉาก)
    iBITRobot.TurnLINE(IR_Dir.Right, 50, IR_TurnSensor.Center, 200)
    iBITRobot.TurnLINE(IR_Dir.Right, 50, IR_TurnSensor.Center, 200)
    // 6. กลับมาแยกที่ 1 แล้วเลี้ยวขวาลงไปทาง START
    iBITRobot.ForwardTIME(200, MIN, MAX, KP, KD)
    iBITRobot.ForwardLINECount(IR_Find.Center, 1, MIN, MAX, KP, KD)
    iBITRobot.ForwardTIME(100, MIN, MAX, KP, KD)
    basic.pause(100)
    iBITRobot.TurnLINE(IR_Dir.Right, 50, IR_TurnSensor.Center, 200)
    // 7. เข้า START แล้ววางกระป๋อง
    iBITRobot.ForwardTIME(200, MIN, MAX, KP, KD)
    iBITRobot.ForwardLINECount(IR_Find.Center, 1, MIN, MAX, KP, KD)
    iBITRobot.ForwardTIME(150, MIN, MAX, KP, KD)
    iBITRobot.putDown()
    basic.showIcon(IconNames.Happy)
})

// ---- กด A+B: วิ่งตามเส้นต่อเนื่อง (ใช้จูน KP KD) กดอีกครั้งเพื่อหยุด ----
let freeRun = false
input.onButtonPressed(Button.AB, function () {
    freeRun = !(freeRun)
    if (!(freeRun)) {
        iBITRobot.motorStop()
    }
})
basic.forever(function () {
    if (freeRun) {
        iBITRobot.Follower(80, 100, KP, KD)
    }
})
