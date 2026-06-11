# Project Context — FRA263/264 Robot Arm Controller
## สำหรับผู้อ่านและ Claude AI รุ่นต่อไป

---

## 1. ภาพรวมโปรเจค

ระบบควบคุมแขนกล 1 แกน (1-DOF Rotary) บน **STM32 NUCLEO-G474RE** ทำงานร่วมกับ PC ผ่าน **Base System** (Python WebSocket + Modbus RTU) และ **CAN Bus** (Gripper Node)

### Hardware
| ชิ้นส่วน | รายละเอียด |
|---|---|
| MCU | STM32 NUCLEO-G474RE (170 MHz, Cortex-M4) |
| Motor | Brush DC 24V |
| Motor Driver | Cyton MD20A (DIR + PWM) |
| Encoder | Quadrature, 8192 counts/rev (X4 mode) via TIM2 |
| Gripper Node | CAN Bus node (relay-based, Node ID 0x10) |
| Proximity Sensor | NPN on GPIOB PIN_0 (ใช้หา home position) |

### Software Stack
```
Base System (PC) ←→ Modbus RTU (19200, 8E1, RS485) ←→ STM32 (Slave addr 21)
                                                             ↕
                                                       CAN Bus 500kbps
                                                             ↕
                                                    Gripper Node (0x10)
```

---

## 2. โครงสร้างไฟล์ที่สำคัญ

```
Core/
├── Inc/
│   ├── CANBUS.h        ← CAN Bus driver (ใหม่ - สร้างในโปรเจคนี้)
│   ├── Gripper.h       ← Gripper control via GPIO relay
│   ├── kalman.h        ← 4-state Kalman filter
│   ├── Modbus.h        ← Modbus RTU register map + API
│   ├── Mode.h          ← Robot mode state machine
│   ├── robot_arm.h     ← Cascade PID + S-Curve + Kalman
│   ├── scurve_trajectory.h
│   └── proximity.h
└── Src/
    ├── CANBUS.c        ← CAN Bus driver (ใหม่)
    ├── Gripper.c       ← GPIO relay control
    ├── kalman.c        ← Kalman filter implementation
    ├── main.c          ← Entry point + main loop
    ├── Modbus.c        ← Modbus RTU slave
    ├── Mode.c          ← Mode state machine (Jog/Auto/Home/Test)
    ├── robot_arm.c     ← Motor control core
    └── scurve_trajectory.c
```

---

## 3. Modbus Register Map (ใช้กับ Base System)

| Address (hex) | Name | Description |
|---|---|---|
| 0x00 | REG_HEARTBEAT | PC ส่ง 18537, STM32 ตอบ 22881 |
| 0x01 | REG_MODE | Mode bits: HOME=1, JOG=2, AUTO=4, SET_HOME=8, TEST=16 |
| 0x02 | REG_GRIPPER_MANUAL | UP=0, DOWN=1, OPEN=2, CLOSE=4 |
| 0x03 | REG_GRIPPER_SEQ | Pick=1, Place=2 |
| 0x04 | REG_GRIPPER_AUTO_EN | Enable gripper in auto mode |
| 0x05 | REG_JOG | Jog step (signed degrees) |
| 0x06 | REG_TEST_TYPE | 0=Precision, 1=Performance |
| 0x07 | REG_PERF_VEL | Performance test velocity |
| 0x08 | REG_PERF_ACC | Performance test acceleration |
| 0x09 | REG_PREC_INIT | Precision test start position |
| 0x10 | REG_PREC_FINAL | Precision test end position |
| 0x11 | REG_PREC_REPEAT | Precision test repeat count |
| 0x12–0x21 | REG_PICKPLACE_START | Pick/Place sequence slots |
| 0x22 | REG_PICKPLACE_COUNT | จำนวน pick/place pairs |
| 0x23 | REG_P2P_UNIT | 0=degree, 1=index |
| 0x24 | REG_P2P_TARGET | P2P target value (ใช้เป็น Set Home target) |
| 0x25 | REG_SOFT_STOP | 1=stop |
| 0x26 | REG_SENSORS | Gripper sensors bitmask |
| 0x27 | REG_TASK | Task status |
| 0x28 | REG_POSITION | pos_deg × 10 |
| 0x29 | REG_VELOCITY | vel_deg_s × 10 |
| 0x30 | REG_ACCELERATION | accel × 10 |
| 0x31 | REG_EMERGENCY | 1=emergency stop |

---

## 4. Control Architecture (robot_arm.c)

```
S-Curve Trajectory (g_arm.traj)
        │ ref_pos_deg, ref_vel_deg_s
        ▼
Outer PID @ 100 Hz (pos_error → vel_cmd)
   Zone switching: normal zone / fine zone (≤5 deg)
        │ vel_cmd_hold
        ▼
Inner PID @ 1 kHz (vel_error → pwm_pid)
   + Velocity Feedforward
        │ pwm_cmd
        ▼
Motor (TIM1 PWM + DIR GPIO)
        │ encoder pulses
        ▼
Kalman Filter 4-state [θ, ω, i, τ_disturbance]
        │ pos_deg, vel_deg_s (filtered)
        └── feedback → Outer PID
```

### PID Gains (robot_arm.h)
| Parameter | Normal Zone | Fine Zone (≤5°) |
|---|---|---|
| Pos Kp | 1.0 | 1.0 |
| Pos Ki | 0.0001 | 0.0 |
| Vel Kp | 2.0 | 1.2 |
| Vel Ki | 0.0 | 0.0 |
| Deadband | 45 PWM counts | - |
| Done tolerance | 0.5° / 1.0 deg/s | - |
| Settle timeout | 800 ms | - |
| Approach dir | +1.0 (CCW) | - |
| Backlash comp | 6° overshoot | - |

### S-Curve Parameters (scurve_trajectory.h)
- V_max = 350 deg/s
- A_max = 3500 deg/s²
- J_max = 8000 deg/s³

---

## 5. Mode State Machine (Mode.c)

### Modes
- **ROBOT_IDLE**: รอคำสั่ง
- **ROBOT_JOG**: หมุนทีละ step จาก REG_JOG
- **ROBOT_AUTO**: Pick & Place sequence อัตโนมัติ
- **ROBOT_GOHOME**: หมุนกลับไป home_offset_deg
- **ROBOT_SET_HOME**: บันทึก REG_P2P_TARGET เป็น home position
- **ROBOT_TEST**: Precision หรือ Performance test

### Critical: Jog Latch Mechanism
**ปัญหาเดิม:** reg[5] (REG_JOG) ถูก clear ก่อนที่ Mode_Jog จะอ่านค่าได้

**วิธีแก้:** ใน main.c หลัง Modbus_Protocal_Worker() ทำ latch ทันที:
```c
dbg_reg5_snap = reg[REG_JOG].U16;
if (dbg_reg5_snap != 0) {
    modbus_jog_latch = (int16_t)dbg_reg5_snap;
    reg[REG_JOG].U16 = 0;  // clear ที่นี่
}
```
`modbus_jog_latch` ถูก define ใน Modbus.c และ Mode_Jog อ่านจาก latch แทน register โดยตรง

---

## 6. Modbus (Modbus.c) — Critical Fix

**ปัญหาเดิม:** `HAL_UARTEx_ReceiveToIdle_DMA` คืน HAL_BUSY เงียบๆ ทำให้รับได้แค่ frame แรก

**วิธีแก้:** เพิ่ม `HAL_UART_AbortReceive()` ก่อน restart DMA ในทุก iteration ของ state machine

---

## 7. Gripper (Gripper.c)

### GPIO Mapping
| Relay | Pin | Function |
|---|---|---|
| UP | PB2 | Gripper Up Solenoid |
| DOWN | PB1 | Gripper Down Solenoid |
| CLOSE | PB15 | Gripper Close Solenoid |
| OPEN | PB14 | Gripper Open Solenoid |

### HandleManual bit map (ตาม README)
- 0x0000 = UP
- 0x0001 = DOWN
- 0x0002 = OPEN
- 0x0004 = CLOSE

### HandleSequence bit map
- 0x0001 = Pick (DoPick)
- 0x0002 = Place (DoPlace)

**หมายเหตุ:** UP (0x0000) จะไม่ผ่าน `if (reg != 0)` ปกติ — แก้ด้วย edge-detection ใน main.c

---

## 8. CAN Bus (CANBUS.c / CANBUS.h) — ใหม่

### Protocol Spec v1.0.1
- CAN 2.0A, 11-bit ID, 500 kbps
- STM32 เป็น **Master**, Gripper Node (0x10) เป็น Slave
- FDCAN1, Prescaler=34, NomTimeSeg1=7, NomTimeSeg2=2

### CAN ID Structure: [Func(3bit)][NodeID(8bit)]
| CAN ID | Direction | Purpose |
|---|---|---|
| 0x010 | Node→Master | EMCY (emergency) |
| 0x110 | Node→Master | Real-time opto data |
| 0x210 | Master→Node | Command request (relay write/opto read) |
| 0x310 | Node→Master | Command response |
| 0x410 | Master→Node | Config request |
| 0x510 | Node→Master | Config response |
| 0x600 | Master→All | Master heartbeat (ส่งทุก 500ms) |
| 0x710 | Node→Master | Node heartbeat |

### Relay Bank 0 Mapping (บน Node)
| Relay | Function |
|---|---|
| Relay 0 | Gripper Up Solenoid |
| Relay 1 | Gripper Down Solenoid |
| Relay 2 | Gripper Close Solenoid |
| Relay 3 | Gripper Open Solenoid |

### API
```c
CANBUS_Init(&hfdcan1);         // เรียกใน main() USER CODE BEGIN 2
CANBUS_Process();               // เรียกทุกรอบ while(1)
CANBUS_Gripper_Up/Down/Open/Close/Off();
CANBUS_Gripper_DoPick();        // Down→Close→Up (blocking)
CANBUS_Gripper_DoPlace();       // Down→Open→Up (blocking)

// ใน stm32g4xx_it.c:
void HAL_FDCAN_RxFifo0Callback(...) { CANBUS_RxCallback(...); }
```

### Status Variables (ดูใน Live Expression)
- `can_node_state`: 0x05 = Operational, 0xFF = Failsafe
- `can_relay_state`: relay state ปัจจุบัน
- `can_opto_state`: opto input state
- `can_last_node_tick`: HAL_GetTick() ล่าสุดที่รับ heartbeat

### การตั้งค่า (CubeMX)
ต้อง enable FDCAN1 ใน .ioc ก่อน:
- Prescaler = 34
- Nom Time Seg 1 = 7
- Nom Time Seg 2 = 2

---

## 9. Proximity Sensor Homing

**Logic:**
1. Boot → `Proximity_Read_Sensor()` → หมุน 360° ด้วย `RobotArm_Move`
2. ใน main.c while loop: ถ้า `proximity == 1` ขณะหมุน → `RobotArm_Stop()` + reset pos/Kalman = 0

**ตำแหน่งโค้ด main.c:**
```c
proximity = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
if (proximity == 1) { check = 55555; }  // ปัจจุบัน: แค่ monitor
```
(proximity stop logic ถูก comment ออกชั่วคราว — ให้ uncomment เมื่อต้องการ)

---

## 10. Base System

### Version ปัจจุบัน
- `main_v1_1.exe` (จาก GitHub commit e0f963b)
- `frontend-image-v1_1.tar` (จาก GitHub commit a6e34a4)
- GitHub: https://github.com/SUNTADTAWAN/FRA263-264_BaseSystem

### Run คำสั่ง
```powershell
docker load -i frontend-image-v1_1.tar
docker-compose up -d
.\main_v1_1.exe
```

---

## 11. Debug Variables (Live Expression)

| Variable | Location | ความหมาย |
|---|---|---|
| `check` | Mode.c | 1=Jog Move, 3=JOG mode, 4=after Mode_Jog, 33=AUTO, 44=after Auto |
| `current_mode` | Mode.c | enum: IDLE=0, GOHOME=1, JOG=2, AUTO=3, SET_HOME=4, TEST=5 |
| `g_arm.pos_deg` | robot_arm | ตำแหน่งปัจจุบัน (Kalman filtered) |
| `g_arm.ref_pos_deg` | robot_arm | ตำแหน่ง reference จาก S-Curve |
| `g_arm.done` | robot_arm | true = หยุดนิ่งแล้ว |
| `g_arm.running` | robot_arm | true = กำลังเคลื่อนที่ |
| `uart_rx_count` | main.c | จำนวน Modbus frames ที่รับได้ |
| `dbg_crc_fail` | main.c | จำนวน CRC errors |
| `dbg_last_write_addr` | Modbus.c | address ล่าสุดที่ถูก write |
| `dbg_last_write_value` | Modbus.c | value ล่าสุดที่ถูก write |
| `dbg_last_fc` | Modbus.c | Function code ล่าสุด (6=FC06, 16=FC10) |
| `modbus_jog_latch` | Modbus.c | Jog step ที่ถูก latch ไว้ |
| `dbg_jog_raw` | Mode.c | reg[5] ขณะที่ Mode_Jog อ่าน |
| `dbg_jog_sticky` | Mode.c | ค่า jog ที่ Mode_Jog เห็นล่าสุด (sticky) |
| `home_offset_deg` | Mode.c | ตำแหน่ง home ที่ Set Home |
| `can_node_state` | CANBUS.c | สถานะ CAN node |
| `can_relay_state` | CANBUS.c | สถานะ relay ปัจจุบัน |

---

## 12. สิ่งที่ยังต้องทำ / Known Issues

### ยังไม่เสร็จ
1. **Joystick integration** — Joystick_Process() ยังถูก comment ออก เพราะ HAL_Delay ใน Gripper จะรบกวน Modbus
2. **Proximity homing** — logic หยุดเมื่อเจอ sensor ถูก comment ออก รอ tune offset ให้ตรง
3. **CAN Bus testing** — ยังไม่ได้ทดสอบกับ node จริง (ต้องต่อสาย + ต้อง enable FDCAN ใน CubeMX)
4. **PID tuning** — ยังต้องปรับ gain ให้ smooth ขึ้น โดยเฉพาะ fine zone

### Known Issues
- **reg[5] (REG_JOG) ปัญหา:** ค่า jog ไม่เข้า reg[5] ตรงๆ — แก้ด้วย `modbus_jog_latch` ใน main.c
- **Gripper UP (0x0000):** ต้อง detect edge change เพราะ `if (reg != 0)` จะบล็อค — แก้แล้วใน main.c
- **Set Home:** ใช้ REG_P2P_TARGET (0x24) เป็น target angle

---

## 13. TIM Assignments

| Timer | Function |
|---|---|
| TIM1 | PWM output สำหรับ motor (8500 period) |
| TIM2 | Encoder input (motor) |
| TIM3 | Encoder input (joystick rotary) |
| TIM6 | 1kHz interrupt → Mode_TrajTick |
| TIM7 | 1kHz interrupt → RobotArm_ControlTick |
| TIM16 | Modbus T3.5 one-pulse timer |
| FDCAN1 | CAN Bus 500kbps |
| LPUART1 | Modbus RTU 19200 baud 8E1 + DMA |
| I2C1 | LCD display |

---

## 14. คำสั่งที่ใช้บ่อย

### Build & Flash
- Clean + Build: **Project → Clean Project → Build Project**
- Flash: Run As Debug (F11)

### Docker (Base System)
```powershell
cd D:\STU2-2568
docker load -i frontend-image-v1_1.tar
docker-compose up -d
.\main_v1_1.exe
```

### GitHub Base System
```powershell
cd D:\STU2-2568\BaseSystem
git log --oneline   # ดู commit history
git checkout <hash> -- <file>  # ดึงไฟล์จาก commit
```
