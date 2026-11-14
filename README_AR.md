# Milestone 2 – FINAL

```
Milestone2_FINAL/
  Hand_Calculations_Milestone2.docx   الحسابات باليد (Word)
  Circuit_Diagram.png / .pdf          الدايرة كاملة بالـ pins
  MATLAB/                             كل كود الماتلاب  ← خليه الـ Current Folder
    Simscape_STEP/                    الروبوت من الـ STEP في Simscape Multibody
  ESP32_Robot/ESP32_Robot.ino         كود الـ ESP32 (الماتلاب هو اللي عامله)
```

## الترتيب (الـ Current Folder = MATLAB)

| الأمر | بيعمل إيه |
|---|---|
| `robot_model` | الـ parameters + الـ state space + الـ TF + الـ steady state + الـ ramp + الـ PI + Bode + Root locus |
| `robot_kinematics` | Forward و Inverse kinematics |
| `rst_design` | RST controller (Diophantine) ومقارنته بالـ PI |
| `robot_animation` / `record_animation` | الروبوت من الـ CAD بيطلع الـ ramp / فيديو على الـ Desktop |
| `export_results` | كل الرسومات PNG + كل الأرقام في Desktop\Milestone2_Results |
| `build_robot_sim` | Simulink: الموديل الكامل + PI + ramp |
| `addpath('Simscape_STEP')` ثم `save_simscape_video` | Simscape: الـ STEP بيتحرك بالموديل |
| **`generate_esp32_code`** | **الماتلاب بيكتب كود الـ ESP32 من أرقام الموديل** ← `ESP32_Robot\ESP32_Robot.ino` |
| `test_robot_connection` | (مرة واحدة والعجل مرفوع) يختبر الـ WiFi واتجاه العجل والـ encoder والـ PI |
| `build_robot_wireless` | Simulink بيحرّك الروبوت الحقيقي على **WiFi** + sliders + scopes live |

## الروبوت الحقيقي على WiFi
1. وصّل حسب `Circuit_Diagram.png`. اظبط الـ LM2596 على 5.0 V الأول.
2. `generate_esp32_code` ← افتح `ESP32_Robot\ESP32_Robot.ino` في Arduino IDE ← Board: **ESP32 Dev Module** ← Upload.
3. الـ ESP32 بيعمل شبكة WiFi اسمها **Robot_ESP32** (الباسورد **12345678**). وصّل اللابتوب عليها.
   أول مرة: لو Windows سأل عن الـ firewall للماتلاب ← Allow (Private + Public).
4. `test_robot_connection` (والعجل مرفوع) وصلّح اللي يقولك عليه.
   لو قالك غيّر `DIR_L` أو `ENC_SIGN_R`... غيّرها في `generate_esp32_code.m` واعمل generate و upload تاني.
5. `build_robot_wireless` ← Run ← حرّك الـ SPEED slider والـ TURN knob.
   `mode`: 1 = PI ، 2 = RST ، 0 = open loop (`V_test`). حدود الأمان: 0.4 m/s و 2 rad/s.

لو حبيت Bluetooth بدل WiFi: `generate_esp32_code('bt')` وفي الماتلاب `ESP32_LINK = "bt"; ESP32_PORT = "COMx";`
