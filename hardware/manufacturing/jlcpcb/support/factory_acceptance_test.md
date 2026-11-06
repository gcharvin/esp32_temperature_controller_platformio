# Prototype assembly inspection and functional-test quotation

Please quote these checks separately from assembly if needed. They are
acceptance instructions, **not test results already obtained**. No ESP32,
LCD, sensor module or heater is supplied for factory installation.

## Inspection and bare-board tests

- Use the exact approved BOM. Record any proposed substitution before fitting.
- Request100% bare-board opens/shorts testing. `bare_board_test.d356` is the
  CAM test netlist; the raw KiCad export is retained for traceability. On B,
  intentional copper NT1 joins PGND/GND, so these names are normalized to one
  physical test net. Other names and every test-point coordinate are unchanged.
  The IPC file retains its native0.0001inch units; Gerbers/CPL/drill files are
  metric. Do not apply an arbitrary coordinate origin shift.
- AOI and visual inspection of solder joints, bridges, polarity and orientation;
  confirm the LM2596 tab joint inspection process. Handle MSL lots correctly.
- Inspect C8/C10 positive pad1, LED cathode1, D9 cathode1, Q1 G-D-S1-2-3,
  Q2 tab2 and LM2596 tab3/PGND. Follow the supplied assembly PDF and pin map.
- Assemble both A socket rows perpendicular to A and parallel to each other;
  centres25.40mm, contacts2.54mm. Do not fit an ESP32. Supply JP1 shunt loose.
- Trim THT leads to a target≤2mm below the board. Deliver fully depanelized,
  with deburred edges and no fixture/rail/tab residue in clip contact areas.

## Electrical check A, without external modules

1. JP1 open; J6/J7 disconnected; no DevKit or peripherals. Confirm no hard
   rail short before energizing. Inject regulated5.00V at J1 pin2, GND at pin1,
   with an initially50mA-limited supply. Stop on persistent current limiting.
2. Record supply current, J1 voltage and U2 pin5 / local3V3. Accept3.23–3.37V
   at room temperature with a calibrated instrument. Both5V/3V3 LEDs should
   light. No firmware is needed.
3. Confirm the JP1 link is open. Fit the shunt only temporarily, USB absent,
   and measure the5V supply at U1 socket15 with respect to14/17. Remove the
   shunt again before delivery.
4. Check connector pinout/continuity against the supplied map. I²C pullups
   belong on A; do not add parts to B as an unapproved correction.

## Electrical check B, without A or heater

1. No connections at J1/J3/J4/J6/J7. Apply24.0V at J2 pin1, PGND at pin2,
   with a current-limited laboratory supply; start unloaded and inspect for
   persistent abnormal current. Do not reverse the source for this check.
2. Record VIN_PROTECTED, +5V_BUCK beforeF2, J1 pin2 afterF2 and U2 pin5 /3V3.
   At room temperature accept **4.90–5.15V at J1**, and3.23–3.37V at U2 pin5.
   This5V acceptance band is deliberately narrower than the regulator's full
   datasheet envelope, to reserve cable-drop margin for the prototype.
3. Load J1 pin2 to pin1 with0.10A,0.50A,0.75A, using a suitable load and a
   source current limit sufficient for normal operation (e.g.0.5A at24V).
   Record J1 voltage at each step; accept the same DC range. Remove the load
   after the check. B logic current is additional; total5V budget remains0.8A.
4. With no commands connected, check Q1 gate-to-source<0.2V after settling;
   HEAT LED should stay off. VIN/5V/3V3 LEDs should light. No heater is attached.
5. If oscilloscopes are included in the quote, record startup/steady-state5V
   and ripple with a short ground connection. Engineering targets are no
   startup overshoot above5.25V and ripple<50mVpp; record method/bandwidth.

## Delivery report

Provide board identification, actual fitted part list, bare-board test outcome,
inspection outcome, measured values, instrument identification and any rework.
If functional checks cannot be performed, state that explicitly in the quote;
they will be performed at prototype commissioning and must not be marked done.
This test does not qualify100W operation, hot faults, the enclosure, USB source
sharing or thermal protection. Those tests follow receipt under the bench plan.
