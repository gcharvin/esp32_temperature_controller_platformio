# Draft request for quotation — NOT SENT

Please quote fabrication and complete assembly of a small prototype batch:
**5 × PCB A (display) and 5 × PCB B (temperature)**. The attached package is
for engineering review and quotation; it is not an approved production order.

| Item | A v0.4 | B v0.5 |
| --- | --- | --- |
| Functional PCB outline | 110 × 75 mm | 90 × 65 mm |
| Construction | 2-layer FR-4, 1.6 mm | 2-layer FR-4, 1.6 mm |
| Finished external copper | 1 oz each side | **2 oz each side** |
| SMT placement | Top side, 13 components | Top side, 38 components |
| THT assembly | 8 native items + 2 separate sockets | 9 native items |
| Scope | All listed SMT and THT parts soldered | All listed SMT and THT parts soldered |

Please quote lead-free compatible FR4 Tg>=150C, **ENIG**, green soldermask and
white legends. Any lower-cost finish is a separately priced, unapproved alternative. Quote A Economic versus
Standard if compatible with all selected parts. For B, quote **Standard with
an approved thermal profile**: TI specifies a peak reflow limit of **245 °C**
for LM2596S-5.0/NOPB, MSL3. The advertised Economic 255 ± 5 °C profile is not
acceptable for that component. Confirm lot-specific moisture handling and
measured component peak temperature, rather than relying only on an oven setpoint.
Kingbright LED lots also include MSL3 parts. Check all actual lot labels.

B's functional height is 65 mm. Please propose removable manufacturing rails
and tooling to meet your Standard assembly envelope, for example **5 mm
rails above and below B, panel outline 90 × 75 mm**. This is an engineering
proposal, not a released panel design. Put fiducials/tooling holes on the
rails and submit the panel drawing, tab locations and component orientations
for review. Please deliver the functional boards fully depanelized with flush,
deburred edges: their perimeter engages printed enclosure clips/rails. Do not
enlarge the functional PCB, put tooling holes into it or leave tabs in the
clip-contact areas. Indicate contour/thickness tolerances and inspection.

The attached SMT BOM/CPL pairs have matching designators. Rotations are native
KiCad values; approve against actual assembly-library orientation for each
polarized/asymmetric part, including MOSFETs, LEDs, D9 and LM2596 tab/pin3.
Through-hole parts require the separate THT lists and socket instructions.
The three-page assembly drawing, per-pad net/coordinate map and physical THT
placement list are supplied in `assembly_review/`. Local coordinates start at
the upper-left board corner in the top view; native coordinates are also
included. Quote and align the two socket rows separately, not at U1's centroid.

**Special purchasing/assembly points:**

- A/U1 is a customer-owned removable ESP32 DevKit, **not a soldered module**.
  Quote two Harwin M20-7821546 1×15 sockets under the two groups of U1 pads,
  25.4 mm between rows, 8.5 mm housing height. Do not treat U1 as one placed
  component. Jumper header M20-9990246 and separate shunt M7566-05 are also
  requested; the shunt may be supplied loose for final service configuration.
- B/NT1 is a copper net tie: **no component to buy or mount**.
- Exact-match public catalogue IDs have been added where verified. This does
  not establish reserved or live assembly stock. Please confirm quantity,
  availability, lead time, losses/minimum quantities and price for every line.
- Exact sourcing remains requested for **EEUFR1V331UB**, **WSL1206R0560FEA**
  and the Harwin parts above. RC0603FR-0747KL is verified as C105579 in the
  public catalogue; live assembly stock still requires confirmation.
  Do not change manufacturer merely because another part has a similar MPN.
- C10's exact height, capacitance/ESR and R13's value are design constraints.
  Alternatives must be proposed with manufacturer data, land pattern,
  dimensions and electrical rationale before substitution approval.
- R13's local footprint follows Vishay's recommended lands for WSL1206 in the
  0.006..0.200 ohm range: two 1.65 x 1.93 mm rectangles, 3.45 mm centre spacing,
  1.80 mm inner gap. Please review stencil apertures and process for this part.
- Please itemize sourcing, PCB, stencil, setup, SMT, THT, inspection,
  depanelization, lead time and delivery charges. Include factory DFM issues.

Please use `fabrication_specification.json` as the build specification and
return `factory_response.csv` with the quote/DFM/placement-preview evidence.
Nominal finished socket holes are1.05mm; confirm finished-hole tolerance and
socket insertion/soldering process. Supply the JP1 shunt loose, not installed.
Trim THT leads to a target<=2mm beyond the solder side.

Quote100% bare-board electrical test using `bare_board_test.d356`; raw KiCad
output is included separately. B/NT1 intentionally joins PGND/GND, normalized
only in the CAM test netlist. All other nets/coordinates remain unchanged.
Please quote the inspections and low-power functional checks in
`factory_acceptance_test.md`, with no DevKit, firmware or heater required.
Do not mark those checks done if they are not included in your service.

Optional cable quotation: one straight POWER, I2C and CTRL harness per A+B
set,300mm, as specified in `harness_pin_map.csv`/`wiring_and_harnesses.md`.

Sources for process requirements:
[TI LM2596](https://www.ti.com/lit/ds/symlink/lm2596.pdf#page=37),
[JLC assembly capabilities](https://jlcpcb.com/capabilities/pcb-assembly-capabilities).
