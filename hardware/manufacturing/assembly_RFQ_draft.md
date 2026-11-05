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

Please provide lead-free fabrication/assembly options, green soldermask and
lead-free HASL versus ENIG pricing if available. Quote A Economic versus
Standard if compatible with all selected parts. For B, quote **Standard with
an approved thermal profile**: TI specifies a peak reflow limit of **245 °C**
for LM2596S-5.0/NOPB, MSL3. The advertised Economic 255 ± 5 °C profile is not
acceptable for that component. Confirm lot-specific moisture handling and
measured component peak temperature, rather than relying only on an oven setpoint.

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
  and the Harwin parts above. RC0603FR-0747KL has a discovered C105579
  identifier but its current JLC page check was rate-limited; confirm it.
  Do not change manufacturer merely because another part has a similar MPN.
- C10's exact height, capacitance/ESR and R13's value are design constraints.
  Alternatives must be proposed with manufacturer data, land pattern,
  dimensions and electrical rationale before substitution approval.
- Please itemize sourcing, PCB, stencil, setup, SMT, THT, inspection,
  depanelization, lead time and delivery charges. Include factory DFM issues.

Sources for process requirements:
[TI LM2596](https://www.ti.com/lit/ds/symlink/lm2596.pdf#page=37),
[JLC assembly capabilities](https://jlcpcb.com/capabilities/pcb-assembly-capabilities).
