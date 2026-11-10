# /// script
# requires-python = ">=3.11,<3.14"
# dependencies = ["cadquery==2.6.1", "pillow>=11,<12"]
# ///
"""Check quote alternatives without editing/re-exporting the released enclosure."""
import hashlib
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
MECHANICAL = HERE.parents[1] / 'mechanical'
sys.path.insert(0, str(MECHANICAL))
import build_enclosure as b

b.OUT = HERE / 'mechanical'
b.OUT.mkdir(exist_ok=True)
b.TAG = 'quote_alternatives_20261110'
board = next(x for x in b.LAYOUT['boards'] if x['name'] == 'pcb_b_temperature')
c10 = next(x for x in board['components'] if x['reference'] == 'C10')
old_height = c10['height_mm']
# Panasonic FS 8x15 case: L+1.5 maximum; retain the larger original D10 envelope.
c10['height_mm'] = 16.5
# HCTL maximum body 38.8 x 2.65 x 8.65 is inside the existing Harwin envelope.
assert all(a <= z for a, z in zip([38.8, 2.65, 8.65], b.MODULES['devkit']['socket']['maximum_body_mm']))
b.render_previews = lambda: None
b.main(validate=False)
paths = b.verify_assembly_paths()
c10_part = next(p for p in b.PARTS if p['name'] == 'B_C10_ENVELOPE')
contacts = []
for part in b.PARTS:
    if part is c10_part or not part['physical'] or part['name'] == 'PCB_B':
        continue
    overlap = b._overlap(c10_part['shape'], part['shape'])
    if overlap > 1e-4:
        contacts.append(dict(other=part['name'], volume_mm3=overlap))
report = dict(scope='Quote-only mechanical alternatives, conservative original D10 C10 footprint envelope',
              C10_old_height_mm=old_height, C10_alternative_max_height_mm=16.5,
              socket_alternative_max_body_mm=[38.8, 2.65, 8.65],
              socket_existing_envelope_mm=b.MODULES['devkit']['socket']['maximum_body_mm'],
              static_C10_collisions=contacts, service_stages=len(paths['tests']),
              assembly_failures=paths['failures'],
              source_sha256={str(p.relative_to(b.ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in b.SOURCE_INPUTS},
              physical_fit_verified=False)
(b.OUT / 'alternative_clearance.json').write_text(json.dumps(report, indent=2)+'\n')
assert not contacts, contacts
print(json.dumps(report, indent=2))
