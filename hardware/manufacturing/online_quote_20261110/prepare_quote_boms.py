"""Generate complete quote variants. Native CAD and released ZIPs stay unchanged."""
import csv
import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
SOURCE = HERE.parent / 'jlcpcb'
changes = {
    'C2167399': ('Walsin', '0603B105K160CT', 'C108463'),
    'C2167467': ('Samsung', 'CL10A106KP8NNNC', 'C19702'),
    'C1498285': ('Panasonic', 'EEUFS1V331LB', 'C1584641'),
    'C2090415': ('Vishay', 'WSL1206R0500FEA', 'C844686'),
    'C5879381': ('Kingbright', 'APT2012LSYCK/J3-PRV', 'C5879382'),
}
manifest = {'status': 'QUOTE ONLY - not a manufacturing release', 'boards': {}}
for board, tag, count in [('A', 'A04', 24), ('B', 'B05', 48)]:
    src = SOURCE / ('upload_' + board) / (tag + '_BOM_ALL_ASSEMBLED.csv')
    with src.open(newline='', encoding='utf-8-sig') as f:
        reader = csv.DictReader(f)
        fields = reader.fieldnames
        rows = list(reader)
    assert len(rows) == count
    modified = []
    for row in rows:
        old = row.copy()
        code = row['LCSC Part #']
        if board == 'A' and row['Designator'] in ('J8', 'J9'):
            replacement = ('HCTL', 'PM254-1-15-Z-8.5', 'C2897378')
        elif board == 'A' and row['Designator'] == 'JP1':
            replacement = ('Harwin', 'M20-9990246', 'C3332353')
        elif board == 'B' and row['Designator'] == 'C10':
            replacement = ('Panasonic', 'EEUFS1V331LB', 'C1584641')
        elif board == 'B' and row['Designator'] == 'R13':
            replacement = ('Vishay', 'WSL1206R0500FEA', 'C844686')
        else:
            replacement = changes.get(code)
        if replacement:
            row.update(zip(['Manufacturer', 'Manufacturer Part', 'LCSC Part #'], replacement))
        if board == 'B' and row['Designator'] == 'R13':
            row['Comment'] = '50mR/1%/0.25W/C10_damping_QUOTE_ONLY'
        if board == 'B' and row['Designator'] == 'D8':
            row['Comment'] = 'APT2012LSYCK/J3-PRV/HEAT_YELLOW_QUOTE_ONLY'
        if row != old:
            modified.append({'designator': row['Designator'], 'old': old, 'quote': row})
    assert all(row['LCSC Part #'].startswith('C') for row in rows)
    dest = HERE / (tag + '_BOM_STOCK_ALTERNATIVES_QUOTE_ONLY.csv')
    with dest.open('w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    manifest['boards'][board] = {'source_sha256': hashlib.sha256(src.read_bytes()).hexdigest(),
                                  'quote_sha256': hashlib.sha256(dest.read_bytes()).hexdigest(),
                                  'placements': count, 'changes': modified}
(HERE / 'quote_bom_changes.json').write_text(json.dumps(manifest, indent=2)+'\n')
