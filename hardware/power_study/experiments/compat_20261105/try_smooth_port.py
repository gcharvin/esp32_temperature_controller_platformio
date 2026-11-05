"""Experimental unvalidated derivative; never edits or publishes the vendor library.

All ideal IF thresholds become tanh decisions of voltage width epsilon.
The hold-latch nodes receive 1 Gohm leakage to resolve their floating DC state.
This is a compatibility experiment, not a manufacturer model validation.
"""
from pathlib import Path
import re, hashlib, json
BASE = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
SOURCE = BASE / 'vendor_model/LMR36520A_PSPICE_TRANS/LMR36520A_PSPICE_TRANS/LMR36520A_TRANS.LIB'

def top_split(s, chars):
    depth = 0
    start = 0
    parts = []
    for i, c in enumerate(s):
        if c in '({': depth += 1
        elif c in ')}': depth -= 1
        elif depth == 0 and c in chars:
            parts.append(s[start:i]); start = i + 1
    parts.append(s[start:])
    return parts

def cond(s, w):
    if len(parts := top_split(s, '|')) > 1:
        return '(1-' + '*'.join('(1-' + cond(p, w) + ')' for p in parts) + ')'
    if len(parts := top_split(s, '&')) > 1:
        return '(' + '*'.join(cond(p, w) for p in parts) + ')'
    depth = 0
    for i, c in enumerate(s):
        if c in '({': depth += 1
        elif c in ')}': depth -= 1
        elif depth == 0 and c in '<>':
            a, b = s[:i].strip(), s[i+1:].strip()
            if b.startswith('='): b = b[1:]
            sign = '1' if c == '>' else '-1'
            return f'(0.5*(1+{sign}*tanh((({a})-({b}))/{w})))'
    raise ValueError('Unknown condition ' + s)

def smooth(s, w):
    while m := re.search(r'\bif\s*\(', s, re.I):
        start, j = m.start(), m.end()
        depth, k = 1, j
        while depth:
            if s[k] == '(': depth += 1
            if s[k] == ')': depth -= 1
            k += 1
        parts = top_split(s[j:k-1], ',')
        assert len(parts) == 3, parts
        a, b, c = parts
        f = cond(a, w); b = smooth(b, w); c = smooth(c, w)
        s = s[:start] + f'(({f})*({b})+(1-({f}))*({c}))' + s[k:]
    return s

src = SOURCE.read_text()
flat = re.sub(r'\n\s*\+', ' ', src)
manifest = {'source_sha256': hashlib.sha256(SOURCE.read_bytes()).hexdigest(), 'vendor_validation': False, 'experiments': []}
for width in ('100u', '1m', '10m'):
    model = '\n'.join(smooth(line, width) if not line.startswith('*') else line for line in flat.splitlines())
    model = re.sub(r'(?im)^(\.SUBCKT SRLATCHRHP_BASIC_NEW[^\n]*)', r'\1\nR_port_q Q 0 1G\nR_port_qb QB 0 1G', model)
    name = f'all_if_smooth_{width}'
    (OUT / (name + '.lib')).write_text('* EXPERIMENTAL UNVALIDATED SMOOTH NGSPICE PORT\n' + model)
    deck = (BASE / 'benches/startup.cir').read_text()
    deck = re.sub(r'(?m)^\.include.*', f'.include "{name}.lib"', deck).replace('12m', '2m').replace('vntol=1u', 'vntol=1u method=gear')
    (OUT / (name + '.cir')).write_text(deck)
    manifest['experiments'].append({'library': name + '.lib', 'smoothing_width_SPICE': width, 'sha256': hashlib.sha256((OUT / (name + '.lib')).read_bytes()).hexdigest()})
fine = (OUT / 'all_if_smooth_1m.cir').read_text().replace('.tran 100n 2m 0 100n', '.tran 5n 2m 0 5n').replace('method=gear', 'method=gear maxord=2 trtol=1 itl4=200')
(OUT / 'all_if_smooth_fine.cir').write_text(fine)
(OUT / 'port_manifest.json').write_text(json.dumps(manifest, indent=2))
(OUT / '.gitignore').write_text('*.lib\n*.dat\n*.log\n*.cir\nrunner.py\n')
