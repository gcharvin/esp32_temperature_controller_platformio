"""Quote-only engineering study; does not change the released fabrication data.

Uses the existing unmodified vendor models and verified transient runner.
Run with KiCad Python, then evaluate_simulations.py validation_plan.json.
"""
import copy
import json
from pathlib import Path
import re
import subprocess
import sys

HERE = Path(__file__).resolve().parent
POWER = HERE.parents[1] / 'power_study'
BASE = POWER / 'lm2596'


def prepare():
    plan = json.loads((BASE / 'validation_plan.json').read_text())
    plan['candidate'] = 'QUOTE ONLY: WSL1206R0500FEA + EEUFS1V331LB'
    plan['scope'] += (' Quote-only substitution study: 50 mOhm +/-1% damping resistor '
                      'in the C10 branch, not in the load return; same Vishay WSL1206 '
                      'package. Panasonic FS 330uF/35V, 41mOhm maximum impedance at '
                      '20C and 123mOhm at -10C. Constant ESR approximation only; '
                      'does not qualify PCB parasitics, hardware, thermal behavior '
                      'or a complete cold-temperature operating range.')
    plan['external_components'].update(
        Cout_added_series_resistance_ohm=0.050,
        Cout_lower_series_bound_ohm=0.049,
        output_capacitor_example='Panasonic EEUFS1V331LB; quote-only alternative',
        output_capacitor_source='https://industrial.panasonic.com/ww/products/pt/aluminum-cap-lead/models/EEUFS1V331LB')
    cases = copy.deepcopy(plan['cases'])
    stress = copy.deepcopy(next(c for c in cases if c['name'] == 'load_step_derated'))
    stress.update(name='upper_esr_stress', deck='benches/upper_esr_stress.cir')
    cases.append(stress)
    out = HERE / 'simulation'
    (out / 'benches').mkdir(parents=True, exist_ok=True)
    for case in cases:
        source = BASE / ('benches/load_step_derated.cir' if case['name'] == 'upper_esr_stress' else case['deck'])
        deck = source.read_text()
        # Preserve the vendor libraries exactly; only external component values change.
        vendor = (POWER / 'vendor_model').as_posix()
        deck = deck.replace('../../vendor_model', vendor)
        esr = '0.049' if case['name'] == 'esr_lower_bound' else '0.18' if case['name'] == 'upper_esr_stress' else '0.1'
        deck = re.sub(r'^RESR esr 0 .+$', 'RESR esr 0 ' + esr, deck, flags=re.M)
        (out / case['deck']).write_text(deck)
    plan['cases'] = cases
    (out / 'validation_plan.json').write_text(json.dumps(plan, indent=2) + '\n')
    return out, plan


if __name__ == '__main__':
    out, plan = prepare()
    for case in plan['cases']:
        subprocess.run([sys.executable, str(POWER / 'run_ngspice.py'),
                        str(out / case['deck']), '--vectors',
                        'v(vout)', 'v(vin)', 'v(sw)', 'i(l1)'], check=True)
