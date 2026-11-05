# /// script
# requires-python = ">=3.11"
# dependencies = ["numpy>=2,<3"]
# ///
"""Measure out-of-bound ESR tests separately; never overwrite main results."""
import hashlib
import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
plan = json.loads((HERE/'sensitivity_plan.json').read_text())
results = dict(scope=plan['scope'], criteria=plan['criteria'], cases=[])
for case in plan['cases']:
    deck = HERE/case['deck']
    run = json.loads(deck.with_suffix('.run.json').read_text())
    assert run['status'] == 'complete'
    assert run['input_sha256'][str(deck.resolve())] == hashlib.sha256(deck.read_bytes()).hexdigest()
    datafile = deck.with_suffix('.dat')
    with datafile.open() as stream:
        names = next(stream).lower().split()
    data = np.loadtxt(datafile, skiprows=1)
    assert np.isfinite(data).all()
    t, v = data[:,0], data[:,names.index('v(vout)')]
    assert abs(t[-1]-run['expected_stop_s']) < 1e-12
    start,end = case['regulated_window_s']
    regulated = (t>=start)&(t<=end)
    start,end = case['ripple_window_s']
    quiet = (t>=start)&(t<=end)
    lo,hi = plan['criteria']['output_range_V']
    vmin,vmax,ripple = float(v[regulated].min()),float(v[regulated].max()),float(np.ptp(v[quiet]))
    results['cases'].append(dict(name=case['name'], completed_until_s=float(t[-1]),
        deck_sha256=hashlib.sha256(deck.read_bytes()).hexdigest(),
        waveform_sha256=hashlib.sha256(datafile.read_bytes()).hexdigest(),
        output_min_V=vmin, output_max_V=vmax, ripple_Vpp=ripple,
        regulated_range=lo<=vmin<=vmax<=hi,
        settled_ripple=ripple<=plan['criteria']['ripple_max_Vpp']))
(HERE/'sensitivity_results.json').write_text(json.dumps(results,indent=2)+'\n')
print(json.dumps(results,indent=2))
