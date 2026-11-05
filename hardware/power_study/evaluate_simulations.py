# /// script
# requires-python = ">=3.11"
# dependencies = ["numpy>=2,<3", "matplotlib>=3.9,<4"]
# ///
"""Measure completed buck transients and render reproducible review plots.

Usage: uv run evaluate_simulations.py validation_plan.json
The plan names each deck, evaluation interval, ripple interval and shutdown
criterion explicitly. No partial transient is accepted as a functional result.
"""
import argparse
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def evaluate(planpath):
    plan = json.loads(planpath.read_text(encoding='utf-8'))
    destination = planpath.parent / 'results'
    destination.mkdir(exist_ok=True)
    summary = dict(candidate=plan['candidate'], model=plan['model'],
                   scope=plan['scope'], criteria=plan['criteria'], cases=[])
    lower, upper = plan['criteria']['output_range_V']
    for case in plan['cases']:
        deck = (planpath.parent / case['deck']).resolve()
        runpath = deck.with_suffix('.run.json')
        run = json.loads(runpath.read_text(encoding='utf-8'))
        if run['status'] != 'complete':
            raise RuntimeError('Incomplete transient: '+str(deck))
        for filename, expected in run.get('input_sha256', {}).items():
            if digest(Path(filename)) != expected:
                raise RuntimeError('An input changed since simulation: '+filename)
        datafile = deck.with_suffix('.dat')
        with datafile.open() as stream:
            names = next(stream).lower().split()
        data = np.loadtxt(datafile, skiprows=1)
        if not np.isfinite(data).all():
            raise RuntimeError('Non-finite waveform: '+str(deck))
        t = data[:, 0]
        if np.any(np.diff(t) < 0) or abs(t[-1]-run['expected_stop_s']) > 1e-10:
            raise RuntimeError('Invalid time axis: '+str(deck))
        vout = data[:, names.index('v(vout)')]
        start, end = case['regulated_window_s']
        sel = (t >= start) & (t <= end)
        rt0, rt1 = case['ripple_window_s']
        quiet = (t >= rt0) & (t <= rt1)
        if np.count_nonzero(sel) < 10 or np.count_nonzero(quiet) < 10:
            raise RuntimeError('Not enough evaluated samples')
        vmin, vmax = float(vout[sel].min()), float(vout[sel].max())
        # Average weighted by simulator time, not by its adaptive sample density.
        mean = float(np.trapezoid(vout[quiet], t[quiet])/(t[quiet][-1]-t[quiet][0]))
        ripple = float(np.ptp(vout[quiet]))
        checks = dict(regulated_range=(lower <= vmin <= vmax <= upper),
                      settled_ripple=ripple <= plan['criteria']['ripple_max_Vpp'],
                      startup_overvoltage=float(vout[t <= start].max()) <= upper,
                      startup_reaches_regulation=bool(np.any(vout[t <= start] >= lower)))
        item = dict(name=case['name'], deck=str(deck.relative_to(planpath.parent)),
                    deck_sha256=digest(deck), waveform_sha256=digest(datafile),
                    samples=len(t), completed_until_s=float(t[-1]),
                    regulated_window_s=[start, end], output_min_V=vmin, output_max_V=vmax,
                    ripple_window_s=[rt0, rt1], output_mean_V=mean, ripple_Vpp=ripple,
                    startup_max_V=float(vout[t <= start].max()),
                    final_output_V=float(vout[-1]), checks=checks)
        item['inputs_hashed_at_run_start'] = bool(run.get('input_sha256'))
        current_name = next((n for n in ('l1#branch', 'i(l1)') if n in names), None)
        if current_name:
            item['inductor_peak_A'] = float(data[:, names.index(current_name)].max())
        if 'shutdown_below_V' in case:
            checks['shutdown_discharge'] = float(vout[-1]) < case['shutdown_below_V']
            if 'v(sw)' in names:
                last_ms = t >= t[-1]-0.001
                item['shutdown_switch_max_V_last_ms'] = float(data[last_ms, names.index('v(sw)')].max())
                input_min = float(data[last_ms, names.index('v(vin)')].min())
                checks['shutdown_no_high_switch_pulses_last_ms'] = item['shutdown_switch_max_V_last_ms'] < .5*input_min
        item['all_criteria_pass'] = all(checks.values())
        summary['cases'].append(item)
        fig, axes = plt.subplots(2, 1, figsize=(10, 6), layout='constrained')
        # Preserve all switching extrema: min/max bins, not a sparse sample.
        for ax, window in zip(axes, [(0, t[-1]), (rt0, rt1)]):
            chosen = np.flatnonzero((t >= window[0]) & (t <= window[1]))
            # Smaller inset is cheap to draw without reduction.
            step = max(1, len(chosen)//10000)
            indices = []
            for i in range(0, len(chosen), step):
                block = chosen[i:i+step]
                indices.extend(sorted({int(block[0]), int(block[-1]),
                                       int(block[np.argmin(vout[block])]),
                                       int(block[np.argmax(vout[block])])}))
            ax.plot(t[indices]*1000, vout[indices], linewidth=.7, color='#096b91')
            ax.axhline(lower, color='#a64b2a', linestyle='--', linewidth=.7)
            ax.axhline(upper, color='#a64b2a', linestyle='--', linewidth=.7)
            ax.set(xlabel='Temps (ms)', ylabel='Sortie (V)', xlim=(window[0]*1000, window[1]*1000))
            ax.grid(alpha=.2)
        axes[0].set_title(plan['candidate']+' — '+case['name'])
        axes[1].set_title(f'Fenêtre stabilisée : moyenne {mean:.4f} V ; ondulation {ripple*1000:.1f} mV crête à crête')
        fig.savefig(destination/(case['name']+'.png'), dpi=140)
        fig.savefig(destination/(case['name']+'.svg'))
        plt.close(fig)
    summary['all_criteria_pass'] = all(c['all_criteria_pass'] for c in summary['cases'])
    (destination/'simulation_results.json').write_text(json.dumps(summary, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(summary, indent=2))
    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('plan', type=Path)
    args = parser.parse_args()
    result = evaluate(args.plan.resolve())
    if not result['all_criteria_pass']:
        raise SystemExit('A completed simulation did not meet all electrical targets')
