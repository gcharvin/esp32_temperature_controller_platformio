"""Evaluate only the exploratory 12 V startup without replacing 24 V results."""
import hashlib
import json
import math
from pathlib import Path

ROOT=Path(__file__).resolve().parent
DECK=ROOT/'benches/integrated_B_startup_12V.cir'


def main():
    run=json.loads(DECK.with_suffix('.run.json').read_text(encoding='utf-8'))
    if run['status']!='complete' or run['errors']:
        raise ValueError('A complete transient is required')
    for name,digest in run['input_sha256'].items():
        if hashlib.sha256(Path(name).read_bytes()).hexdigest()!=digest:
            raise ValueError('Simulation input changed: '+name)
    baseline=(ROOT/'benches/integrated_B_startup.cir').read_text(encoding='utf-8').splitlines()[1:]
    variant=DECK.read_text(encoding='utf-8').splitlines()[1:]
    if variant!=[line.replace('VIN vin 0 PWL(0 0 100u 24)','VIN vin 0 PWL(0 0 100u 12)') for line in baseline]:
        raise ValueError('Unexpected circuit change besides 24 V ->12 V')
    settled_min=ripple_min=math.inf
    settled_max=ripple_max=start_max=peak=-math.inf
    settled_count=ripple_count=rows=0
    last=-math.inf
    reached=False
    with DECK.with_suffix('.dat').open() as stream:
        names=next(stream).lower().split()
        for line in stream:
            values=list(map(float,line.split()))
            if not all(math.isfinite(v) for v in values):
                raise ValueError('Nonfinite waveform')
            t,v=values[0],values[names.index('v(vout)')]
            if t<last:
                raise ValueError('Invalid time axis')
            last=t
            rows+=1
            peak=max(peak,abs(values[names.index('l1#branch')]))
            if t<=0.004:
                start_max=max(start_max,v)
                reached|=v>=4.75
            if 0.004<=t<=0.008:
                settled_min=min(settled_min,v);settled_max=max(settled_max,v);settled_count+=1
            if 0.007<=t<=0.008:
                ripple_min=min(ripple_min,v);ripple_max=max(ripple_max,v);ripple_count+=1
    if abs(last-0.008)>1e-10 or rows!=run['rows'] or min(settled_count,ripple_count)<10:
        raise ValueError('Incomplete waveform')
    checks=dict(settled_range=4.75<=settled_min<=settled_max<=5.25,
                settled_ripple=ripple_max-ripple_min<0.05,
                startup_overvoltage=start_max<=5.25,startup_reaches_regulation=reached)
    result=dict(status='criteria passed' if all(checks.values()) else 'criteria FAILED',
                input_V=12, load_A=0.5, stop_s=last, rows=rows,
                settled_window_s=[0.004,0.008], settled_min_V=settled_min,settled_max_V=settled_max,
                ripple_window_s=[0.007,0.008], ripple_Vpp=ripple_max-ripple_min,
                startup_max_V=start_max,inductor_peak_A=peak, checks=checks,
                waveform_sha256=hashlib.sha256(DECK.with_suffix('.dat').read_bytes()).hexdigest(),
                scope='Exploratory nominal buck core only, same components/C9 as 24 V. No full board 12 V qualification, upstream protection, heater, tolerance, thermal or source-sharing claim.')
    (ROOT/'startup_12V_results.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result,indent=2))
    if not all(checks.values()):
        raise ValueError('12 V startup criteria failed')


if __name__=='__main__':
    main()
