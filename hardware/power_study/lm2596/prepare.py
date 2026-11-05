"""Fetch checksum-pinned official models and build ngspice buck benches.

No installer, account, model modification or production PCB edit is involved.
Use the Python shipped with Codex or KiCad. Vendor files stay in the ignored
power_study/vendor_model directory; this source regenerates them.
"""
import hashlib
import io
import json
from pathlib import Path
import urllib.request
import zipfile

HERE = Path(__file__).resolve().parent
VENDOR = HERE.parent / 'vendor_model'
TI_URL = 'https://www.ti.com/lit/zip/snvma62'
TI_SHA = 'c8d6730cb669b577e013acd14653ef2be12035e778b3b4449e384c93bab8636f'
MODEL_SHA = 'bf883d939dbdce9c9146f9fd6d6c090e7cfb7afb5aaa01b6b8391ca06dbe8134'
DIODE_URL = 'https://www.vishay.com/docs/88171/ss36.txt'
DIODE_SHA = 'ed8ecea02b1440cbb7a3db34dc57a7a88581f34b71ffd8795607a8f3758b202a'


def fetch():
    VENDOR.mkdir(exist_ok=True)
    archive = VENDOR / 'lm2596_snvma62.zip'
    if not archive.exists():
        archive.write_bytes(urllib.request.urlopen(TI_URL, timeout=30).read())
    assert hashlib.sha256(archive.read_bytes()).hexdigest() == TI_SHA, 'TI archive changed; review before updating SHA'
    with zipfile.ZipFile(io.BytesIO(archive.read_bytes())) as z:
        name = next(n for n in z.namelist() if n.upper().endswith('LM2596_5P0_TRANS.LIB'))
        data = z.read(name)
    assert hashlib.sha256(data).hexdigest() == MODEL_SHA
    (VENDOR / 'LM2596_5P0_TRANS.LIB').write_bytes(data)
    diode = VENDOR / 'ss36.lib'
    if not diode.exists():
        diode.write_bytes(urllib.request.urlopen(DIODE_URL, timeout=30).read())
    assert hashlib.sha256(diode.read_bytes()).hexdigest() == DIODE_SHA, 'Vishay model changed; review before updating SHA'


def prepare():
    fetch()
    benches = HERE / 'benches'
    benches.mkdir(exist_ok=True)
    cases = [
        dict(name='startup', vin='PWL(0 0 100u 24)', load='0.5',
             stop='8m', regulated_window_s=[0.004, 0.008], ripple_window_s=[0.007, 0.008]),
        dict(name='load_step', vin='PWL(0 0 100u 24)',
             load='PWL(0 0.1 5m 0.1 5.01m 0.8 9m 0.8 9.01m 0.1)',
             stop='12m', regulated_window_s=[0.004, 0.012], ripple_window_s=[0.008, 0.009]),
        dict(name='load_step_derated', vin='PWL(0 0 100u 24)', cout='264u', inductance='80u',
             load='PWL(0 0.1 5m 0.1 5.01m 0.8 9m 0.8 9.01m 0.1)',
             stop='12m', regulated_window_s=[0.004, 0.012], ripple_window_s=[0.008, 0.009]),
        dict(name='esr_lower_bound', vin='PWL(0 0 100u 24)', cout='264u', inductance='80u', esr='0.056',
             load='PWL(0 0.1 5m 0.1 5.01m 0.8 9m 0.8 9.01m 0.1)',
             stop='12m', regulated_window_s=[0.004, 0.012], ripple_window_s=[0.008, 0.009]),
        dict(name='line_step', vin='PWL(0 0 100u 24 5m 24 5.01m 26.4 8m 26.4 8.01m 18)', load='0.8',
             stop='12m', regulated_window_s=[0.004, 0.012], ripple_window_s=[0.011, 0.012]),
        dict(name='disable', vin='PWL(0 0 100u 24)', load='0.5',
             disable='PWL(0 0 5m 0 5.01m 3.3)', stop='15m',
             regulated_window_s=[0.004, 0.005], ripple_window_s=[0.004, 0.005], shutdown_below_V=0.5),
    ]
    for case in cases:
        case['deck'] = 'benches/' + case['name'] + '.cir'
        text = f'''LM2596 fixed 5 V - {case['name']} - original TI and Vishay models
.include "../../vendor_model/LM2596_5P0_TRANS.LIB"
.include "../../vendor_model/ss36.lib"
VIN vin 0 {case['vin']}
VDIS disable 0 {case.get('disable','0')}
XU1 vin vout sw 0 disable LM2596_5P0_TRANS
CIN vin cinret 100u
RIN_ESR cinret 0 0.1
L1 sw lind {case.get('inductance','100u')}
RDCR lind vout 0.118
XD1 0 sw ss36
COUT vout esr {case.get('cout','330u')}
RESR esr 0 {case.get('esr','0.1')}
VLOADCMD loadcmd 0 {case['load']}
GLOAD vout 0 VALUE={{V(loadcmd)*LIMIT(V(vout)/4.75,0,1)}}
.options method=gear maxord=2 reltol=0.003 abstol=1n vntol=1u
.tran 100n {case['stop']} 0 100n
.end
'''
        (HERE / case['deck']).write_text(text, encoding='utf-8')
    plan = dict(
        candidate='LM2596S-5.0, 24 V to 5 V / 0.8 A study',
        model=dict(name='LM2596_5P0_TRANS', version='TI Final 1.00 / 2015-07-06', modified=False,
                   source_url=TI_URL, archive_sha256=TI_SHA, sha256=MODEL_SHA,
                   diode=dict(name='ss36', source_url=DIODE_URL, sha256=DIODE_SHA, modified=False)),
        scope='Electrical transient study after upstream protection and before existing F2. Excludes thermal behavior, parasitic PCB/cable effects, EMI, USB sharing, MCU firmware and manufacture qualification.',
        criteria=dict(output_range_V=[4.75,5.25], ripple_max_Vpp=0.05),
        external_components=dict(L_H=100e-6, L_DCR_ohm=0.118, Cin_F=100e-6, Cin_ESR_ohm=0.1,
                                 Cout_F=330e-6, Cout_total_series_ohm=0.1, Cout_added_series_resistance_ohm=0.056, Cout_lower_series_bound_ohm=0.056,
                                 Cout_tolerance_case_F=264e-6,
                                 output_capacitor_example='Panasonic EEUFR1V331U, 330 uF / 35 V, +/-20%, 43 mOhm max impedance at 100 kHz and 20 C. Add an explicit 56 mOhm series resistor in the capacitor branch to bound minimum ESR. Total 100 mOhm rounds up the capacitor 43 mOhm maximum impedance plus 56 mOhm resistor; lower-bound case assumes zero capacitor ESR and 56 mOhm total. Constant ESR at switching frequency is an approximation, not a fitted capacitor model; temperature, ESL and aging are not qualified.',
                                 output_capacitor_source='https://industrial.panasonic.com/ww/products/pt/aluminum-cap-lead/models/EEUFR1V331UB',
                                 diode='Vishay SS36, SMC, 60 V / 3 A, official unmodified SPICE3 model',
                                 inductor='Bourns SRP1770TA-101M, 100 uH +/-20%, DCR max 118 mOhm, Isat1 6.5 A at 20% loss; modeled linearly. Derated case L=80 uH and C=264 uF. Startup peaks and physical integration require review.',
                                 inductor_source='https://www.bourns.com/docs/Product-Datasheets/srp1770ta.pdf'),
        cases=[{k:v for k,v in c.items() if k in ['name','deck','regulated_window_s','ripple_window_s','shutdown_below_V']} for c in cases],
    )
    (HERE / 'validation_plan.json').write_text(json.dumps(plan, indent=2)+'\n', encoding='utf-8')
    # Deliberately outside the proposed 56..100 mOhm room-temperature model.
    # Keep these sensitivity results separate from nominal acceptance.
    sensitivity = dict(plan)
    sensitivity['scope'] = ('Out-of-bound sensitivity study, not nominal acceptance: '
        '20 mOhm total ESR implies the proposed physical 56 mOhm series resistor is missing; '
        '150 mOhm exceeds the proposed room-temperature capacitor approximation. '
        'Both use L=80 uH and C=264 uF. Temperature is not qualified by this test.')
    sensitivity['cases'] = []
    base = (benches / 'load_step_derated.cir').read_text(encoding='utf-8')
    for name, esr in [('stress_esr_20m', '0.02'), ('stress_esr_150m', '0.15')]:
        (benches / (name+'.cir')).write_text(
            base.replace('load_step_derated -', name+' -').replace('RESR esr 0 0.1', 'RESR esr 0 '+esr),
            encoding='utf-8')
        sensitivity['cases'].append(dict(name=name, deck='benches/'+name+'.cir',
                                        regulated_window_s=[0.004, 0.012], ripple_window_s=[0.008, 0.009]))
    (HERE / 'sensitivity_plan.json').write_text(json.dumps(sensitivity, indent=2)+'\n', encoding='utf-8')
    print('Prepared', len(cases), 'ngspice cases; vendor model checksums verified.')


if __name__ == '__main__':
    prepare()
