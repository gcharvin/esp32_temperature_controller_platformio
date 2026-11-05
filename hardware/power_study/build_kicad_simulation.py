"""Generate the separate, simulation-only KiCad front end for the buck study.

Uses native Sim.* fields and the installed KiCad CLI's actual SPICE exporter.
The routed A/B hardware and the manufacturing study are deliberately untouched.
"""
import argparse
import copy
import json
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'scripts'))
import create_drafts as s

CLI = Path('C:/Program Files/KiCad/10.0/bin/kicad-cli.exe')
DEFAULT_MODEL = HERE / 'vendor_model/LMR36520A_PSPICE_TRANS/LMR36520A_PSPICE_TRANS/LMR36520A_TRANS.LIB'


def set_fields(sheet, ref, fields):
    for i, raw in enumerate(sheet.items):
        node = s.parse(raw)
        if node[0] != 'symbol':
            continue
        props = {s.unquote(p[1]): p for p in s.children(node, 'property')}
        if s.unquote(props['Reference'][2]) != ref:
            continue
        for key, value in fields.items():
            if key in props:
                props[key][2] = s.q(value)
            else:
                node.append(s.parse(f'(property {s.q(key)} {s.q(value)} (at 0 0 0) (effects (font (size 1.27 1.27)) (hide yes)))'))
        sheet.items[i] = s.dump(node)
        return
    raise ValueError(ref)


def generate_lmr36520(model, options):
    dest = HERE / 'kicad_simulation'
    dest.mkdir(exist_ok=True)
    model = model.resolve()
    if not model.is_file():
        raise FileNotFoundError(model)
    import os
    relative_model = Path(os.path.relpath(model, dest)).as_posix()
    name = 'buck_5v_simulation'
    sheet = s.Sheet(name, 'KiCad/ngspice simulation - LMR36520A buck - not a PCB design')
    source = s.parse((HERE / 'buck_5v/buck_5v.kicad_sch').read_text(encoding='utf-8'))
    lib = copy.deepcopy(next(x for x in s.children(s.child(source, 'lib_symbols'), 'symbol')
                             if s.unquote(x[1]) == 'Lab:LMR36520ADDAR'))
    s.CUSTOM['LMR36520ADDAR'] = lib
    sheet.section(20, 18, 'KICAD / NGSPICE - CONVERTER-ONLY SIMULATION BENCH')
    sheet.text(20, 27, 'Editable source values and explicit model-to-symbol pin mapping. Open the matching .wbk in Simulator.\nNot the routed PCB B. No upstream protection, USB sharing, thermal or enclosure validation.', 1.1)
    # These three sources implement the startup case from benches/startup.cir.
    sheet.add('Simulation_SPICE:VPWL', 'V1', 'VIN: 0 -> 24V / 100us', 35, 58, {'1': 'vin', '2': '0'}, '')
    sheet.add('Simulation_SPICE:VPWL', 'V2', 'EN: on at 210us', 35, 108, {'1': 'en', '2': '0'}, '')
    sheet.add('Simulation_SPICE:VDC', 'V3', '0.5A command', 35, 158, {'1': 'loadcmd', '2': '0'}, '')
    sheet.add('Simulation_SPICE:0', '#GND01', '0', 35, 196, {'1': '0'}, '')
    set_fields(sheet, 'V1', {'Sim.Device': 'V', 'Sim.Type': 'PWL', 'Sim.Params': 'pwl="0 0 100u 24"', 'Sim.Pins': '1=+ 2=-'})
    set_fields(sheet, 'V2', {'Sim.Device': 'V', 'Sim.Type': 'PWL', 'Sim.Params': 'pwl="0 0 200u 0 210u 3.3"', 'Sim.Pins': '1=+ 2=-'})
    set_fields(sheet, 'V3', {'Sim.Device': 'V', 'Sim.Type': 'DC', 'Sim.Params': 'dc=0.5', 'Sim.Pins': '1=+ 2=-'})
    sheet.add('Lab:LMR36520ADDAR', 'U1', 'LMR36520A_TRANS', 110, 108,
              {'1': '0', '2': 'vin', '3': 'en', '4': 'pg', '5': 'fb', '6': 'vcc', '7': 'boot', '8': 'sw', '9': '0'}, '')
    # Raw SPICE device avoids KiCad's narrower model-editor parser rejecting
    # vendor expressions before ngspice can apply PSpice compatibility. The
    # explicit include is handled by ngspice, not reinterpreted by this script.
    set_fields(sheet, 'U1', {'Sim.Device': 'SPICE',
                            'Sim.Pins': '1=1 2=9 3=3 4=5 5=4 6=8 7=2 8=7 9=6',
                            'Sim.Params': 'type="X" model="LMR36520A_TRANS STEADY_STATE=0 FASTSS=0"'})
    def passive(libid, ref, value, x, y, a, b):
        sheet.add(libid, ref, value, x, y, {'1': a, '2': b}, '')
        device = libid.split(':')[1]
        set_fields(sheet, ref, {'Sim.Device': device, 'Sim.Type': '=', 'Sim.Params': f'{device.lower()}={value}', 'Sim.Pins': '1=+ 2=-'})
    passive('Device:R', 'R1', '100k', 85, 58, 'vin', 'en')
    for ref, value, x in [('C1', '4.7u', 135), ('C2', '4.7u', 170), ('C3', '220n', 205)]:
        passive('Device:C', ref, value, x, 58, 'vin', '0')
    passive('Device:C', 'C4', '100n', 180, 108, 'boot', 'sw')
    passive('Device:C', 'C5', '1u', 180, 158, 'vcc', '0')
    passive('Device:L', 'L1', '10u', 245, 58, 'sw', 'lind')
    passive('Device:R', 'R5', '29.8m', 285, 58, 'lind', 'vout')
    passive('Device:R', 'R2', '100k', 245, 108, 'vout', 'fb')
    passive('Device:R', 'R3', '24.9k', 285, 108, 'fb', '0')
    passive('Device:R', 'R4', '100k', 325, 108, 'vout', 'pg')
    for i, x in enumerate([230, 270, 310, 350], 1):
        passive('Device:C', f'C{i + 5}', '22u', x, 158, 'vout', f'co{i}')
        passive('Device:R', f'R{i + 5}', '5m', x, 204, f'co{i}', '0')
    # Generic B-source retains the exact conditioned load from the source deck.
    sheet.add('Simulation_SPICE:BSOURCE', 'B1', 'Conditioned current load', 380, 78, {'1': 'vout', '2': '0'}, '')
    set_fields(sheet, 'B1', {'Sim.Device': 'SPICE', 'Sim.Params': 'type="B" model="i={V(/loadcmd)*LIMIT(V(/vout)/4.75,0,1)}"', 'Sim.Pins': '1=1 2=2'})
    sheet.text(20, 235, 'U1 mapping: model AGND BOOT EN FB PG PGND SW VCC VIN = physical 1 7 3 5 4 9 8 6 2.\nGround pins 1 and 9 are joined to SPICE node 0. PG is a 5V test point, not ESP32-safe.\nR(DCR)=29.8mOhm; each output capacitor has 5mOhm ESR. Load command is reduced below 4.75V.', 1.1)
    sheet.text(20, 257, 'Startup: VIN ramps 0 -> 24V in 100us; EN reaches 3.3V at 210us; conditioned load 0.5A.\nChange source simulation properties to run a different stimulus, or use the scripted benches for the full suite.', 1.1)
    sheet.text(20, 280, '.include "' + relative_model + '"', 0.9)
    sheet.text(20, 287, '.options ' + options, 1.0)
    tree = f'(kicad_sch (version 20250114) (generator "eeschema") (uuid {sheet.uuid}) (paper "A3") (title_block (title {s.q(sheet.title)}) (date "2026-11-05") (rev "0.2 SIMULATION"))\n'
    tree += '(lib_symbols\n' + '\n'.join(s.dump(x) for x in sheet.libs.values()) + ')\n'
    tree += '\n'.join(sheet.items) + '\n(sheet_instances (path "/" (page "1"))) (embedded_fonts no))\n'
    sch = dest / (name + '.kicad_sch')
    sch.write_text(tree, encoding='utf-8')
    local = copy.deepcopy(lib)
    local[1] = s.q('LMR36520ADDAR')
    (dest / 'simulation.kicad_sym').write_text('(kicad_symbol_lib (version 20241209) (generator "kicad_symbol_editor") ' + s.dump(local) + ')\n', encoding='utf-8')
    (dest / 'sym-lib-table').write_text('(sym_lib_table (version 7) (lib (name "Lab") (type "KiCad") (uri "${KIPRJMOD}/simulation.kicad_sym") (options "") (descr "Simulation bench only")))\n', encoding='utf-8')
    (dest / (name + '.kicad_pro')).write_text(json.dumps({'meta': {'filename': name + '.kicad_pro', 'version': 1}}, indent=2) + '\n', encoding='utf-8')
    workbook = {'version': 6, 'last_sch_text_sim_command': '', 'tabs': [{
        'analysis': 'TRAN', 'commands': ['.tran 100n 12m 0 100n', '.kicad adjustpaths', '.save all'],
        'dottedSecondary': False, 'margins': {'bottom': 45, 'left': 70, 'right': 140, 'top': 30},
        'measurements': [], 'showGrid': True, 'traces': [
            {'color': 'rgb(228, 26, 28)', 'signal': 'V(/vout)', 'trace_type': 257},
            {'color': 'rgb(55, 126, 184)', 'signal': 'V(/vin)', 'trace_type': 257},
            {'color': 'rgb(77, 175, 74)', 'signal': 'I(L1)', 'trace_type': 258}]}], 'user_defined_signals': []}
    (dest / (name + '.wbk')).write_text(json.dumps(workbook, indent=2) + '\n', encoding='utf-8')
    subprocess.run([str(CLI), 'sch', 'export', 'netlist', '--format', 'spice', '--output', str(dest / (name + '.cir')), str(sch)], check=True)
    subprocess.run([str(CLI), 'sch', 'export', 'pdf', '--output', str(dest / (name + '.pdf')), str(sch)], check=True)
    return dest


def generate_lm2596(model, options):
    """Make one actual, editable KiCad project per stimulus, with native models."""
    import os
    # Read the actual scripted benches, avoiding two independently edited sets
    # of component values and load-step times.
    cases = {}
    for bench in sorted((HERE / 'lm2596/benches').glob('*.cir')):
        lines = {line.split()[0].upper(): line.split() for line in bench.read_text().splitlines()
                 if line.strip() and not line.startswith(('*', '+'))}
        def stimulus(ref):
            value = ' '.join(lines[ref][3:])
            return value[4:-1] if value.upper().startswith('PWL(') else '0 ' + value
        cases[bench.stem] = dict(vin=stimulus('VIN'), off=stimulus('VDIS'), load=stimulus('VLOADCMD'),
            cout=lines['COUT'][3], inductance=lines['L1'][3], dcr=lines['RDCR'][3], esr=lines['RESR'][3],
            cin=lines['CIN'][3], cin_esr=lines['RIN_ESR'][3], tran=' '.join(lines['.TRAN']),
            options=' '.join(lines['.OPTIONS'][1:]))
    if not cases:
        raise RuntimeError('Run lm2596/prepare.py before generating the GUI projects')
    model = model.resolve()
    if not model.is_file():
        raise FileNotFoundError(model)
    outputs = []
    for case, cfg in cases.items():
        dest = HERE / 'kicad_simulation' / case
        dest.mkdir(parents=True, exist_ok=True)
        name = 'lm2596_5v_' + case
        sheet = s.Sheet(name, f'LM2596S-5 KiCad/ngspice simulation - {case}')
        relative_model = Path(os.path.relpath(model, dest)).as_posix()
        sheet.section(20, 18, 'KICAD / NGSPICE - LM2596 FIXED 5V - ' + case.upper())
        sheet.text(20, 29, 'Converter-only simulation study. Not installed or routed on PCB B.\nTI regulator and Vishay SS36 models are unmodified. Final component and thermal qualification remain required.', 1.1)
        # The symbol's numbered pins match the LM2596S-5 physical package.
        sheet.add('Regulator_Switching:LM2596S-5', 'U1', 'LM2596S-5', 126, 102,
                  {'1': 'vin', '2': 'sw', '3': '0', '4': 'vout', '5': 'off'}, '')
        set_fields(sheet, 'U1', {'Sim.Device': 'SPICE', 'Sim.Pins': '1=1 2=3 3=4 4=2 5=5',
                                'Sim.Params': 'type="X" model="LM2596_5P0_TRANS"'})
        def passive(libid, ref, value, x, y, a, b):
            sheet.add(libid, ref, value, x, y, {'1': a, '2': b}, '')
            device = libid.split(':')[1]
            set_fields(sheet, ref, {'Sim.Device': device, 'Sim.Type': '=', 'Sim.Params': f'{device.lower()}={value}', 'Sim.Pins': '1=+ 2=-'})
        for ref, net, xy, pwl in [
                ('V1', 'vin', (35, 70), cfg['vin']),
                ('V2', 'off', (35, 130), cfg['off']),
                ('V3', 'loadcmd', (35, 190), cfg['load'])]:
            sheet.add('Simulation_SPICE:VPWL', ref, {'V1': '24V supply stimulus', 'V2': 'OFF high / RUN low', 'V3': 'Load command / A per V'}[ref], *xy, {'1': net, '2': '0'}, '')
            set_fields(sheet, ref, {'Sim.Device': 'V', 'Sim.Type': 'PWL', 'Sim.Params': f'pwl="{pwl}"', 'Sim.Pins': '1=+ 2=-'})
        sheet.add('Simulation_SPICE:0', '#GND01', '0', 75, 224, {'1': '0'}, '')
        sheet.flag('#FLG01', 85, 224, '0')
        sheet.flag('#FLG02', 115, 224, 'vin')
        passive('Device:C', 'C1', cfg['cin'], 85, 68, 'vin', 'cinret')
        passive('Device:R', 'R4', cfg['cin_esr'], 85, 124, 'cinret', '0')
        passive('Device:L', 'L1', cfg['inductance'], 210, 68, 'sw', 'lind')
        passive('Device:R', 'R1', cfg['dcr'], 255, 68, 'lind', 'vout')
        passive('Device:C', 'C2', cfg['cout'], 255, 133, 'vout', 'esr')
        passive('Device:R', 'R2', cfg['esr'], 255, 184, 'esr', '0')
        # A finite monitor load gives the otherwise behavioral-only command
        # node an explicit connection for ERC; V3 is an ideal voltage source.
        passive('Device:R', 'R3', '1Meg', 85, 190, 'loadcmd', '0')
        sheet.add('Device:D_Schottky', 'D1', 'Vishay SS36', 190, 145, {'1': 'sw', '2': '0'}, '')
        set_fields(sheet, 'D1', {'Sim.Device': 'SPICE', 'Sim.Pins': '1=2 2=1', 'Sim.Params': 'type="X" model="ss36"'})
        sheet.add('Simulation_SPICE:BSOURCE', 'B1', 'Conditioned load', 340, 105, {'1': 'vout', '2': '0'}, '')
        set_fields(sheet, 'B1', {'Sim.Device': 'SPICE', 'Sim.Params': 'type="B" model="i={V(/loadcmd)*LIMIT(V(/vout)/4.75,0,1)}"', 'Sim.Pins': '1=1 2=2'})
        sheet.text(20, 244, 'U1 node order VIN FB OUT GND ON_OFF_N = physical pins 1 4 2 3 5. FB directly senses VOUT.\nR1 = inductor DCR; R4 = input ESR; R2 = lumped output series resistance. Intended hardware adds 56mOhm.\nStress ESR cases may violate that hardware bound. V3 commands a load, reduced below 4.75V.', 1.0)
        sheet.text(20, 258, '.include "' + relative_model + '"', 0.9)
        sheet.text(20, 265, '.include "' + Path(os.path.relpath(model.parent / 'ss36.lib', dest)).as_posix() + '"', 0.9)
        sheet.text(20, 272, '.options ' + (options or cfg['options']), 0.9)
        sheet.text(20, 279, cfg['tran'], 0.9)
        tree = f'(kicad_sch (version 20250114) (generator "eeschema") (uuid {sheet.uuid}) (paper "A3") (title_block (title {s.q(sheet.title)}) (date "2026-11-05") (rev "0.2 SIMULATION"))\n'
        tree += '(lib_symbols\n' + '\n'.join(s.dump(x) for x in sheet.libs.values()) + ')\n'
        tree += '\n'.join(sheet.items) + '\n(sheet_instances (path "/" (page "1"))) (embedded_fonts no))\n'
        sch = dest / (name + '.kicad_sch')
        sch.write_text(tree, encoding='utf-8')
        (dest / (name + '.kicad_pro')).write_text(json.dumps({'meta': {'filename': name + '.kicad_pro', 'version': 1},
            'schematic': {'ngspice': {'fix_include_paths': True, 'model_mode': 2,
                                    'meta': {'version': 0}, 'workbook_filename': name + '.wbk'}}}, indent=2) + '\n', encoding='utf-8')
        workbook = {'version': 6, 'last_sch_text_sim_command': cfg['tran'], 'tabs': [{
            'analysis': 'TRAN', 'commands': [cfg['tran'], '.kicad adjustpaths', '.save all'],
            'dottedSecondary': False, 'margins': {'bottom': 45, 'left': 70, 'right': 140, 'top': 30},
            'measurements': [], 'showGrid': True, 'traces': [
                {'color': 'rgb(228, 26, 28)', 'signal': 'V(/vout)', 'trace_type': 257},
                {'color': 'rgb(55, 126, 184)', 'signal': 'V(/vin)', 'trace_type': 257}]}], 'user_defined_signals': []}
        (dest / (name + '.wbk')).write_text(json.dumps(workbook, indent=2) + '\n', encoding='utf-8')
        subprocess.run([str(CLI), 'sch', 'export', 'netlist', '--format', 'spice', '--output', str(dest / (name + '.cir')), str(sch)], check=True)
        subprocess.run([str(CLI), 'sch', 'export', 'pdf', '--output', str(dest / (name + '.pdf')), str(sch)], check=True)
        exported = (dest / (name + '.cir')).read_text()
        assert 'XU1 /vin /vout /sw 0 /off LM2596_5P0_TRANS' in exported
        assert 'XD1 0 /sw ss36' in exported
        subprocess.run([str(CLI), 'sch', 'erc', str(sch), '--output', str(dest / (name + '_erc.rpt')), '--exit-code-violations'], check=True)
        outputs.append(dest)
    return outputs


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--part', choices=['lm2596', 'lmr36520'], default='lm2596')
    parser.add_argument('--model', type=Path)
    parser.add_argument('--options', help='Override canonical bench solver options')
    args = parser.parse_args()
    if args.part == 'lm2596':
        print(generate_lm2596(args.model or HERE / 'vendor_model/LM2596_5P0_TRANS.LIB', args.options))
    else:
        print(generate_lmr36520(args.model or DEFAULT_MODEL, args.options or 'reltol=0.003 abstol=1n vntol=1u'))
