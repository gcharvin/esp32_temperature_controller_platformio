"""One-shot A0.4.3/B0.5.3 sourcing revision, preserving all copper/pads.

KiCad Python --apply. Uses reviewed November 10 quote alternatives. No network,
supplier submission, order, firmware flash or purchase is performed.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import shutil
import tempfile

import pcbnew as p
import create_drafts as s
from route_diagnostics import copper_signature

ROOT = Path(__file__).resolve().parents[1]
QUOTE = ROOT/'manufacturing/online_quote_20261110'
SOCKET = 'LabHardware:ESP32_30P_HCTL_PM254_Rows25.40mm'
C10_FP = 'LabHardware:CP_Radial_EEUFS1V331LB_P5_D10_Keepout'
SOCKET_SOURCE = 'https://www.lcsc.com/datasheet/C2897378.pdf'


def read(path): return json.loads(path.read_text(encoding='utf-8'))
def write(path, data): path.write_text(json.dumps(data, indent=2, ensure_ascii=False)+'\n', encoding='utf-8')
def sha(path): return hashlib.sha256(path.read_bytes()).hexdigest()


def pin_geometry(board):
    return {(f.GetReference(), pad.GetNumber(), pad.m_Uuid.AsString()):
            (tuple(pad.GetPosition()), tuple(pad.GetSize()), tuple(pad.GetDrillSize()),
             pad.GetShape(), pad.GetNetname(), pad.GetOrientationDegrees())
            for f in board.GetFootprints() for pad in f.Pads()}


def main():
    choices_path = ROOT/'manufacturing/component_selection.json'
    choices = read(choices_path)
    assert 'A0.4.2' in choices['revision'], 'One-shot revision already applied or unexpected source'
    backup = Path(tempfile.mkdtemp(prefix='hardware-before-stock-revision-'))
    for name in ('pcb_a_display', 'pcb_b_temperature'):
        for suffix in ('.kicad_sch', '.kicad_pcb', '_bom_draft.csv'):
            src = ROOT/name/(name+suffix); shutil.copy2(src, backup/src.name)
    shutil.copy2(choices_path, backup/choices_path.name)
    print('Backup:', backup, flush=True)
    new_parts = {
        'C1608X7R1C105K080AC': ('0603B105K160CT', 'Walsin', 'https://www.ic-components.se/files/d4/0603B105K160CT.pdf'),
        'C1608X5R1A106K080AC': ('CL10A106KP8NNNC', 'Samsung', 'https://static6.arrow.com/aropdfconversion/f24e86c802cdb5d71185c025570539bb11b8d108/specsheetcl10a106kp8nnn20250808.pdf'),
        'EEUFR1V331UB': ('EEUFS1V331LB', 'Panasonic', 'https://industrial.panasonic.com/ww/products/pt/aluminum-cap-lead/models/EEUFS1V331LB'),
        'WSL1206R0560FEA': ('WSL1206R0500FEA', 'Vishay', 'https://www.vishay.com/docs/30100/wsl.pdf'),
        'APT2012LSECK/J4-PRV': ('APT2012LSYCK/J3-PRV', 'Kingbright', 'https://www.kingbrightusa.com/images/catalog/spec/apt2012lsyck-j3-prv.pdf'),
    }
    for table in choices['boards'].values():
        for choice in table.values():
            if choice['MPN'] in new_parts:
                choice['MPN'], choice['Manufacturer'], choice['Source'] = new_parts[choice['MPN']]
                choice['Decision'] = 'ENGINEERING_SELECTED_STOCK_REVISION_2026_11_10'
    choices['boards']['A']['U1']['Footprint'] = SOCKET
    choices['extra_assembly_items']['A_U1_SOCKETS'].update(
        MPN='PM254-1-15-Z-8.5', Manufacturer='HCTL', Source=SOCKET_SOURCE)
    choices['boards']['B']['C10'].update(Footprint=C10_FP,
        Requirements='330uF 35V +/-20%; EEUFS1V331LB formed 5mm pitch, D8 nominal/D8.5 max, H15 nominal/H16.5 max; 41mOhm max impedance at100kHz/20C. D10 native keepout/lands retained conservatively; pad1 positive.')
    choices['boards']['B']['R13'].update(Value='50m / 1% / 0.25W',
        Requirements='50mOhm +/-1%, 0.25W, Vishay WSL1206 6..200mOhm lands; in C10 return only. Qualified seven-case study includes49mOhm lower bound and180mOhm upper stress.')
    choices['boards']['B']['D8'].update(Value='APT2012LSYCK/J3-PRV',
        Requirements='yellow low-current LED 0805; 1.85V typical at2mA, MSL3, cathode pad1. R12 remains2.2kOhm; replaces orange with yellow HEAT indicator.')
    choices.update(revision='A0.4.3 / B0.5.3 reviewed stock alternatives 2026-11-10',
                   qualification='manufacturing/online_quote_20261110/README.md; native integration authorized, factory approval and order remain pending')
    write(choices_path, choices)
    from build_component_models import main as build_component_models
    build_component_models()
    layout_path=ROOT/'placement.json';layout=read(layout_path);preservation={}
    for code, name in [('A','pcb_a_display'),('B','pcb_b_temperature')]:
        table=choices['boards'][code];folder=ROOT/name
        path=folder/(name+'.kicad_sch');tree=s.parse(path.read_text(encoding='utf-8'))
        for node in s.children(tree,'symbol'):
            props={s.unquote(v[1]):v for v in s.children(node,'property')}
            ref=s.unquote(props['Reference'][2]);choice=table.get(ref)
            if choice is None:continue
            for key, val in [('Value',choice['Value']),('Footprint',choice['Footprint']),('MPN',choice['MPN']),
                             ('Manufacturer',choice['Manufacturer']),('Datasheet',choice['Source']),
                             ('Assembly',choice['Assembly']),('Design_Status',choice['Decision'])]:
                if key in props:props[key][2]=s.q(val)
        title=s.child(tree,'title_block');s.child(title,'rev')[1]=s.q('0.4.3' if code=='A' else '0.5.3')
        for node in s.children(tree,'text'):
            node[1]=s.q(s.unquote(node[1]).replace('Harwin M20-7821546','HCTL PM254-1-15-Z-8.5')
                        .replace('56m','50m').replace('56 m','50 m').replace('EEUFR1V331UB','EEUFS1V331LB'))
        path.write_text('('+tree[0]+'\n'+'\n'.join(s.dump(n) for n in tree[1:])+')\n',encoding='utf-8')
        pcb=folder/(name+'.kicad_pcb');board=p.LoadBoard(str(pcb));before=copper_signature(board);pads=pin_geometry(board)
        for fp in board.GetFootprints():
            ref=fp.GetReference();choice=table[ref]
            fp.SetValue(choice['Value']);fp.SetFPID(p.LIB_ID(*choice['Footprint'].split(':')))
            for key,val in [('MPN',choice['MPN']),('Manufacturer',choice['Manufacturer']),('Datasheet',choice['Source']),
                            ('Assembly',choice['Assembly']),('Design_Status',choice['Decision'])]:
                fp.SetField(key,val);fp.GetField(key).SetVisible(False)
            if code=='A' and ref=='U1':
                fp.SetLibDescription('Two HCTL PM254-1-15-Z-8.5 sockets, pitch2.54mm, rows25.40mm, holes1.05mm; existing conservative courtyard retained.')
                p.PCB_IO_MGR.FindPlugin(p.PCB_IO_MGR.KICAD_SEXP).FootprintSave(str(ROOT/'lab_hardware.pretty'),fp)
            if code=='B' and ref=='C10':
                fp.SetLibDescription('Panasonic EEUFS1V331LB: D8 nominal, H16.5 max, formed P5. Original D10 keepout, pads and holes retained; custom maximum-height VRML envelope.')
                fp.Models().clear();model=p.FP_3DMODEL()
                model.m_Filename='${KIPRJMOD}/../lab_hardware.3dshapes/EEUFS1V331LB_envelope.wrl'
                fp.Add3DModel(model)
                p.PCB_IO_MGR.FindPlugin(p.PCB_IO_MGR.KICAD_SEXP).FootprintSave(str(ROOT/'lab_hardware.pretty'),fp)
        for drawing in board.GetDrawings():
            if isinstance(drawing,p.PCB_TEXT):drawing.SetText(drawing.GetText().replace('HARWIN SOCKETS','HCTL SOCKETS'))
        board.GetTitleBlock().SetRevision('0.4.3' if code=='A' else '0.5.3')
        assert before==copper_signature(board) and pads==pin_geometry(board), 'Copper/pad/net geometry changed'
        p.SaveBoard(str(pcb),board)
        reopened=p.LoadBoard(str(pcb))
        assert before==copper_signature(reopened) and pads==pin_geometry(reopened)
        preservation[code]=dict(tracks_vias_preserved=len(before),pads_preserved=len(pads),
            copper_sha256=hashlib.sha256(json.dumps(before,sort_keys=True).encode()).hexdigest())
        with (folder/(name+'_bom_draft.csv')).open('w',encoding='utf-8',newline='') as f:
            fields=['Reference','Value','Footprint','Design_Status','MPN','Manufacturer'];w=csv.DictWriter(f,fieldnames=fields);w.writeheader()
            for ref,c in table.items():w.writerow(dict(Reference=ref,Value=c['Value'],Footprint=c['Footprint'],Design_Status=c['Decision'],MPN=c['MPN'],Manufacturer=c['Manufacturer']))
        spec=next(b for b in layout['boards'] if b['name']==name)
        for row in spec['components']:
            c=table[row['reference']];row.update(value=c['Value'],footprint=c['Footprint'])
            if code=='B' and row['reference']=='C10':row['height_mm']=16.5
    layout['revision']='A0.4.3 / B0.5.3 stock alternatives; copper/pads and conservative keepouts unchanged'
    write(layout_path,layout)
    modulepath=ROOT/'module_selection.json';modules=read(modulepath)
    modules['revision']='A0.4.3 / B0.5.3 stock alternatives; power/UART topology preserved'
    dev=modules['devkit'];dev['footprint']=SOCKET
    dev['spacing_reason']='25.40mm row spacing measured; two HCTL PM254 sockets, existing1.05mm holes and conservative courtyard retained.'
    dev['socket'].update(MPN='PM254-1-15-Z-8.5',Manufacturer='HCTL',supplier_id='C2897378',
        maximum_body_mm=[38.8,2.65,8.65],source=SOCKET_SOURCE,drawing_revision='HCTL PM254-1-N-Z-8.5-XX(L11.7), revision A',
        manufacturer_recommended_hole_mm=[1.02,1.02],
        limits='Manufacturer nominal recommended hole1.02mm; native1.05mm retained for0.64x0.40mm leads. Finished-hole tolerance/assembly acceptance and physical mating require factory/hardware review.')
    modules['buck'].update(output_C10='EEUFS1V331LB / 330uF 35V / formedP5 / Hmax16.5mm',
        output_damping_R13='50mOhm / 1% / 0.25W / Vishay WSL1206R0500FEA')
    write(modulepath,modules)
    landpath=ROOT/'manufacturing/land_pattern_review.json';lands=read(landpath);lands['checked_date']='2026-11-10'
    lands['R13']['MPN_candidate']='WSL1206R0500FEA'
    lands['C10'].update(MPN_candidate='EEUFS1V331LB',source=choices['boards']['B']['C10']['Source'],footprint=C10_FP,
        body_diameter_mm=8,body_diameter_max_mm=8.5,body_height_mm=15,body_height_max_mm=16.5,
        mechanical_envelope_height_mm=16.5,
        limits='LB formed5mm lead pitch required. Original D10 native keepout/lands retained conservatively; body max8.5mm fits within it. Custom VRML represents maximum body height; factory polarity/process approval pending.')
    lands['A_U1_SOCKETS'].update(MPN='PM254-1-15-Z-8.5',Manufacturer='HCTL',source=SOCKET_SOURCE,
        footprint=SOCKET,body_max_mm=[38.8,2.65,8.65],recommended_hole_mm=[1.02,1.02],
        drawing_revision=dev['socket']['drawing_revision'],notes=dev['socket']['limits'])
    led=lands['LED_APT2012'];led['MPNs'][1]='APT2012LSYCK/J3-PRV';led['sources'][1]=choices['boards']['B']['D8']['Source']
    led['drawing']='page1 recommended soldering pattern; greenV8B2025-03-17, yellowV5B2025-03-14'
    write(landpath,lands)
    param=ROOT/'mechanical/enclosure_parameters.json';params=read(param)
    params.update(revision='0.12 - stock alternatives: HCTL sockets and C10 height16.5mm; six printed pieces retained',export_tag='v12')
    write(param,params)
    simulation=read(QUOTE/'simulation/results/simulation_results.json')
    assert simulation['all_criteria_pass'] and len(simulation['cases'])==7
    for case in simulation['cases']:
        assert sha(QUOTE/'simulation'/case['deck'])==case['deck_sha256']
    binding=ROOT/'power_study/lm2596/current_acceptance.json'
    files=[QUOTE/'simulation/results/simulation_results.json',QUOTE/'simulation/validation_plan.json',QUOTE/'qualify_buck_alternatives.py']
    files += [QUOTE/'simulation'/c['deck'] for c in simulation['cases']]
    write(binding,dict(revision='B0.5.3',C10='EEUFS1V331LB',R13='WSL1206R0500FEA',R13_ohm=.05,
        plan='manufacturing/online_quote_20261110/simulation/validation_plan.json',
        results='manufacturing/online_quote_20261110/simulation/results/simulation_results.json',
        source_sha256={f.relative_to(ROOT).as_posix():sha(f) for f in files},
        note='November10 quote qualification promoted to native revision; original six-case FR/56mOhm study remains historical. No physical qualification inferred.'))
    write(ROOT/'review/stock_revision_copper_validation.json',dict(status='PASS',revision=choices['revision'],boards=preservation,
        electrical_connectivity_or_routing_changed=False,order_placed=False))
    print('Native stock revision applied; all tracks, vias, pad UUIDs, dimensions and nets preserved.',flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--apply',action='store_true');args=parser.parse_args()
    assert args.apply,'Explicit --apply required'
    main()
