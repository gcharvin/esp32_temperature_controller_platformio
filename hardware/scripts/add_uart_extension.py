"""Incrementally add J10 UART to native A/B, preserving existing copper.

KiCad Python --apply. A: GPIO17 TX / GPIO16 RX. B temperature: pins2/3 NC,
reserved for future B functions. Native J8/J9 remain free for socket upload aliases.
"""
import argparse
import copy
import csv
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import pcbnew as p
import create_drafts as s
from create_placement import pos, xy, mm, text
from route_diagnostics import copper_signature, native_extract

ROOT=Path(__file__).resolve().parents[1]
CLI='C:/Program Files/KiCad/10.0/bin/kicad-cli.exe'
FP='Connector_JST:JST_XH_B3B-XH-A_1x03_P2.50mm_Vertical'
VALUE='A-B UART / 3V3 / GND TX_A RX_A'


def write_tree(path,tree):
    path.write_text('('+tree[0]+'\n'+'\n'.join(s.dump(n) for n in tree[1:])+')\n',encoding='utf-8')


def change_tx_symbol(symbol):
    for sub in s.children(symbol,'symbol'):
        for pin in s.children(sub,'pin'):
            if s.unquote(s.child(pin,'number')[1])=='22':
                assert s.unquote(s.child(pin,'name')[1])=='GPIO17'
                pin[1]='output'


def schematic(name,choice):
    path=ROOT/name/(name+'.kicad_sch');tree=s.parse(path.read_text(encoding='utf-8'))
    refs={s.unquote(v[2]) for n in s.children(tree,'symbol') for v in s.children(n,'property') if s.unquote(v[1])=='Reference'}
    assert 'J10' not in refs,'J10 already exists; do not apply twice'
    is_a=name=='pcb_a_display'
    if is_a:
        lib=next(n for n in s.children(s.child(tree,'lib_symbols'),'symbol') if s.unquote(n[1])=='Lab:Diymore_ESP32_30P')
        change_tx_symbol(lib)
        u=next(n for n in s.children(tree,'symbol') if any(s.unquote(v[1])=='Reference' and s.unquote(v[2])=='U1' for v in s.children(n,'property')))
        ux,uy=map(float,s.child(u,'at')[1:3]);assert s.child(u,'at')[3]=='0'
        for num,net in [('21','UART_A_RX'),('22','UART_A_TX')]:
            pin=next(pin for sub in s.children(lib,'symbol') for pin in s.children(sub,'pin') if s.unquote(s.child(pin,'number')[1])==num)
            at=s.child(pin,'at');sx=round(ux+float(at[1]),4);sy=round(uy-float(at[2]),4)
            nc=[n for n in s.children(tree,'no_connect') if math.dist(list(map(float,s.child(n,'at')[1:3])),[sx,sy])<1e-5]
            assert len(nc)==1;tree.remove(nc[0]);ex=round(sx+5.08,4)
            tree.append(s.parse(f'(wire (pts (xy {sx} {sy}) (xy {ex} {sy})) (stroke (width 0) (type default)) (uuid {s.uid(name+num+"UARTwire")}))'))
            tree.append(s.parse(f'(label {s.q(net)} (at {ex} {sy} 0) (effects (font (size 1.016 1.016)) (justify left bottom)) (uuid {s.uid(name+num+"UARTlabel")}))'))
    base=float(s.child(tree,'paper')[3])+12
    sheet=s.Sheet(name,'')
    sheet.section(20,base,'UART EXTENSION - J10 - 3.3 V LOGIC, NOT RS232 VOLTAGE LEVELS')
    sheet.connector('J10',VALUE,65,base+25,['GND','UART_A_TX','UART_A_RX'] if is_a else ['GND',None,None],3,FP)
    sheet.text(110,base+18,'1 GND / 2 TX_A -> RX_B / 3 RX_A <- TX_B. Straight cable; no supply rail.',1.2)
    sheet.text(110,base+27,('A: GPIO17 TX (U1.22), GPIO16 RX (U1.21); independent of USB UART0 and PWM.' if is_a else
        'B temperature: contacts 2 and 3 intentionally NC; connector fitted for interface compatibility.'),1.2)
    sheet.text(110,base+36,'Reserved extension; future B must implement its UART peripheral/transceiver. No hot-plug.',1.2)
    for item in sheet.items:
        node=s.parse(item)
        if node[0]=='symbol':
            for key,value in [('MPN',choice['MPN']),('Manufacturer',choice['Manufacturer']),('Assembly',choice['Assembly']),
                              ('Design_Status',choice['Decision']),('Datasheet',choice['Source'])]:
                prop=next((v for v in s.children(node,'property') if s.unquote(v[1])==key),None)
                if prop:prop[2]=s.q(value)
                else:node.append(s.parse(f'(property {s.q(key)} {s.q(value)} (at 0 0 0) (effects (font (size 1 1)) hide))'))
        tree.append(node)
    libs=s.child(tree,'lib_symbols');existing={s.unquote(n[1]) for n in s.children(libs,'symbol')}
    libs.extend(v for key,v in sheet.libs.items() if key not in existing)
    s.child(tree,'paper')[3]=str(base+75)
    s.child(s.child(tree,'title_block'),'rev')[1]=s.q('0.4.2' if is_a else '0.5.2')
    s.child(s.child(tree,'title_block'),'date')[1]=s.q('2026-11-10')
    write_tree(path,tree)
    bompath=path.with_name(name+'_bom_draft.csv')
    with bompath.open(encoding='utf-8-sig',newline='') as f:
        reader=csv.DictReader(f);fields=reader.fieldnames;rows=list(reader)
    rows.append({key:{'Reference':'J10','Value':VALUE,'Footprint':FP,'Design_Status':choice['Decision'],**choice}.get(key,'') for key in fields})
    with bompath.open('w',encoding='utf-8',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader();writer.writerows(rows)
    subprocess.run([CLI,'sch','export','netlist','--format','kicadxml','--output',str(ROOT/'review'/(name+'.net.xml')),str(path)],check=True,capture_output=True)


def board(name,choice,position):
    path=ROOT/name/(name+'.kicad_pcb');b=p.LoadBoard(str(path));original=copper_signature(b)
    netlist=ET.parse(ROOT/'review'/(name+'.net.xml'))
    comp=next(c for c in netlist.findall('./components/comp') if c.attrib['ref']=='J10')
    nets={v.GetNetname():v for v in b.GetNetsByNetcode().values()}
    pinmap={}
    for net in netlist.findall('./nets/net'):
        key=net.attrib['name']
        if key not in nets:nets[key]=p.NETINFO_ITEM(b,key);b.Add(nets[key])
        for node in net.findall('node'):pinmap[node.attrib['ref'],node.attrib['pin']]=nets[key]
    fp=p.FootprintLoad('C:/Program Files/KiCad/10.0/share/kicad/footprints/Connector_JST.pretty',FP.split(':')[1])
    assert fp is not None
    fp.SetFPID(p.LIB_ID(*FP.split(':')));fp.SetReference('J10');fp.SetValue(VALUE)
    fp.SetPosition(pos(*position));fp.SetPath(p.KIID_PATH('/'+s.uid(name)+'/'+comp.findtext('tstamps')))
    fp.SetSheetname(name);fp.SetSheetfile(name+'.kicad_sch');b.Add(fp)
    for key,value in [('MPN',choice['MPN']),('Manufacturer',choice['Manufacturer']),('Assembly',choice['Assembly']),
                      ('Design_Status',choice['Decision']),('Datasheet',choice['Source'])]:
        fp.SetField(key,value);fp.GetField(key).SetVisible(False)
    fp.Value().SetVisible(False);fp.Reference().SetTextSize(xy(.8,.8));fp.Reference().SetTextThickness(mm(.12))
    fp.Reference().SetPosition(pos(position[0]+2.5,position[1]+4.5) if name=='pcb_a_display' else pos(position[0]-4.2,position[1]))
    for pad in fp.Pads():pad.SetNet(pinmap['J10',pad.GetNumber()])
    if name=='pcb_a_display':
        u=next(f for f in b.GetFootprints() if f.GetReference()=='U1')
        for pad in u.Pads():
            if pad.GetNumber() in ['21','22']:
                pad.SetNet(pinmap['U1',pad.GetNumber()])
                if hasattr(pad,'SetPinType'):pad.SetPinType('output' if pad.GetNumber()=='22' else 'input')
    if name=='pcb_a_display':
        text(b,'UART 3V3',position[0]+2.5,position[1]-5,.8,p.F_SilkS)
        text(b,'1G 2TXA 3RXA',position[0]+2.5,position[1]+6,.8,p.F_SilkS)
    else:
        text(b,'UART 3V3',position[0]+13.5,5.5,.8,p.F_SilkS)
        text(b,'NC 2/3',position[0]+13.5,7.0,.8,p.F_SilkS)
    b.GetTitleBlock().SetRevision('0.4.2' if name=='pcb_a_display' else '0.5.2')
    after=copper_signature(b)
    assert original==after,'Existing copper changed'
    p.ZONE_FILLER(b).Fill(b.Zones());p.SaveBoard(str(path),b)
    return dict(before_count=len(original),before_signature_sha256=hashlib.sha256(json.dumps(original,sort_keys=True).encode()).hexdigest(),UUIDs=list(original),new_J10_origin_mm=position)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--apply',action='store_true');args=parser.parse_args()
    assert args.apply,'Use --apply for this authorized one-shot native revision'
    selection=ROOT/'manufacturing/component_selection.json';choices=json.loads(selection.read_text(encoding='utf-8'))
    assert all('J10' not in choices['boards'][c] for c in 'AB'),'UART revision already applied'
    backup=Path(tempfile.mkdtemp(prefix='pcb-before-uart-'))
    for name in ['pcb_a_display','pcb_b_temperature']:
        for suffix in ['.kicad_sch','.kicad_pcb','.kicad_pro','_bom_draft.csv']:
            shutil.copy2(ROOT/name/(name+suffix),backup/(name+suffix))
    for path in [selection,ROOT/'module_selection.json',ROOT/'placement.json',ROOT/'interface_ab.json',ROOT/'lab_draft.kicad_sym']:
        shutil.copy2(path,backup/path.name)
    print('Native pre-UART backup:',backup,flush=True)
    libpath=ROOT/'lab_draft.kicad_sym';lib=s.parse(libpath.read_text(encoding='utf-8'))
    change_tx_symbol(next(n for n in s.children(lib,'symbol') if s.unquote(n[1])=='Diymore_ESP32_30P'))
    write_tree(libpath,lib)
    preserved={}
    for code,name,position in [('A','pcb_a_display',[73.5,31]),('B','pcb_b_temperature',[42,7.5])]:
        choice=copy.deepcopy(choices['boards'][code]['J6'])
        choice.update(Value=VALUE,Requirements=choice['Requirements']+'; dedicated 3.3V UART, 1 GND / 2 TX_A / 3 RX_A; pins2/3 NC on temperature B')
        choices['boards'][code]['J10']=choice
        schematic(name,choice);preserved[name]=board(name,choice,position)
    choices.update(revision='A0.4.2 / B0.5.2 dedicated UART extension 2026-11-10',
        scope='70 physical native PCB components; excludes virtual ESP32, copper net tie and22 copper test pads; sockets and shunt separate')
    selection.write_text(json.dumps(choices,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    modulepath=ROOT/'module_selection.json';modules=json.loads(modulepath.read_text(encoding='utf-8'))
    modules['devkit']['used_pin_nets'].update({'21':'UART_A_RX','22':'UART_A_TX'})
    modules['revision']='A0.4.2 / B0.5.2 UART, original power topology'
    modulepath.write_text(json.dumps(modules,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    interfacepath=ROOT/'interface_ab.json';interface=json.loads(interfacepath.read_text(encoding='utf-8'))
    interface['revision']='0.4 / dedicated UART J10 / A0.4.2 B0.5.2'
    interface['harness']='three required cables plus optional reserved UART cable; straight-through; connections unpowered, no hot-plug'
    interface['connectors'].append(dict(reference='J10',function='Dedicated 3.3V UART; optional on temperature prototype',required_for_temperature=False,
        pins=[dict(pin=1,net='GND',direction='common'),dict(pin=2,net='UART_A_TX',direction='A->future B RX',gpio_on_a=17,connected_on_b=False),
              dict(pin=3,net='UART_A_RX',direction='future B TX->A',gpio_on_a=16,connected_on_b=False)]))
    interfacepath.write_text(json.dumps(interface,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    (ROOT/'review/uart_extension_copper_baseline.json').write_text(json.dumps(dict(backup=str(backup),boards=preserved),indent=2)+'\n',encoding='utf-8')
    from add_test_points import refresh_placement
    refresh_placement()
    layoutpath=ROOT/'placement.json';layout=json.loads(layoutpath.read_text(encoding='utf-8'))
    layout['revision']='A0.4.2 / B0.5.2 dedicated UART J10; existing power/control interfaces preserved'
    for spec in layout['boards']:
        row=next(r for r in spec['components'] if r['reference']=='J10')
        row.update(height_mm=9.8,status='JST-XH3 dedicated UART 3.3V; connector fitted, no UART endpoint on temperature B')
    layoutpath.write_text(json.dumps(layout,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    print('J10 added on both native boards. Route new branches before DRC/parity and fabrication exports.',flush=True)


if __name__=='__main__':main()
