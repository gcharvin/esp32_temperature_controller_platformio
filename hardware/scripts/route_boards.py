"""DSN settings and SES import for local Freerouting 2.5.0.

Use --prepare before the native CLI, --import after routing. No cloud API used.
Power nets: 2.4 mm; PGND heater return is explicitly fixed in the board source.
"""
import argparse
from pathlib import Path
import re
import xml.etree.ElementTree as ET
import pcbnew as p
from create_drafts import q

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'review/routing'
POWER={'/VIN_RAW','/VIN_FUSED','/VIN_PROTECTED','/HEATER_LOW'}

def prepare(name):
    source=ROOT/name/(name+'.kicad_pcb');board=p.LoadBoard(str(source))
    for item in board.GetDrawings():
        if isinstance(item,p.PCB_TEXT) and item.GetText() in ('A-B 5V','A-B I2C','A-B CTRL') and name=='pcb_b_temperature':
            pt=item.GetPosition();item.SetPosition(p.VECTOR2I(pt.x,p.FromMM(115)))
    p.SaveBoard(str(source),board)
    path=OUT/(name+'.dsn');assert p.ExportSpecctraDSN(board,str(path))
    dsn=path.read_text(encoding='utf-8')
    nets=[n.attrib['name'] for n in ET.parse(ROOT/'review'/(name+'.net.xml')).getroot().findall('./nets/net')]
    power=POWER.intersection(nets) if name=='pcb_b_temperature' else set()
    via='Via[0-1]_800:400_um'
    oldvia=re.search(r'    \(padstack "Via\[0-1\]_600:300_um".*?\n    \)',dsn,re.S).group()
    newvia=oldvia.replace('600:300','800:400').replace('F.Cu 600','F.Cu 800').replace('B.Cu 600','B.Cu 800')
    dsn=dsn.replace(oldvia,newvia).replace('Via[0-1]_600:300_um',via)
    start=dsn.index('    (class kicad_default');end=dsn.index('  (wiring',start)
    def klass(label,names,width,clearance):
        return f'    (class {label} '+ ' '.join(q(n) for n in names)+f' (circuit (use_via "{via}")) (rule (width {width}) (clearance {clearance})))\n'
    classes=klass('kicad_default',[n for n in nets if n not in power],250,220)
    if power:classes+=klass('HeaterPower',sorted(power),2400,350)
    dsn=dsn[:start]+classes+'  )\n'+dsn[end:]
    path.write_text(dsn,encoding='utf-8')
    print('Prepared',path.name,'power classes',sorted(power))

def import_session(name):
    path=ROOT/name/(name+'.kicad_pcb');board=p.LoadBoard(str(path))
    assert len(list(board.GetTracks())) <= (6 if name=='pcb_b_temperature' else 0), 'Refuse to overwrite completed native routing with an intermediate SES'
    assert p.ImportSpecctraSES(board,str(OUT/(name+'.ses')))
    p.SaveBoard(str(path),board)
    print('Imported',name,'tracks/vias',len(list(board.GetTracks())))

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--prepare',action='store_true');parser.add_argument('--import',dest='import_',action='store_true');parser.add_argument('--board',choices=['pcb_a_display','pcb_b_temperature'])
    args=parser.parse_args()
    for name in [args.board] if args.board else ['pcb_a_display','pcb_b_temperature']:
        if args.prepare:prepare(name)
        if args.import_:import_session(name)
