"""Export IPC-D-356 and normalize only the explicitly designed B ground tie.

Keep raw KiCad output alongside the CAM test file. The logical PGND/GND
separation is a routing rule; NT1 makes them one physical bare-board net.
No other net name, test-point coordinate or record is changed.
"""
import hashlib
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
CLI='C:/Program Files/KiCad/10.0/bin/kicad-cli.exe'


def export(code,name,folder):
    pcb=ROOT/name/(name+'.kicad_pcb');raw=folder/'bare_board_test_raw_kicad.d356'
    subprocess.run([CLI,'pcb','export','ipcd356','--output',str(raw),str(pcb)],check=True,capture_output=True,text=True)
    lines=raw.read_text(encoding='utf-8').splitlines();assert lines[:2]==['P  CODE 00','P  UNITS CUST 0']
    changed=0;result=[]
    if code=='B':
        nets=ET.parse(folder/'netlist.xml')
        pins={(n.get('ref'),n.get('pin')):net.get('name') for net in nets.findall('./nets/net') for n in net.findall('node')}
        assert pins['NT1','1']=='/PGND' and pins['NT1','2']=='/GND'
    for line in lines:
        if code=='B' and line[:3] in ('317','327','367') and line[3:17].strip()=='/PGND':
            line=line[:3]+'/GND'.ljust(14)+line[17:];changed+=1
        result.append(line)
    if code=='B':assert changed>0
    if code=='A':assert changed==0
    header=['C  RFQ test netlist; native IPC-D-356 units retained (0.0001 inch).']
    if code=='B':header+=['C  Intentional copper NT1 joins /PGND and /GND; normalized to /GND.']
    target=folder/'bare_board_test.d356';target.write_text('\n'.join(header+result)+'\n',encoding='utf-8')
    # Prove normalization is confined to the net-name field of expected records.
    for before,after in zip(lines,result):
        if before!=after:
            assert before[3:17].strip()=='/PGND' and after[3:17].strip()=='/GND'
            assert before[:3]==after[:3] and before[17:]==after[17:]
    report=dict(status='Prepared; bare-board tests have not been performed',board=code,
        record_count=sum(line[:3] in ('317','327','367') for line in lines),ground_tie_records_normalized=changed,
        expected_physical_equivalence=['PGND','GND'] if code=='B' else [],all_coordinates_and_other_nets_unchanged=True,
        source_PCB_sha256=hashlib.sha256(pcb.read_bytes()).hexdigest(),
        output_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (raw,target)})
    (folder/'bare_board_test_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    return report
