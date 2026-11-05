"""Read-only validation of native unrouted PCB studies (KiCad Python).

Checks contours, clip-band clearance, real-pad net assignment and DRC geometry.
Unconnected pads are expected; they are counted, never suppressed.
"""
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
import pcbnew as p

ROOT=Path(__file__).resolve().parents[1]
CLI='C:/Program Files/KiCad/10.0/bin/kicad-cli.exe'
DATA=json.loads((ROOT/'placement.json').read_text(encoding='utf-8'))
messages=[]
for spec in DATA['boards']:
    name=spec['name']; board=p.LoadBoard(str(ROOT/name/(name+'.kicad_pcb')))
    width,height,thickness=spec['board_mm']
    edges=board.GetBoardEdgesBoundingBox()
    assert abs(p.ToMM(edges.GetWidth())-width-.15)<.001
    assert abs(p.ToMM(edges.GetHeight())-height-.15)<.001
    assert abs(p.ToMM(board.GetDesignSettings().GetBoardThickness())-thickness)<.001
    is_routed=bool(list(board.GetTracks()))
    pin_nets={}
    xml=ET.parse(ROOT/'review'/(name+'.net.xml')).getroot()
    for net in xml.findall('./nets/net'):
        for node in net.findall('node'):
            pin_nets[(node.attrib['ref'],node.attrib['pin'])]=net.attrib['name']
    footprints={fp.GetReference():fp for fp in board.GetFootprints()}
    assert set(footprints)=={c['reference'] for c in spec['components']}
    provisional=[]
    for row in spec['components']:
        ref=row['reference'];fp=footprints[ref]
        x,y,w,h=row['bounds_mm']
        assert x>=-.15 and x+w<=width+.15 and y>=4 and y+h<=height-4,ref
        if row['footprint'] is None:
            assert len(list(fp.Pads()))==0,ref
            provisional.append(ref)
        else:
            assert fp.GetFPID().GetUniStringLibId()==row['footprint']
            for pad in fp.Pads():
                if not pad.GetNumber():
                    assert not pad.IsOnLayer(p.F_Cu) and not pad.IsOnLayer(p.B_Cu)
                    continue
                assert pad.GetNetname()==pin_nets[(ref,pad.GetNumber())],(name,ref,pad.GetNumber())
    report=ROOT/'review'/('pcb_'+('a' if name=='pcb_a_display' else 'b')+'_placement_drc.json')
    subprocess.run([CLI,'pcb','drc','--format','json','--output',str(report),str(ROOT/name/(name+'.kicad_pcb'))],check=True)
    drc=json.loads(report.read_text(encoding='utf-8'))
    assert drc['violations']==[],drc['violations']
    unconnected=len(drc['unconnected_items'])
    if is_routed:assert unconnected==0
    messages.append(f'{name}: {width} x {height} x {thickness} mm; DRC geometry 0 violations; {unconnected} unconnected items; '+('routed.' if is_routed else 'unrouted.'))
    messages.append('Padless provisional envelopes: '+(', '.join(provisional) if provisional else 'none'))
messages.append('Clip bands and native footprint net assignments verified. No thermal, RF, print tolerance, assembly parity or manufacturing qualification implied.')
(ROOT/'review'/'placement_validation.txt').write_text('\n'.join(messages)+'\n',encoding='utf-8')
print('\n'.join(messages))
