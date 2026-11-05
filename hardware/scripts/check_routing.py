"""Validate routed native boards, schematic parity and actual high-current copper paths.

Run using KiCad Python. Geometry checks do not qualify thermal behavior or module fit.
"""
import collections
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
import pcbnew as p
from create_drafts import parse,child,children,unquote

ROOT=Path(__file__).resolve().parents[1]
CLI='C:/Program Files/KiCad/10.0/bin/kicad-cli.exe'

def power_path(board,net,start,end):
    """Find a continuous >=1.0mm trace path, with no discrete vias.

    Copper intersections use the native effective shapes on each layer, including
    plated connector/device pads. Duplicate same-number package pads are bonded.
    """
    nodes=[]
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            if pad.GetNetname()!=net:continue
            layers=[l for l in (p.F_Cu,p.B_Cu) if pad.IsOnLayer(l)]
            nodes.append({'key':(fp.GetReference(),pad.GetNumber()),'kind':'pad','width':None,'length':0,
                          'shapes':{l:pad.GetEffectiveShape(l) for l in layers}})
    for t in board.GetTracks():
        if t.GetNetname()!=net or isinstance(t,p.PCB_VIA) or t.GetWidth()<p.FromMM(1):continue
        nodes.append({'key':None,'kind':'track','width':p.ToMM(t.GetWidth()),'length':p.ToMM(t.GetLength()),
                      'shapes':{t.GetLayer():t.GetEffectiveShape(t.GetLayer())}})
    graph=[[] for _ in nodes]
    for i,a in enumerate(nodes):
        for j in range(i):
            b=nodes[j]
            bonded=a['key'] is not None and a['key']==b['key']
            if bonded or any(a['shapes'][l].Collide(b['shapes'][l],0) for l in a['shapes'].keys() & b['shapes'].keys()):
                graph[i].append(j);graph[j].append(i)
    starts=[i for i,n in enumerate(nodes) if n['key']==start]
    targets={i for i,n in enumerate(nodes) if n['key']==end}
    previous={i:None for i in starts};queue=collections.deque(starts);found=None
    while queue:
        i=queue.popleft()
        if i in targets:found=i;break
        for j in graph[i]:
            if j not in previous:previous[j]=i;queue.append(j)
    assert found is not None,(net,start,end,'No >=1.0 mm copper path without discrete vias')
    route=[]
    while found is not None:route.append(nodes[found]);found=previous[found]
    tracks=[n for n in route if n['kind']=='track']
    return {'net':net,'from':list(start),'to':list(end),'discrete_vias':0,
            'minimum_track_width_mm':min(n['width'] for n in tracks),
            'track_length_mm':round(sum(n['length'] for n in tracks),3),
            'neck_length_below_2p4_mm':round(sum(n['length'] for n in tracks if n['width']<2.399),3)}

def main():
    results=[]
    for name in ('pcb_a_display','pcb_b_temperature'):
        path=ROOT/name/(name+'.kicad_pcb');board=p.LoadBoard(str(path))
        source=parse(path.read_text(encoding='utf-8'))
        stack=child(child(source,'setup'),'stackup')
        copper={unquote(n[1]):float(child(n,'thickness')[1]) for n in children(stack,'layer') if unquote(n[1]) in ('F.Cu','B.Cu')}
        expected=.07 if name=='pcb_b_temperature' else .035
        assert copper=={'F.Cu':expected,'B.Cu':expected},copper
        report=ROOT/'review/routing'/(name+'_drc.json')
        subprocess.run([CLI,'pcb','drc',str(path),'--schematic-parity','--format','json','--output',str(report),'--exit-code-violations'],check=True)
        d=json.loads(report.read_text(encoding='utf-8'))
        assert not d['violations'] and not d['unconnected_items'] and not d['schematic_parity']
        xml=ET.parse(ROOT/'review'/(name+'.net.xml')).getroot()
        pins={(n.attrib['ref'],n.attrib['pin']):net.attrib['name'] for net in xml.findall('./nets/net') for n in net.findall('node')}
        for fp in board.GetFootprints():
            for pad in fp.Pads():
                if not pad.GetNumber():
                    assert not pad.IsOnLayer(p.F_Cu) and not pad.IsOnLayer(p.B_Cu), 'Only paste-only anonymous pads allowed'
                    continue
                assert pad.GetNetname()==pins[(fp.GetReference(),pad.GetNumber())]
        row={'board':name,'DRC_violations':0,'unconnected_items':0,'schematic_parity_issues':0,
             'footprints':len(list(board.GetFootprints())),
             'tracks':sum(not isinstance(t,p.PCB_VIA) for t in board.GetTracks()),
             'vias':sum(isinstance(t,p.PCB_VIA) for t in board.GetTracks())}
        row['copper_thickness_mm']=copper
        if name=='pcb_a_display':
            host=next(fp for fp in board.GetFootprints() if fp.GetReference()=='U1')
            pads={pad.GetNumber():pad for pad in host.Pads()}
            # Seller top view rotated so antenna is LEFT and USB is RIGHT:
            # original left header is now bottom (EN -> VIN), not mirrored.
            en=pads['1'].GetPosition();gpio23=pads['30'].GetPosition();vin=pads['15'].GetPosition()
            assert en.y>gpio23.y and vin.x>en.x and en.x==gpio23.x
            assert abs(p.ToMM(en.y-gpio23.y)-25.4)<1e-6
            assert abs(p.ToMM(pads['2'].GetPosition().x-en.x)-2.54)<1e-6
            row['host_orientation_checked']='antenna left / USB right; EN bottom-left; GPIO23 top-left; VIN bottom-right'
        if name=='pcb_b_temperature':
            pairs=[('/VIN_RAW',('J2','1'),('F1','1')),('/VIN_FUSED',('F1','2'),('Q2','2')),
                   ('/VIN_PROTECTED',('Q2','3'),('J3','1')),('/HEATER_LOW',('J3','2'),('Q1','2')),
                   ('/PGND',('Q1','3'),('J2','2'))]
            row['high_current_paths']=[power_path(board,*pair) for pair in pairs]
        results.append(row)
    data={'revision':'A 0.4 / B 0.5 integrated buck','status':'routed prototype, not fabrication release','target_input_V':24,'target_heater_W':100,
          'heater_current_A':100/24,'required_B_outer_copper_um':70,
          'devkit_row_spacing_mm':25.4,'devkit_spacing_status':'ASSUMED, user verification pending',
          'boards':results,'limits':['Physical DevKit dimensions/pinout must be confirmed.',
                                   'Power/thermal and fault testing not performed.',
                                   'Integrated LM2596 input transients, startup and fault conditions not qualified.',
                                   'Enclosure STEP v0.7 geometry reconciled; real fit, cables, clips and thermal tests pending.']}
    (ROOT/'review/routing/routing_validation.json').write_text(json.dumps(data,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(data,indent=2))

if __name__=='__main__':main()
