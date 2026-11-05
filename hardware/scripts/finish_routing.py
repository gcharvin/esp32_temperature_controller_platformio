"""Apply explicit native-board corrections after SES import, then run DRC."""
import xml.etree.ElementTree as ET
from pathlib import Path
import pcbnew as p

ROOT=Path(__file__).resolve().parents[1]

def add_segment(board,net,start,end,width,layer=p.B_Cu):
    a=p.VECTOR2I(p.FromMM(start[0]),p.FromMM(start[1]));b=p.VECTOR2I(p.FromMM(end[0]),p.FromMM(end[1]))
    if any(not isinstance(t,p.PCB_VIA) and t.GetLayer()==layer and {tuple(t.GetStart()),tuple(t.GetEnd())}=={tuple(a),tuple(b)} for t in board.GetTracks()):return
    t=p.PCB_TRACK(board);t.SetStart(a);t.SetEnd(b);t.SetWidth(p.FromMM(width));t.SetLayer(layer)
    t.SetNet(next(n for n in board.GetNetsByNetcode().values() if n.GetNetname()==net));board.Add(t)

def reverse_gate_route(path):
    # Edit route primitives directly to avoid a KiCad 10 SWIG ownership issue
    # observed when removing tracks and adding vias in the same Python process.
    from create_drafts import parse, dump, child, q
    import uuid
    tree=parse(path.read_text(encoding='utf-8'))
    tree=[n for n in tree if not (isinstance(n,list) and n[0] in ('segment','via') and child(n,'net')[1]=='"/REV_GATE"')]
    def obsolete_power_route(n):
        if not (isinstance(n,list) and n[0] in ('segment','via') and child(n,'net')[1]=='"/VIN_PROTECTED"'):return False
        if n[0]=='via':return tuple(round(float(v),4) for v in child(n,'at')[1:])==(107.2921,79.1695)
        points=[tuple(round(float(v),4) for v in child(n,key)[1:]) for key in ('start','end')]
        return any(point in points for point in [(107.2921,79.1695),(106.1805,79.1695),(119.4616,67)])
    tree=[n for n in tree if not obsolete_power_route(n)]
    for x,y in [(98,69.7),(108.5,82),(94,82)]:
        tree.append(parse(f'(via (at {x} {y}) (size 0.8) (drill 0.4) (layers "F.Cu" "B.Cu") (net "/REV_GATE") (uuid "{uuid.uuid4()}"))'))
    for points,layer in [([(99.85,69.7),(98,69.7)],'F.Cu'),
                         ([(98,69.7),(98,80),(108.5,80),(108.5,82)],'B.Cu'),
                         ([(108.5,82),(106.65,82)],'F.Cu'),
                         ([(96.175,82),(94,82)],'F.Cu'),
                         ([(94,82),(94,80),(98,80)],'B.Cu')]:
        for a,b in zip(points,points[1:]):
            tree.append(parse(f'(segment (start {a[0]} {a[1]}) (end {b[0]} {b[1]}) (width 0.25) (layer "{layer}") (net "/REV_GATE") (uuid "{uuid.uuid4()}"))'))
    path.write_text('('+tree[0]+'\n'+'\n'.join(dump(n) for n in tree[1:])+')\n',encoding='utf-8')

def main():
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument('--board',choices=['pcb_a_display','pcb_b_temperature']);args=parser.parse_args()
    for name in [args.board] if args.board else ('pcb_a_display','pcb_b_temperature'):
        path=ROOT/name/(name+'.kicad_pcb');board=p.LoadBoard(str(path))
        comps={c.attrib['ref']:c for c in ET.parse(ROOT/'review'/(name+'.net.xml')).getroot().findall('./components/comp')}
        for fp in board.GetFootprints():
            comp=comps[fp.GetReference()]
            for field in comp.findall('./fields/field'):
                key=field.attrib['name']
                if key not in ('Footprint','Datasheet','Description'):
                    fp.SetField(key,field.text or '')
                    fp.GetField(key).SetVisible(False)
            ds=comp.find('./fields/field[@name="Datasheet"]')
            if ds is not None:fp.SetField('Datasheet',ds.text or '')
        for track in board.GetTracks():
            # Do not call PCB_VIA.GetWidth without specifying its layer.
            if not isinstance(track,p.PCB_VIA) and track.GetWidth()<p.FromMM(.2):
                track.SetWidth(p.FromMM(.2))
        if name=='pcb_b_temperature':
            add_segment(board,'/PGND',(74.08,85),(76,85),.5)
            add_segment(board,'/PGND',(76,85),(77,84),.5)
            add_segment(board,'/PGND',(99.9172,82),(99.9172,84),.3)
            for t in board.GetTracks():
                if not isinstance(t,p.PCB_VIA) and t.GetNetname()=='/VIN_PROTECTED' and t.GetWidth()<p.FromMM(1.2):t.SetWidth(p.FromMM(1.2))
            points=[(99.85,78),(110,78),(121.15,78),(126,73.15),(126,70),(129,67),(135,67)]
            for a,b in zip(points,points[1:]):add_segment(board,'/VIN_PROTECTED',a,b,2.4,p.F_Cu)
        p.SaveBoard(str(path),board)
        if name=='pcb_b_temperature':reverse_gate_route(path)

if __name__=='__main__':main()
