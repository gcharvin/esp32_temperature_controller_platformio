"""Add local heater suppression, decoupling, copper stackup and ground pours.

Native PCB files remain authoritative; this is the prototype's final copper stage.
"""
from pathlib import Path
import xml.etree.ElementTree as ET
import pcbnew as p
from create_drafts import parse,dump,child,children,q,uid
from create_placement import pos,xy,mm
from finish_routing import add_segment

ROOT=Path(__file__).resolve().parents[1]
LIB=Path('C:/Program Files/KiCad/10.0/share/kicad/footprints')

def stackup(path,copper):
    tree=parse(path.read_text(encoding='utf-8'));setup=child(tree,'setup')
    if 'pcb_b_temperature' in str(path):
        def obsolete(n):
            if not (isinstance(n,list) and n[0] in ('segment','via') and child(n,'net')[1]=='"/VIN_PROTECTED"'):return False
            if n[0]=='via':return tuple(map(float,child(n,'at')[1:]))==(140,81)
            points=[tuple(map(float,child(n,key)[1:])) for key in ('start','end')]
            return any(point in points for point in [(146,81),(146,63.5),(138.5,63.5)])
        tree=[n for n in tree if not obsolete(n)]
        tree.append(parse(f'(via (at 140 81) (size 0.8) (drill 0.4) (layers "F.Cu" "B.Cu") (net "/VIN_PROTECTED") (uuid "{uid("flyback-via")}"))'))
        setup=child(tree,'setup')
    setup[:]=[n for n in setup if not (isinstance(n,list) and n[0]=='stackup')]
    core=1.6-2*copper
    setup.insert(1,parse(f'''(stackup
      (layer "F.SilkS" (type "Top Silk Screen"))
      (layer "F.Paste" (type "Top Solder Paste"))
      (layer "F.Mask" (type "Top Solder Mask") (thickness 0.01) (epsilon_r 3.3) (loss_tangent 0))
      (layer "F.Cu" (type "copper") (thickness {copper}))
      (layer "dielectric 1" (type "core") (thickness {core}) (material "FR4") (epsilon_r 4.5) (loss_tangent 0.02))
      (layer "B.Cu" (type "copper") (thickness {copper}))
      (layer "B.Mask" (type "Bottom Solder Mask") (thickness 0.01) (epsilon_r 3.3) (loss_tangent 0))
      (layer "B.Paste" (type "Bottom Solder Paste"))
      (layer "B.SilkS" (type "Bottom Silk Screen"))
      (copper_finish "ENIG") (dielectric_constraints no))'''))
    path.write_text('('+tree[0]+'\n'+'\n'.join(dump(n) for n in tree[1:])+')\n',encoding='utf-8')

def zone(board,name,netname,layer,bounds,thermal=True):
    if any(z.GetZoneName()==name for z in board.Zones()):return
    z=p.ZONE(board);z.SetZoneName(name);z.SetLayer(layer)
    z.SetNet(next(n for n in board.GetNetsByNetcode().values() if n.GetNetname()==netname))
    z.SetLocalClearance(mm(.3));z.SetMinThickness(mm(.25))
    z.SetPadConnection(p.ZONE_CONNECTION_THT_THERMAL if thermal else p.ZONE_CONNECTION_FULL)
    z.SetThermalReliefGap(mm(.3));z.SetThermalReliefSpokeWidth(mm(.4))
    z.SetIslandRemovalMode(p.ISLAND_REMOVAL_MODE_ALWAYS)
    x,y,w,h=bounds;outline=z.Outline();outline.NewOutline()
    for xx,yy in [(x,y),(x+w,y),(x+w,y+h),(x,y+h)]:
        point=pos(xx,yy);outline.Append(point.x,point.y)
    board.Add(z)

def main():
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument('--board',choices=['pcb_a_display','pcb_b_temperature']);args=parser.parse_args()
    for name,width,height in [('pcb_a_display',110,75),('pcb_b_temperature',90,65)]:
        if args.board and name!=args.board:continue
        path=ROOT/name/(name+'.kicad_pcb');stackup(path,.07 if name=='pcb_b_temperature' else .035)
        board=p.LoadBoard(str(path));xml=ET.parse(ROOT/'review'/(name+'.net.xml')).getroot()
        comps={c.attrib['ref']:c for c in xml.findall('./components/comp')}
        pin_nets={(n.attrib['ref'],n.attrib['pin']):net.attrib['name'] for net in xml.findall('./nets/net') for n in net.findall('node')}
        nets={n.GetNetname():n for n in board.GetNetsByNetcode().values()}
        existing={f.GetReference() for f in board.GetFootprints()}
        if name=='pcb_b_temperature':
            for ref,x,y,angle in [('D4',81,23,0),('C8',23,23.85,270)]:
                if ref in existing:continue
                comp=comps[ref];libid=comp.findtext('footprint');nick,part=libid.split(':')
                fp=p.FootprintLoad(str(LIB/(nick+'.pretty')),part);assert fp is not None
                fp.SetFPID(p.LIB_ID(nick,part));fp.SetReference(ref);fp.SetValue(comp.findtext('value'))
                fp.SetPosition(pos(x,y));fp.SetOrientationDegrees(angle)
                fp.SetPath(p.KIID_PATH('/'+uid(name)+'/'+comp.findtext('tstamps')))
                fp.SetSheetname(name);fp.SetSheetfile(name+'.kicad_sch');board.Add(fp)
                fp.Value().SetVisible(False);fp.Reference().SetTextSize(xy(.8,.8));fp.Reference().SetTextThickness(mm(.12))
                for pad in fp.Pads():pad.SetNet(nets[pin_nets[(ref,pad.GetNumber())]])
            # High-current flyback pulse path; no current rating inferred from small-signal routing.
            for net,points,layer in [('/VIN_PROTECTED',[(138.85,78),(138.85,81),(140,81)],p.F_Cu),
                                    ('/VIN_PROTECTED',[(140,81),(144,77),(144,63.5),(138.5,63.5),(135,67)],p.B_Cu),
                                    ('/HEATER_LOW',[(143.15,78),(143.15,70.07),(140.08,67)],p.F_Cu)]:
                for a,b in zip(points,points[1:]):add_segment(board,net,a,b,1.0,layer)
            # Local input bulk capacitor return, to the fixed PGND spine on the bottom.
            add_segment(board,'/PGND',(83,82.35),(83,84),1.0,p.B_Cu)
            zone(board,'Q2_DRAIN_HEAT_SPREAD','/VIN_FUSED',p.F_Cu,(20,5,32,16),False)
        for fp in board.GetFootprints():
            comp=comps[fp.GetReference()];fp.SetValue(comp.findtext('value'))
            for field in comp.findall('./fields/field'):
                if field.attrib['name'] not in ('Footprint','Description'):
                    fp.SetField(field.attrib['name'],field.text or '')
                    fp.GetField(field.attrib['name']).SetVisible(False)
        zone(board,'LOGIC_GND_BOTTOM','/GND',p.B_Cu,(1.8,4.3,width-3.6,height-8.6))
        p.ZONE_FILLER(board).Fill(board.Zones())
        p.SaveBoard(str(path),board)
        print('Filled copper and set stackup:',name)

if __name__=='__main__':main()
