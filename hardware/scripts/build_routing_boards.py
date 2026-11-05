"""Physical prototype boards + DSN. Run before routing, never over routed files."""
import copy
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import pcbnew as p
from create_placement import A as OLD_A, rule_area, rect, text, pos, xy, mm, dimension
from create_drafts import uid

ROOT=Path(__file__).resolve().parents[1]
LIB=Path('C:/Program Files/KiCad/10.0/share/kicad/footprints')
A={ref:(row[0],row[1],0) for ref,row in OLD_A.items()}
B={
 'J2':(9,12,270),'F1':(27,12,0),'Q2':(43,17,0),'D3':(45,27,0),'R8':(37,27,0),
 'D2':(59,12,0),'D4':(81,23,0),'C8':(23,23.85,270),'J3':(75,12,0),'Q1':(66,23,0),'J8':(9,30,0),
 'J9':(9,44,0),'F2':(23,42,0),'U2':(32,44,0),'C1':(27,47,0),'C2':(36,47,0),
 'U3':(44,44,0),'C3':(44,39,0),'R1':(39,44,0),'R2':(39,48,0),
 'U5':(60,35,0),'C6':(56,32,0),'C7':(56,38,0),'R3':(64,29,0),'R4':(75,28,0),
 'U4':(76,40,0),'C5':(80,35,0),'R5':(85,45,0),'R6':(85,39,0),'C4':(81,43,0),'R7':(71,45,0),'J4':(81,52,0),
 'J1':(15,55,0),'J6':(33,55,0),'J7':(53,55,0)}

def track(board,net,points,width=2.4,layer=p.F_Cu,locked=True):
    for start,end in zip(points,points[1:]):
        t=p.PCB_TRACK(board);t.SetStart(pos(*start));t.SetEnd(pos(*end));t.SetWidth(mm(width))
        t.SetLayer(layer);t.SetNet(net);t.SetLocked(locked);board.Add(t)

def project_rules(name):
    path=ROOT/name/(name+'.kicad_pro');d=json.loads(path.read_text(encoding='utf-8'))
    ns=d.setdefault('net_settings',{})
    default=ns['classes'][0];default.update(clearance=.20,track_width=.25,via_diameter=.8,via_drill=.4)
    ns['classes']=[default]
    ns['netclass_patterns']=[]
    if name=='pcb_b_temperature':
        power=copy.deepcopy(default);power.update(name='HeaterPower',clearance=.3,track_width=2.4,via_diameter=2,via_drill=1,priority=0)
        ns['classes'].append(power)
        ns['netclass_patterns']=[{'netclass':'HeaterPower','pattern':'/'+net} for net in ('VIN_RAW','VIN_FUSED','VIN_PROTECTED','HEATER_LOW')]
    rules=d.setdefault('board',{}).setdefault('design_settings',{}).setdefault('rules',{})
    rules.update(min_clearance=.2,min_track_width=.2,min_via_diameter=.6,min_through_hole_diameter=.3,min_hole_clearance=.25,min_copper_edge_clearance=.5)
    path.write_text(json.dumps(d,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')

def build(name,width,height,placement):
    path=ROOT/name/(name+'.kicad_pcb')
    assert not list(p.LoadBoard(str(path)).GetTracks()), 'Refuse routed source'
    project_rules(name)
    board=p.BOARD();board.SetFileName(str(path));board.GetDesignSettings().SetBoardThickness(mm(1.6))
    rect(board,0,0,width,height,p.Edge_Cuts)
    dimension(board,(0,0),(width,0),-9);dimension(board,(0,0),(0,height),9)
    for y in (0,height-4):rule_area(board,0,y,width,4,True);rect(board,0,y,width,4,p.Cmts_User)
    for x in (0,width-1.5):rule_area(board,x,4,1.5,height-8,True);rect(board,x,4,1.5,height-8,p.Cmts_User)
    for x in (0,width-2):rule_area(board,x,8,2,3,True)
    text(board,name+' / ROUTING PROTOTYPE 0.3',0,-16,1.4)
    text(board,'NOT RELEASED FOR FABRICATION / module and thermal checks pending',0,height+5,1)
    if name=='pcb_a_display':
        rule_area(board,0,17,12.5,46);rect(board,0,17,12.5,46,p.Cmts_User)
        text(board,'ANTENNA - RF TEST REQUIRED',0,15,.8,p.Cmts_User)
        text(board,'U1 ROW SPACING 25.40 mm ASSUMED - CHECK PHYSICAL BOARD',4,height+8,1)
    else:
        text(board,'24V / 100W RESISTIVE / 2 OZ Cu / 500 Hz PWM',4,height+8,1)
        text(board,'XL4015 is OFF-BOARD; set 5.00 V before connecting',4,height+11,1)
    xml=ET.parse(ROOT/'review'/(name+'.net.xml')).getroot()
    nets={};pin_nets={}
    for net in xml.findall('./nets/net'):
        item=p.NETINFO_ITEM(board,net.attrib['name']);board.Add(item);nets[net.attrib['name']]=item
        for node in net.findall('node'):pin_nets[(node.attrib['ref'],node.attrib['pin'])]=item
    comps={c.attrib['ref']:c for c in xml.findall('./components/comp')}
    expected=set(comps)-({'U1'} if name=='pcb_b_temperature' else set())
    assert set(placement)==expected,(set(placement)^expected)
    records=[];fps={}
    for ref,(x,y,angle) in placement.items():
        comp=comps[ref];libid=comp.findtext('footprint')
        assert libid,(name,ref,'missing manufacturing footprint')
        nick,part=libid.split(':')
        libpath=ROOT/'lab_hardware.pretty' if nick=='LabHardware' else LIB/(nick+'.pretty')
        fp=p.FootprintLoad(str(libpath),part);assert fp is not None,(ref,libid)
        fp.SetFPID(p.LIB_ID(nick,part));fp.SetReference(ref);fp.SetValue(comp.findtext('value'))
        fp.SetPosition(pos(x,y));fp.SetOrientationDegrees(angle)
        fp.SetPath(p.KIID_PATH('/'+uid(name)+'/'+comp.findtext('tstamps')))
        fp.SetSheetname(name);fp.SetSheetfile(name+'.kicad_sch')
        board.Add(fp);fps[ref]=fp
        fp.Reference().SetTextSize(xy(.8,.8));fp.Reference().SetTextThickness(mm(.12))
        fp.Reference().SetLayer(p.F_SilkS);fp.Value().SetVisible(False)
        for pad in fp.Pads():
            number=pad.GetNumber()
            if number:
                assert (ref,number) in pin_nets,(ref,number)
                pad.SetNet(pin_nets[(ref,number)])
        bounds=fp.GetBoundingBox(False,False)
        records.append({'reference':ref,'value':comp.findtext('value'),'origin_mm':[x,y],'rotation_deg':angle,
                        'footprint':libid,'bounds_mm':[round(p.ToMM(v),3) for v in (bounds.GetLeft()-mm(60),bounds.GetTop()-mm(55),bounds.GetWidth(),bounds.GetHeight())],
                        'status':'physical footprint; prototype design review pending',
                        'height_mm':9.8 if ref.startswith('J') else 20 if ref=='U1' else 17 if ref=='Q1' else 1.5})
    if name=='pcb_b_temperature':
        # Fixed high-current return on bottom, entering Q1 source from the right.
        # No heater current through logic GND / JST cable returns.
        track(board,nets['/PGND'],[(9,17.08),(9,21),(17,29),(77,29),(77,26),(74,23),(71.08,23)],2.4,p.B_Cu)
        # Short gate loop, driver bypass and gate pulldown; refine if DRC requires.
    labels={'J1':'A-B 5V','J6':'A-B I2C','J7':'A-B CTRL','J2':'24V IN' if name.endswith('temperature') else 'LCD','J3':'HEATER' if name.endswith('temperature') else 'ENCODER','J4':'NTC' if name.endswith('temperature') else 'TMP117'}
    if name=='pcb_b_temperature':labels.update(J8='BUCK IN 24V',J9='BUCK OUT 5V')
    for ref,label in labels.items():
        fp=fps[ref];pt=fp.GetPosition();text(board,label,p.ToMM(pt.x)-60,p.ToMM(pt.y)-55-7.5,.85,p.F_SilkS)
    p.SaveBoard(str(path),board)
    loaded=p.LoadBoard(str(path))
    dest=ROOT/'review/routing';dest.mkdir(exist_ok=True)
    assert p.ExportSpecctraDSN(loaded,str(dest/(name+'.dsn')))
    return {'name':name,'board_mm':[width,height,1.6],
            'coordinates':'component-side view, origin upper-left',
            'clip_bands_mm':[[0,0,width,4],[0,height-4,width,4]],
            'side_support_bands_mm':[[0,4,1.5,height-8],[width-1.5,4,1.5,height-8]],
            'clip_side_contact_patches_mm':[[0,8,2,3],[width-2,8,2,3]],'components':records}

def main():
    data=json.loads((ROOT/'placement.json').read_text(encoding='utf-8'))
    data['boards']=[build('pcb_a_display',110,75,A),build('pcb_b_temperature',90,65,B)]
    data.update(revision='routing prototype 0.3',status='routing preparation; DevKit pitch assumed 25.40 mm')
    data['external_modules']=[{'reference':'B_U1','model':'TECNOULAB XL4015 CC/CV','source':'https://www.amazon.fr/dp/B0C6R3FMT3','dimensions_mm':[51,26.3,14],'mounting':'off-board, flexible wires; support in enclosure to revise'}]
    (ROOT/'placement.json').write_text(json.dumps(data,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')

if __name__=='__main__':main()
