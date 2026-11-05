"""Create unrouted mechanical placement studies with KiCad's bundled Python.

Known schematic footprints are used as-is. Unspecified packages are padless
mechanical envelopes, explicitly labelled TBD. Native PCB files remain editable.
No manufacturing output is produced. Existing boards require --overwrite.
"""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import pcbnew as p

ROOT=Path(__file__).resolve().parents[1]
FPLIB=Path('C:/Program Files/KiCad/10.0/share/kicad/footprints')
ORIGIN=(60,55)

# Coordinates relative to the upper-left of each board, viewed from components.
# An envelope's width/height are planning allowances, not measured packages.
A={
 'U1':(32,40,(60,32)), 'U2':(80,55,None),
 'J1':(94,12,None),'J6':(94,28,None),'J7':(88,45,None),
 'J2':(66,10,(16,9)), 'J3':(72,64,(16,9)),
 'J4':(44,10,(14,9)), 'J5':(23,10,(14,9)), 'JP1':(72,40,None),
 'C1':(75,55,None),'C2':(85,55,None),'C3':(88,60,None),
 'R1':(62,61,None),'R2':(65,57,None),'R3':(68,57,None),
 'R4':(64,22,None),'R5':(67,22,None)}
B={
 'U1':(22,29,(30,22)), 'U2':(23,45,None),
 'U3':(54,42,(5,5)), 'U4':(73,29,None), 'Q1':(52,28,(12,12)),
 'F1':(28,10,(8,5)), 'D1':(42,10,(8,5)), 'D2':(56,10,(8,5)),
 'J2':(12,10,(12,10)), 'J3':(78,10,(12,10)), 'J4':(80,44,(10,8)),
 'J1':(15,53,None),'J6':(34,53,None),'J7':(56,53,None),
 'C1':(18,45,None),'C2':(28,45,None),'C3':(50,42,None),
 'C4':(83,36,None),'C5':(68,29,None),
 'R1':(46,42,None),'R2':(46,46,None),'R3':(58,36,None),
 'R4':(62,36,None),'R5':(83,28,None),'R6':(83,32,None),'R7':(68,35,None)}

def mm(x):return p.FromMM(x)
def xy(x,y):return p.VECTOR2I(mm(x),mm(y))
def pos(x,y):return xy(x+ORIGIN[0],y+ORIGIN[1])

def line(board,x1,y1,x2,y2,layer=p.Dwgs_User,width=.15,parent=None):
    obj=p.PCB_SHAPE(parent or board)
    obj.SetShape(p.SHAPE_T_SEGMENT);obj.SetLayer(layer);obj.SetWidth(mm(width))
    obj.SetStart(pos(x1,y1) if parent is None else xy(x1,y1))
    obj.SetEnd(pos(x2,y2) if parent is None else xy(x2,y2))
    (parent or board).Add(obj)

def rect(board,x,y,w,h,layer=p.Dwgs_User,parent=None):
    for a,b,c,d in [(x,y,x+w,y),(x+w,y,x+w,y+h),(x+w,y+h,x,y+h),(x,y+h,x,y)]:
        line(board,a,b,c,d,layer,parent=parent)

def text(board,txt,x,y,size=1,layer=p.Dwgs_User):
    obj=p.PCB_TEXT(board);obj.SetText(txt);obj.SetPosition(pos(x,y))
    obj.SetTextSize(xy(size,size));obj.SetTextThickness(mm(.15));obj.SetLayer(layer)
    obj.SetHorizJustify(p.GR_TEXT_H_ALIGN_LEFT);board.Add(obj)

def rule_area(board,x,y,w,h,footprints=False):
    z=p.ZONE(board);z.SetIsRuleArea(True)
    layers=p.LSET();layers.AddLayer(p.F_Cu);layers.AddLayer(p.B_Cu);z.SetLayerSet(layers)
    z.SetDoNotAllowTracks(True);z.SetDoNotAllowVias(True);z.SetDoNotAllowZoneFills(True)
    z.SetDoNotAllowPads(True);z.SetDoNotAllowFootprints(footprints)
    outline=z.Outline();outline.NewOutline()
    for px,py in [(x,y),(x+w,y),(x+w,y+h),(x,y+h)]:
        point=pos(px,py);outline.Append(point.x,point.y)
    board.Add(z)

def dimension(board,start,end,height):
    d=p.PCB_DIM_ALIGNED(board);d.SetLayer(p.Dwgs_User)
    d.SetStart(pos(*start));d.SetEnd(pos(*end));d.SetHeight(mm(height))
    d.SetUnitsMode(p.DIM_UNITS_MODE_MM);d.SetPrecision(p.DIM_PRECISION_X_X)
    d.SetTextSize(xy(1.2,1.2));d.SetTextThickness(mm(.15));d.Update();board.Add(d)

def envelope_fp(board,name,ref,width,height):
    fp=p.FOOTPRINT(board);fp.SetReference(ref);fp.SetValue('TBD - ENVELOPE ONLY')
    fp.SetFPID(p.LIB_ID('PlacementOnly',name+'_'+ref+'_TBD'))
    rect(board,-width/2,-height/2,width,height,p.F_Fab,parent=fp)
    rect(board,-width/2,-height/2,width,height,p.F_CrtYd,parent=fp)
    fp.SetAttributes(p.FP_EXCLUDE_FROM_POS_FILES)
    folder=ROOT/'placement_only.pretty';folder.mkdir(exist_ok=True)
    p.PCB_IO_MGR.FindPlugin(p.PCB_IO_MGR.KICAD_SEXP).FootprintSave(str(folder),fp)
    return fp

def build(name,width,height,placement):
    board=p.BOARD();board.GetDesignSettings().SetBoardThickness(mm(1.6))
    nets={};pin_nets={}
    xml=ET.parse(ROOT/'review'/(name+'.net.xml')).getroot()
    for n in xml.findall('./nets/net'):
        net=p.NETINFO_ITEM(board,n.attrib['name']);board.Add(net);nets[n.attrib['name']]=net
        for node in n.findall('node'):
            pin_nets[(node.attrib['ref'],node.attrib['pin'])]=net
    rect(board,0,0,width,height,p.Edge_Cuts)
    dimension(board,(0,0),(width,0),-9)
    dimension(board,(0,0),(0,height),9)
    rect(board,0,0,width,4);rect(board,0,height-4,width,4)
    rule_area(board,0,0,width,4,True);rule_area(board,0,height-4,width,4,True)
    for x in (0,width-2):
        rule_area(board,x,8,2,3,True)
        rect(board,x,8,2,3,p.Cmts_User)
    text(board,'CLIP / SUPPORT BAND - NO COPPER / PARTS',3,2.7,.9)
    for x in (8,width-20):
        rect(board,x,0,12,4,p.Cmts_User);rect(board,x,height-4,12,4,p.Cmts_User)
    if name=='pcb_a_display':
        rule_area(board,0,17,20,46)
        rect(board,0,17,20,46,p.Cmts_User)
        text(board,'RF RESERVE',1,20,.8,p.Cmts_User)
        text(board,'USB access from rear / side service space',25,height+10,1)
    text(board,name+' / PLACEMENT STUDY 0.1',0,-17,1.5)
    text(board,'UNROUTED - TBD outlines are NOT validated footprints',0,height+5,1.1)
    records=[]
    components={c.attrib['ref']:c for c in xml.findall('./components/comp')}
    assert set(components)==set(placement), (set(components)-set(placement),set(placement)-set(components))
    for ref,(x,y,envelope) in placement.items():
        comp=components[ref];libid=comp.findtext('footprint') or ''
        if libid:
            lib,part=libid.split(':',1)
            fp=p.FootprintLoad(str(FPLIB/(lib+'.pretty')),part)
            if fp is None:raise RuntimeError(libid)
            fp.SetFPID(p.LIB_ID(lib,part))
            status='schematic footprint'
        else:
            assert envelope,ref
            fp=envelope_fp(board,name,ref,*envelope);status='unmeasured planning envelope, NO PADS'
        fp.SetReference(ref);fp.SetPosition(pos(x,y));board.Add(fp)
        fp.Value().SetVisible(False)
        fp.Reference().SetLayer(p.F_SilkS);fp.Reference().SetTextSize(xy(.8,.8))
        fp.Reference().SetTextThickness(mm(.12))
        for pad in fp.Pads():
            net=pin_nets.get((ref,pad.GetNumber()))
            if net:pad.SetNet(net)
        box=fp.GetBoundingBox(False,False)
        xmin=p.ToMM(box.GetLeft())-ORIGIN[0];ymin=p.ToMM(box.GetTop())-ORIGIN[1]
        w=p.ToMM(box.GetWidth());h=p.ToMM(box.GetHeight())
        if envelope:
            xmin,ymin=x-envelope[0]/2,y-envelope[1]/2;w,h=envelope
            fp.Reference().SetPosition(pos(xmin,ymin-1.2))
        records.append({'reference':ref,'value':comp.findtext('value'),'origin_mm':[x,y],
                        'bounds_mm':[round(v,3) for v in [xmin,ymin,w,h]],
                        'footprint':libid or None,'status':status})
    path=ROOT/name/(name+'.kicad_pcb');p.SaveBoard(str(path),board)
    (ROOT/name/'fp-lib-table').write_text('(fp_lib_table (version 7) (lib (name "PlacementOnly") (type "KiCad") (uri "${KIPRJMOD}/../placement_only.pretty") (options "") (descr "UNMEASURED planning envelopes, no pads; not manufacturing footprints")))\n',encoding='utf-8')
    # Round-trip checks verify usable native files, closed outline and net assignment.
    loaded=p.LoadBoard(str(path))
    assert len(list(loaded.GetFootprints()))==len(placement)
    assert len(list(loaded.GetTracks()))==0
    assert loaded.GetBoardEdgesBoundingBox().GetWidth()==mm(width)+mm(.15)
    for row in records:
        x,y,w,h=row['bounds_mm']
        assert x>=-.15 and x+w<=width+.15 and y>=4 and y+h<=height-4,(name,row)
    return {'name':name,'board_mm':[width,height,1.6],
            'coordinates':'component-side view, origin upper-left; placement origin may be pad 1',
            'clip_bands_mm':[[0,0,width,4],[0,height-4,width,4]],
            'clip_contact_patches_mm':[[x,y,12,4] for x in (8,width-20) for y in (0,height-4)],
            'clip_side_contact_patches_mm':[[0,8,2,3],[width-2,8,2,3]],
            'components':records}

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--overwrite',action='store_true')
    args=parser.parse_args()
    if any(ROOT.glob('pcb_*/*.kicad_pcb')) and not args.overwrite:
        raise SystemExit('Existing PCB study: use --overwrite only to discard edits intentionally.')
    boards=[build('pcb_a_display',110,75,A),build('pcb_b_temperature',90,65,B)]
    data={'revision':'placement study 0.1','status':'proposed dimensions, not production ready',
          'case_external_mm':{'width':145,'height':100,'depth':115},'wall_mm':3,
          'case_coordinates':'x left-to-right, y top-to-bottom, d front-to-rear',
          'assembly':{'A_board_origin_x_y_d_mm':[8,12,45],'B_board_origin_x_y_d_mm':[40,18,85],
                      'lcd_front_envelope_x_y_w_h_mm':[8,12,98,60],
                      'lcd_back_depth_max_mm':28,'encoder_center_x_y_mm':[126,45],
                      'A_rear_component_allowance_mm':20,'B_rear_component_allowance_mm':20,
                      'B_extraction_direction':'rear (+depth), unplug all cables unpowered',
                      'rear_panel_removable':True},
          'boards':boards}
    (ROOT/'placement.json').write_text(json.dumps(data,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    print('Created editable unrouted PCB studies and placement.json')

if __name__=='__main__':main()
