"""One-shot copper-only bench pads, native schematic parity and preserved routes.

KiCad Python --apply; then route_test_points.py and refresh placement.
No components are bought or pasted for these B.Cu/B.Mask test pads.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
import tempfile
import pcbnew as p
import create_drafts as s
from create_placement import pos,xy,mm
from route_diagnostics import native_extract,copper_signature

ROOT=Path(__file__).resolve().parents[1]
LIB='C:/Program Files/KiCad/10.0/share/kicad/footprints/TestPoint.pretty'
CLI='C:/Program Files/KiCad/10.0/bin/kicad-cli.exe'
FP='TestPoint:TestPoint_Pad_D1.5mm'
SPECS={
 'pcb_a_display':[
  ('GND','J1','1'),('+5V_AB','J1','2'),('+5V_DEVKIT','JP1','2'),('+3V3_A','J5','2'),
  ('I2C_SDA','J6','2'),('I2C_SCL','J6','3'),('PWM0','J7','2'),('ENABLE','J7','3'),
  ('FAULT_N','J7','4'),('ENC_SW','J3','3')],
 'pcb_b_temperature':[
  ('PGND','J2','2'),('VIN_PROTECTED','J3','1'),('+5V_BUCK','C10','1'),('+5V_AB','J1','2'),
  ('+3V3_B','J4','1'),('NTC_RAW','J4','2'),('NTC_ADC','U4','4'),('PWM0','J7','2'),
  ('ENABLE','J7','3'),('FAULT_N','J7','4'),('GATE_DRV','R3','1'),('GND','J1','1')]
}


def write_tree(path,tree):
    path.write_text('('+tree[0]+'\n'+'\n'.join(s.dump(n) for n in tree[1:])+')\n',encoding='utf-8')


def schematic(name,rows):
    path=ROOT/name/(name+'.kicad_sch');tree=s.parse(path.read_text(encoding='utf-8'))
    assert not any('"TP1"'==prop[2] for node in s.children(tree,'symbol') for prop in s.children(node,'property') if s.unquote(prop[1])=='Reference')
    bottom=max(float(s.child(n,'at')[2]) for n in tree if isinstance(n,list) and n[0] in ('symbol','text') and s.child(n,'at'))
    base=math.ceil((bottom+24)/10)*10
    sheet=s.Sheet(name,'');sheet.section(20,base,'BENCH TEST PADS - BACK SIDE B.Cu - COPPER ONLY, DO NOT POPULATE')
    for i,row in enumerate(rows):
        sheet.add('Connector:TestPoint',row['ref'],row['net'],40+(i%6)*63,base+19+(i//6)*32,
                  {'1':row['net']},FP,status='COPPER_ONLY_BACK_SIDE_TEST_POINT')
    for value in sheet.items:
        node=s.parse(value)
        if node[0]=='symbol':
            for key,value_ in [('MPN',''),('Manufacturer',''),('Assembly','NONE_COPPER')]:
                node.append(s.parse(f'(property {s.q(key)} {s.q(value_)} (at 0 0 0) (effects (font (size 1 1)) hide))'))
        tree.append(node)
    libs=s.child(tree,'lib_symbols');existing={s.unquote(n[1]) for n in s.children(libs,'symbol')}
    libs.extend(v for k,v in sheet.libs.items() if k not in existing)
    s.child(tree,'paper')[:]=['paper',s.q('User'),'420',str(base+120)]
    s.child(s.child(tree,'title_block'),'rev')[1]=s.q('0.4.1' if name=='pcb_a_display' else '0.5.1')
    write_tree(path,tree)
    bompath=ROOT/name/(name+'_bom_draft.csv')
    with bompath.open(encoding='utf-8-sig',newline='') as f:reader=csv.DictReader(f);fields=reader.fieldnames;bom=list(reader)
    bom.extend(dict(Reference=r['ref'],Value=r['net'],Footprint=FP,Design_Status='COPPER_ONLY_BACK_SIDE_TEST_POINT',MPN='',Manufacturer='') for r in rows)
    with bompath.open('w',encoding='utf-8',newline='') as f:w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(bom)
    subprocess.run([CLI,'sch','export','netlist','--format','kicadxml','--output',str(ROOT/'review'/(name+'.net.xml')),str(path)],check=True,capture_output=True)


def seg_distance(x,y,a,b):
    dx,dy=b[0]-a[0],b[1]-a[1];den=dx*dx+dy*dy
    t=max(0,min(1,((x-a[0])*dx+(y-a[1])*dy)/den)) if den else 0
    return math.hypot(x-a[0]-t*dx,y-a[1]-t*dy)


def choose_position(anchor,label,net,data,used,spec):
    width,height=spec['board_mm'][:2]
    keepouts=spec['clip_bands_mm']+spec['side_support_bands_mm']+spec['clip_side_contact_patches_mm']
    if spec['name']=='pcb_a_display':keepouts=keepouts+[[0,17,12.5,46]]
    def overlap(a,b):return a[0]<b[2] and b[0]<a[2] and a[1]<b[3] and b[1]<a[3]
    candidates=[];textw=max(3,len(label)*.53)
    for ix in range(3,int(width-2)):
      for iy in range(5,int(height-5)):
        x,y=float(ix),float(iy);padbox=[x-1,y-1,x+1,y+1];text=[x-textw/2,y+1.2,x+textw/2,y+2.2]
        if any(overlap(b,[kx,ky,kx+kw,ky+kh]) for b in (padbox,text) for kx,ky,kw,kh in keepouts):continue
        if text[0]<1.6 or text[2]>width-1.6:continue
        if any(overlap(a,b) for a in (padbox,text) for b in used):continue
        clear=True
        for pad in data['pads']:
            if 1 not in pad['layers']:continue
            b=pad['bounds'];expanded=[b[0]-.35,b[1]-.35,b[2]+.35,b[3]+.35]
            # Even same-net copper must remain a separate probe landing.
            if overlap(padbox,expanded) or overlap(text,expanded):clear=False;break
        if not clear:continue
        if any(1 in t['layers'] and t['net']!=net and seg_distance(x,y,t['a'],t['b'])<1.13+t['width']/2 for t in data['tracks']):continue
        candidates.append((math.hypot(x-anchor[0],y-anchor[1]),x,y,padbox,text))
    assert candidates,('No probe location',label)
    _,x,y,padbox,text=min(candidates);used.extend([padbox,text]);return [x,y]


def refresh_placement():
    path=ROOT/'placement.json';data=json.loads(path.read_text(encoding='utf-8'))
    for spec in data['boards']:
        board=p.LoadBoard(str(ROOT/spec['name']/(spec['name']+'.kicad_pcb')))
        old={r['reference']:r for r in spec['components']};records=[]
        for fp in board.GetFootprints():
            ref=fp.GetReference();b=fp.GetBoundingBox(False,False);row=old.get(ref,{})
            row.update(reference=ref,value=fp.GetValue(),origin_mm=[round(p.ToMM(fp.GetPosition().x)-60,5),round(p.ToMM(fp.GetPosition().y)-55,5)],
              rotation_deg=fp.GetOrientationDegrees(),footprint=fp.GetFPID().GetUniStringLibId(),
              bounds_mm=[round(p.ToMM(b.GetLeft())-60,5),round(p.ToMM(b.GetTop())-55,5),round(p.ToMM(b.GetWidth()),5),round(p.ToMM(b.GetHeight()),5)])
            if ref.startswith('TP'):row.update(height_mm=0,status='Copper-only B.Cu/B.Mask probe pad; no purchased component')
            records.append(row)
        spec['components']=sorted(records,key=lambda r:r['reference'])
    data['revision']='A0.4.1 / B0.5.1 - back-side test pads; no power topology change'
    path.write_text(json.dumps(data,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')


def finish_style():
    data=json.loads((ROOT/'test_points.json').read_text(encoding='utf-8'))
    local_name='ProbePad_D1.5mm_NoPaste'
    proto=s.parse((Path(LIB)/'TestPoint_Pad_D1.5mm.kicad_mod').read_text(encoding='utf-8'))
    proto[1]=s.q(local_name)
    proto[:]=[node for node in proto if not (isinstance(node,list) and node[0].startswith('fp_') and s.child(node,'layer') and s.unquote(s.child(node,'layer')[1])=='F.SilkS')]
    (ROOT/'lab_hardware.pretty'/(local_name+'.kicad_mod')).write_text('(footprint '+proto[1]+'\n'+'\n'.join(s.dump(n) for n in proto[2:])+')\n',encoding='utf-8')
    selection=json.loads((ROOT/'manufacturing/component_selection.json').read_text(encoding='utf-8'))
    for name,rows in data['boards'].items():
        path=ROOT/name/(name+'.kicad_pcb');board=p.LoadBoard(str(path))
        for fp in board.GetFootprints():
            if not fp.GetReference().startswith('TP'):continue
            fp.SetFPID(p.LIB_ID('LabHardware',local_name))
            fp.SetField('Design_Status','COPPER_ONLY_BACK_SIDE_TEST_POINT')
            for field in fp.GetFields():
                if field.GetLayer() in (p.B_Fab,p.B_SilkS):field.SetMirrored(True)
                field.SetVisible(False)
            for item in list(fp.GraphicalItems()):
                if isinstance(item,p.PCB_SHAPE) and item.GetLayer()==p.B_SilkS:fp.Remove(item)
        for item in board.GetDrawings():
            if isinstance(item,p.PCB_TEXT) and item.GetLayer()==p.B_SilkS and item.GetText().startswith('TP'):
                item.SetText(item.GetText().split()[0])
        p.SaveBoard(str(path),board)
        sch=ROOT/name/(name+'.kicad_sch');tree=s.parse(sch.read_text(encoding='utf-8'))
        for node in s.children(tree,'symbol'):
            props={s.unquote(pr[1]):s.unquote(pr[2]) for pr in s.children(node,'property')}
            if props.get('Reference','').startswith('TP'):
                s.child(node,'in_bom')[1]='no'
                next(pr for pr in s.children(node,'property') if s.unquote(pr[1])=='Footprint')[2]=s.q('LabHardware:'+local_name)
        write_tree(sch,tree)
        for row in rows:
            row['label']=row['ref']
            selection['boards']['A' if name=='pcb_a_display' else 'B'][row['ref']]['Footprint']='LabHardware:'+local_name
        bp=ROOT/name/(name+'_bom_draft.csv')
        with bp.open(encoding='utf-8-sig',newline='') as f:reader=csv.DictReader(f);fields=reader.fieldnames;br=list(reader)
        for row in br:
            if row['Reference'].startswith('TP'):row['Footprint']='LabHardware:'+local_name
        with bp.open('w',encoding='utf-8',newline='') as f:w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(br)
    data['footprint']='LabHardware:'+local_name
    (ROOT/'manufacturing/component_selection.json').write_text(json.dumps(selection,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    (ROOT/'test_points.json').write_text(json.dumps(data,indent=2)+'\n',encoding='utf-8')
    refresh_placement()


def apply():
    backup=Path(tempfile.mkdtemp(prefix='lab-before-testpads-'));print('Original files:',backup,flush=True)
    layout=json.loads((ROOT/'placement.json').read_text(encoding='utf-8'))
    selection=json.loads((ROOT/'manufacturing/component_selection.json').read_text(encoding='utf-8'))
    result={'revision':'A0.4.1 / B0.5.1','footprint':FP,'side':'B.Cu / B.Mask, no paste','boards':{},'original_copper':{}}
    for name,items in SPECS.items():
        path=ROOT/name/(name+'.kicad_pcb');board=p.LoadBoard(str(path));before=copper_signature(board)
        assert not any(fp.GetReference().startswith('TP') for fp in board.GetFootprints()),'Already applied'
        result['original_copper'][name]=dict(PCB_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            UUIDs=sorted(before),signature_sha256=hashlib.sha256(json.dumps(before,sort_keys=True).encode()).hexdigest(),count=len(before))
        for suffix in ('.kicad_pcb','.kicad_sch','_bom_draft.csv'):shutil.copy2(ROOT/name/(name+suffix),backup/(name+suffix))
        copper=backup/(name+'_copper.json');native_extract(path,copper);data=json.loads(copper.read_text())
        nets={n.GetNetname():n for n in board.GetNetsByNetcode().values()};used=[];rows=[]
        spec=next(b for b in layout['boards'] if b['name']==name)
        for i,(net,anchorref,anchorpin) in enumerate(items,1):
            target=next(pad for pad in data['pads'] if pad['ref']==anchorref and pad['pin']==anchorpin)
            assert target['net']=='/'+net,(name,net,target)
            ref='TP'+str(i);label=ref+' '+net.replace('I2C_','').replace('VIN_PROTECTED','VIN24').replace('+5V_DEVKIT','VIN5').replace('+5V_BUCK','5VBUCK').replace('GATE_DRV','GATE')
            at=choose_position([target['x'],target['y']],label,'/'+net,data,used,spec)
            fp=p.FootprintLoad(LIB,'TestPoint_Pad_D1.5mm');fp.SetReference(ref);fp.SetValue(net);fp.SetPosition(pos(*at));board.Add(fp)
            fp.Flip(fp.GetPosition(),False);fp.SetFPID(p.LIB_ID('TestPoint','TestPoint_Pad_D1.5mm'))
            fp.SetPath(p.KIID_PATH('/'+s.uid(name)+'/'+s.uid(name+ref)))
            for field,value in [('MPN',''),('Manufacturer',''),('Assembly','NONE_COPPER')]:fp.SetField(field,value)
            fp.Reference().SetText(label);fp.Reference().SetTextSize(xy(.8,.8));fp.Reference().SetTextThickness(mm(.12))
            # Full ref+net text is a separate silk object; reference stays canonical.
            fp.Reference().SetText(ref);fp.Reference().SetVisible(False);fp.Value().SetVisible(False)
            text=p.PCB_TEXT(board);text.SetText(label);text.SetPosition(pos(at[0],at[1]+1.7));text.SetTextSize(xy(.8,.8));text.SetTextThickness(mm(.12))
            text.SetLayer(p.B_SilkS);text.SetMirrored(True);board.Add(text)
            for pad in fp.Pads():pad.SetNet(nets['/'+net])
            rows.append(dict(ref=ref,net=net,at_mm=at,label=label,anchor=[anchorref,anchorpin],assembly='NONE_COPPER'))
            code='A' if name=='pcb_a_display' else 'B'
            selection['boards'][code][ref]=dict(Value=net,Footprint=FP,MPN='',Manufacturer='',Requirements='Exposed1.5mm ENIG test pad on back; no paste, no fitted part',Source='',Assembly='NONE_COPPER',Decision='COPPER_ONLY_TEST_POINT')
        assert copper_signature(board)==before
        p.SaveBoard(str(path),board);schematic(name,rows);result['boards'][name]=rows
    (ROOT/'test_points.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    (ROOT/'manufacturing/component_selection.json').write_text(json.dumps(selection,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    finish_style();print('Pads added; route and DRC required. Original tracks preserved.')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--apply',action='store_true');parser.add_argument('--refresh-placement',action='store_true');parser.add_argument('--finish-style',action='store_true');args=parser.parse_args()
    if args.refresh_placement:refresh_placement()
    elif args.finish_style:finish_style()
    elif args.apply:apply()
    else:parser.error('Use --apply once or --refresh-placement after routing')
