"""One-shot incremental revision of the routed prototype: six diagnostic LEDs.

Run with KiCad Python. Existing native schematics/footprints/tracks are preserved.
Refuses a second application. New branches are routed separately, then DRC checked.
"""
import argparse
import csv
import json
from pathlib import Path
import shutil
import tempfile
import xml.etree.ElementTree as ET
import subprocess
import pcbnew as p
import create_drafts as s
from create_placement import pos, xy, mm, text

ROOT=Path(__file__).resolve().parents[1]
LIB=Path('C:/Program Files/KiCad/10.0/share/kicad/footprints')
CLI='C:/Program Files/KiCad/10.0/bin/kicad-cli.exe'
LED_FP='LED_SMD:LED_0805_2012Metric_Pad1.15x1.40mm_HandSolder'
GREEN='APT2012LZGCK'
ORANGE='APT2012LSECK/J4-PRV'
SPECS={
 'pcb_a_display':[
  dict(led='D1',resistor='R6',label='5V_AB',rail='+5V_AB',ground='GND',ohms=1500,voltage=5,
       led_at=[99,55,0],resistor_at=[94,55,0]),
  dict(led='D2',resistor='R7',label='3V3_A',rail='+3V3_A',ground='GND',ohms=470,voltage=3.3,
       led_at=[99,64,0],resistor_at=[94,64,0])],
 'pcb_b_temperature':[
  dict(led='D5',resistor='R9',label='24V',rail='VIN_PROTECTED',ground='PGND',ohms=15000,voltage=24,
       led_at=[22,37,0],resistor_at=[22,32,270],label_at=[17.2,32]),
  dict(led='D6',resistor='R10',label='5V',rail='+5V_AB',ground='GND',ohms=1500,voltage=5,
       led_at=[28,37,0],resistor_at=[28,32,270]),
  dict(led='D7',resistor='R11',label='3V3',rail='+3V3_B',ground='GND',ohms=470,voltage=3.3,
       led_at=[45,34,0],resistor_at=[45,30,270],label_at=[42,32]),
  dict(led='D8',resistor='R12',label='HEAT',rail='GATE_DRV',ground='PGND',ohms=2200,voltage=5,
       led_at=[66,39,0],resistor_at=[66,35,270])]
}
for board,rows in SPECS.items():
 for row in rows:
  row['color']='orange' if row['label']=='HEAT' else 'green'
  row['mpn']=ORANGE if row['color']=='orange' else GREEN
  row['vf_typ_V']=1.8 if row['color']=='orange' else 2.65
  row['net']='LED_'+('A_' if board=='pcb_a_display' else 'B_')+row['led']
  row['resistor_fp']='Resistor_SMD:R_'+('0805_2012Metric' if row['voltage']==24 else '0603_1608Metric')
  row['resistor_value']={1500:'1.5k',470:'470R',15000:'15k',2200:'2.2k'}[row['ohms']]
  row['datasheet']='https://www.kingbrightusa.com/images/catalog/spec/'+('apt2012lseck-j4-prv.pdf' if row['color']=='orange' else 'apt2012lzgck.pdf')

def write_tree(path,tree):
 path.write_text('('+tree[0]+'\n'+'\n'.join(s.dump(n) for n in tree[1:])+')\n',encoding='utf-8')

def schematic(name,rows):
 path=ROOT/name/(name+'.kicad_sch');tree=s.parse(path.read_text(encoding='utf-8'))
 existing={s.unquote(v[2]) for n in s.children(tree,'symbol') for v in s.children(n,'property') if s.unquote(v[1])=='Reference'}
 assert not existing.intersection({r[k] for r in rows for k in ('led','resistor')}),'Diagnostic revision already present'
 sheet=s.Sheet(name,'')
 sheet.section(20,307,'DIAGNOSTICS - RAIL PRESENCE AND HEATER GATE DRIVE')
 for index,row in enumerate(rows):
  x=50.8+index*66.04
  sheet.text(x-18,317,row['led']+' / '+row['label'],1.4)
  sheet.add('Device:R',row['resistor'],row['resistor_value']+' / 1% / '+('0.125W' if row['voltage']==24 else '0.1W'),x,332.74,
            {'1':row['rail'],'2':row['net']},row['resistor_fp'],status='Diagnostic current limit')
  sheet.add('Device:LED',row['led'],row['mpn'],x,355.6,{'1':row['ground'],'2':row['net']},LED_FP,
            status='Diagnostic '+row['color']+' / 0805 / low-current',rotation=90)
  # A continuous series chain is easier to read than two repeated net stubs.
  remove={s.uid(name+ref+pin+suffix) for ref,pin in [(row['resistor'],'2'),(row['led'],'2')] for suffix in ('wire','label')}
  sheet.items=[v for v in sheet.items if s.unquote(s.child(s.parse(v),'uuid')[1]) not in remove]
  sheet.items.append(f'(wire (pts (xy {x} 336.55) (xy {x} 351.79)) (stroke (width 0) (type default)) (uuid {s.uid(name+row["net"]+"series")}))')
  sheet.items.append(f'(label {s.q(row["net"])} (at {x} 344.17 0) (effects (font (size 1.016 1.016)) (justify left bottom)) (uuid {s.uid(name+row["net"]+"series-label")}))')
  for value in sheet.items:
   n=s.parse(value)
   if n[0]!='symbol':continue
   props={s.unquote(v[1]):v for v in s.children(n,'property')}
   if s.unquote(props['Reference'][2])!=row['led']:continue
   for key,dy in [('Reference',-2.54),('Value',0)]:
    s.child(props[key],'at')[1:]=[str(x+5.08),str(355.6+dy),'90']
    s.child(s.child(props[key],'effects'),'justify')[1]='right'
   props['Datasheet'][2]=s.q(row['datasheet'])
   extra=s.parse(s.dump(props['Design_Status']));extra[1:3]=[s.q('MPN'),s.q(row['mpn'])];n.append(extra)
   sheet.items[sheet.items.index(value)]=s.dump(n)
 sheet.text(20,377,'Green LEDs show rail presence only, not regulated voltage accuracy or ESP32 execution.\nHEAT (B/D8): 5 V driver output after PWM AND ENABLE; brightness follows duty, NOT measured heater current.\nNTC on B/J4: 1 = +3V3_B, 2 = NTC_RAW; passive two-wire 15k B3920 probe, either wire orientation.',1.1)
 libs=s.child(tree,'lib_symbols');present={s.unquote(n[1]) for n in s.children(libs,'symbol')}
 libs.extend(v for k,v in sheet.libs.items() if k not in present)
 s.child(tree,'paper')[:]=['paper',s.q('User'),'420','390']
 s.child(s.child(tree,'title_block'),'rev')[1]=s.q('0.4')
 tree.extend(s.parse(v) for v in sheet.items)
 if name=='pcb_b_temperature':
  for node in s.children(tree,'symbol'):
   props={s.unquote(v[1]):v for v in s.children(node,'property')}
   if s.unquote(props['Reference'][2])=='J4':props['Value'][2]=s.q('TEMP / NTC 15k B3920')
 write_tree(path,tree)
 bompath=ROOT/name/(name+'_bom_draft.csv')
 with bompath.open(encoding='utf-8',newline='') as f:
  reader=csv.DictReader(f);fields=reader.fieldnames;bom=list(reader)
 if 'MPN' not in fields:fields.append('MPN')
 for row in sheet.bom:
  row['MPN']=next((r['mpn'] for r in rows if r['led']==row['Reference']),'')
  bom.append(row)
 for row in bom:
  if name=='pcb_b_temperature' and row['Reference']=='J4':row['Value']='TEMP / NTC 15k B3920'
 with bompath.open('w',encoding='utf-8',newline='') as f:
  writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader();writer.writerows(bom)
 subprocess.run([CLI,'sch','export','netlist','--format','kicadxml','--output',str(ROOT/'review'/(name+'.net.xml')),str(path)],check=True)

def board(name,rows):
 path=ROOT/name/(name+'.kicad_pcb');board=p.LoadBoard(str(path))
 comps={v.attrib['ref']:v for v in ET.parse(ROOT/'review'/(name+'.net.xml')).getroot().findall('./components/comp')}
 xml=ET.parse(ROOT/'review'/(name+'.net.xml')).getroot()
 nets={n.GetNetname():n for n in board.GetNetsByNetcode().values()}
 pin_nets={}
 for net in xml.findall('./nets/net'):
  name_=net.attrib['name']
  if name_ not in nets:
   nets[name_]=p.NETINFO_ITEM(board,name_);board.Add(nets[name_])
  for n in net.findall('node'):pin_nets[(n.attrib['ref'],n.attrib['pin'])]=nets[name_]
 for row in rows:
  for kind in ('led','resistor'):
   ref=row[kind];comp=comps[ref];libid=comp.findtext('footprint');nick,part=libid.split(':')
   fp=p.FootprintLoad(str(LIB/(nick+'.pretty')),part);assert fp is not None
   fp.SetFPID(p.LIB_ID(nick,part));fp.SetReference(ref);fp.SetValue(comp.findtext('value'))
   x,y,angle=row[kind+'_at'];fp.SetPosition(pos(x,y));fp.SetOrientationDegrees(angle)
   fp.SetPath(p.KIID_PATH('/'+s.uid(name)+'/'+comp.findtext('tstamps')))
   fp.SetSheetname(name);fp.SetSheetfile(name+'.kicad_sch');board.Add(fp)
   fp.Value().SetVisible(False);fp.Reference().SetTextSize(xy(.8,.8));fp.Reference().SetTextThickness(mm(.12));fp.Reference().SetLayer(p.F_SilkS)
   fp.Reference().SetTextAngle(p.EDA_ANGLE(0,p.DEGREES_T));fp.Reference().SetPosition(pos(x+2.8,y) if angle else pos(x,y+1.8))
   for field in comp.findall('./fields/field'):
    if field.attrib['name'] not in ('Footprint','Description'):
     fp.SetField(field.attrib['name'],field.text or '');fp.GetField(field.attrib['name']).SetVisible(False)
   for pad in fp.Pads():pad.SetNet(pin_nets[(ref,pad.GetNumber())])
  lx,ly,_=row['led_at']
  tx,ty=row.get('label_at',[lx-1.9,ly-(7.4 if name=='pcb_b_temperature' else 2.8)])
  text(board,row['label'],tx,ty,.8,p.F_SilkS)
 if name=='pcb_b_temperature':
  next(fp for fp in board.GetFootprints() if fp.GetReference()=='F2').Reference().SetPosition(pos(20,44.8))
  for fp in board.GetFootprints():
   if fp.GetReference()=='J4':
    fp.SetValue(comps['J4'].findtext('value'));fp.Reference().SetPosition(pos(87.5,52))
  for obj in board.GetDrawings():
   if isinstance(obj,p.PCB_TEXT) and obj.GetText()=='NTC':
    obj.SetText('TEMP / NTC 15k');obj.SetPosition(pos(75,48));obj.SetTextSize(xy(.8,.8))
  text(board,'1=3V3  2=NTC',75,60.5,.8,p.F_SilkS)
 for obj in board.GetDrawings():
  if isinstance(obj,p.PCB_TEXT) and 'ROUTING PROTOTYPE 0.3' in obj.GetText():obj.SetText(obj.GetText().replace('0.3','0.4 DIAGNOSTICS'))
 p.SaveBoard(str(path),board)

def placement():
 path=ROOT/'placement.json';data=json.loads(path.read_text(encoding='utf-8'))
 for spec in data['boards']:
  name=spec['name'];board=p.LoadBoard(str(ROOT/name/(name+'.kicad_pcb')))
  old={row['reference']:row for row in spec['components']};records=[]
  for fp in board.GetFootprints():
   ref=fp.GetReference();bounds=fp.GetBoundingBox(False,False)
   row=old.get(ref,{})
   row.update(reference=ref,value=fp.GetValue(),origin_mm=[round(p.ToMM(fp.GetPosition().x)-60,4),round(p.ToMM(fp.GetPosition().y)-55,4)],
              rotation_deg=fp.GetOrientationDegrees(),footprint=fp.GetFPID().GetUniStringLibId(),
              bounds_mm=[round(p.ToMM(v),4) for v in (bounds.GetLeft()-mm(60),bounds.GetTop()-mm(55),bounds.GetWidth(),bounds.GetHeight())])
   if ref in {r[k] for r in SPECS[name] for k in ('led','resistor')}:row.update(height_mm=.75 if ref.startswith('D') else .6,status='diagnostic component / physical package selected')
   records.append(row)
  spec['components']=sorted(records,key=lambda v:v['reference'])
 data['revision']='routed prototype 0.4 diagnostics / 2026-11-05'
 path.write_text(json.dumps(data,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')

def main():
 parser=argparse.ArgumentParser();parser.add_argument('--refresh-placement',action='store_true');args=parser.parse_args()
 if args.refresh_placement:placement();return
 backup=Path(tempfile.mkdtemp(prefix='lab-pcb-before-diagnostics-'))
 for name in SPECS:
  for suffix in ('.kicad_sch','.kicad_pcb','_bom_draft.csv'):
   shutil.copy2(ROOT/name/(name+suffix),backup/(name+suffix))
 print('Original native files saved at',backup,flush=True)
 for name,rows in SPECS.items():schematic(name,rows);board(name,rows)
 (ROOT/'diagnostic_leds.json').write_text(json.dumps({'revision':'0.4','boards':SPECS},indent=2)+'\n',encoding='utf-8')
 placement()
 print('Six series-limited LEDs added; original routes preserved. Route new branches and refill zones before DRC.')

if __name__=='__main__':main()
