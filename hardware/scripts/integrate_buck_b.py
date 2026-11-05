"""One-shot, staged LM2596 integration. Never touches PCB A.

Run with KiCad Python and --workdir; inspect staged ERC/DRC before publishing.
Native v0.4 files are the input, not the historical bootstrap generators.
"""
import argparse, copy, csv, json, subprocess, shutil
from pathlib import Path
import xml.etree.ElementTree as ET
import pcbnew as p
import create_drafts as s
from create_placement import pos,xy,mm,text

ROOT=Path(__file__).resolve().parents[1]
CLI='C:/Program Files/KiCad/10.0/bin/kicad-cli.exe'
LIB=Path('C:/Program Files/KiCad/10.0/share/kicad/footprints')
NAME='pcb_b_temperature'
SPECS=[
 ('Regulator_Switching:LM2596S-5','U1','LM2596S-5.0/NOPB',70,425,{'1':'VIN_PROTECTED','2':'BUCK_SW','3':'PGND','4':'+5V_BUCK','5':'PGND'},'Package_TO_SOT_SMD:TO-263-5_TabPin3','LM2596S-5.0/NOPB'),
 ('Device:L','L1','100uH / SRP1770TA-101M',150,425,{'1':'BUCK_SW','2':'+5V_BUCK'},'Inductor_SMD:L_Bourns_SRP1770TA_16.9x16.9mm','SRP1770TA-101M'),
 ('Device:D_Schottky','D9','SS36 / 60V 3A',225,425,{'1':'BUCK_SW','2':'PGND'},'Diode_SMD:D_SMC','SS36-E3/57T'),
 ('Device:C','C9','100nF / 50V X7R',290,425,{'1':'VIN_PROTECTED','2':'PGND'},'Capacitor_SMD:C_0805_2012Metric',''),
 ('Device:C_Polarized','C10','330uF / 35V',350,425,{'1':'+5V_BUCK','2':'BUCK_CAP_RETURN'},'Capacitor_THT:CP_Radial_D10.0mm_P5.00mm','EEUFR1V331UB'),
 ('Device:R','R13','56m / 1% / 0.25W',290,475,{'1':'BUCK_CAP_RETURN','2':'PGND'},'Resistor_SMD:R_1206_3216Metric',''),
 ('Device:NetTie_2','NT1','GND STAR / NO COMPONENT',90,475,{'1':'PGND','2':'GND'},'NetTie:NetTie-2_SMD_Pad2.0mm',''),
]
PLACEMENT={
 'U1':(18,35,180),'L1':(39.5,41.5,0),'D9':(33,25,90),'C8':(17,24,0),
 'C9':(26.5,42.8,90),'C10':(10,46.5,0),'R13':(20.5,45,0),'NT1':(8,31,0),
 'F2':(25,48,0),'U2':(52,44,0),'C1':(53,40,0),'C2':(54.5,48.5,0),
 'U3':(62,46,0),'C3':(65,42,0),'R1':(57,46,0),'R2':(59,49,0),'R8':(39,28,0),
 'D5':(5,43,90),'R9':(5,38,90),'D6':(52,31.5,90),'R10':(52,26.5,90),
 'D7':(72,49,90),'R11':(75,49,0),
}

def write_tree(path,tree):
 path.write_text('('+tree[0]+'\n'+'\n'.join(s.dump(n) for n in tree[1:])+')\n',encoding='utf-8')

def stage(work):
 assert 'LM2596S-5.0/NOPB' not in (ROOT/NAME/(NAME+'.kicad_sch')).read_text(), 'Refuse reapplying migration to integrated native schematic'
 work.mkdir(parents=True,exist_ok=True)
 target=work/NAME;target.mkdir(exist_ok=True)
 shutil.copy2(ROOT/NAME/'sym-lib-table',target/'sym-lib-table')
 shutil.copy2(ROOT/'lab_draft.kicad_sym',work/'lab_draft.kicad_sym')
 for ext in ['kicad_sch','kicad_pcb','kicad_pro']:
  shutil.copy2(ROOT/NAME/(NAME+'.'+ext),target/(NAME+'.'+ext))
 source=target/(NAME+'.kicad_sch');tree=s.parse(source.read_text())
 oldrefs={'U1','J8','J9'}
 ids=set()
 for ref in oldrefs:
  ids.add(s.uid(NAME+ref))
  for pin in range(1,6):
   for suffix in ['wire','label','NC']:ids.add(s.uid(NAME+ref+str(pin)+suffix))
 def keep(n):
  if not isinstance(n,list):return True
  uuid=s.child(n,'uuid')
  if uuid and s.unquote(uuid[1]) in ids:return False
  if n[0]=='symbol':
   props={s.unquote(v[1]):s.unquote(v[2]) for v in s.children(n,'property')}
   if props.get('Reference') in oldrefs:return False
  return True
 tree=[n for n in tree if keep(n)]
 for n in s.children(tree,'text'):
  if any(k in s.unquote(n[1]) for k in ['XL4015','shunt','Buck module']):
   n[1]=s.q('Integrated LM2596 fixed 5V. NT1 joins PGND/GND at input star; heater return stays separate.\nF2 remains 1A. USB source sharing and thermal/fault qualification still pending.')
 sheet=s.Sheet(NAME,'')
 sheet.section(20,401,'B5 - INTEGRATED BUCK / FIXED 5V / SYSTEM BUDGET 0.8A')
 for lib,ref,value,x,y,nets,fp,mpn in SPECS:
  sheet.add(lib,ref,value,x,y,nets,fp,status='Prototype 0.5 integrated buck; thermal and fault qualification pending')
  if mpn:
   n=s.parse(sheet.items[-1]);n.append(s.parse(f'(property "MPN" {s.q(mpn)} (at {x} {y} 0) (effects (font (size 1 1)) hide))'));sheet.items[-1]=s.dump(n)
 sheet.text(20,503,'C8 is the local 100uF/50V input reservoir; C9 is HF bypass. C10 negative returns through R13.\nLM2596 tab/pin 3 = PGND. Pin 5 hard-wired low (buck always enabled with 24V present).\nNT1 is a copper net tie, not an isolation barrier. No external XL4015 or J8/J9.\nMatched electrical study: hardware/power_study/lm2596. Physical startup, fault and thermal tests remain required.',1)
 libs=s.child(tree,'lib_symbols');present={s.unquote(n[1]) for n in s.children(libs,'symbol')}
 libs.extend(v for k,v in sheet.libs.items() if k not in present)
 tree.extend(s.parse(i) for i in sheet.items)
 s.child(tree,'paper')[:]=['paper',s.q('User'),'420','530']
 s.child(s.child(tree,'title_block'),'rev')[1]=s.q('0.5 INTEGRATED BUCK')
 write_tree(source,tree)
 for args in [('sch','export','netlist','--format','kicadxml','--output',str(work/'B.net.xml')),('sch','export','pdf','--output',str(work/'B.pdf')),('sch','erc','--output',str(work/'B_erc.rpt'),'--exit-code-violations')]:
  subprocess.run([CLI,*args,str(source)],check=True)
 xml=ET.parse(work/'B.net.xml').getroot();comps={c.attrib['ref']:c for c in xml.findall('./components/comp')}
 pins={(n.attrib['ref'],n.attrib['pin']):net.attrib['name'] for net in xml.findall('./nets/net') for n in net.findall('node')}
 path=target/(NAME+'.kicad_pcb');bt=s.parse(path.read_text())
 def keep_board(n):
  if not isinstance(n,list):return True
  if n[0]=='footprint':
   props={s.unquote(v[1]):s.unquote(v[2]) for v in s.children(n,'property')}
   return props.get('Reference') not in ['J8','J9']
  if n[0] in ['segment','via']:
   net=s.unquote(s.child(n,'net')[1])
   preserve=net in ['/VIN_RAW','/VIN_FUSED','/VIN_PROTECTED','/HEATER_LOW']
   if net=='/PGND' and n[0]=='segment' and float(s.child(n,'width')[1])>=2.39:preserve=True
   if net=='/VIN_PROTECTED' and n[0]=='segment' and (min(float(s.child(n,k)[2]) for k in ['start','end'])>=84 or min(float(s.child(n,k)[1]) for k in ['start','end'])<99.84):preserve=False
   return preserve
  if n[0]=='zone':return s.child(n,'keepout') is not None
  if n[0]=='gr_text':
   val=s.unquote(n[1]);layer=s.unquote(s.child(n,'layer')[1])
   if any(k in val for k in ['XL4015','BUCK IN','BUCK OUT','0.3','0.4']):return False
   if layer=='F.SilkS' and val in ['24V','5V','3V3']:return False
  return True
 bt=[n for n in bt if keep_board(n)];write_tree(path,bt)
 board=p.LoadBoard(str(path))
 nets={n.GetNetname():n for n in board.GetNetsByNetcode().values()}
 for name in set(pins.values())-set(nets):
  n=p.NETINFO_ITEM(board,name);board.Add(n);nets[name]=n
 fps={f.GetReference():f for f in board.GetFootprints()}
 for ref,comp in comps.items():
  if ref not in fps:
   nick,part=comp.findtext('footprint').split(':');fp=p.FootprintLoad(str(LIB/(nick+'.pretty')),part)
   assert fp is not None,(ref,nick,part)
   fp.SetFPID(p.LIB_ID(nick,part));fp.SetReference(ref);board.Add(fp);fps[ref]=fp
  fp=fps[ref];fp.SetValue(comp.findtext('value'))
  fp.SetPath(p.KIID_PATH('/'+s.uid(NAME)+'/'+comp.findtext('tstamps')))
  fp.SetSheetname(NAME);fp.SetSheetfile(NAME+'.kicad_sch')
  if ref in PLACEMENT:
   x,y,a=PLACEMENT[ref];fp.SetPosition(pos(x,y));fp.SetOrientationDegrees(a)
  fp.Value().SetVisible(False);fp.Reference().SetTextSize(xy(.8,.8));fp.Reference().SetTextThickness(mm(.12))
  for field in comp.findall('./fields/field'):
   if field.attrib['name'] not in ['Footprint','Description']:
    fp.SetField(field.attrib['name'],field.text or '');fp.GetField(field.attrib['name']).SetVisible(False)
  for pad in fp.Pads():
   if pad.GetNumber():pad.SetNet(nets[pins[(ref,pad.GetNumber())]])
 text(board,'PCB B / 0.5 INTEGRATED LM2596 / 24V 100W HEATER',0,-16,1.2)
 text(board,'5V BUCK ON BOARD - NO XL4015 / NO J8 J9',4,76,1)
 # Remove outdated LED labels, then put diagnostics near their new footprints.
 for label,x,y in [('24V',3,46),('5V',51,35),('3V3',70,53.5)]:text(board,label,x,y,.8,p.F_SilkS)
 for ref,x,y in [('D5',3,42),('D7',72,46.5),('C3',66,43.5),('C10',8,40.5)]:
  fps[ref].Reference().SetPosition(pos(x,y));fps[ref].Reference().SetTextAngle(p.EDA_ANGLE(0,p.DEGREES_T))
 critical(board,fps,nets)
 p.SaveBoard(str(path),board)
 shutil.copy2(path,work/'pre_router.kicad_pcb')
 # Persist BOM from the authoritative netlist, not historical module selections.
 with (target/(NAME+'_bom_draft.csv')).open('w',newline='',encoding='utf-8') as f:
  w=csv.DictWriter(f,fieldnames=['Reference','Value','Footprint','Design_Status','MPN']);w.writeheader()
  for ref,c in comps.items():
   fields={v.attrib['name']:v.text or '' for v in c.findall('./fields/field')}
   w.writerow(dict(Reference=ref,Value=c.findtext('value'),Footprint=c.findtext('footprint'),Design_Status=fields.get('Design_Status',''),MPN=fields.get('MPN','')))
 print('Staged integration:',target)

def critical(board,fps,nets):
 from build_routing_boards import track
 # Front-side switching loop. Only connections between real assigned pads.
 def at(ref,pin):
  pad=next(pad for pad in fps[ref].Pads() if pad.GetNumber()==str(pin))
  return (p.ToMM(pad.GetPosition().x)-60,p.ToMM(pad.GetPosition().y)-55)
 for net,points,width,layer in [
  ('/BUCK_SW',[at('U1',2),(30.5,36.7),(31.825,38.025),at('L1',1)],1.2,p.F_Cu),
  ('/BUCK_SW',[at('D9',1),(30.5,30.9),(30.5,36.7)],1.2,p.F_Cu),
  ('/PGND',[at('D9',2),(31,20.5),(24,20.5),at('C8',2)],1.2,p.F_Cu),
  ('/PGND',[at('C8',2),(20.5,28),(18,30),(16.5,35)],1.2,p.F_Cu),
  ('/PGND',[at('NT1',1),(6,28)],1.2,p.F_Cu),
  ('/PGND',[(6,28),(5,27),(5,21),(9,21)],1.2,p.B_Cu),
 ]:track(board,nets[net],points,width,layer)
 via=p.PCB_VIA(board);via.SetPosition(pos(6,28));via.SetWidth(mm(1.6));via.SetDrill(mm(.8));via.SetLayerPair(p.F_Cu,p.B_Cu);via.SetNet(nets['/PGND']);board.Add(via)
 # Keep converter current local; heater return remains the original wide spine.
 from finalize_copper import zone
 zone(board,'BUCK_PGND_THERMAL','/PGND',p.F_Cu,(10,30,18,11),False)
 zone(board,'LOGIC_GND_BOTTOM','/GND',p.B_Cu,(1.8,4.3,86.4,56.4))
 zone(board,'Q2_DRAIN_HEAT_SPREAD','/VIN_FUSED',p.F_Cu,(20,5,32,16),False)

if __name__=='__main__':
 a=argparse.ArgumentParser();a.add_argument('--workdir',type=Path,required=True);args=a.parse_args();stage(args.workdir.resolve())

