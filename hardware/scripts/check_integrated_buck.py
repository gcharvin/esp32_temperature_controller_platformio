"""Bind the real PCB B netlist/geometry to the separately simulated buck study."""
import hashlib,json,re,subprocess
from pathlib import Path
import xml.etree.ElementTree as ET
import pcbnew as p

ROOT=Path(__file__).resolve().parents[1]
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def main():
 name='pcb_b_temperature';source=ROOT/name/(name+'.kicad_pcb');board=p.LoadBoard(str(source))
 xml=ET.parse(ROOT/'review'/f'{name}.net.xml').getroot()
 comps={v.attrib['ref']:v for v in xml.findall('./components/comp')}
 pins={(n.attrib['ref'],n.attrib['pin']):net.attrib['name'].removeprefix('/') for net in xml.findall('./nets/net') for n in net.findall('node')}
 assert comps['U1'].findtext('value')=='LM2596S-5.0/NOPB'
 assert 'J8' not in comps and 'J9' not in comps
 for ref,pin,net in [('U1','1','VIN_PROTECTED'),('U1','2','BUCK_SW'),('U1','3','PGND'),('U1','4','+5V_BUCK'),('U1','5','PGND'),('NT1','1','PGND'),('NT1','2','GND')]:assert pins[ref,pin]==net
 assert comps['C8'].findtext('value').startswith('100uF')
 assert comps['C9'].findtext('value').startswith('100nF')
 assert comps['L1'].findtext('value').startswith('100uH')
 assert comps['C10'].findtext('value').startswith('330uF')
 assert comps['R13'].findtext('value').startswith('50m')
 assert comps['R13'].findtext('./fields/field[@name="MPN"]')=='WSL1206R0500FEA'
 assert comps['C10'].findtext('./fields/field[@name="MPN"]')=='EEUFS1V331LB'
 assert {k for k,n in pins.items() if n=='BUCK_CAP_RETURN'}=={('C10','2'),('R13','1')}
 fps={f.GetReference():f for f in board.GetFootprints()}
 # Connector origins are the v0.4 mechanical contract, in native mm.
 for ref,expected in {'J1':(75,110),'J2':(69,67),'J3':(135,67),'J4':(141,107),'J6':(93,110),'J7':(113,110)}.items():
  pt=fps[ref].GetPosition();assert (p.ToMM(pt.x),p.ToMM(pt.y))==expected
 assert '1,2' in source.read_text() or 'net_tie_pad_groups' in source.read_text()
 study=ROOT/'power_study/lm2596'
 binding_path=study/'current_acceptance.json';binding=json.loads(binding_path.read_text())
 assert binding['C10']=='EEUFS1V331LB' and binding['R13']=='WSL1206R0500FEA' and binding['R13_ohm']==.05
 for relative,digest in binding['source_sha256'].items():assert sha(ROOT/relative)==digest,'Stale buck qualification '+relative
 planpath=ROOT/binding['plan'];plan=json.loads(planpath.read_text())
 resultpath=ROOT/binding['results'];results=json.loads(resultpath.read_text())
 assert results['all_criteria_pass'] and len(results['cases'])==7
 assert plan['external_components']['Cout_added_series_resistance_ohm']==.05
 assert plan['external_components']['Cout_lower_series_bound_ohm']==.049
 for case in results['cases']:
  assert sha(planpath.parent/case['deck'])==case['deck_sha256']
  assert case['all_criteria_pass'] and case['inputs_hashed_at_run_start']
 # New startup core adds the actual local bypass. ESR/DCR are modeled losses,
 # not extra BOM parts. Upstream/PCB parasitics are deliberately not included.
 deck=study/'benches/integrated_B_startup.cir'
 base=(study/'benches/startup.cir').read_text()
 deck.write_text(base.replace('LM2596 fixed 5 V - startup', 'LM2596 core from PCB B v0.5 exported netlist - startup')
                 .replace('CIN vin cinret 100u', 'CIN vin cinret '+comps['C8'].findtext('value').split('F')[0])
                 .replace('COUT vout esr 330u', 'COUT vout esr '+comps['C10'].findtext('value').split('F')[0])
                 .replace('L1 sw lind 100u', 'L1 sw lind '+comps['L1'].findtext('value').split('H')[0])
                 .replace('.options ', 'CLOCAL vin 0 '+comps['C9'].findtext('value').split('F')[0]+'\n.options '),encoding='utf-8')
 report=dict(revision='PCB B 0.5.3 integrated LM2596 stock alternatives',
   source_sha256={str(path.relative_to(ROOT)):sha(path) for path in [source,source.with_suffix('.kicad_sch'),ROOT/'review'/f'{name}.net.xml',binding_path,planpath,resultpath]},
   dimensions_mm=[90,65,1.6],copper_um=70,connector_positions_preserved=True,
   external_buck_removed=True,ground_star='NT1 copper only, PGND pin1 / logic GND pin2',
   output_budget_A=.8,fuse_F2_A=1,acceptance_study_cases_passed=7,
   core_deck=str(deck.relative_to(ROOT)),core_deck_sha256=sha(deck),
   limits=['Core simulation excludes PCB/cable parasitics, upstream protection, thermal and heater coupling.',
           'Capacitor maximum impedance41mOhm plus physical R13 50mOhm approximated conservatively as100mOhm total; separate49mOhm lower and180mOhm upper stress cases passed.',
           'Current STEP fit checked separately against B v0.5; real cable, clip and thermal tests pending.',
           'R13/C9 purchasing candidates documented; R13 manufacturer land pattern checked separately. Supplier approval and physical startup/thermal/fault tests pending.'])
 (ROOT/'review/routing/integrated_buck_validation.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
 print(json.dumps(report,indent=2))

if __name__=='__main__':main()
