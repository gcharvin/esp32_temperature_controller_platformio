"""Build a standalone KiCad buck proposal and documented vendor-model benches.

This never edits the routed production-prototype A/B files.
Run with the installed KiCad Python. Schematics and SPICE decks share this spec.
"""
import copy
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import urllib.request
import zipfile

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/'scripts'))
import create_drafts as s

CLI='C:/Program Files/KiCad/10.0/bin/kicad-cli.exe'
VENDOR_URL='https://www.ti.com/lit/zip/snvmbx7'
MODEL='vendor_model/LMR36520A_PSPICE_TRANS/LMR36520A_PSPICE_TRANS/LMR36520A_TRANS.LIB'

def model_download():
    archive=HERE/'lmr36520a_vendor_model.zip'
    if not archive.exists():archive.write_bytes(urllib.request.urlopen(VENDOR_URL).read())
    assert hashlib.sha256(archive.read_bytes()).hexdigest()=='ee5a5dd82e0e97264d8e87e0f177fe0c5aed5941b60c926d4e9b88cdf792f1e1','Manufacturer archive changed; review the model before updating its checksum'
    with zipfile.ZipFile(archive) as z:
        assert z.testzip() is None
        for name in z.namelist():
            dest=(HERE/'vendor_model'/name).resolve()
            assert dest.is_relative_to((HERE/'vendor_model').resolve())
        z.extractall(HERE/'vendor_model')
    model=HERE/MODEL
    assert '.SUBCKT LMR36520A_TRANS AGND BOOT EN FB PG PGND SW VCC VIN' in model.read_text()
    return {'download_url':VENDOR_URL,'archive_sha256':hashlib.sha256(archive.read_bytes()).hexdigest(),
            'model_sha256':hashlib.sha256(model.read_bytes()).hexdigest(),'model_relative_path':MODEL,
            'model_version':'TI Final 1.10, 12FEB2020; original, unmodified',
            'model_temperature_effects':False}

def proposal():
    sheet=s.Sheet('buck_5v','STUDY ONLY - Integrated 24V to 5V buck for PCB B')
    # The family is pin-compatible according to TI. Keep an explicit local
    # symbol with the selected part's name, datasheet and thermal pad number.
    sym=s.native('Regulator_Switching','LMR33620ADDA')
    sym[1]=s.q('Study:LMR36520ADDAR')
    for sub in s.children(sym,'symbol'):
        sub[1]=s.q(s.unquote(sub[1]).replace('LMR33620ADDA','LMR36520ADDAR'))
    props={s.unquote(v[1]):v for v in s.children(sym,'property')}
    props['Value'][2]=s.q('LMR36520ADDAR')
    props['Datasheet'][2]=s.q('https://www.ti.com/lit/ds/symlink/lmr36520.pdf')
    sheet.libs['Study:LMR36520ADDAR']=sym
    s.CUSTOM['LMR36520ADDAR']=sym
    sheet.section(20,18,'POWER STUDY 0.1 - 5V / 0.8A SYSTEM TARGET, 2A CONVERTER RATING')
    sheet.text(20,27,'Proposal only: not installed or routed on PCB B v0.4. Read README.md before integration.\nProtected 24V input. Common ground proposal replaces the external CC/CV module and its possible shunt.',1.1)
    sheet.connector('JIN','FROM B / protected input',38,57,['VIN_PROTECTED','GND'],2)
    for ref,x,val in [('CIN1',90,'4.7uF / 100V X7S'),('CIN2',140,'4.7uF / 100V X7S'),('CHF',190,'220nF / 100V X7R')]:
        sheet.add('Device:C',ref,val,x,57,{'1':'VIN_PROTECTED','2':'GND'},'Capacitor_SMD:C_1210_3225Metric' if ref!='CHF' else 'Capacitor_SMD:C_0805_2012Metric')
    # Use the native symbol through Sheet's custom-library path.
    sheet.add('Lab:LMR36520ADDAR','U_BUCK','LMR36520ADDAR',102,123,
              {'1':'GND','2':'VIN_PROTECTED','3':'BUCK_EN','4':'PG_5V_TP','5':'FB','6':'VCC_INTERNAL','7':'BOOT','8':'SW','9':'GND'},
              'Package_SO:Texas_HSOP-8-1EP_3.9x4.9mm_P1.27mm_ThermalVias',status='STUDY / datasheet DDA land pattern and thermal process to verify')
    # Normalize lib ID: self-contained symbol, no external custom library needed.
    sheet.libs.pop('Study:LMR36520ADDAR',None)
    sheet.libs['Lab:LMR36520ADDAR'][1]=s.q('Lab:LMR36520ADDAR')
    sheet.add('Device:R','REN','100k / 0805',35,113,{'1':'VIN_PROTECTED','2':'BUCK_EN'},'Resistor_SMD:R_0805_2012Metric')
    sheet.connector('JDIS','CLOSE TO DISABLE BUCK',35,161,['BUCK_EN','GND'],2,'Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical')
    sheet.add('Device:C','CB','100nF / 25V',172,110,{'1':'BOOT','2':'SW'},'Capacitor_SMD:C_0603_1608Metric')
    sheet.add('Device:C','CVCC','1uF / 25V',172,151,{'1':'VCC_INTERNAL','2':'GND'},'Capacitor_SMD:C_0603_1608Metric')
    sheet.add('Device:L','LBUCK','10uH / XAL6060-103MEC',240,100,{'1':'SW','2':'+5V_BUCK'},'Inductor_SMD:L_Coilcraft_XAL6060-XXX',status='Shielded, land pattern to verify against Coilcraft drawing',rotation=90)
    for i,x in enumerate([255,295,335,375],1):
        sheet.add('Device:C','CO'+str(i),'22uF / 25V X5R',x,137,{'1':'+5V_BUCK','2':'GND'},'Capacitor_SMD:C_1210_3225Metric')
    sheet.add('Device:R','RTOP','100k / 1%',245,192,{'1':'+5V_BUCK','2':'FB'},'Resistor_SMD:R_0603_1608Metric')
    sheet.add('Device:R','RBOT','24.9k / 1%',295,192,{'1':'FB','2':'GND'},'Resistor_SMD:R_0603_1608Metric')
    sheet.add('Device:R','RPG','100k / 1%',345,192,{'1':'+5V_BUCK','2':'PG_5V_TP'},'Resistor_SMD:R_0603_1608Metric')
    sheet.connector('JOUT','TO B / F2 INPUT',375,57,['+5V_BUCK','GND'],2)
    for i,(net,x) in enumerate([('VIN_PROTECTED',238),('GND',268),('+5V_BUCK',300)],1):sheet.flag('#FLG'+str(i),x,57,net)
    sheet.text(20,225,'OUTPUT: +5V_BUCK -> existing F2 (1A) -> +5V_AB -> A/B electronics. Heater power stays on protected 24V.\nVCC_INTERNAL is not an external 5V output. PG_5V_TP is a 5V-domain scope point, not ESP32-safe.\nFinal integration must pull PG to 3V3 if used by MCU, and deliberately connect logic/power returns; no CC shunt remains.',1.1)
    sheet.text(20,246,'Nominal output: 1V * (1 + 100k / 24.9k) = 5.016V. 4 x 22uF output bank; simulate 44uF effective worst case.\nModel: TI SNVMBX7A / LMR36520A_TRANS, unencrypted PSpice. Temperature and enclosure cooling are not modeled.\nNative ngspice attempts are documented as NON-CONVERGENT, not successful functional validation.',1.1)
    sheet.text(20,272,'No manufacturing release: PCB layout, capacitor DC bias, thermal pad/vias, USB power sharing, sourcing and bench tests pending.',1.1)
    # Canonical numeric designators for an annotatable KiCad study.
    names={'U_BUCK':'U1','JIN':'J1','JDIS':'JP1','JOUT':'J2','CIN1':'C1','CIN2':'C2','CHF':'C3','CB':'C4','CVCC':'C5',
           'CO1':'C6','CO2':'C7','CO3':'C8','CO4':'C9','LBUCK':'L1','REN':'R1','RTOP':'R2','RBOT':'R3','RPG':'R4'}
    for i,item in enumerate(sheet.items):
        n=s.parse(item)
        if n[0]!='symbol':continue
        props_={s.unquote(v[1]):v for v in s.children(n,'property')}
        old=s.unquote(props_['Reference'][2])
        if old not in names:continue
        props_['Reference'][2]=s.q(names[old])
        instances=s.child(n,'instances')
        project=s.child(instances,'project');path_=s.child(project,'path');s.child(path_,'reference')[1]=s.q(names[old])
        sheet.items[i]=s.dump(n)
    for row in sheet.bom:row['Reference']=names.get(row['Reference'],row['Reference'])
    dest=HERE/'buck_5v';dest.mkdir(exist_ok=True)
    local=copy.deepcopy(sheet.libs['Lab:LMR36520ADDAR']);local[1]=s.q('LMR36520ADDAR')
    (dest/'buck_study.kicad_sym').write_text('(kicad_symbol_lib (version 20241209) (generator "kicad_symbol_editor") '+s.dump(local)+')\n',encoding='utf-8')
    (dest/'sym-lib-table').write_text('(sym_lib_table (version 7) (lib (name "Lab") (type "KiCad") (uri "${KIPRJMOD}/buck_study.kicad_sym") (options "") (descr "Standalone buck study")))\n',encoding='utf-8')
    tree=f'(kicad_sch (version 20250114) (generator "eeschema") (uuid {sheet.uuid}) (paper "A3") (title_block (title {s.q(sheet.title)}) (date "2026-11-05") (rev "0.1 STUDY"))\n'
    tree+='(lib_symbols\n'+'\n'.join(s.dump(v) for v in sheet.libs.values())+')\n'
    tree+='\n'.join(sheet.items)+'\n(sheet_instances (path "/" (page "1"))) (embedded_fonts no))\n'
    source=dest/'buck_5v.kicad_sch';source.write_text(tree,encoding='utf-8')
    (dest/'buck_5v.kicad_pro').write_text(json.dumps({'meta':{'filename':'buck_5v.kicad_pro','version':1}},indent=2)+'\n')
    with (dest/'buck_5v_bom_study.csv').open('w',newline='',encoding='utf-8') as f:
        writer=csv.DictWriter(f,fieldnames=['Reference','Value','Footprint','Design_Status']);writer.writeheader();writer.writerows(row for row in sheet.bom if not row['Reference'].startswith('#'))
    for command,ext in [('netlist','.net.xml'),('pdf','.pdf')]:
        args=[CLI,'sch','export',command,'--output',str(dest/('buck_5v'+ext)),str(source)]
        if command=='netlist':args[4:4]=['--format','kicadxml']
        subprocess.run(args,check=True)
    subprocess.run([CLI,'sch','erc',str(source),'--output',str(dest/'buck_5v_erc.rpt'),'--exit-code-violations'],check=True)

def benches():
    folder=HERE/'benches';folder.mkdir(exist_ok=True)
    cases={
     'startup':dict(vin='PWL(0 0 100u 24)',en='PWL(0 0 200u 0 210u 3.3)',load='0.5',cap=22,stop='12m'),
     'load_step':dict(vin='PWL(0 0 100u 24)',en='PWL(0 0 200u 0 210u 3.3)',load='PWL(0 0.1 8m 0.1 8.001m 0.8 10m 0.8 10.001m 0.1)',cap=22,stop='12m'),
     'load_step_derated':dict(vin='PWL(0 0 100u 24)',en='PWL(0 0 200u 0 210u 3.3)',load='PWL(0 0.1 8m 0.1 8.001m 0.8 10m 0.8 10.001m 0.1)',cap=11,stop='12m'),
     'line_step':dict(vin='PWL(0 0 100u 24 8m 24 8.01m 26.4 10m 26.4 10.01m 18)',en='PWL(0 0 200u 0 210u 3.3)',load='0.5',cap=22,stop='12m'),
     'disable':dict(vin='PWL(0 0 100u 24)',en='PWL(0 0 200u 0 210u 3.3 10m 3.3 10.001m 0)',load='0.5',cap=22,stop='12m')}
    for name,cfg in cases.items():
        deck=f'''LMR36520ADDAR - {name} - STUDY / PSpice vendor model
.include "../{MODEL}"
VIN vin 0 {cfg['vin']}
VEN en 0 {cfg['en']}
REN vin en 100k
XU1 0 boot en fb pg 0 sw vcc vin LMR36520A_TRANS PARAMS: STEADY_STATE=0 FASTSS=0
CIN1 vin 0 4.7u
CIN2 vin 0 4.7u
CHF vin 0 220n
CB boot sw 100n
CVCC vcc 0 1u
LBUCK sw lind 10u
RDCR lind vout 29.8m
RTOP vout fb 100k
RBOT fb 0 24.9k
RPG vout pg 100k
VLOADCMD loadcmd 0 {cfg['load']}
GLOAD vout 0 VALUE={{V(loadcmd)*LIMIT(V(vout)/4.75,0,1)}}
'''
        for i in range(1,5):deck+=f'CO{i} vout co{i} {cfg["cap"]}u\nRESR{i} co{i} 0 5m\n'
        for old,new in [('CIN1','C1'),('CIN2','C2'),('CHF','C3'),('CB ','C4 '),('CVCC','C5'),('LBUCK','L1'),('REN ','R1 '),('RTOP','R2'),('RBOT','R3'),('RPG ','R4 ')]:deck=deck.replace(old,new)
        for i in range(1,5):deck=deck.replace('CO'+str(i)+' ', 'C'+str(i+5)+' ')
        deck+=f'.options reltol=0.003 abstol=1n vntol=1u\n.tran 100n {cfg["stop"]} 0 100n\n.probe V(vout) V(vin) V(pg) I(L1)\n.end\n'
        (folder/(name+'.cir')).write_text(deck,encoding='utf-8')
    model=model_download()
    (HERE/'simulation_plan.json').write_text(json.dumps(dict(candidate='LMR36520ADDAR',model=model,
      cases=cases,execution_status='PSpice benches prepared; not executed in vendor-supported PSpice. ngspice compatibility attempts failed.',
      targets=dict(output_min_V=4.75,output_max_V=5.25,settled_ripple_target_Vpp=.05,load_A=[.1,.8],
                   nominal_output_V=1+100000/24900,system_fuse_A=1,converter_rated_A=2),
      scope='Converter only, after upstream protection and before F2. No semiconductor temperature model, fuse/TVS/inversion/firmware/USB/PCB parasitics.'),indent=2)+'\n')

if __name__=='__main__':proposal();benches()
