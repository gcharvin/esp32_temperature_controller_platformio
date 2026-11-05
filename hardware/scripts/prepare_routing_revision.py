"""Build prototype routing revision from the reviewed circuit and purchased modules.

KiCad Python. Explicit one-shot revision: refuses routed boards. Does not touch
project settings. DevKit row spacing is a USER-AUTHORIZED hypothesis, not measured.
"""
import copy
import csv
import json
from pathlib import Path
import subprocess
import pcbnew as p
import create_drafts as s
from create_placement import rule_area, rect, text, pos, xy, mm

ROOT=Path(__file__).resolve().parents[1]
LIB=Path('C:/Program Files/KiCad/10.0/share/kicad/footprints')
CLI='C:/Program Files/KiCad/10.0/bin/kicad-cli.exe'
ROW_SPACING=25.4
SOCKET='LabHardware:Diymore_ESP32_30P_Rows25.40mm_ASSUMED'
XH=lambda n:f'Connector_JST:JST_XH_B{n}B-XH-A_1x{n:02d}_P2.50mm_Vertical'
RFP='Resistor_SMD:R_0603_1608Metric'
CFP='Capacitor_SMD:C_0603_1608Metric'
FUSE='Fuse:Fuse_Littelfuse-NANO2-451_453'
TERM='TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-2-5.08_1x02_P5.08mm_Horizontal'

# Clockwise socket numbering, top view / USB at bottom before PCB rotation.
LEFT=['EN','GPIO36','GPIO39','GPIO34','GPIO35','GPIO32','GPIO33','GPIO25','GPIO26','GPIO27','GPIO14','GPIO12','GPIO13','GND','VIN']
RIGHT=['3V3','GND','GPIO15','GPIO2','GPIO4','GPIO16','GPIO17','GPIO5','GPIO18','GPIO19','GPIO21','GPIO3','GPIO1','GPIO22','GPIO23']
PIN_NAMES={str(i+1):n for i,n in enumerate(LEFT+RIGHT)}
USED={'GPIO4':'ENC_A','GPIO19':'ENC_B','GPIO23':'ENC_SW','GPIO21':'I2C_SDA','GPIO22':'I2C_SCL','GPIO25':'PWM0','GPIO26':'PWM1','GPIO27':'ENABLE','GPIO32':'DIN0','GPIO33':'FAULT_N','GND':'GND','VIN':'+5V_DEVKIT'}

def write_sheet(sheet):
    folder=ROOT/sheet.name
    source=f'(kicad_sch (version 20250114) (generator "eeschema") (uuid {sheet.uuid}) (paper "A3") (title_block (title {s.q(sheet.title)}) (date "2026-11-05") (rev "0.3 ROUTING PROTOTYPE") (company "Laboratory hardware platform") (comment 1 "PROTOTYPE - DEVKIT PITCH TO VERIFY - NOT RELEASED FOR FABRICATION"))\n'
    source+='(lib_symbols\n'+'\n'.join(s.dump(sym) for sym in sheet.libs.values())+')\n'
    source+='\n'.join(sheet.items)+'\n(sheet_instances (path "/" (page "1"))) (embedded_fonts no))\n'
    (folder/(sheet.name+'.kicad_sch')).write_text(source,encoding='utf-8')
    with (folder/(sheet.name+'_bom_draft.csv')).open('w',encoding='utf-8',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=['Reference','Value','Footprint','Design_Status'])
        writer.writeheader();writer.writerows(r for r in sheet.bom if not r['Reference'].startswith('#'))
    for fmt,ext in [('kicadxml','.net.xml')]:
        subprocess.run([CLI,'sch','export','netlist','--format',fmt,'--output',str(ROOT/'review'/(sheet.name+ext)),str(folder/(sheet.name+'.kicad_sch'))],check=True)

def create_socket():
    fp=p.FOOTPRINT(None);fp.SetReference('U1');fp.SetValue('DIymore CH340 30P; PITCH ASSUMED')
    fp.SetFPID(p.LIB_ID('LabHardware',SOCKET.split(':')[1]))
    # Local long axis X; antenna left and USB right. Header strips 2 x 15.
    for i in range(15):
        # With antenna left / USB right, the module's original LEFT header
        # becomes the BOTTOM row. Preserve handedness: do not mirror the module.
        for number,y in [(i+1,ROW_SPACING/2),(30-i,-ROW_SPACING/2)]:
            pad=p.PAD(fp);pad.SetNumber(str(number));pad.SetAttribute(p.PAD_ATTRIB_PTH)
            pad.SetShape(p.PAD_SHAPE_RECT if number in (1,16) else p.PAD_SHAPE_OVAL)
            pad.SetSize(xy(1.8,1.8));pad.SetDrillSize(xy(1,1));pad.SetLayerSet(p.PAD.PTHMask())
            pad.SetPosition(xy(-17.78+2.54*i,y));fp.Add(pad)
    from create_placement import line
    for y in (-ROW_SPACING/2,ROW_SPACING/2):rect(None,-19.05,y-1.27,38.1,2.54,p.F_CrtYd,parent=fp)
    rect(None,-27,-13.5,54,27,p.F_Fab,parent=fp)
    line(None,27,-5,29,-5,p.F_Fab,parent=fp);line(None,29,-5,29,5,p.F_Fab,parent=fp);line(None,29,5,27,5,p.F_Fab,parent=fp)
    fp.Reference().SetPosition(xy(0,-16));fp.Value().SetVisible(False)
    dest=ROOT/'lab_hardware.pretty';dest.mkdir(exist_ok=True)
    p.PCB_IO_MGR.FindPlugin(p.PCB_IO_MGR.KICAD_SEXP).FootprintSave(str(dest),fp)

def a_sheet():
    pins=[]
    outputs={'GPIO22','GPIO25','GPIO26','GPIO27'}
    for num,name in PIN_NAMES.items():
        typ='power_in' if name=='VIN' or num=='14' else 'power_out' if name=='3V3' else 'passive' if name=='GND' else 'output' if name in outputs else 'bidirectional' if name=='GPIO21' else 'input'
        pins.append((num,name,typ,'L' if int(num)<=15 else 'R'))
    s.CUSTOM['Diymore_ESP32_30P']=s.module('Diymore_ESP32_30P',pins,40.64)
    sheet=s.make_a()
    # Replace U1 and its generated stubs with a physical pin-numbered symbol.
    # Use a fresh sheet for that host, then preserve all other reviewed sections.
    replacement=s.Sheet(sheet.name,sheet.title)
    replacement.add('Lab:Diymore_ESP32_30P','U1','diymore CH340 USB-C / 30P',90,62,
                    {num:USED.get(name) for num,name in PIN_NAMES.items()},SOCKET,
                    status='Row pitch 25.40 mm ASSUMED; verify purchased board before fabrication')
    old_ids=set()
    # Reuse deterministic UUIDs belonging to host drawing/stubs only.
    original=s.Sheet(sheet.name,sheet.title)
    original.add('Lab:DevKit_Interface','U1','ELEGOO ESP32 DevKit - logical pins',90,62,
                 {'5V':'+5V_DEVKIT','GND':'GND','21':'I2C_SDA','22':'I2C_SCL','4':'ENC_A','19':'ENC_B','23':'ENC_SW','25':'PWM0','26':'PWM1','27':'ENABLE','32':'DIN0','33':'FAULT_N'},status='Header numbering / outline to measure; no footprint')
    original_items=set(original.items)
    sheet.items=[x for x in sheet.items if x not in original_items]+replacement.items
    sheet.libs.pop('Lab:DevKit_Interface');sheet.libs.update(replacement.libs)
    sheet.bom=[r for r in sheet.bom if r['Reference']!='U1']+replacement.bom
    sheet.items=[x.replace('GPIO numbers are chip GPIOs, NOT physical header pad numbers.', 'Physical pad numbers defined; row spacing 25.40 mm ASSUMED.').replace('NO 12 V, heater current or 3V3 rail.', 'NO 24 V, heater current or 3V3 rail.') for x in sheet.items]
    for i,item in enumerate(sheet.items):
        if item.startswith('(symbol (lib_id '):
            node=s.parse(item);props={s.unquote(v[1]):v for v in s.children(node,'property')}
            ref=s.unquote(props['Reference'][2])
            if ref in ('J2','J3','J4','J5'):props['Footprint'][2]=s.q(XH(4));props['Design_Status'][2]=s.q('JST B4B-XH-A physical package selected')
            sheet.items[i]=s.dump(node)
    for row in sheet.bom:
        if row['Reference'] in ('J2','J3','J4','J5'):row['Footprint']=XH(4);row['Design_Status']='JST B4B-XH-A physical package selected'
    return sheet

def b_sheet():
    sheet=s.Sheet('pcb_b_temperature','PCB B - 24 V / 100 W resistive heater + external XL4015')
    sheet.section(15,15,'B1 - 24 V DC / 100 W MAX HEATER DESIGN TARGET')
    sheet.connector('J2','24V IN / + PGND',30,40,['VIN_RAW','PGND'],2,TERM)
    sheet.add('Device:Fuse','F1','6.3A / 125VDC / 045106.3MRL',77,40,{'1':'VIN_RAW','2':'VIN_FUSED'},FUSE,rotation=90)
    sheet.add('Transistor_FET:Q_PMOS_GDS','Q2','DMP6023LE-13 / reverse input',127,43,{'1':'REV_GATE','2':'VIN_FUSED','3':'VIN_PROTECTED'},'Package_TO_SOT_SMD:SOT-223-3_TabPin2')
    sheet.add('Device:D_Zener','D3','BZT52C12 / 12V gate clamp',174,38,{'1':'VIN_PROTECTED','2':'REV_GATE'},'Diode_SMD:D_SOD-123')
    sheet.resistor('R8','100k',172,66,'REV_GATE','PGND')
    sheet.add('Device:D_TVS','D2','SMBJ24CA / transients NOT qualified',227,40,{'1':'PGND','2':'VIN_PROTECTED'},'Diode_SMD:D_SMB',rotation=90)
    sheet.connector('J3','HEATER / + switched return',284,40,['VIN_PROTECTED','HEATER_LOW'],2,TERM)
    sheet.connector('J8','To XL4015 IN / + PGND',340,40,['VIN_PROTECTED','PGND'],2,TERM)
    sheet.add('Device:D_Schottky','D4','STPS5L60U / cable flyback',284,68,{'1':'VIN_PROTECTED','2':'HEATER_LOW'},'Diode_SMD:D_SMB')
    sheet.add('Device:C_Polarized','C8','100uF / 50V / EEUFR1H101',335,77,{'1':'VIN_PROTECTED','2':'PGND'},'Capacitor_THT:CP_Radial_D8.0mm_P3.50mm')
    sheet.text(15,88,'Regulated 24 V supply only. F1 is a short-circuit protection candidate, not a 100 W limiter.\nPMOS: drain = fused input, source = protected output. Zener cathode at source.\nTVS clamping/transient margin of the purchased 32 V module is NOT qualified; no industrial surge claim.')
    sheet.section(15,103,'B2 - EXTERNAL XL4015 / SEPARATE INPUT AND OUTPUT RETURNS')
    sheet.add('Lab:Buck_Module_TBD','U1','TECNOULAB XL4015 CC/CV / OFF-BOARD',85,130,
              {'1':'VIN_PROTECTED','2':'PGND','3':'+5V_BUCK','4':'GND'},'',status='Purchased module; wire to screw terminals. No PCB mounting pads. Set 5.00V before connection.')
    # Module belongs to BOM but is mechanically off-board.
    for i,item in enumerate(sheet.items):
        if item.startswith('(symbol (lib_id "Lab:Buck_Module_TBD")'):
            sheet.items[i]=item.replace('(on_board yes)','(on_board no)')
    sheet.connector('J9','From XL4015 OUT / GND +5V',164,124,['GND','+5V_BUCK'],2,XH(2))
    sheet.add('Device:Fuse','F2','1A / 125VDC / 0451001.MRL',215,124,{'1':'+5V_BUCK','2':'+5V_AB'},FUSE,rotation=90)
    sheet.add('Regulator_Linear:AP2112K-3.3','U2','AP2112K-3.3',280,130,{'1':'+5V_AB','2':'GND','3':'+5V_AB','4':None,'5':'+3V3_B'})
    sheet.cap('C1','1uF / 16V X7R',242,150,'+5V_AB');sheet.cap('C2','1uF / 10V X7R',328,143,'+3V3_B')
    s.add_interface(sheet,[(35,175),(95,175),(158,175)])
    sheet.text(15,207,'PGND = XL4015 IN- / heater return. GND = XL4015 OUT- / A-B logic reference.\nNo copper short between these returns on B: preserve possible module CC sense shunt. NON-ISOLATED module.\nJ9 is 5 V ONLY. LCD/ESP32 power budget and module settings require bench verification.')
    sheet.section(200,169,'B3 - PWM AND ENABLE / 5 V GATE DRIVE')
    sheet.add('74xGxx:74LVC1G08','U3','SN74LVC1G08DBVR',235,193,{'1':'PWM0','2':'ENABLE','3':'GND','4':'GATE_CMD','5':'+3V3_B'},'Package_TO_SOT_SMD:SOT-23-5')
    sheet.resistor('R1','47k',204,219,'PWM0','GND');sheet.resistor('R2','47k',235,219,'ENABLE','GND');sheet.cap('C3','100nF / 10V X7R',267,212,'+3V3_B')
    driverpins=[('1','VDD','power_in','L'),('2','GND','power_in','L'),('3','IN+','input','L'),('4','IN-','input','L'),('5','OUT','output','R')]
    s.CUSTOM['UCC27517_DBV']=s.module('UCC27517_DBV',driverpins,15.24)
    sheet.add('Lab:UCC27517_DBV','U5','UCC27517DBVR',308,193,{'1':'+5V_AB','2':'PGND','3':'GATE_CMD','4':'PGND','5':'GATE_DRV'},'Package_TO_SOT_SMD:SOT-23-5',status='TI physical DBV pinout; TTL input, UVLO, output referenced to PGND')
    sheet.add('Device:C','C6','100nF / 16V X7R',345,215,{'1':'+5V_AB','2':'PGND'},CFP)
    sheet.add('Device:C','C7','1uF / 16V X7R',373,215,{'1':'+5V_AB','2':'PGND'},CFP)
    sheet.add('Device:R','R3','22R',351,175,{'1':'GATE_DRV','2':'GATE'},RFP,rotation=90)
    sheet.add('Transistor_FET:IRLZ44N','Q1','IRLZ44NPBF / gate 5 V',383,178,{'1':'GATE','2':'HEATER_LOW','3':'PGND'},'Package_TO_SOT_THT:TO-220-3_Vertical')
    sheet.resistor('R4','100k',365,235,'GATE','PGND')
    sheet.section(15,228,'B4 - NTC / ADS1115 ADDRESS 0x49')
    sheet.connector('J4','NTC 15k B3920',30,252,['+3V3_B','NTC_RAW'],2,XH(2))
    sheet.resistor('R5','8.45k 1%',74,252,'NTC_RAW','GND');sheet.resistor('R6','1k',111,252,'NTC_RAW','NTC_ADC')
    sheet.cap('C4','100nF / 10V X7R',148,252,'NTC_ADC')
    sheet.add('Analog_ADC:ADS1115IDGS','U4','ADS1115IDGSR / 0x49',206,258,{'1':'+3V3_B','2':'FAULT_N','3':'GND','4':'NTC_ADC','5':None,'6':None,'7':None,'8':'+3V3_B','9':'I2C_SDA','10':'I2C_SCL'})
    sheet.cap('C5','100nF / 10V X7R',251,259,'+3V3_B');sheet.resistor('R7','10k',290,258,'+3V3_B','FAULT_N')
    sheet.text(15,283,'24 V / 100 W at 500 Hz PWM is a PROTOTYPE target. Independent thermal cutoff still required for unattended use.\nFirmware must handle open/short NTC, stale ADC, fault shutdown and explicit enable. No firmware changes in this revision.',1.0)
    for i,(net,x,y) in enumerate([('VIN_RAW',370,80),('PGND',393,80),('VIN_PROTECTED',370,110),('GND',393,110),('+5V_AB',375,145)],1):sheet.flag(f'#FLG0{i}',x,y,net)
    return sheet

def main():
    for path in ROOT.glob('pcb_*/*.kicad_pcb'):
        assert not list(p.LoadBoard(str(path)).GetTracks()), 'Refuse to overwrite routed PCB'
    create_socket()
    (ROOT/'pcb_a_display/fp-lib-table').write_text('(fp_lib_table (version 7) (lib (name "LabHardware") (type "KiCad") (uri "${KIPRJMOD}/../lab_hardware.pretty") (options "") (descr "Prototype socket; row pitch requires physical verification")))\n',encoding='utf-8')
    sheets=[a_sheet(),b_sheet()]
    for sheet in sheets:write_sheet(sheet)
    # Synchronize local custom symbols, including the physical gate driver.
    library='(kicad_symbol_lib (version 20241209) (generator "kicad_symbol_editor")\n'
    for name,sym in s.CUSTOM.items():
        sym=copy.deepcopy(sym);sym[1]=s.q(name);library+=s.dump(sym)+'\n'
    (ROOT/'lab_draft.kicad_sym').write_text(library+')\n',encoding='utf-8')
    print('Physical socket hypothesis and 24 V power circuit written; PCB generation follows separately.')

if __name__=='__main__':main()
