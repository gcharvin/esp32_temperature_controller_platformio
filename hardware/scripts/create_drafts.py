"""One-shot bootstrap of the initial KiCad schematics; native files are authoritative.

Uses only Python's standard library and the installed KiCad symbol libraries.
Do not rerun over hand-edited schematics without an explicit --overwrite.
"""
import argparse
import copy
import csv
import json
import math
from pathlib import Path
import re
import uuid

ROOT = Path(__file__).resolve().parents[1]
LIB = Path('C:/Program Files/KiCad/10.0/share/kicad/symbols')
NS = uuid.UUID('8d592695-98e3-40a5-a414-51808994985c')

def uid(key):
    return str(uuid.uuid5(NS, key))

def q(value):
    return json.dumps(str(value), ensure_ascii=False)

def parse(text):
    tokens = iter(re.findall(r'"(?:\\.|[^"\\])*"|[()]|[^\s()]+', text))
    def read(token):
        if token != '(':
            return token
        result = []
        for token in tokens:
            if token == ')':
                return result
            result.append(read(token))
        raise ValueError('Unclosed S-expression')
    return read(next(tokens))

def dump(node):
    return '(' + ' '.join(dump(x) if isinstance(x, list) else x for x in node) + ')'

def children(node, key):
    return [x for x in node if isinstance(x, list) and x[0] == key]

def child(node, key):
    return next(iter(children(node, key)), None)

def unquote(token):
    return json.loads(token) if token.startswith('"') else token

LIBRARIES = {}

def native(lib, name):
    if lib not in LIBRARIES:
        root = parse((LIB / (lib + '.kicad_sym')).read_text(encoding='utf-8'))
        LIBRARIES[lib] = {unquote(s[1]): s for s in children(root, 'symbol')}
    def resolve(name):
        sym = copy.deepcopy(LIBRARIES[lib][name])
        parent = child(sym, 'extends')
        if parent:
            base_name = unquote(parent[1])
            base = resolve(base_name)
            overridden = {unquote(x[1]) for x in children(sym, 'property')}
            sym = sym[:2] + [x for x in base[2:] if not (isinstance(x, list) and x[0] == 'property' and unquote(x[1]) in overridden)] + [x for x in sym[2:] if not (isinstance(x, list) and x[0] in ('extends', 'embedded_fonts'))]
            for sub in children(sym, 'symbol'):
                suffix = unquote(sub[1])[len(base_name):]
                sub[1] = q(name + suffix)
        return sym
    sym = resolve(name)
    sym[1] = q(lib + ':' + name)
    return sym

def module(name, pins, width=25.4):
    # pins: (number, name, electrical_type, side), logical module interface.
    sides = {s: [p for p in pins if p[3] == s] for s in ('L', 'R')}
    height = max(len(p) for p in sides.values()) * 2.54 + 5.08
    pieces = [f'(symbol {q("Lab:" + name)} (pin_names (offset 1.016)) (in_bom yes) (on_board yes)',
              '(property "Reference" "U" (at 0 0 0) (effects (font (size 1.27 1.27))))',
              f'(property "Value" {q(name)} (at 0 0 0) (effects (font (size 1.27 1.27))))',
              f'(symbol {q(name + "_0_1")} (rectangle (start {-width/2} {height/2}) (end {width/2} {-height/2}) (stroke (width 0.254) (type default)) (fill (type background))))',
              f'(symbol {q(name + "_1_1")}']
    for side, group in sides.items():
        for i, (num, label, kind, _) in enumerate(group):
            x = (-1 if side == 'L' else 1) * (width / 2 + 5.08)
            y = (len(group) - 1) * 1.27 - i * 2.54
            angle = 0 if side == 'L' else 180
            pieces.append(f'(pin {kind} line (at {x} {y} {angle}) (length 5.08) (name {q(label)} (effects (font (size 1.016 1.016)))) (number {q(num)} (effects (font (size 1.016 1.016)))))')
    pieces.append('))')
    return parse(' '.join(pieces))

CUSTOM = {}
CUSTOM['DevKit_Interface'] = module('DevKit_Interface', [
    ('5V','5V','power_in','L'), ('GND','GND','power_in','L'),
    ('21','GPIO21 SDA','bidirectional','L'),('22','GPIO22 SCL','output','L'),
    ('4','GPIO4 ENC_A','input','L'),('19','GPIO19 ENC_B','input','L'),('23','GPIO23 ENC_SW','input','L'),
    ('25','GPIO25 PWM0','output','R'),('26','GPIO26 PWM1','output','R'),
    ('27','GPIO27 ENABLE','output','R'),('32','GPIO32 DIN0','input','R'),('33','GPIO33 FAULT_N','input','R')], 40.64)
CUSTOM['Buck_Module_TBD'] = module('Buck_Module_TBD', [('1','VIN+','power_in','L'),('2','VIN-','power_in','L'),('3','5V OUT+','power_out','R'),('4','OUT-','passive','R')])
CUSTOM['NMOS_TBD'] = module('NMOS_TBD', [('1','GATE','input','L'),('2','DRAIN','passive','R'),('3','SOURCE','passive','R')],20.32)

INTERFACE = [
    {'reference':'J1','function':'5V power','pins':[
        {'pin':1,'net':'GND','direction':'common'},
        {'pin':2,'net':'+5V_AB','direction':'B->A'}]},
    {'reference':'J6','function':'I2C bus, no power rail','pins':[
        {'pin':1,'net':'GND','direction':'common'},
        {'pin':2,'net':'I2C_SDA','direction':'bidirectional'},
        {'pin':3,'net':'I2C_SCL','direction':'A->B'}]},
    {'reference':'J7','function':'Control and reserved signals','pins':[
        {'pin':1,'net':'GND','direction':'common'},
        {'pin':2,'net':'PWM0','direction':'A->B'},
        {'pin':3,'net':'ENABLE','direction':'A->B'},
        {'pin':4,'net':'FAULT_N','direction':'B->A, open-drain active-low'},
        {'pin':5,'net':'PWM1','direction':'A->B','connected_on_b':False},
        {'pin':6,'net':'DIN0','direction':'B->A','connected_on_b':False}]}]

def add_interface(sheet, positions):
    for port, (x,y) in zip(INTERFACE, positions):
        ref=port['reference']; n=len(port['pins'])
        nets=[p['net'] if sheet.name!='pcb_b_temperature' or p.get('connected_on_b',True) else None for p in port['pins']]
        sheet.connector(ref,{'J1':'A-B POWER / XH-2','J6':'A-B I2C / XH-3','J7':'A-B CTRL / XH-6'}[ref],x,y,nets,n,
                        f'Connector_JST:JST_XH_B{n}B-XH-A_1x{n:02d}_P2.50mm_Vertical')

class Sheet:
    def __init__(self, name, title):
        self.name, self.title = name, title
        self.uuid = uid(name)
        self.libs, self.items, self.bom, self.connections = {}, [], [], {}
    def text(self, x, y, text, size=1.27):
        self.items.append(f'(text {q(text)} (at {x} {y} 0) (effects (font (size {size} {size})) (justify left bottom)) (uuid {uid(self.name+text)}))')
    def section(self, x, y, text):
        self.text(x,y,text,1.8)
    def add(self, libid, ref, value, x, y, nets, footprint=None, status='provisional', rotation=0):
        x, y = round(round(x/1.27)*1.27,4), round(round(y/1.27)*1.27,4)
        sym = copy.deepcopy(CUSTOM[libid.split(':')[1]]) if libid.startswith('Lab:') else native(*libid.split(':'))
        self.libs[libid] = sym
        props = {unquote(p[1]): unquote(p[2]) for p in children(sym,'property')}
        fp = props.get('Footprint','') if footprint is None else footprint
        pins=[]
        for sub in children(sym, 'symbol'):
            pins += children(sub, 'pin')
        # All schematic components are unit 1. Local symbols avoid inherited dependencies.
        ymin = min([-float(child(p,'at')[2]) for p in pins] + [-5])
        is_small = libid in ('Device:R','Device:C','Device:Fuse','Device:D_Schottky','Device:D_TVS')
        for sub in children(sym, 'symbol'):
            for rect in children(sub, 'rectangle'):
                ymin=min(ymin,-float(child(rect,'start')[2]),-float(child(rect,'end')[2]))
        if rotation:
            ymin = -7.62
        px, py = (x+5.08,y-1.27) if is_small and rotation == 0 else (x, y+ymin-6.35)
        if libid == 'Device:D_Schottky':
            px, py = x-10.16,y+6.35
        if libid == 'Device:D_TVS':
            px, py = x+25.4,y-1.27
        property_angle=rotation
        angle = math.radians(rotation)
        def transform(lx,ly):
            return (round(x+lx*math.cos(angle)-ly*math.sin(angle),4), round(y-lx*math.sin(angle)-ly*math.cos(angle),4))
        symbol_id = uid(self.name+ref)
        parts = [f'(symbol (lib_id {q(libid)}) (at {x} {y} {rotation}) (unit 1) (in_bom yes) (on_board yes) (dnp no) (uuid {symbol_id})',
                 f'(property "Reference" {q(ref)} (at {px} {py} {property_angle}) (effects (font (size 1.27 1.27)) (justify left)))',
                 f'(property "Value" {q(value)} (at {px} {py+2.54} {property_angle}) (effects (font (size 1.27 1.27)) (justify left)))',
                 f'(property "Footprint" {q(fp)} (at {x} {y} 0) (effects (font (size 1.27 1.27)) hide))',
                 f'(property "Datasheet" {q(props.get("Datasheet",""))} (at {x} {y} 0) (effects (font (size 1.27 1.27)) hide))',
                 f'(property "Design_Status" {q(status)} (at {x} {y} 0) (effects (font (size 1.27 1.27)) hide))']
        seen=set()
        for pin in pins:
            num = unquote(child(pin,'number')[1])
            if num in seen:
                continue
            seen.add(num)
            parts.append(f'(pin {q(num)} (uuid {uid(self.name+ref+num)}))')
            a = child(pin,'at')
            sx,sy = transform(float(a[1]),float(a[2]))
            net=nets.get(num)
            self.connections[(ref,num)] = net
            key=self.name+ref+num
            if net is None:
                self.items.append(f'(no_connect (at {sx} {sy}) (uuid {uid(key+"NC")}))')
            else:
                theta=math.radians(float(a[3])+rotation)
                ex,ey=round(sx-5.08*math.cos(theta),4), round(sy+5.08*math.sin(theta),4)
                self.items.append(f'(wire (pts (xy {sx} {sy}) (xy {ex} {ey})) (stroke (width 0) (type default)) (uuid {uid(key+"wire")}))')
                vertical=abs(ey-sy)>1
                label_angle=90 if vertical else 0
                # Put the text outside the stub, away from the symbol.
                justify='right' if (not vertical and ex<sx) or (vertical and ey>sy) else 'left'
                self.items.append(f'(label {q(net)} (at {ex} {ey} {label_angle}) (effects (font (size 1.016 1.016)) (justify {justify} bottom)) (uuid {uid(key+"label")}))')
        assert not (set(nets)-seen), (ref, set(nets)-seen)
        parts.append(f'(instances (project {q(self.name)} (path {q("/"+self.uuid)} (reference {q(ref)}) (unit 1)))))')
        self.items.append(' '.join(parts))
        self.bom.append({'Reference':ref,'Value':value,'Footprint':fp,'Design_Status':status})
    def resistor(self, ref, value, x,y,a,b):
        self.add('Device:R',ref,value,x,y,{'1':a,'2':b},'Resistor_SMD:R_0603_1608Metric')
    def cap(self,ref,value,x,y,net):
        self.add('Device:C',ref,value,x,y,{'1':net,'2':'GND'},'Capacitor_SMD:C_0603_1608Metric')
    def connector(self,ref,value,x,y,nets,n=4,footprint=''):
        self.add(f'Connector_Generic:Conn_01x{n:02d}',ref,value,x,y,{str(i): net for i,net in enumerate(nets,1)},footprint)
    def flag(self,ref,x,y,net):
        self.add('power:PWR_FLAG',ref,'PWR_FLAG',x,y,{'1':net},'',status='ERC supply declaration')
    def write(self):
        folder=ROOT/self.name
        folder.mkdir(exist_ok=True)
        out=f'(kicad_sch (version 20250114) (generator "eeschema") (uuid {self.uuid}) (paper "A3") (title_block (title {q(self.title)}) (date "2026-11-04") (rev "0.2 DRAFT") (company "Laboratory hardware platform") (comment 1 "DESIGN REVIEW ONLY - NOT FOR FABRICATION"))\n'
        out+='(lib_symbols\n'+'\n'.join(dump(s) for s in self.libs.values())+')\n'
        out+='\n'.join(self.items)+f'\n(sheet_instances (path "/" (page "1"))) (embedded_fonts no))\n'
        (folder/(self.name+'.kicad_sch')).write_text(out,encoding='utf-8')
        # Empty project settings let KiCad use its default ERC matrix, no suppressed checks.
        (folder/(self.name+'.kicad_pro')).write_text(json.dumps({'meta':{'filename':self.name+'.kicad_pro','version':1}},indent=2)+'\n',encoding='utf-8')
        (folder/'sym-lib-table').write_text('(sym_lib_table (version 7) (lib (name "Lab") (type "KiCad") (uri "${KIPRJMOD}/../lab_draft.kicad_sym") (options "") (descr "Logical module interfaces; physical mappings pending")))\n',encoding='utf-8')
        with (folder/(self.name+'_bom_draft.csv')).open('w',newline='',encoding='utf-8') as f:
            writer=csv.DictWriter(f,fieldnames=['Reference','Value','Footprint','Design_Status'])
            writer.writeheader();writer.writerows(row for row in self.bom if not row['Reference'].startswith('#'))

def make_a():
    s=Sheet('pcb_a_display','PCB A - Display and ESP32 carrier')
    s.section(20,20,'A1 - HOST AND COMMON INTERFACE / LOGIC 3.3 V')
    s.add('Lab:DevKit_Interface','U1','ELEGOO ESP32 DevKit - logical pins',90,62,
          {'5V':'+5V_DEVKIT','GND':'GND','21':'I2C_SDA','22':'I2C_SCL','4':'ENC_A','19':'ENC_B','23':'ENC_SW','25':'PWM0','26':'PWM1','27':'ENABLE','32':'DIN0','33':'FAULT_N'},status='Header numbering / outline to measure; no footprint')
    add_interface(s,[(190,55),(250,55),(325,55)])
    s.text(185,90,'J1 POWER: 1 GND / 2 +5V. J6 I2C: 1 GND / 2 SDA / 3 SCL.\nJ7 CTRL: 1 GND / 2 PWM0 / 3 ENABLE / 4 FAULT_N / 5 PWM1 / 6 DIN0.\nStraight-through cables, JST-XH 2.50 mm. NO 12 V, heater current or 3V3 rail.')
    s.text(20,96,'U1 is a removable DevKit, not a bare ESP32.\nGPIO numbers are chip GPIOs, NOT physical header pad numbers.')
    s.section(20,115,'A2 - LOCAL POWER / LCD 3.3 V AS CURRENT PROTOTYPE')
    s.connector('JP1','5V DevKit link / remove for service',42,139,['+5V_AB','+5V_DEVKIT'],2,'Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical')
    s.add('Regulator_Linear:AP2112K-3.3','U2','AP2112K-3.3 peripherals',110,143,{'1':'+5V_AB','2':'GND','3':'+5V_AB','4':None,'5':'+3V3_A'})
    s.cap('C1','1uF',78,150,'+5V_AB'); s.cap('C2','1uF',153,143,'+3V3_A')
    s.cap('C3','10uF',190,143,'+5V_AB')
    s.connector('J2','LCD20x4 + backpack / 3V3',265,139,['GND','+3V3_A','I2C_SDA','I2C_SCL'])
    s.text(20,172,'JP1 fitted in normal use. Do not parallel external 5 V and USB VBUS.\nUSB maintenance: B off, disconnect J1/J6/J7, remove JP1; no peripheral operation in this mode.\nPowered serial debugging requires a verified VBUS-isolated USB connection. No automatic power mux.')
    s.text(250,164,'LCD header order is a proposed cable order.\nCheck actual backpack pin order, 3.3 V operation\nand backlight current before PCB layout.')
    s.section(20,198,'A3 - USER INPUT / I2C PERIPHERALS')
    s.connector('J3','Encoder / A B SW GND',43,222,['ENC_A','ENC_B','ENC_SW','GND'])
    s.resistor('R1','10k',83,226,'+3V3_A','ENC_A')
    s.resistor('R2','10k',117,226,'+3V3_A','ENC_B')
    s.resistor('R3','10k',151,226,'+3V3_A','ENC_SW')
    s.resistor('R4','4.7k',188,226,'+3V3_A','I2C_SDA')
    s.resistor('R5','4.7k',226,226,'+3V3_A','I2C_SCL')
    s.connector('J4','TMP117 module / addr 0x48',286,215,['GND','+3V3_A','I2C_SDA','I2C_SCL'])
    s.connector('J5','BH1750 module / addr 0x23',286,241,['GND','+3V3_A','I2C_SDA','I2C_SCL'])
    s.text(20,265,'Only R4/R5 are the main I2C pull-ups. Remove or account for pull-ups on all breakout modules and LCD.\nDevKit 3V3 is not joined to +3V3_A. U2 supplies peripherals only; check current and thermal budget.\nI2C cable A-B: short internal harness. ADC on B = 0x49, TMP117 on A = 0x48.')
    for i,(net,x) in enumerate([('+5V_AB',337),('+5V_DEVKIT',362),('GND',387)],1):
        s.flag(f'#FLG0{i}',x,110,net)
    return s

def make_b():
    s=Sheet('pcb_b_temperature','PCB B - Temperature acquisition and heater power')
    s.section(20,20,'B1 - 12 V DC POWER INPUT / PROTECTED HEATER SUPPLY')
    s.connector('J2','12V DC input / + then 0V',40,45,['VIN_RAW','GND'],2)
    s.add('Device:Fuse','F1','TBD current',85,45,{'1':'VIN_RAW','2':'VIN_FUSED'},'',rotation=90)
    s.add('Device:D_Schottky','D1','Reverse polarity / TBD rating',150,45,{'1':'VIN_PROTECTED','2':'VIN_FUSED'},'',rotation=0)
    s.add('Device:D_TVS','D2','TVS TBD',220,45,{'1':'GND','2':'VIN_PROTECTED'},'',rotation=90)
    s.add('Lab:Buck_Module_TBD','U1','12V -> 5V module / TBD',300,45,{'1':'VIN_PROTECTED','2':'GND','3':'+5V_AB','4':'GND'},status='Logical interface only; rating, pinout and footprint TBD')
    s.text(20,75,'F1, D1, D2, U1 and Q1 require load current / transient / thermal sizing. D1 is a series diode, with conduction losses.\n12 V prototype only. No 24 V qualification. Main load returns directly to input GND; keep analog ground paths quiet.')
    s.section(20,98,'B2 - PASSIVE MODULE INTERFACE / LOCAL 3.3 V')
    add_interface(s,[(40,123),(90,123),(150,125)])
    s.add('Regulator_Linear:AP2112K-3.3','U2','AP2112K-3.3',120,171,{'1':'+5V_AB','2':'GND','3':'+5V_AB','4':None,'5':'+3V3_B'})
    s.cap('C1','1uF',83,171,'+5V_AB'); s.cap('C2','1uF',160,171,'+3V3_B')
    s.text(20,200,'J1/J6/J7: same pin order as A. J7.5 / J7.6 reserved, not connected on B.\nFAULT_N = ADC ALERT/RDY: configure comparator in firmware; no independent thermal cutoff.\nAll three cables: connect/disconnect ONLY with both boards unpowered. No 3V3 rail between A and B.')
    s.section(210,98,'B3 - HEATER COMMAND / HARDWARE ENABLE')
    s.add('74xGxx:74LVC1G08','U3','74LVC1G08',253,126,{'1':'PWM0','2':'ENABLE','3':'GND','4':'GATE_CMD','5':'+3V3_B'})
    s.resistor('R1','47k',210,148,'PWM0','GND'); s.resistor('R2','47k',247,148,'ENABLE','GND')
    s.cap('C3','100nF',288,143,'+3V3_B')
    s.resistor('R3','100R',325,126,'GATE_CMD','GATE')
    s.add('Lab:NMOS_TBD','Q1','NMOS / 3V3 gate / TBD',371,133,{'1':'GATE','2':'HEATER_LOW','3':'GND'},status='Logical G/D/S only; device, physical pads and footprint TBD')
    s.resistor('R4','100k',326,158,'GATE','GND')
    s.connector('J3','Heater output',385,177,['VIN_PROTECTED','HEATER_LOW'],2)
    s.text(210,190,'Gate drive = PWM0 AND ENABLE. Both default low with A unplugged.\nA crashed with ENABLE high is not detected by this gate: add watchdog / independent thermal protection before unattended use.')
    s.section(20,210,'B4 - NTC INPUT / ADC / ADC ADDRESS 0x49')
    # NTC upper leg +3V3 -> NTC -> divider node -> 8.45k -> GND, matching old firmware topology.
    s.connector('J4','NTC 15k B3920 probe',45,238,['+3V3_B','NTC_RAW'],2)
    s.resistor('R5','8.45k 1%',84,247,'NTC_RAW','GND')
    s.resistor('R6','1k',127,247,'NTC_RAW','NTC_ADC')
    s.cap('C4','100nF',168,247,'NTC_ADC')
    s.add('Analog_ADC:ADS1115IDGS','U4','ADS1115IDGS / 0x49',233,245,{'1':'+3V3_B','2':'FAULT_N','3':'GND','4':'NTC_ADC','5':None,'6':None,'7':None,'8':'+3V3_B','9':'I2C_SDA','10':'I2C_SCL'})
    s.cap('C5','100nF',285,245,'+3V3_B')
    s.resistor('R7','10k',324,228,'+3V3_B','FAULT_N')
    s.text(20,277,'R_NTC = R5 * (Vexc / Vnode - 1). Calibrate R5 and Vexc.\nOld 8400 ohm firmware constant must change to actual R5.\nProbe fault / stale ADC: firmware must disable heater.\nAIN1..3 unused. ADDR=VDD => 0x49. I2C pull-ups on A only.')
    s.flag('#FLG01',365,224,'VIN_RAW'); s.flag('#FLG02',389,224,'GND')
    s.flag('#FLG03',365,85,'VIN_PROTECTED')
    return s

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--overwrite',action='store_true')
    args=parser.parse_args()
    if any(ROOT.glob('pcb_*/*.kicad_sch')) and not args.overwrite:
        raise SystemExit('Native schematics exist; use --overwrite only to intentionally discard their edits.')
    for sheet in (make_a(),make_b()):
        sheet.write()
    (ROOT/'interface_ab.json').write_text(json.dumps({'revision':'0.2','logic_voltage_V':3.3,'power_voltage_V':5,'connector_family':'JST-XH','pitch_mm':2.5,'harness':'three short internal straight-through cables; all connections made unpowered, no hot-plug','connectors':INTERFACE},indent=2)+'\n',encoding='utf-8')
    lib='(kicad_symbol_lib (version 20241209) (generator "kicad_symbol_editor")\n'
    for name,sym in CUSTOM.items():
        sym=copy.deepcopy(sym);sym[1]=q(name);lib+=dump(sym)+'\n'
    (ROOT/'lab_draft.kicad_sym').write_text(lib+')\n',encoding='utf-8')
    print('Created PCB A and PCB B native KiCad draft projects.')

if __name__=='__main__':
    main()
