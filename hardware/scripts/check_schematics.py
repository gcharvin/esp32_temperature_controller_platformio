"""Export and check the actual KiCad schematics, including the A/B cable contract.

Example (KiCad Python, or any Python >=3.10):
  python hardware/scripts/check_schematics.py --kicad-cli <path-to-kicad-cli>
This never regenerates or changes the source schematics.
"""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]

def run(*args):
    subprocess.run([str(a) for a in args],check=True)

def read_netlist(path):
    pins={}
    groups={}
    root=ET.parse(path).getroot()
    for net in root.findall('./nets/net'):
        name=net.attrib['name'].removeprefix('/')
        nodes={(n.attrib['ref'],n.attrib['pin']) for n in net.findall('node')}
        groups[name]=nodes
        for node in nodes:
            if node in pins:
                raise AssertionError(f'Pin appears on two nets: {node}')
            pins[node]=name
    return pins,groups

def check_pin(pins, ref, pin, expected):
    actual=pins.get((ref,str(pin)))
    assert actual == expected, f'{ref}.{pin}: expected {expected}, got {actual}'

def check_nc(pins,groups,ref,pin):
    net=pins[(ref,str(pin))]
    assert net.startswith('unconnected-') and len(groups[net]) == 1, (ref,pin,net)

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--kicad-cli',default=shutil.which('kicad-cli') or 'C:/Program Files/KiCad/10.0/bin/kicad-cli.exe')
    args=parser.parse_args()
    out=ROOT/'review'
    out.mkdir(exist_ok=True)
    boards={}
    for name in ('pcb_a_display','pcb_b_temperature'):
        source=ROOT/name/(name+'.kicad_sch')
        run(args.kicad_cli,'sch','erc',source,'--output',out/(name+'_erc.rpt'),'--exit-code-violations')
        run(args.kicad_cli,'sch','export','netlist','--format','kicadxml','--output',out/(name+'.net.xml'),source)
        run(args.kicad_cli,'sch','export','pdf','--output',out/(name+'.pdf'),source)
        boards[name]=read_netlist(out/(name+'.net.xml'))
    contract=json.loads((ROOT/'interface_ab.json').read_text(encoding='utf-8'))
    for name,(pins,groups) in boards.items():
        exported=ET.parse(out/(name+'.net.xml')).getroot()
        footprints={c.attrib['ref']: c.findtext('footprint') for c in exported.findall('./components/comp')}
        for port in contract['connectors']:
            ref=port['reference']; count=len(port['pins'])
            assert {p for r,p in pins if r==ref} == {str(row['pin']) for row in port['pins']}, (name,ref,'unexpected pin count')
            expected_fp=f'Connector_JST:JST_XH_B{count}B-XH-A_1x{count:02d}_P2.50mm_Vertical'
            assert footprints.get(ref)==expected_fp, (name,ref,'wrong JST footprint')
            for row in port['pins']:
                if row['net'] is None or (name=='pcb_b_temperature' and not row.get('connected_on_b',True)):
                    check_nc(pins,groups,ref,row['pin'])
                else:
                    check_pin(pins,ref,row['pin'],row['net'])
            assert all(not n.startswith(('VIN','+3V3')) and n!='HEATER_LOW' for (r,_),n in pins.items() if r==ref)
        assert {n for (r,_),n in pins.items() if r=='J1'} == {'GND','+5V_AB'}
        assert {n for (r,_),n in pins.items() if r=='J6'} == {'GND','I2C_SDA','I2C_SCL'}
    a,ag=boards['pcb_a_display']; b,bg=boards['pcb_b_temperature']
    # Physical 30-contact socket numbering; GPIO labels are not pad numbers.
    for pin,net in [(15,'+5V_DEVKIT'),(14,'GND'),(17,'GND'),(26,'I2C_SDA'),(29,'I2C_SCL'),(8,'PWM0'),(9,'PWM1'),(10,'ENABLE'),(6,'DIN0'),(7,'FAULT_N'),(20,'ENC_A'),(25,'ENC_B'),(30,'ENC_SW')]:
        check_pin(a,'U1',pin,net)
    check_pin(a,'JP1',1,'+5V_AB');check_pin(a,'JP1',2,'+5V_DEVKIT')
    check_pin(a,'U2',5,'+3V3_A');check_pin(a,'J2',2,'+3V3_A')
    for pin,net in [(1,'+3V3_B'),(8,'+3V3_B'),(3,'GND'),(4,'NTC_ADC'),(9,'I2C_SDA'),(10,'I2C_SCL'),(2,'FAULT_N')]:
        check_pin(b,'U4',pin,net)
    for ref,pin,net in [('U3',1,'PWM0'),('U3',2,'ENABLE'),('U3',4,'GATE_CMD'),('Q1',1,'GATE'),('Q1',2,'HEATER_LOW'),('Q1',3,'PGND'),('J3',1,'VIN_PROTECTED'),('J3',2,'HEATER_LOW'),('Q2',2,'VIN_FUSED'),('Q2',3,'VIN_PROTECTED'),('Q2',1,'REV_GATE'),('D3',1,'VIN_PROTECTED'),('D3',2,'REV_GATE'),('D4',1,'VIN_PROTECTED'),('D4',2,'HEATER_LOW'),('F1',1,'VIN_RAW'),('F1',2,'VIN_FUSED'),('F2',1,'+5V_BUCK'),('F2',2,'+5V_AB'),('U5',1,'+5V_AB'),('U5',2,'PGND'),('U5',3,'GATE_CMD'),('U5',4,'PGND'),('U5',5,'GATE_DRV'),('C8',1,'VIN_PROTECTED'),('C8',2,'PGND'),('R5',1,'NTC_RAW'),('R5',2,'GND'),('R6',1,'NTC_RAW'),('R6',2,'NTC_ADC')]:
        check_pin(b,ref,pin,net)
    assert bg['GND'].isdisjoint(bg['PGND']), 'Retain separately routed logic/power returns joined only by NT1'
    for ref,pin,net in [('U1',1,'VIN_PROTECTED'),('U1',2,'BUCK_SW'),('U1',3,'PGND'),('U1',4,'+5V_BUCK'),('U1',5,'PGND'),('L1',1,'BUCK_SW'),('L1',2,'+5V_BUCK'),('D9',1,'BUCK_SW'),('D9',2,'PGND'),('C9',1,'VIN_PROTECTED'),('C9',2,'PGND'),('C10',1,'+5V_BUCK'),('C10',2,'BUCK_CAP_RETURN'),('R13',1,'BUCK_CAP_RETURN'),('R13',2,'PGND'),('NT1',1,'PGND'),('NT1',2,'GND')]:
        check_pin(b,ref,pin,net)
    assert not any(ref in ['J8','J9'] for ref,pin in b), 'External buck wiring must be removed'
    bc={c.attrib['ref']:c for c in ET.parse(out/'pcb_b_temperature.net.xml').getroot().findall('./components/comp')}
    assert bc['U1'].findtext('value')=='LM2596S-5.0/NOPB'
    assert bc['U1'].findtext('footprint')=='Package_TO_SOT_SMD:TO-263-5_TabPin3'
    assert bc['L1'].findtext('value').startswith('100uH') and bc['C10'].findtext('value').startswith('330uF') and bc['R13'].findtext('value').startswith('56m')
    # Independent diagnostic contract: check rail, series resistor, LED polarity
    # and return. In particular, HEAT must follow the enabled driver output and
    # no indicator may load the NTC divider or add another PGND-to-GND connection.
    diagnostics=[]
    contracts={
        'pcb_a_display': [('D1','R6','+5V_AB','GND','1.5k'),('D2','R7','+3V3_A','GND','470R')],
        'pcb_b_temperature': [('D5','R9','VIN_PROTECTED','PGND','15k'),('D6','R10','+5V_AB','GND','1.5k'),
                              ('D7','R11','+3V3_B','GND','470R'),('D8','R12','GATE_DRV','PGND','2.2k')]}
    for name,rows in contracts.items():
        pins,groups=boards[name]
        comps={c.attrib['ref']:c for c in ET.parse(out/(name+'.net.xml')).getroot().findall('./components/comp')}
        for led,resistor,rail,ground,resistance in rows:
            check_pin(pins,resistor,1,rail);check_pin(pins,led,1,ground)
            series=pins[(resistor,'2')]
            assert groups[series]=={(resistor,'2'),(led,'2')},(led,'wrong series chain')
            assert comps[resistor].findtext('value').split(' / ')[0]==resistance
            assert comps[led].findtext('footprint')=='LabHardware:LED_Kingbright_APT2012_Manufacturer'
            assert comps[led].findtext('value')==('APT2012LSECK/J4-PRV' if led=='D8' else 'APT2012LZGCK')
            diagnostics.append(dict(board=name,led=led,resistor=resistor,supply=rail,return_net=ground,
                                    series_only=True,cathode_pin=1,anode_pin=2))
    check_pin(b,'J4',1,'+3V3_B');check_pin(b,'J4',2,'NTC_RAW')
    assert bg['NTC_RAW']=={('J4','2'),('R5','1'),('R6','1')},'Diagnostic circuits must not load the NTC'
    (out/'routing/diagnostic_validation.json').write_text(json.dumps(dict(LEDs=diagnostics,
        NTC_divider_unloaded=True,HEAT_measures='gate driver output, not heater current'),indent=2)+'\n',encoding='utf-8')
    for name,groups in [('A',ag),('B',bg)]:
        allowed_rails={'+3V3_A'} if name=='A' else {'+3V3_B'}
        assert allowed_rails.issubset(groups)
        other_rail='+3V3_B' if name=='A' else '+3V3_A'
        assert other_rail not in groups
    summary='Both schematics: ERC 0 errors / 0 warnings.\nA/B interface, isolated supply rails, ADC address strap, heater enable/gate, NTC and power path checked against exported netlists.\nSix diagnostic LED series chains, polarity, rail/return assignments and unloaded NTC divider verified.\nThese checks do not qualify current ratings, footprints, PCB routing or safety.\n'
    (out/'validation.txt').write_text(summary,encoding='utf-8')
    print(summary)

if __name__=='__main__':
    main()
