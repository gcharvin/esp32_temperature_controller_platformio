"""Check the proposal's actual KiCad netlist and calculate design margins.

These are circuit-connectivity and analytical checks, not SPICE validation.
"""
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import re

HERE=Path(__file__).resolve().parent
tree=ET.parse(HERE/'buck_5v/buck_5v.net.xml').getroot()
components={v.attrib['ref']:v for v in tree.findall('./components/comp')}
def numeric_value(ref):
    value=components[ref].findtext('value')
    match=re.match(r'([\d.]+)([kmunp]?)',value)
    assert match,(ref,value)
    return float(match[1])*{'':1,'k':1e3,'m':1e-3,'u':1e-6,'n':1e-9,'p':1e-12}[match[2]]
pins={(n.attrib['ref'],n.attrib['pin']):net.attrib['name'].removeprefix('/')
      for net in tree.findall('./nets/net') for n in net.findall('node')}
for pin,net in [('1','GND'),('2','VIN_PROTECTED'),('3','BUCK_EN'),('4','PG_5V_TP'),('5','FB'),
                ('6','VCC_INTERNAL'),('7','BOOT'),('8','SW'),('9','GND')]:
    assert pins[('U1',pin)]==net,(pin,net)
for ref,a,b in [('C1','VIN_PROTECTED','GND'),('C2','VIN_PROTECTED','GND'),('C3','VIN_PROTECTED','GND'),
                ('C4','BOOT','SW'),('C5','VCC_INTERNAL','GND'),('L1','SW','+5V_BUCK'),
                ('R1','VIN_PROTECTED','BUCK_EN'),('R2','+5V_BUCK','FB'),('R3','FB','GND'),
                ('R4','+5V_BUCK','PG_5V_TP'),('JP1','BUCK_EN','GND')]:
    assert (pins[(ref,'1')],pins[(ref,'2')])==(a,b),ref
for ref in ['C6','C7','C8','C9']:assert (pins[(ref,'1')],pins[(ref,'2')])==('+5V_BUCK','GND')
assert {ref for ref,pin in pins if pins[(ref,pin)]=='VCC_INTERNAL'}=={'U1','C5'},'Internal VCC must not supply external loads'

rtop=numeric_value('R2');rbot=numeric_value('R3');inductance=numeric_value('L1')
assert components['U1'].findtext('value')=='LMR36520ADDAR'
assert rtop==100000 and rbot==24900 and abs(inductance-10e-6)<1e-12
bank=sum(numeric_value(ref) for ref in ['C6','C7','C8','C9'])
assert abs(bank-88e-6)<1e-12
vnom=1+rtop/rbot
vmin=.985*(1+rtop*.99/(rbot*1.01))
vmax=1.015*(1+rtop*1.01/(rbot*.99))
rows=[]
for vin in (18,24,26.4):
    ripple=5*(1-5/vin)/(inductance*400e3)
    rows.append(dict(input_V=vin,duty_ideal=5/vin,nominal_inductor_ripple_App=ripple,
                     peak_A_at_0p8A=.8+ripple/2))
assert vmin>4.75 and vmax<5.25
report=dict(status='analytical checks only; functional switching simulation remains a separate gate',
    KiCad_pin_mapping_checked=True,internal_VCC_unloaded=True,
    output_V_nominal=vnom,output_V_static_corners_1pct_resistors_and_1p5pct_reference=[vmin,vmax],
    rows=rows,output_bank_rated_uF=88,output_bank_effective_sensitivity_uF=44,
    minimum_inductance_datasheet_rule_uH=.42*5/400e3*1e6,selected_inductance_uH=10,
    selected_inductance_minus20pct_uH=8,
    estimated_converter_loss_at_5W_if_90pct_efficiency_W=5*(1/.9-1),
    loss_caveat='Illustrative estimate using an efficiency assumption; not a thermal prediction or measurement.',
    pending=['SPICE completion and functional criteria','actual capacitor DC-bias curves and sourcing',
             'loop stability across final output bank and loads','PCB layout and thermal validation',
             'integration of logic/power returns and USB supply selection'])
(HERE/'analytical_checks.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
print(json.dumps(report,indent=2))
