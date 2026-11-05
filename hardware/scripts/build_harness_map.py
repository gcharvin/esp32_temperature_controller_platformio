"""Build the three straight A/B harness definitions from independent pin contracts.

Checks both fresh schematic netlists, including B's deliberately NC signals.
Does not infer connector orientation from wire colours or a photograph.
"""
import csv
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]


def main():
    contract=ROOT/'interface_ab.json';data=json.loads(contract.read_text(encoding='utf-8'))
    paths=[ROOT/'review'/(name+'.net.xml') for name in ('pcb_a_display','pcb_b_temperature')]
    pinmaps=[]
    for path in paths:
        pins={}
        for net in ET.parse(path).findall('./nets/net'):
            for node in net.findall('node'):pins[node.get('ref'),node.get('pin')]=net.get('name').removeprefix('/')
        pinmaps.append(pins)
    rows=[]
    for connector in data['connectors']:
        ref=connector['reference'];count=len(connector['pins']);power=ref=='J1'
        for item in connector['pins']:
            pin=str(item['pin']);a,b=(pins[ref,pin] for pins in pinmaps)
            assert a==item['net']
            if item.get('connected_on_b',True):assert b==a
            else:assert b.startswith('unconnected-')
            rows.append(dict(Harness={'J1':'POWER','J6':'I2C','J7':'CTRL'}[ref],
                End_A='A/'+ref,Pin_A=pin,End_B='B/'+ref,Pin_B=pin,Net_A=a,
                Net_B=b if item.get('connected_on_b',True) else 'NC - reserved for other B variants',
                Housing_each_end='XHP-'+str(count),Contact_each_end='SXH-001T-P0.6',
                Wire_AWG=22 if power else 26,Length_mm=300,Tolerance_mm=10,
                Wire_colour='black' if a=='GND' else 'red' if a=='+5V_AB' else 'number-labelled; colour not authoritative'))
    assert len(rows)==11
    folder=ROOT/'manufacturing';output=folder/'harness_pin_map.csv'
    with output.open('w',encoding='utf-8-sig',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    report=dict(status='PASS / straight pin mapping matches both schematics',wire_count=11,contact_count=22,
        housings={'XHP-2':2,'XHP-3':2,'XHP-6':2},lengths='300+/-10mm for prototype quotation; real cable routing unqualified',
        source_sha256={path.relative_to(ROOT).as_posix():hashlib.sha256(path.read_bytes()).hexdigest()
            for path in [contract,*paths,Path(__file__),output]})
    (folder/'harness_validation.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
