"""Verify all frozen part identities in fresh netlists, native PCBs and selection.

KiCad Python. No supplier substitutes and no implicit ordering approval.
"""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import pcbnew as p

ROOT=Path(__file__).resolve().parents[1]


def main():
    selection=ROOT/'manufacturing/component_selection.json'
    choices=json.loads(selection.read_text(encoding='utf-8'))
    sources=[selection,Path(__file__)];counts={};physical=0
    for code,name in [('A','pcb_a_display'),('B','pcb_b_temperature')]:
        sch=ROOT/name/(name+'.kicad_sch');pcb=sch.with_suffix('.kicad_pcb')
        net=ROOT/'review'/(name+'.net.xml');sources.extend([sch,pcb,net])
        comps={c.attrib['ref']:c for c in ET.parse(net).findall('./components/comp')}
        board=p.LoadBoard(str(pcb));fps={fp.GetReference():fp for fp in board.GetFootprints()}
        table=choices['boards'][code]
        assert set(table)==set(comps)==set(fps)
        for ref,choice in table.items():
            comp=comps[ref];fp=fps[ref]
            fields={f.get('name'):f.text or '' for f in comp.findall('./fields/field')}
            assert comp.findtext('value')==fp.GetValue()==choice['Value'],(code,ref,'value')
            assert comp.findtext('footprint')==fp.GetFPID().GetUniStringLibId()==choice['Footprint'],(code,ref,'footprint')
            for field in ('MPN','Manufacturer','Assembly'):
                assert fields.get(field,'')==fp.GetFieldText(field)==choice[field],(code,ref,field)
            if choice['Assembly'] in ('FACTORY_SMT_QUOTE','FACTORY_THT_QUOTE'):
                assert choice['MPN'] and choice['Manufacturer'] and choice['Source']
                physical+=1
            else:assert not choice['MPN']
        counts[code]=len(table)
    assert physical==70
    result=dict(status='PASS / prototype engineering selection; assembler approval remains external',
        native_references=counts,physical_native_components=physical,extra_sockets=2,shunts_supplied_loose=1,
        all_native_MPN_manufacturer_fields_match=True,substitutions_allowed=False,
        source_sha256={path.relative_to(ROOT).as_posix():hashlib.sha256(path.read_bytes()).hexdigest() for path in sources})
    (ROOT/'review/routing/component_selection_validation.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
