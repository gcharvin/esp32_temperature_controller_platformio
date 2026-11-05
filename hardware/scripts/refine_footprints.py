"""Incrementally assign documented packages without regenerating the schematics.

Run with KiCad's Python. Unresolved modules/power components remain explicit.
"""
import csv
import json
from pathlib import Path
import pcbnew as p
from create_drafts import parse, dump, child, children, q, unquote

ROOT = Path(__file__).resolve().parents[1]
LIB = Path('C:/Program Files/KiCad/10.0/share/kicad/footprints')
XH4 = 'Connector_JST:JST_XH_B4B-XH-A_1x04_P2.50mm_Vertical'
XH2 = 'Connector_JST:JST_XH_B2B-XH-A_1x02_P2.50mm_Vertical'
SELECTIONS = {
    'pcb_a_display': {
        ref: (XH4, 'B4B-XH-A(LF)(SN)', 'JST XH vertical 2.50 mm; cable mapping unchanged')
        for ref in ('J2', 'J3', 'J4', 'J5')
    },
    'pcb_b_temperature': {
        'J4': (XH2, 'B2B-XH-A(LF)(SN)', 'JST XH vertical 2.50 mm; probe +3V3 / NTC_RAW'),
        'U3': ('Package_TO_SOT_SMD:SOT-23-5', 'SN74LVC1G08DBVR',
               'TI DBV: 1=A, 2=B, 3=GND, 4=Y, 5=VCC'),
    },
}


def main():
    for path in ROOT.glob('pcb_*/*.kicad_pcb'):
        assert not list(p.LoadBoard(str(path)).GetTracks()), 'Refuse to alter an already routed revision'
    layout = json.loads((ROOT/'placement.json').read_text(encoding='utf-8'))
    for name, selections in SELECTIONS.items():
        folder = ROOT/name
        path = folder/(name+'.kicad_sch')
        lines = path.read_text(encoding='utf-8').splitlines()
        found = set()
        for i, line in enumerate(lines):
            if not line.startswith('(symbol (lib_id '):
                continue
            node = parse(line)
            props = {unquote(x[1]): x for x in children(node, 'property')}
            ref = unquote(props['Reference'][2])
            if ref not in selections:
                continue
            fp, mpn, status = selections[ref]
            props['Footprint'][2] = q(fp)
            props['Design_Status'][2] = q('Package selected; '+status)
            if ref == 'U3':
                props['Value'][2] = q(mpn)
                props['Datasheet'][2] = q('https://www.ti.com/lit/ds/symlink/sn74lvc1g08.pdf')
            else:
                props['Datasheet'][2] = q('https://www.jst-mfg.com/product/pdf/eng/eXH.pdf')
            if 'MPN' not in props:
                prop = parse(dump(props['Design_Status']))
                prop[1:3] = [q('MPN'), q(mpn)]
                node.append(prop)
            else:
                props['MPN'][2] = q(mpn)
            lines[i] = dump(node)
            found.add(ref)
        assert found == set(selections), (name, found)
        path.write_text('\n'.join(lines)+'\n', encoding='utf-8')

        boardpath = folder/(name+'.kicad_pcb')
        board = p.LoadBoard(str(boardpath))
        assert not list(board.GetTracks()), 'Refuse to replace packages on a routed PCB'
        footprints = {fp.GetReference(): fp for fp in board.GetFootprints()}
        spec = next(b for b in layout['boards'] if b['name'] == name)
        for ref, (libid, mpn, status) in selections.items():
            old = footprints[ref]
            nick, footprint = libid.split(':')
            new = p.FootprintLoad(str(LIB/(nick+'.pretty')), footprint)
            assert new is not None, libid
            new.SetFPID(p.LIB_ID(nick, footprint))
            new.SetReference(ref)
            new.SetValue(mpn)
            new.SetPosition(old.GetPosition())
            new.SetOrientation(old.GetOrientation())
            new.SetPath(old.GetPath())
            # Read the existing schematic net assignments, including new pads.
            import xml.etree.ElementTree as ET
            xml = ET.parse(ROOT/'review'/(name+'.net.xml')).getroot()
            pin_nets = {(n.attrib['ref'], n.attrib['pin']): net.attrib['name']
                        for net in xml.findall('./nets/net') for n in net.findall('node')}
            netmap = {net.GetNetname(): net for net in board.GetNetsByNetcode().values()}
            for pad in new.Pads():
                pad.SetNet(netmap[pin_nets[(ref, pad.GetNumber())]])
            board.Remove(old)
            board.Add(new)
            new.Reference().SetVisible(True)
            new.Value().SetVisible(False)
            row = next(r for r in spec['components'] if r['reference'] == ref)
            bounds = new.GetBoundingBox(False, False)
            row.update(footprint=libid, value=mpn, status='Package selected; '+status,
                       bounds_mm=[round(p.ToMM(v), 3) for v in
                                  (bounds.GetLeft()-p.FromMM(60), bounds.GetTop()-p.FromMM(55),
                                   bounds.GetWidth(), bounds.GetHeight())])
            row['height_mm'] = 9.8 if ref.startswith('J') else 1.45
        # Reserve continuous side support lips introduced by enclosure revision 0.4.
        from create_placement import rule_area, rect
        width, height, _ = spec['board_mm']
        if not spec.get('side_support_bands_mm'):
            for x in (0, width-1.5):
                rule_area(board, x, 4, 1.5, height-8, True)
                rect(board, x, 4, 1.5, height-8, p.Cmts_User)
            spec['side_support_bands_mm'] = [[0, 4, 1.5, height-8], [width-1.5, 4, 1.5, height-8]]
        p.SaveBoard(str(boardpath), board)

        bompath = folder/(name+'_bom_draft.csv')
        with bompath.open(encoding='utf-8', newline='') as f:
            reader = csv.DictReader(f); fields = reader.fieldnames; rows = list(reader)
        if 'MPN' not in fields:
            fields.append('MPN')
        for row in rows:
            if row['Reference'] in selections:
                fp, mpn, status = selections[row['Reference']]
                row.update(Footprint=fp, MPN=mpn, Design_Status='Package selected; '+status)
                if row['Reference'] == 'U3': row['Value'] = mpn
        with bompath.open('w', encoding='utf-8', newline='') as f:
            writer = csv.DictWriter(f, fields); writer.writeheader(); writer.writerows(rows)
    (ROOT/'placement.json').write_text(json.dumps(layout, indent=2, ensure_ascii=False)+'\n', encoding='utf-8')


if __name__ == '__main__':
    main()
